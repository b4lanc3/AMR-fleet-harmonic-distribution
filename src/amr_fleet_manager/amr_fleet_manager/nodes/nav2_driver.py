"""Drive an AMR with Nav2 along the lane route granted by the fleet manager.

The fleet manager decides *where* and *how far* each robot may drive (traffic
control on the lane graph).  This driver turns the granted part of the route
into a dense nav_msgs/Path and hands it to the robot's Nav2 controller_server
(FollowPath action, Regulated Pure Pursuit with the local costmap).  The lane
speed limit of the current segment goes to the controller on `speed_limit`.

The robot pose is the AMCL estimate: map -> odom is taken from `amcl_pose`
(matched with the wheel odometry at the scan stamp) and composed with the
latest wheel odometry, which is what TF does, without subscribing to the
fleet-wide /tf in Python.  The collision_monitor of Nav2 provides the
lidar + 3D camera safety stop.

Whenever traffic control extends the grant the path is re-sent (preempting
the running goal), so the robot keeps rolling instead of stopping at every
node.  A robot that has to wait simply receives a path that ends at the last
granted node.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import CollisionMonitorState, SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ..core.controller import MODE_FOLLOW, PathFollower, wrap
from .docking_client import DOCKED, DOCKING, IDLE, UNDOCKING, ChargerDocking


def _quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _compose(a, b):
    """a * b for planar poses (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def _inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class Nav2Driver:
    PATH_STEP = 0.10          # m between poses of the path sent to the controller
    RESEND_PERIOD = 0.35      # s, min time between two goals
    SOFT_DECEL = 0.6          # m/s^2: early, gentle braking towards the end of the granted segment
    STOP_MARGIN = 0.25        # m before the granted end
    YAW_TOL = 0.12
    ARRIVE_TOL = 0.20         # m: end of the route reached (Nav2 keeps approaching to 0.10)
    MAX_STAMP_GAP_NS = 100_000_000   # amcl_pose vs odometry sample used for map -> odom

    def __init__(self, node: Node, robot_id: str, follower: PathFollower,
                 map_frame: str = 'map') -> None:
        self.node = node
        self.rid = robot_id
        self.f = follower
        self.map_frame = map_frame
        self.client = ActionClient(node, FollowPath, 'follow_path')
        self.goal_handle = None
        self.goal_seq = 0
        self.goal_key: Optional[tuple] = None
        self.goal_done = True
        self.goal_ok_key: Optional[tuple] = None
        self.last_send = -math.inf
        self.last_done = -math.inf
        self.last_error = ''
        self.obstacle_stop = False
        self.server_ready = False
        self.odom_hist: deque = deque(maxlen=120)     # (stamp_ns, (x, y, yaw)) ~4 s
        self.map_odom: Optional[Tuple[float, float, float]] = None
        self.speed_limit = -1.0
        self.limit_pub = node.create_publisher(SpeedLimit, 'speed_limit', QoSProfile(depth=2))
        self.clear_cli = node.create_client(ClearEntireCostmap,
                                            'local_costmap/clear_entirely_local_costmap')
        self.last_clear = -math.inf
        self.docking = ChargerDocking(node, robot_id)
        self.dock_id = ''           # charger at the end of the current route (RouteCommand)
        self.last_cmd = -1
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, latched)
        node.create_subscription(CollisionMonitorState, 'collision_monitor_state',
                                 self._on_cm_state, QoSProfile(depth=5))

    # ------------------------------------------------------------- inputs
    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        st = msg.header.stamp
        self.odom_hist.append((st.sec * 1_000_000_000 + st.nanosec,
                               (p.position.x, p.position.y, _yaw(p.orientation))))

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        if not self.odom_hist:
            return
        st = msg.header.stamp
        t = st.sec * 1_000_000_000 + st.nanosec
        stamp, odom = min(self.odom_hist, key=lambda e: abs(e[0] - t))
        if abs(stamp - t) > self.MAX_STAMP_GAP_NS:
            return        # AMCL lags behind the odometry history: keep the last correction
        p = msg.pose.pose
        est = (p.position.x, p.position.y, _yaw(p.orientation))
        self.map_odom = _compose(est, _inverse(odom))

    def _on_cm_state(self, msg: CollisionMonitorState) -> None:
        self.obstacle_stop = msg.action_type == CollisionMonitorState.STOP

    def pose(self) -> Optional[Tuple[float, float, float]]:
        """AMCL pose of the robot in the map (None until AMCL is localised)."""
        if self.map_odom is None or not self.odom_hist:
            return None
        return _compose(self.map_odom, self.odom_hist[-1][1])

    # --------------------------------------------------------------- step
    def step(self, pose: Tuple[float, float, float], speed: float, now: float) -> None:
        f, s = self.f, self.f.s
        if s.mode != MODE_FOLLOW or not s.waypoints or s.granted < 0:
            self.cancel()
            return
        if self._docking_step(s):
            return
        if not self.server_ready:
            self.server_ready = self.client.server_is_ready()
            if not self.server_ready:
                return
        x, y, yaw = pose
        f._update_reached(x, y)
        self._advance(x, y)
        last = len(s.waypoints) - 1
        d_end = math.dist((x, y), s.waypoints[last])
        if s.granted == last and s.reached < last and (
                d_end < self.ARRIVE_TOL or
                (self.goal_ok_key == (s.command_id, last) and d_end < 0.30)):
            s.reached = last          # at the station / dock (AMCL accuracy)
        self._limit_speed(s, x, y)
        if s.reached >= last:
            yaw_ok = math.isnan(s.final_yaw) or abs(wrap(s.final_yaw - yaw)) < self.YAW_TOL
            if yaw_ok and (self.goal_done or abs(speed) < 0.03 or d_end < 0.08):
                self.cancel()
                if self.dock_id and self.docking.at_staging(self.dock_id, s.command_id, now):
                    return        # docking on the charger, the route completes once docked
                s.complete = True
                return
        elif s.reached >= s.granted:
            # waiting at the end of the granted segment
            if not self.goal_done and abs(speed) < 0.02:
                self.cancel()
            return
        key = (s.command_id, s.granted, s.rev)
        stalled_goal = self.goal_done and now - self.last_done > 1.0
        if key == self.goal_key and not stalled_goal:
            return
        if now - self.last_send < self.RESEND_PERIOD:
            return
        self._send(self._build_path(x, y, yaw), key, now)

    def _docking_step(self, s) -> bool:
        """Charger docking around the route.  True while it owns the robot."""
        dk = self.docking
        if s.command_id != self.last_cmd:
            self.last_cmd = s.command_id
            dk.reset_arrival()
            if dk.state == IDLE:
                dk.attempts = 0
        if dk.state == DOCKED:
            if s.command_id == dk.command_id or (
                    len(s.waypoints) == 1 and self.dock_id == dk.dock_id):
                # docked, or the fleet keeps the robot on this charger
                dk.command_id = s.command_id
                s.reached = len(s.waypoints) - 1
                s.complete = True
                return True
            self.cancel()
            dk.undock()               # back to the staging pose before the new route
            return True
        if dk.state in (DOCKING, UNDOCKING):
            self.cancel()             # FollowPath must not fight the docking server
            return True
        return False

    def _advance(self, x: float, y: float) -> None:
        """Progress along the route by projection on the granted polyline.

        Regulated Pure Pursuit cuts corners by up to its lookahead, so a corner
        node may never come within the follower's reach tolerance.  The robot
        has passed node k once it projects onto segment k -> k+1 (searched a
        few metres ahead only, so a route that passes close to itself is not
        skipped).  The last node keeps the strict final tolerance."""
        s = self.f.s
        wps, cum = s.waypoints, s.cum
        last = len(wps) - 1
        i0 = max(s.reached, 0)
        end = min(s.granted, last)
        best = None
        k = i0
        while k < end and cum[k] - cum[i0] < 4.0:
            (ax, ay), (bx, by) = wps[k], wps[k + 1]
            L2 = (bx - ax) ** 2 + (by - ay) ** 2
            t = 0.0 if L2 < 1e-9 else min(max(((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / L2,
                                               0.0), 1.0)
            d = math.hypot(ax + t * (bx - ax) - x, ay + t * (by - ay) - y)
            if best is None or d <= best[0] + 1e-6:
                best = (d, k, t)
            k += 1
        if best is None or best[0] > 1.0:
            return
        _, k, t = best
        if k + 1 < last and (t >= 0.999 or math.dist((x, y), wps[k + 1]) < self.f.p.reach_tol):
            k += 1
        if k > s.reached:
            s.reached = k

    def _limit_speed(self, s, x: float, y: float) -> None:
        last = len(s.waypoints) - 1
        i = min(s.reached + 1, last)
        lim = s.speed_limits[i] if 0 <= i < len(s.speed_limits) else self.f.p.max_speed
        lim = float(min(lim, self.f.p.max_speed))
        if s.granted < last and s.reached < s.granted:
            # the traffic grant ends before the route: slow down early and
            # gently (instead of full speed then a hard stop) - most of the time
            # the grant is extended before the robot gets there and it never stops
            d_rem = math.dist((x, y), s.waypoints[i]) + s.cum[s.granted] - s.cum[i]
            v = math.sqrt(2.0 * self.SOFT_DECEL * max(0.0, d_rem - self.STOP_MARGIN))
            lim = min(lim, max(0.15, round(v / 0.05) * 0.05))
        if abs(lim - self.speed_limit) > 1e-3:
            self.speed_limit = lim
            msg = SpeedLimit()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.percentage = False
            msg.speed_limit = lim
            self.limit_pub.publish(msg)

    def cancel(self) -> None:
        if self.goal_handle is not None and not self.goal_done:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal_done = True
        self.goal_key = None

    # ------------------------------------------------------------ helpers
    def _build_path(self, x: float, y: float, yaw: float) -> Path:
        s = self.f.s
        last = len(s.waypoints) - 1
        pts = [(x, y)] + list(s.waypoints[s.reached + 1:s.granted + 1])
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.node.get_clock().now().to_msg()
        heading = yaw
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            L = math.hypot(bx - ax, by - ay)
            if L < 1e-3:
                continue
            heading = math.atan2(by - ay, bx - ax)
            n = max(1, int(math.ceil(L / self.PATH_STEP)))
            for i in range(n):
                path.poses.append(self._pose(ax + (bx - ax) * i / n, ay + (by - ay) * i / n, heading))
        end = pts[-1]
        final = heading
        if s.granted == last and not math.isnan(s.final_yaw):
            final = s.final_yaw
        path.poses.append(self._pose(end[0], end[1], final))
        if len(path.poses) == 1:   # only a rotation is left
            path.poses.insert(0, self._pose(x, y, yaw))
        return path

    def _pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        qx, qy, qz, qw = _quat(yaw)
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        return p

    def _send(self, path: Path, key: tuple, now: float) -> None:
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'
        self.goal_seq += 1
        seq = self.goal_seq
        self.goal_key = key
        self.goal_done = False
        self.last_send = now
        fut = self.client.send_goal_async(goal)
        fut.add_done_callback(lambda fu, q=seq: self._on_goal(fu, q))

    def _on_goal(self, fut, seq: int) -> None:
        handle = fut.result()
        if seq != self.goal_seq:
            return
        if handle is None or not handle.accepted:
            self.goal_done = True
            self.last_done = self._now()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda fu, q=seq: self._on_result(fu, q))

    def _on_result(self, fut, seq: int) -> None:
        if seq != self.goal_seq:
            return
        res = fut.result()
        self.goal_done = True
        self.last_done = self._now()
        if res is not None and res.status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_ok_key = self.goal_key
        elif res is not None and res.status == GoalStatus.STATUS_ABORTED:
            err = getattr(res.result, 'error_msg', '') or str(getattr(res.result, 'error_code', ''))
            if err != self.last_error:
                self.node.get_logger().warning(f'{self.rid}: FollowPath aborted ({err}), retrying')
                self.last_error = err
            self._clear_costmap()

    def _clear_costmap(self) -> None:
        """Recovery like Nav2's default behaviour tree: clear the local costmap
        after a controller failure (stale marks of robots that drove away).
        Real obstacles are re-marked by the next scan; traffic control keeps the
        robots apart and the collision monitor still guards the robot."""
        now = self._now()
        if now - self.last_clear < 3.0 or not self.clear_cli.service_is_ready():
            return
        self.last_clear = now
        self.clear_cli.call_async(ClearEntireCostmap.Request())

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9
