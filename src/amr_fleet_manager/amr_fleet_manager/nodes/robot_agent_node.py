"""On-board software of one AMR (runs once per robot, in its namespace).

* subscribes  odom (nav_msgs/Odometry), scan (sensor_msgs/LaserScan),
              camera/points (sensor_msgs/PointCloud2, RGB-D camera, optional),
              fleet/route (amr_fleet_msgs/RouteCommand)
* publishes   fleet/status (amr_fleet_msgs/RobotStatus), battery (sensor_msgs/BatteryState),
              cargo_lift (std_msgs/Float64)

Two drivers move the robot along the waypoints granted by the fleet manager:

* driver:=nav2    the granted route goes to the robot's Nav2 controller_server
                  (FollowPath, Regulated Pure Pursuit + local costmap), the pose
                  is the AMCL estimate and Nav2's collision_monitor (lidar + 3D
                  camera) publishes cmd_vel.  See nav2_driver.py.
* driver:=simple  built-in pure pursuit on map-aligned odometry with a safety
                  field fused from the 2D lidar and the 3D camera -> cmd_vel.

The battery is simulated here (drain while driving, charge while docked and
the fleet manager enables charging).
"""

from __future__ import annotations

import math

import rclpy
from amr_fleet_msgs.msg import RobotStatus, RouteCommand
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from sensor_msgs.msg import BatteryState, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

from ..core.controller import MODE_HOLD, MODE_IDLE, FollowerParams, PathFollower
from .nav2_driver import Nav2Driver


CARGO_UP = 0.235   # cargo_joint position with the load on the deck


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotAgent(Node):
    def __init__(self, namespace: str = None, overrides: dict = None) -> None:
        kw = {}
        if namespace is not None:
            kw['namespace'] = namespace
        if overrides:
            from rclpy.parameter import Parameter
            kw['parameter_overrides'] = [Parameter(k, value=v) for k, v in overrides.items()]
        super().__init__('robot_agent', **kw)
        self.declare_parameter('robot_id', self.get_namespace().strip('/') or 'amr_01')
        self.declare_parameter('driver', 'simple')       # simple | nav2
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('status_rate', 10.0)
        self.declare_parameter('initial_battery', 90.0)
        self.declare_parameter('battery_idle_drain', 0.004)
        self.declare_parameter('battery_move_drain', 0.030)
        self.declare_parameter('battery_charge_rate', 0.25)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('safety_stop_distance', 0.30)
        self.declare_parameter('safety_slow_distance', 0.85)
        self.declare_parameter('safety_half_angle_deg', 35.0)
        self.declare_parameter('command_timeout', 3.0)
        self.declare_parameter('simulate_battery', True)
        # 3D camera safety: obstacles below / above the lidar plane
        self.declare_parameter('use_camera_safety', True)
        self.declare_parameter('camera_x', 0.40)        # camera position in base_footprint [m]
        self.declare_parameter('camera_z', 0.31)
        self.declare_parameter('camera_pitch', 0.12)    # rad, positive = looking down
        self.declare_parameter('camera_timeout', 1.0)

        gp = self.get_parameter
        self.robot_id = gp('robot_id').value
        fp = FollowerParams(max_speed=gp('max_speed').value,
                            safety_stop=gp('safety_stop_distance').value,
                            safety_slow=gp('safety_slow_distance').value)
        self.follower = PathFollower(fp)
        driver = gp('driver').value
        if driver not in ('simple', 'nav2'):
            raise ValueError(f'unknown driver "{driver}" (simple | nav2)')
        self.nav2 = Nav2Driver(self, self.robot_id, self.follower) if driver == 'nav2' else None
        self.half_angle = math.radians(gp('safety_half_angle_deg').value)
        self.battery = float(gp('initial_battery').value)
        self.idle_drain = gp('battery_idle_drain').value
        self.move_drain = gp('battery_move_drain').value
        self.charge_rate = gp('battery_charge_rate').value
        self.cmd_timeout = gp('command_timeout').value
        self.sim_battery = gp('simulate_battery').value

        self.pose = None
        self.v_meas = 0.0
        self.w_meas = 0.0
        self.front_clearance = math.inf
        self.lidar_clearance = math.inf
        self.camera_clearance = math.inf
        self.camera_stamp = None
        self.charge_enable = False
        self.charging = False
        self.last_cmd_time = None
        self.last_step = None

        qos = QoSProfile(depth=10)
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, 'odom', self._on_odom, qos)
        self.create_subscription(LaserScan, 'scan', self._on_scan, sensor_qos)
        self.create_subscription(RouteCommand, 'fleet/route', self._on_route, qos)
        # with Nav2 the collision_monitor and the local costmap use the point cloud
        if gp('use_camera_safety').value and self.nav2 is None:
            self.cam_x, self.cam_z = gp('camera_x').value, gp('camera_z').value
            self.cam_pitch = gp('camera_pitch').value
            self.cam_timeout = gp('camera_timeout').value
            self.create_subscription(PointCloud2, 'camera/points', self._on_points, sensor_qos)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos) if self.nav2 is None else None
        self.cargo_pub = self.create_publisher(Float64, 'cargo_lift', qos)
        self.carrying = False
        self.status_pub = self.create_publisher(RobotStatus, 'fleet/status', qos)
        self.batt_pub = self.create_publisher(BatteryState, 'battery', qos)
        self.create_timer(1.0 / gp('control_rate').value, self._control)
        self.create_timer(1.0 / gp('status_rate').value, self._publish_status)
        self.get_logger().info(f'{self.robot_id}: agent ready, driver {driver} '
                               f'(battery {self.battery:.0f}%)')

    # ---------------------------------------------------------- callbacks
    def _on_odom(self, msg: Odometry) -> None:
        if self.nav2 is not None:
            self.nav2.on_odom(msg)      # wheel odometry, the pose comes from AMCL
        else:
            p = msg.pose.pose
            self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.v_meas = msg.twist.twist.linear.x
        self.w_meas = msg.twist.twist.angular.z

    def _on_scan(self, msg: LaserScan) -> None:
        r = np.asarray(msg.ranges, dtype=np.float32)
        ang = msg.angle_min + np.arange(r.size, dtype=np.float32) * msg.angle_increment
        if self.follower.s.v < -0.01:          # reversing: look backwards
            ang = (ang + 2.0 * math.pi) % (2.0 * math.pi) - math.pi
        ok = (np.abs(ang) <= self.half_angle) & (r > msg.range_min) & (r < msg.range_max)
        self.lidar_clearance = float(r[ok].min()) if ok.any() else math.inf
        self._fuse()

    def _on_points(self, msg: PointCloud2) -> None:
        """Closest 3D obstacle in front of the robot from the RGB-D point cloud.

        Gazebo publishes the cloud in the camera link frame (x forward, z up).
        Points are rotated by the camera pitch, lifted to base height, and the
        floor (z < 5 cm) and anything above 1.9 m are ignored."""
        try:
            pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception:  # pragma: no cover - malformed cloud
            return
        if pts.size == 0:
            self.camera_clearance = math.inf
        else:
            pts = pts[np.isfinite(pts).all(axis=1)][::3]
            c, s_ = math.cos(self.cam_pitch), math.sin(self.cam_pitch)
            x = c * pts[:, 0] + s_ * pts[:, 2]
            z = -s_ * pts[:, 0] + c * pts[:, 2] + self.cam_z
            y = pts[:, 1]
            m = (z > 0.05) & (z < 1.9) & (np.abs(y) < 0.36) & (x > 0.10)
            # report the distance as seen from the lidar position (same reference)
            self.camera_clearance = float(x[m].min()) + self.cam_x - 0.43 if m.any() else math.inf
        self.camera_stamp = self.get_clock().now()
        self._fuse()

    def _fuse(self) -> None:
        cam = self.camera_clearance
        if self.camera_stamp is not None and \
                (self.get_clock().now() - self.camera_stamp).nanoseconds * 1e-9 > self.cam_timeout:
            cam = math.inf
        self.front_clearance = min(self.lidar_clearance, cam)

    def _on_route(self, msg: RouteCommand) -> None:
        self.last_cmd_time = self.get_clock().now()
        wps = [(p.x, p.y) for p in msg.waypoints]
        self.follower.set_command(msg.command_id, msg.mode, wps, list(msg.speed_limits),
                                  msg.granted_index, msg.final_yaw)
        self.charge_enable = bool(msg.charge)
        if bool(msg.carrying) != self.carrying:
            self.carrying = bool(msg.carrying)
            self.cargo_pub.publish(Float64(data=CARGO_UP if self.carrying else 0.0))

    # ------------------------------------------------------------ control
    def _control(self) -> None:
        now = self.get_clock().now()
        dt = 0.05 if self.last_step is None else (now - self.last_step).nanoseconds * 1e-9
        self.last_step = now
        if dt <= 0.0 or dt > 1.0:
            dt = 0.05
        if self.nav2 is not None:
            self.pose = self.nav2.pose() or self.pose
        stale = (self.last_cmd_time is None
                 or (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout)
        if self.pose is not None and stale:
            self.follower.s.mode = MODE_HOLD if self.follower.s.waypoints else MODE_IDLE
        if self.nav2 is not None:
            if self.pose is not None:
                self.nav2.step(self.pose, self.v_meas, now.nanoseconds * 1e-9)
                # the collision monitor only reports while Nav2 is driving
                self.follower.s.obstacle_stop = self.nav2.obstacle_stop and not self.nav2.goal_done
        else:
            twist = Twist()
            if self.pose is not None:
                v, w = self.follower.step(self.pose[0], self.pose[1], self.pose[2], dt,
                                          self.front_clearance)
                twist.linear.x = v
                twist.angular.z = w
            self.cmd_pub.publish(twist)
        if self.sim_battery:
            self.charging = self.charge_enable and abs(self.v_meas) < 0.03
            if self.charging:
                self.battery = min(100.0, self.battery + self.charge_rate * dt)
            else:
                drain = self.idle_drain + self.move_drain * abs(self.v_meas) + 0.01 * abs(self.w_meas)
                self.battery = max(0.0, self.battery - drain * dt)

    def _publish_status(self) -> None:
        if self.pose is None:
            return
        f = self.follower
        st = RobotStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'map'
        st.robot_id = self.robot_id
        st.pose.x, st.pose.y, st.pose.theta = self.pose
        st.linear_velocity = self.v_meas
        st.angular_velocity = self.w_meas
        st.command_id = f.s.command_id
        st.reached_index = f.reached_index
        st.route_complete = f.route_complete
        st.battery_percent = float(self.battery)
        st.charging = self.charging
        st.obstacle_stop = f.s.obstacle_stop
        st.min_obstacle_distance = float(min(self.front_clearance, 99.0))
        self.status_pub.publish(st)
        b = BatteryState()
        b.header = st.header
        b.percentage = float(self.battery / 100.0)
        b.voltage = float(44.0 + 10.0 * self.battery / 100.0)
        b.present = True
        b.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.charging
                                 else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        self.batt_pub.publish(b)
        self.cargo_pub.publish(Float64(data=CARGO_UP if self.carrying else 0.0))


def main_fleet(args=None) -> None:
    """All simulated robot agents in one process (one DDS participant).

    ros2 run amr_fleet_manager fleet_agents --ros-args -p num_robots:=12
    Real robots run `robot_agent` on board instead."""
    rclpy.init(args=args)
    cfg = Node('fleet_agents')
    cfg.declare_parameter('num_robots', 12)
    cfg.declare_parameter('robot_prefix', 'amr_')
    cfg.declare_parameter('initial_batteries', [0.0])
    cfg.declare_parameter('driver', 'simple')
    n = cfg.get_parameter('num_robots').value
    prefix = cfg.get_parameter('robot_prefix').value
    batt = [b for b in cfg.get_parameter('initial_batteries').value]
    use_sim_time = cfg.get_parameter('use_sim_time').value
    driver = cfg.get_parameter('driver').value
    nodes = []
    for i in range(n):
        rid = f'{prefix}{i + 1:02d}'
        ov = {'robot_id': rid, 'use_sim_time': use_sim_time, 'driver': driver}
        if i < len(batt) and batt[i] > 0:
            ov['initial_battery'] = float(batt[i])
        nodes.append(RobotAgent(namespace=rid, overrides=ov))
    ex = rclpy.executors.SingleThreadedExecutor()
    for nd in nodes + [cfg]:
        ex.add_node(nd)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        for nd in nodes + [cfg]:
            nd.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotAgent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
