"""Charger docking with Nav2's docking server (opennav_docking).

The fleet route of an AMR going to charge ends on the charger's dock node,
the staging pose.  There the agent asks the docking server to dock
(DockRobot, navigate_to_staging_pose=False): the server drives the last
0.5 m with its own controller towards the dock pose that this client detects
from the lidar (`core.docking.detect_dock_face`, published on
`detected_dock_pose`), until the bumper touches the charger's contact plate.
Before the next route leaves the charger the robot undocks (UndockRobot:
back to the staging pose).
"""

from __future__ import annotations

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot, UndockRobot
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan

from ..core.docking import detect_dock_face

IDLE, DOCKING, DOCKED, UNDOCKING = 'IDLE', 'DOCKING', 'DOCKED', 'UNDOCKING'


class ChargerDocking:
    SETTLE = 1.0          # s standing at the staging pose before docking (collision monitor hands over)
    MAX_ATTEMPTS = 2

    def __init__(self, node: Node, robot_id: str, lidar_x: float = 0.43) -> None:
        self.node = node
        self.rid = robot_id
        self.lidar_x = lidar_x
        self.dock_cli = ActionClient(node, DockRobot, 'dock_robot')
        self.undock_cli = ActionClient(node, UndockRobot, 'undock_robot')
        self.pose_pub = node.create_publisher(PoseStamped, 'detected_dock_pose', QoSProfile(depth=5))
        self.state = IDLE
        self.dock_id = ''
        self.command_id = -1        # command that docked the robot / is docking it
        self.attempts = 0
        self.failed_cmd = -1        # docking gave up for this command: charge at the staging pose
        self.arrived_at = None
        self.seq = 0
        self.detections = 0

    # ------------------------------------------------------------ detection
    def on_scan(self, msg: LaserScan) -> None:
        if self.state != DOCKING:
            return
        found = detect_dock_face(msg.ranges, msg.angle_min, msg.angle_increment,
                                 lidar_x=self.lidar_x)
        if found is None:
            return
        x, y, yaw = found
        p = PoseStamped()
        p.header.stamp = msg.header.stamp
        p.header.frame_id = f'{self.rid}/base_footprint'
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self.pose_pub.publish(p)
        self.detections += 1

    # ------------------------------------------------------------- control
    def wants_dock(self, command_id: int) -> bool:
        return self.state == IDLE and self.failed_cmd != command_id

    def at_staging(self, dock_id: str, command_id: int, now: float) -> bool:
        """Robot stands on the dock node: dock after a short settle.  Returns True
        while docking is in progress (the route is not complete yet)."""
        if self.state == DOCKING:
            return True
        if not self.wants_dock(command_id):
            return False
        if self.arrived_at is None:
            self.arrived_at = now
            return True
        if now - self.arrived_at < self.SETTLE:
            return True
        if not self.dock_cli.server_is_ready():
            self.node.get_logger().warning(f'{self.rid}: docking server not available, '
                                           f'charging on the staging pose')
            self.failed_cmd = command_id
            return False
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = dock_id
        goal.navigate_to_staging_pose = False
        self.state, self.dock_id, self.command_id = DOCKING, dock_id, command_id
        self.attempts += 1
        self.detections = 0
        self.seq += 1
        seq = self.seq
        self.dock_cli.send_goal_async(goal).add_done_callback(
            lambda fu, q=seq: self._on_goal(fu, q, self._on_dock_result))
        self.node.get_logger().info(f'{self.rid}: docking on {dock_id}')
        return True

    def undock(self) -> None:
        if self.state != DOCKED:
            return
        if not self.undock_cli.server_is_ready():
            self.state = IDLE
            return
        goal = UndockRobot.Goal()
        goal.dock_type = 'charger'
        self.state = UNDOCKING
        self.seq += 1
        seq = self.seq
        self.undock_cli.send_goal_async(goal).add_done_callback(
            lambda fu, q=seq: self._on_goal(fu, q, self._on_undock_result))
        self.node.get_logger().info(f'{self.rid}: undocking from {self.dock_id}')

    def reset_arrival(self) -> None:
        self.arrived_at = None

    # ------------------------------------------------------------ callbacks
    def _on_goal(self, fut, seq: int, on_result) -> None:
        handle = fut.result()
        if seq != self.seq:
            return
        if handle is None or not handle.accepted:
            on_result(None, seq)
            return
        handle.get_result_async().add_done_callback(lambda fu, q=seq: on_result(fu.result(), q))

    def _on_dock_result(self, res, seq: int) -> None:
        if seq != self.seq:
            return
        if res is not None and res.status == GoalStatus.STATUS_SUCCEEDED and res.result.success:
            self.state = DOCKED
            self.node.get_logger().info(f'{self.rid}: docked on {self.dock_id} '
                                        f'({self.detections} dock detections)')
            return
        code = getattr(res.result, 'error_code', -1) if res is not None else -1
        self.state = IDLE
        self.arrived_at = None
        if self.attempts >= self.MAX_ATTEMPTS:
            self.failed_cmd = self.command_id
            self.node.get_logger().warning(f'{self.rid}: docking on {self.dock_id} failed '
                                           f'(error {code}), charging on the staging pose')
        else:
            self.node.get_logger().warning(f'{self.rid}: docking on {self.dock_id} failed '
                                           f'(error {code}), retrying')

    def _on_undock_result(self, res, seq: int) -> None:
        if seq != self.seq:
            return
        ok = res is not None and res.status == GoalStatus.STATUS_SUCCEEDED
        if not ok:
            self.node.get_logger().warning(f'{self.rid}: undocking failed, driving on')
        self.state = IDLE
        self.attempts = 0
        self.arrived_at = None
