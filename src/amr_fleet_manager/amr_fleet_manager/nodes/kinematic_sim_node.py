"""Gazebo-free simulator for the whole fleet (for weak PCs and CI).

For each robot it provides exactly the interfaces the Gazebo bridge provides:
  sub /<id>/cmd_vel            geometry_msgs/Twist
  pub /<id>/odom               nav_msgs/Odometry   (map-aligned, like the gz OdometryPublisher)
  pub /<id>/scan               sensor_msgs/LaserScan (180 deg front lidar, sees walls,
                                                      objects and other robots)
  pub /<id>/joint_states       sensor_msgs/JointState (wheel angles for robot_state_publisher)
  pub /tf                      <id>/odom -> <id>/base_footprint
  pub /tf_static               map -> <id>/odom (identity)
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from ..core.layout import all_objects, build_graph, load_layout
from ..sim.headless import spawn_poses
from ..sim.kinematic import KinematicWorld


class KinematicSimNode(Node):
    def __init__(self) -> None:
        super().__init__('kinematic_sim')
        self.declare_parameter('layout_file', '')
        self.declare_parameter('num_robots', 12)
        self.declare_parameter('robot_prefix', 'amr_')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('scan_samples', 181)
        gp = self.get_parameter
        path = gp('layout_file').value
        if not path:
            from ament_index_python.packages import get_package_share_directory
            import os
            path = os.path.join(get_package_share_directory('amr_bringup'), 'config',
                                'factory_layout.yaml')
        layout = load_layout(path)
        graph = build_graph(layout)
        n = gp('num_robots').value
        prefix = gp('robot_prefix').value
        self.world = KinematicWorld(layout, objects=all_objects(layout, graph))
        self.ids = [f'{prefix}{i + 1:02d}' for i in range(n)]
        for rid, (x, y, yaw) in zip(self.ids, spawn_poses(graph, n)):
            self.world.add_robot(rid, x, y, yaw, 100.0)
        qos = QoSProfile(depth=10)
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_pubs, self.scan_pubs, self.js_pubs = {}, {}, {}
        for rid in self.ids:
            self.create_subscription(Twist, f'/{rid}/cmd_vel',
                                     lambda m, r=rid: self._on_cmd(r, m), qos)
            self.odom_pubs[rid] = self.create_publisher(Odometry, f'/{rid}/odom', qos)
            self.scan_pubs[rid] = self.create_publisher(LaserScan, f'/{rid}/scan', sensor_qos)
            self.js_pubs[rid] = self.create_publisher(JointState, f'/{rid}/joint_states', qos)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()
        statics = []
        for rid in self.ids:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'map'
            t.child_frame_id = f'{rid}/odom'
            t.transform.rotation.w = 1.0
            statics.append(t)
        self.static_tf.sendTransform(statics)
        self.dt = 1.0 / gp('rate').value
        self.samples = gp('scan_samples').value
        self.last = None
        self.create_timer(self.dt, self._step)
        self.create_timer(1.0 / gp('scan_rate').value, self._scan)
        self.get_logger().info(f'Kinematic simulator: {n} robots')

    def _on_cmd(self, rid: str, msg: Twist) -> None:
        r = self.world.robots[rid]
        r.cmd_v = max(-0.3, min(1.2, msg.linear.x))
        r.cmd_w = max(-1.5, min(1.5, msg.angular.z))

    def _step(self) -> None:
        now = self.get_clock().now()
        dt = self.dt if self.last is None else (now - self.last).nanoseconds * 1e-9
        self.last = now
        dt = min(max(dt, 0.0), 0.2)
        self.world.step(dt, {})
        stamp = now.to_msg()
        tfs = []
        for rid, r in self.world.robots.items():
            od = Odometry()
            od.header.stamp = stamp
            od.header.frame_id = f'{rid}/odom'
            od.child_frame_id = f'{rid}/base_footprint'
            od.pose.pose.position.x = r.x
            od.pose.pose.position.y = r.y
            od.pose.pose.orientation.z = math.sin(r.yaw / 2)
            od.pose.pose.orientation.w = math.cos(r.yaw / 2)
            od.twist.twist.linear.x = r.v
            od.twist.twist.angular.z = r.w
            self.odom_pubs[rid].publish(od)
            t = TransformStamped()
            t.header = od.header
            t.child_frame_id = od.child_frame_id
            t.transform.translation.x = r.x
            t.transform.translation.y = r.y
            t.transform.rotation = od.pose.pose.orientation
            tfs.append(t)
            js = JointState()
            js.header.stamp = stamp
            js.name = ['left_wheel_joint', 'right_wheel_joint']
            js.position = [r.wheel_left, r.wheel_right]
            self.js_pubs[rid].publish(js)
        self.tf.sendTransform(tfs)

    def _scan(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for rid in self.ids:
            ranges = self.world.scan(rid, n=self.samples)
            msg = LaserScan()
            msg.header.stamp = stamp
            msg.header.frame_id = f'{rid}/lidar_link'
            msg.angle_min = -math.pi / 2
            msg.angle_max = math.pi / 2
            msg.angle_increment = math.pi / (self.samples - 1)
            msg.range_min = 0.08
            msg.range_max = 12.0
            msg.ranges = [float(r) if r < 12.0 else float('inf') for r in ranges]
            self.scan_pubs[rid].publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KinematicSimNode()
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
