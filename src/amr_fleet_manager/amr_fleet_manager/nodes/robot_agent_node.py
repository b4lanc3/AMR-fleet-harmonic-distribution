"""On-board software of one AMR (runs once per robot, in its namespace).

* subscribes  odom (nav_msgs/Odometry, map frame), scan (sensor_msgs/LaserScan)
              fleet/route (amr_fleet_msgs/RouteCommand)
* publishes   cmd_vel (geometry_msgs/Twist), fleet/status (amr_fleet_msgs/RobotStatus),
              battery (sensor_msgs/BatteryState)

The path follower drives along the waypoints granted by the fleet manager and
applies the lidar safety field.  The battery is simulated here (drain while
driving, charge while docked and the fleet manager enables charging).
"""

from __future__ import annotations

import math

import rclpy
from amr_fleet_msgs.msg import RobotStatus, RouteCommand
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, LaserScan

from ..core.controller import MODE_HOLD, MODE_IDLE, FollowerParams, PathFollower


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotAgent(Node):
    def __init__(self) -> None:
        super().__init__('robot_agent')
        self.declare_parameter('robot_id', self.get_namespace().strip('/') or 'amr_01')
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

        gp = self.get_parameter
        self.robot_id = gp('robot_id').value
        fp = FollowerParams(max_speed=gp('max_speed').value,
                            safety_stop=gp('safety_stop_distance').value,
                            safety_slow=gp('safety_slow_distance').value)
        self.follower = PathFollower(fp)
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
        self.charge_enable = False
        self.charging = False
        self.last_cmd_time = None
        self.last_step = None

        qos = QoSProfile(depth=10)
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, 'odom', self._on_odom, qos)
        self.create_subscription(LaserScan, 'scan', self._on_scan, sensor_qos)
        self.create_subscription(RouteCommand, 'fleet/route', self._on_route, qos)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.status_pub = self.create_publisher(RobotStatus, 'fleet/status', qos)
        self.batt_pub = self.create_publisher(BatteryState, 'battery', qos)
        self.create_timer(1.0 / gp('control_rate').value, self._control)
        self.create_timer(1.0 / gp('status_rate').value, self._publish_status)
        self.get_logger().info(f'{self.robot_id}: agent ready (battery {self.battery:.0f}%)')

    # ---------------------------------------------------------- callbacks
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.v_meas = msg.twist.twist.linear.x
        self.w_meas = msg.twist.twist.angular.z

    def _on_scan(self, msg: LaserScan) -> None:
        best = math.inf
        a = msg.angle_min
        reverse = self.follower.s.v < -0.01
        for r in msg.ranges:
            ang = a if not reverse else a + math.pi
            ang = (ang + math.pi) % (2 * math.pi) - math.pi
            if abs(ang) <= self.half_angle and msg.range_min < r < msg.range_max:
                best = min(best, r)
            a += msg.angle_increment
        self.front_clearance = best

    def _on_route(self, msg: RouteCommand) -> None:
        self.last_cmd_time = self.get_clock().now()
        wps = [(p.x, p.y) for p in msg.waypoints]
        self.follower.set_command(msg.command_id, msg.mode, wps, list(msg.speed_limits),
                                  msg.granted_index, msg.final_yaw)
        self.charge_enable = bool(msg.charge)

    # ------------------------------------------------------------ control
    def _control(self) -> None:
        now = self.get_clock().now()
        dt = 0.05 if self.last_step is None else (now - self.last_step).nanoseconds * 1e-9
        self.last_step = now
        if dt <= 0.0 or dt > 1.0:
            dt = 0.05
        twist = Twist()
        if self.pose is not None:
            stale = (self.last_cmd_time is None
                     or (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout)
            if stale:
                self.follower.s.mode = MODE_HOLD if self.follower.s.waypoints else MODE_IDLE
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
