"""ROS 2 wrapper of FleetCore.

Topics (global):
  /fleet/state          amr_fleet_msgs/FleetState       robots, tasks, chargers (2 Hz)
  /fleet/production     amr_fleet_msgs/ProductionState  lines and KPIs (1 Hz)
  /fleet/events         amr_fleet_msgs/FleetEvent       task / traffic / yield / charge log
  /fleet/snapshot_json  std_msgs/String                 everything as JSON (dashboard)
  /fleet/markers        visualization_msgs/MarkerArray  dynamic RViz markers
  /fleet/static_markers visualization_msgs/MarkerArray  layout (latched)
  /factory/line_state   std_msgs/String                 every line, carton and product on the way
                                                        -> FactoryLines system of Gazebo (4 Hz)
  /map                  nav_msgs/OccupancyGrid          factory map (latched, publish_map:=true;
                                                        with Nav2 the map_server serves /map)
Per robot (<id> = amr_01 ...):
  /<id>/fleet/status    amr_fleet_msgs/RobotStatus      (subscribed)
  /<id>/fleet/route     amr_fleet_msgs/RouteCommand     (published)
Services:
  /fleet/submit_task        amr_fleet_msgs/SubmitTask
  /fleet/set_robot_priority amr_fleet_msgs/SetRobotPriority
  /fleet/pause              std_srvs/SetBool
  /fleet/demo_head_on       std_srvs/Trigger   (two robots meet head-on on the two-way link lane)
"""

from __future__ import annotations

import json
import math
from typing import Dict

import rclpy
from amr_fleet_msgs.msg import (FleetEvent, FleetState, LineState, ProductionState,
                                RobotState, RobotStatus, RouteCommand, TaskInfo)
from amr_fleet_msgs.srv import SetRobotPriority, SubmitTask
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import MarkerArray

from ..core.fleet import Command, FleetCore, FleetParams, RobotStatusIn
from ..core.layout import all_objects, build_graph, graph_to_dict, load_layout
from .visualization import dynamic_markers, occupancy_grid, static_markers
from .executor import spin_nodes


def robot_ids_from(n: int, prefix: str) -> list:
    return [f'{prefix}{i + 1:02d}' for i in range(n)]


class FleetManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('fleet_manager')
        self.declare_parameter('layout_file', '')
        self.declare_parameter('num_robots', 12)
        self.declare_parameter('robot_prefix', 'amr_')
        self.declare_parameter('robot_ids', [''])
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('state_rate', 2.0)
        self.declare_parameter('marker_rate', 4.0)
        self.declare_parameter('heartbeat', 1.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('priority_mode', 'task')
        self.declare_parameter('battery_low', 30.0)
        self.declare_parameter('battery_critical', 15.0)
        self.declare_parameter('battery_full', 95.0)
        self.declare_parameter('battery_opportunistic', 50.0)
        self.declare_parameter('station_capacity', 2)
        self.declare_parameter('start_paused', False)
        self.declare_parameter('production', True)
        self.declare_parameter('station_handover', False)
        # Nav2 mode: map_server publishes the AMCL map and AMCL publishes map -> odom
        self.declare_parameter('publish_map', True)
        self.declare_parameter('publish_odom_tf', True)

        gp = self.get_parameter
        layout_file = gp('layout_file').value
        if not layout_file:
            from ament_index_python.packages import get_package_share_directory
            import os
            layout_file = os.path.join(get_package_share_directory('amr_bringup'), 'config',
                                       'factory_layout.yaml')
        self.layout = load_layout(layout_file)
        ids = [i for i in gp('robot_ids').value if i]
        if not ids:
            ids = robot_ids_from(gp('num_robots').value, gp('robot_prefix').value)
        self.robot_ids = ids
        params = FleetParams(priority_mode=gp('priority_mode').value,
                             battery_low=gp('battery_low').value,
                             battery_critical=gp('battery_critical').value,
                             battery_full=gp('battery_full').value,
                             battery_opportunistic=gp('battery_opportunistic').value,
                             station_capacity=gp('station_capacity').value,
                             production=gp('production').value,
                             station_handover=gp('station_handover').value)
        self.graph = build_graph(self.layout)
        self.core = FleetCore(self.layout, ids, params, graph=self.graph)
        if gp('start_paused').value:
            self.core.set_paused(True)
        self.frame = gp('frame_id').value
        self.heartbeat = gp('heartbeat').value
        self.robot_index = {rid: i for i, rid in enumerate(ids)}

        qos = QoSProfile(depth=10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.route_pubs = {}
        self.last_sent: Dict[str, tuple] = {}
        self.last_sent_time: Dict[str, float] = {}
        for rid in ids:
            self.create_subscription(RobotStatus, f'/{rid}/fleet/status', self._on_status, qos)
            self.route_pubs[rid] = self.create_publisher(RouteCommand, f'/{rid}/fleet/route', qos)
        self.state_pub = self.create_publisher(FleetState, '/fleet/state', qos)
        self.prod_pub = self.create_publisher(ProductionState, '/fleet/production', qos)
        self.event_pub = self.create_publisher(FleetEvent, '/fleet/events', QoSProfile(depth=100))
        self.json_pub = self.create_publisher(String, '/fleet/snapshot_json', qos)
        # Gazebo station arms: "<station>:<robot>" when they took / put the load
        self.create_subscription(String, '/factory/station_handover', self._on_handover, qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/fleet/markers', qos)
        self.static_pub = self.create_publisher(MarkerArray, '/fleet/static_markers', latched)
        self.publish_map = gp('publish_map').value
        self.publish_odom_tf = gp('publish_odom_tf').value
        if self.publish_map:
            self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.layout_pub = self.create_publisher(String, '/fleet/layout_json', latched)
        # cartons on the pedestals and products on the belts of Gazebo follow the MES
        self.line_pub = self.create_publisher(String, '/factory/line_state', qos)

        self.create_service(SubmitTask, '/fleet/submit_task', self._srv_submit)
        self.create_service(SetRobotPriority, '/fleet/set_robot_priority', self._srv_priority)
        self.create_service(SetBool, '/fleet/pause', self._srv_pause)
        self.create_service(Trigger, '/fleet/demo_head_on', self._srv_demo)

        self.static_tf = StaticTransformBroadcaster(self)
        self._event_seq = 0
        self._snapshot = None
        self.create_timer(1.0 / gp('rate').value, self._tick)
        self.create_timer(1.0 / gp('state_rate').value, self._publish_state)
        self.create_timer(1.0 / gp('marker_rate').value, self._publish_markers)
        self.create_timer(1.0, self._publish_production)
        self.create_timer(0.25, self._publish_lines)
        self._publish_static()
        self.get_logger().info(
            f'Fleet manager: {len(ids)} robots, {len(self.graph.nodes)} graph nodes, '
            f'{len(self.graph.stations)} stations, {len(self.graph.chargers)} chargers, '
            f'{len(self.graph.yield_bays)} yield bays')
        for w in getattr(self.graph, 'build_warnings', []):
            self.get_logger().warning(w)

    # ------------------------------------------------------------- helpers
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_handover(self, msg: String) -> None:
        station, _, rid = msg.data.partition(':')
        self.core.station_handover(station, rid)

    def _on_status(self, msg: RobotStatus) -> None:
        self.core.update_status(RobotStatusIn(
            msg.robot_id, msg.pose.x, msg.pose.y, msg.pose.theta, msg.linear_velocity,
            msg.angular_velocity, msg.command_id, msg.reached_index, msg.route_complete,
            msg.battery_percent, msg.charging, msg.obstacle_stop, msg.min_obstacle_distance,
            self._now()))

    def _send(self, c: Command, now: float) -> None:
        key = c.key()
        if self.last_sent.get(c.robot_id) == key and \
                now - self.last_sent_time.get(c.robot_id, -1e9) < self.heartbeat:
            return
        self.last_sent[c.robot_id] = key
        self.last_sent_time[c.robot_id] = now
        msg = RouteCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.command_id = c.command_id
        msg.mode = c.mode
        msg.node_ids = c.node_ids
        msg.waypoints = [Point(x=float(x), y=float(y), z=0.0) for x, y in c.waypoints]
        msg.speed_limits = [float(v) for v in c.speed_limits]
        msg.granted_index = int(c.granted_index)
        msg.final_yaw = float(c.final_yaw)
        msg.charge = bool(c.charge)
        msg.carrying = bool(c.carrying)
        if c.node_ids and c.node_ids[-1] in self.graph.chargers:
            msg.dock_id = c.node_ids[-1]
        self.route_pubs[c.robot_id].publish(msg)

    def _tick(self) -> None:
        now = self._now()
        cmds = self.core.tick(now)
        for c in cmds.values():
            self._send(c, now)
        self._flush_events()

    def _flush_events(self) -> None:
        for e in list(self.core.events):
            if e['seq'] <= self._event_seq:
                continue
            self._event_seq = e['seq']
            msg = FleetEvent()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.level = e['level']
            msg.category = e['category']
            msg.robot_id = e['robot']
            msg.text = e['text']
            self.event_pub.publish(msg)
            # rclpy forbids one call site logging with different severities
            text = f"[{e['category']}] {e['text']}"
            if e['level'] >= 1:
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)

    # ----------------------------------------------------------- publishing
    def _publish_state(self) -> None:
        snap = self.core.snapshot()
        self._snapshot = snap
        msg = FleetState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.paused = snap['paused']
        msg.sim_time_s = float(snap['sim_time_s'])
        for r in snap['robots']:
            rs = RobotState()
            for k in ('robot_id', 'state', 'state_detail', 'cargo', 'task_id', 'task_type',
                      'task_description', 'pickup', 'dropoff', 'current_goal', 'route_summary',
                      'next_task_id', 'next_task_description', 'blocked_by', 'yielding_to'):
                setattr(rs, k, str(r[k]))
            rs.priority_rank = int(r['priority_rank'])
            rs.priority_score = float(r['priority_score'])
            rs.base_rank = int(r['base_rank'])
            rs.pose.x, rs.pose.y, rs.pose.theta = (float(r['pose']['x']), float(r['pose']['y']),
                                                   float(r['pose']['theta']))
            rs.speed = float(r['speed'])
            rs.battery_percent = float(r['battery_percent'])
            rs.charging = bool(r['charging'])
            for k in ('eta_goal_s', 'eta_task_s', 'distance_remaining_m',
                      'next_task_eta_start_s', 'waiting_time_s', 'odometer_m'):
                setattr(rs, k, float(r[k]))
            rs.route_node_ids = list(r['route_node_ids'])
            rs.route_points = [Point(x=float(x), y=float(y), z=0.0) for x, y in r['route_points']]
            rs.granted_count = int(r['granted_count'])
            rs.tasks_completed = int(r['tasks_completed'])
            msg.robots.append(rs)
        for t in snap['tasks']:
            ti = TaskInfo()
            for k in ('task_id', 'task_type', 'description', 'status', 'pickup', 'dropoff',
                      'robot_id', 'next_for_robot'):
                setattr(ti, k, str(t[k]))
            ti.priority = int(t['priority'])
            ti.age_s = float(t['age_s'])
            ti.eta_s = float(t['eta_s'])
            msg.tasks.append(ti)
        msg.charger_ids = snap['charger_ids']
        msg.charger_occupants = snap['charger_occupants']
        msg.charger_queue = snap['charger_queue']
        msg.tasks_completed = int(snap['tasks_completed'])
        msg.yield_maneuvers = int(snap['yield_maneuvers'])
        msg.deadlocks_resolved = int(snap['deadlocks_resolved'])
        self.state_pub.publish(msg)

        full = dict(snap)
        full['production'] = self.core.production.snapshot()
        full['events'] = list(self.core.events)[-60:]
        full['time'] = self._now()
        self.json_pub.publish(String(data=json.dumps(clean_json(full), default=_json_default)))

    def _publish_production(self) -> None:
        p = self.core.production.snapshot()
        msg = ProductionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.elapsed_s = float(p['elapsed_s'])
        for ln in p['lines']:
            ls = LineState()
            for k in ('line_id', 'line_type', 'name', 'status'):
                setattr(ls, k, str(ln[k]))
            for k in ('input_buffer', 'input_capacity', 'inbound', 'work_in_progress',
                      'output_buffer', 'output_capacity', 'produced'):
                setattr(ls, k, int(ln[k]))
            ls.utilization = float(ln['utilization'])
            msg.lines.append(ls)
        for k in ('kits_delivered', 'products_made', 'boxes_transferred', 'products_packaged',
                  'finished_stored'):
            setattr(msg, k, int(p[k]))
        msg.throughput_per_hour = float(p['throughput_per_hour'])
        self.prod_pub.publish(msg)

    def _publish_lines(self) -> None:
        self.line_pub.publish(String(data=self.core.production.gz_state()))

    def _publish_markers(self) -> None:
        if self._snapshot is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(dynamic_markers(self._snapshot, self.robot_index, self.frame,
                                                stamp, self.graph))

    def _publish_static(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.static_pub.publish(static_markers(self.layout, self.graph, self.frame, stamp))
        objects = all_objects(self.layout, self.graph)
        if self.publish_map:
            self.map_pub.publish(occupancy_grid(self.layout, self.frame, stamp, objects=objects))
        layout = dict(self.layout)
        layout['objects'] = objects
        self.layout_pub.publish(String(data=json.dumps(clean_json(
            {'layout': layout, 'graph': graph_to_dict(self.graph),
             'robots': self.robot_ids}), default=_json_default)))
        if not self.publish_odom_tf:
            return
        # map -> <robot>/odom (ground-truth odometry is map aligned)
        tfs = []
        for rid in self.robot_ids:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.frame
            t.child_frame_id = f'{rid}/odom'
            t.transform.rotation.w = 1.0
            tfs.append(t)
        self.static_tf.sendTransform(tfs)

    # ------------------------------------------------------------- services
    def _srv_submit(self, req, res):
        ok, tid, msg = self.core.submit_task(req.pickup, req.dropoff, req.priority,
                                             req.description)
        res.accepted, res.task_id, res.message = ok, tid, msg
        return res

    def _srv_priority(self, req, res):
        ok = True
        msgs = []
        if req.base_rank > 0:
            ok = self.core.set_base_rank(req.robot_id, req.base_rank)
            msgs.append(f'rank={req.base_rank}')
        if req.set_paused:
            ok = self.core.set_robot_paused(req.robot_id, req.paused) and ok
            msgs.append(f'paused={req.paused}')
        res.success = ok
        res.message = ', '.join(msgs) if ok else f'unknown robot {req.robot_id}'
        return res

    def _srv_demo(self, req, res):
        res.success, res.message = self.core.start_demo_head_on()
        return res

    def _srv_pause(self, req, res):
        self.core.set_paused(bool(req.data))
        res.success = True
        res.message = 'paused' if req.data else 'running'
        return res


def clean_json(o):
    """Replace NaN / inf (invalid JSON) by None, recursively."""
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {k: clean_json(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean_json(v) for v in o]
    if isinstance(o, set):
        return [clean_json(v) for v in sorted(o)]
    return o


def _json_default(o):
    if isinstance(o, set):
        return sorted(o)
    if isinstance(o, float) and (math.isinf(o) or math.isnan(o)):
        return None
    return str(o)


def main(args=None) -> None:
    rclpy.init(args=args)
    spin_nodes([FleetManagerNode()])


if __name__ == '__main__':
    main()
