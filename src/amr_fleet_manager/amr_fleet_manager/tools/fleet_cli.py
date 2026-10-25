"""Terminal view of the fleet and small helpers around the fleet services.

  ros2 run amr_fleet_manager fleet_cli status            # live table (Ctrl+C to quit)
  ros2 run amr_fleet_manager fleet_cli task A1 D2 --priority 2
  ros2 run amr_fleet_manager fleet_cli rank amr_07 1     # make amr_07 the top base rank
  ros2 run amr_fleet_manager fleet_cli pause | resume
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from amr_fleet_msgs.msg import FleetState
from amr_fleet_msgs.srv import SetRobotPriority, SubmitTask
from rclpy.node import Node
from std_srvs.srv import SetBool


def fmt_s(s: float) -> str:
    if s is None or s != s or s > 1e6:
        return '-'
    s = int(max(0, s))
    return f'{s // 60}m{s % 60:02d}s' if s >= 60 else f'{s}s'


def table(msg: FleetState) -> str:
    head = (f'{"ROBOT":7} {"P":>3} {"STATE":12} {"TASK":6} {"GOAL":8} {"ETA":>6} {"ETA_TASK":>8} '
            f'{"BATT":>5}  {"ROUTE":34} {"NEXT":28} WAIT/YIELD')
    lines = [f't={fmt_s(msg.sim_time_s)}  tasks done={msg.tasks_completed}  '
             f'yields={msg.yield_maneuvers}  deadlocks={msg.deadlocks_resolved}  '
             f'{"PAUSED" if msg.paused else "running"}', head, '-' * len(head)]
    for r in msg.robots:
        wait = f'yield->{r.yielding_to}' if r.yielding_to else (f'wait {r.blocked_by}' if r.blocked_by else '')
        lines.append(
            f'{r.robot_id:7} {r.priority_rank:>3} {r.state:12} {r.task_id:6} {r.current_goal[:8]:8} '
            f'{fmt_s(r.eta_goal_s):>6} {fmt_s(r.eta_task_s) if r.task_id else "-":>8} '
            f'{r.battery_percent:5.0f}  {r.route_summary[:34]:34} {r.next_task_description[:28]:28} {wait}')
    return '\n'.join(lines)


class Cli(Node):
    def __init__(self) -> None:
        super().__init__('fleet_cli')


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description='AMR fleet command line')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('status')
    t = sub.add_parser('task')
    t.add_argument('pickup')
    t.add_argument('dropoff')
    t.add_argument('--priority', type=int, default=1)
    r = sub.add_parser('rank')
    r.add_argument('robot')
    r.add_argument('rank', type=int)
    p = sub.add_parser('pause_robot')
    p.add_argument('robot')
    p = sub.add_parser('resume_robot')
    p.add_argument('robot')
    sub.add_parser('pause')
    sub.add_parser('resume')
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = Cli()
    try:
        if args.cmd == 'status':
            def show(msg):
                sys.stdout.write('\x1b[2J\x1b[H' + table(msg) + '\n')
                sys.stdout.flush()
            node.create_subscription(FleetState, '/fleet/state', show, 10)
            rclpy.spin(node)
        elif args.cmd == 'task':
            res = _call(node, SubmitTask, '/fleet/submit_task',
                        pickup=args.pickup, dropoff=args.dropoff, priority=args.priority)
            print(res)
        elif args.cmd in ('rank', 'pause_robot', 'resume_robot'):
            kw = {'robot_id': args.robot}
            if args.cmd == 'rank':
                kw['base_rank'] = args.rank
            else:
                kw.update(set_paused=True, paused=args.cmd == 'pause_robot')
            print(_call(node, SetRobotPriority, '/fleet/set_robot_priority', **kw))
        elif args.cmd in ('pause', 'resume'):
            print(_call(node, SetBool, '/fleet/pause', data=args.cmd == 'pause'))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _call(node, srv_type, name, **fields):
    cli = node.create_client(srv_type, name)
    if not cli.wait_for_service(timeout_sec=5.0):
        return f'service {name} not available'
    req = srv_type.Request()
    for k, v in fields.items():
        setattr(req, k, v)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    return fut.result()


if __name__ == '__main__':
    main()
