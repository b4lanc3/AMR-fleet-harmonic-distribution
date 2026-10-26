"""Web dashboard for the fleet (no external dependencies).

Open http://localhost:8080 while the simulation runs.  The page shows, for
every robot: what it is doing, its route, ETA, next task, priority rank,
battery, who it waits for / gives way to, plus a live 2D map, production
lines, the charging station and the event log.  It can also create manual
tasks, change a robot's base priority, pause a robot or the whole fleet.

Data source: /fleet/snapshot_json and /fleet/layout_json (fleet_manager).
"""

from __future__ import annotations

import json
import os
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import rclpy
from amr_fleet_msgs.srv import SetRobotPriority, SubmitTask
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

WEB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')
MIME = {'.html': 'text/html; charset=utf-8', '.js': 'application/javascript; charset=utf-8',
        '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml', '.ico': 'image/x-icon'}


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__('fleet_dashboard')
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('web_dir', '')
        self.web_dir = self.get_parameter('web_dir').value or self._find_web_dir()
        self.state: Optional[str] = None
        self.layout: Optional[str] = None
        self.cond = threading.Condition()
        self.version = 0
        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/fleet/snapshot_json', self._on_state, 10,
                                 callback_group=cb)
        self.create_subscription(String, '/fleet/layout_json', self._on_layout, latched,
                                 callback_group=cb)
        self.cli_task = self.create_client(SubmitTask, '/fleet/submit_task', callback_group=cb)
        self.cli_prio = self.create_client(SetRobotPriority, '/fleet/set_robot_priority',
                                           callback_group=cb)
        self.cli_pause = self.create_client(SetBool, '/fleet/pause', callback_group=cb)
        self.cli_demo = self.create_client(Trigger, '/fleet/demo_head_on', callback_group=cb)
        port = self.get_parameter('port').value
        host = self.get_parameter('host').value
        handler = _make_handler(self)
        self.httpd = ThreadingHTTPServer((host, port), handler)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f'Dashboard: http://localhost:{port}  (web dir {self.web_dir})')

    def _find_web_dir(self) -> str:
        try:
            from ament_index_python.packages import get_package_share_directory
            d = os.path.join(get_package_share_directory('amr_fleet_manager'), 'web')
            if os.path.isdir(d):
                return d
        except Exception:
            pass
        return WEB_DIR

    def _on_state(self, msg: String) -> None:
        with self.cond:
            self.state = msg.data
            self.version += 1
            self.cond.notify_all()

    def _on_layout(self, msg: String) -> None:
        self.layout = msg.data

    # service calls from the HTTP threads
    def call(self, client, request, timeout: float = 3.0):
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        fut = client.call_async(request)
        done = threading.Event()
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            return None
        return fut.result()


def _make_handler(node: DashboardNode):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):  # keep the console quiet
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code: int = 200) -> None:
            self._send(code, json.dumps(obj).encode(), 'application/json; charset=utf-8')

        def do_GET(self):  # noqa: N802
            path = self.path.split('?')[0]
            if path == '/api/state':
                body = (node.state or '{}').encode()
                return self._send(200, body, 'application/json; charset=utf-8')
            if path == '/api/layout':
                body = (node.layout or '{}').encode()
                return self._send(200, body, 'application/json; charset=utf-8')
            if path == '/api/stream':
                return self._stream()
            if path in ('', '/'):
                path = '/index.html'
            fpath = os.path.normpath(os.path.join(node.web_dir, path.lstrip('/')))
            if not fpath.startswith(os.path.normpath(node.web_dir)) or not os.path.isfile(fpath):
                return self._send(404, b'not found', 'text/plain')
            with open(fpath, 'rb') as f:
                data = f.read()
            ext = os.path.splitext(fpath)[1]
            return self._send(200, data, MIME.get(ext, 'application/octet-stream'))

        def _stream(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            last = -1
            try:
                while rclpy.ok():
                    with node.cond:
                        node.cond.wait_for(lambda: node.version != last, timeout=5.0)
                        data, last = node.state, node.version
                    if data is None:
                        self.wfile.write(b': waiting\n\n')
                    else:
                        self.wfile.write(b'data: ' + data.encode() + b'\n\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return

        def do_POST(self):  # noqa: N802
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except json.JSONDecodeError:
                return self._json({'ok': False, 'message': 'bad json'}, 400)
            try:
                return self._post(self.path.split('?')[0], body)
            except (AttributeError, TypeError, ValueError, AssertionError) as exc:
                # wrong field types / values out of range for the ROS message
                return self._json({'ok': False, 'message': f'bad request: {exc}'}, 400)

        def _post(self, path, body):
            if path == '/api/task':
                req = SubmitTask.Request()
                req.pickup = str(body.get('pickup', ''))
                req.dropoff = str(body.get('dropoff', ''))
                req.priority = int(body.get('priority', 1))
                req.description = str(body.get('description', ''))
                res = node.call(node.cli_task, req)
                if res is None:
                    return self._json({'ok': False, 'message': 'fleet manager not reachable'}, 503)
                return self._json({'ok': res.accepted, 'task_id': res.task_id,
                                   'message': res.message})
            if path == '/api/priority':
                req = SetRobotPriority.Request()
                req.robot_id = str(body.get('robot_id', ''))
                req.base_rank = int(body.get('base_rank', 0))
                if 'paused' in body:
                    req.set_paused = True
                    req.paused = bool(body['paused'])
                res = node.call(node.cli_prio, req)
                if res is None:
                    return self._json({'ok': False, 'message': 'fleet manager not reachable'}, 503)
                return self._json({'ok': res.success, 'message': res.message})
            if path == '/api/demo':
                res = node.call(node.cli_demo, Trigger.Request())
                if res is None:
                    return self._json({'ok': False, 'message': 'fleet manager not reachable'}, 503)
                return self._json({'ok': res.success, 'message': res.message})
            if path == '/api/pause':
                req = SetBool.Request()
                req.data = bool(body.get('paused', True))
                res = node.call(node.cli_pause, req)
                if res is None:
                    return self._json({'ok': False, 'message': 'fleet manager not reachable'}, 503)
                return self._json({'ok': res.success, 'message': res.message})
            return self._json({'ok': False, 'message': 'unknown endpoint'}, HTTPStatus.NOT_FOUND)

    return Handler


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardNode()
    # the HTTP server has its own threads; service futures complete in this
    # single-threaded executor (MultiThreadedExecutor busy-spins on Jazzy)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except RuntimeError:          # Humble: context shut down by Ctrl+C while spinning
        if rclpy.ok():
            raise
    finally:
        node.httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
