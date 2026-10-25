"""Executor used by the fleet's Python nodes.

rclpy's SingleThreadedExecutor rebuilds the wait set of every subscription,
timer and client of the process for each callback.  The fleet nodes have many
of them (15 agents in one process, one subscription per robot in the fleet
manager and the kinematic simulator), so that overhead alone saturated a core
with 15 robots.  The EventsExecutor (rclpy >= 7.1, experimental) dispatches
ready callbacks directly: ~4x less CPU.  Parameter `executor:=single` falls
back to the SingleThreadedExecutor.
"""

from __future__ import annotations

from typing import Sequence

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node


def make_executor(node: Node):
    if not node.has_parameter('executor'):
        node.declare_parameter('executor', 'events')     # events | single
    if node.get_parameter('executor').value == 'events':
        try:
            from rclpy.experimental import EventsExecutor
            return EventsExecutor()
        except ImportError:
            node.get_logger().warning('EventsExecutor not available, using SingleThreadedExecutor')
    return SingleThreadedExecutor()


def spin_nodes(nodes: Sequence[Node]) -> None:
    """Spin the nodes (the first one carries the `executor` parameter), then shut down."""
    ex = make_executor(nodes[0])
    for nd in nodes:
        ex.add_node(nd)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for nd in nodes:
            nd.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
