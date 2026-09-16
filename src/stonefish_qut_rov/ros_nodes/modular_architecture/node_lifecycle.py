"""Run node cleanup once, before invalidating the ROS context."""
import signal
import threading

import rclpy
from rclpy.signals import SignalHandlerOptions


def run_node(factory, args=None, cleanup="stop"):
    requested = threading.Event()
    previous = {}
    node = None
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[signum] = signal.signal(signum, lambda *_: requested.set())
        node = factory()
        while rclpy.ok() and not requested.is_set() and not getattr(node, "_shutdown_requested", False):
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                try:
                    getattr(node, cleanup)()
                finally:
                    node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            for signum, handler in previous.items():
                signal.signal(signum, handler)
