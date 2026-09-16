"""Exercise viewer thermal transitions without SSH, video, or gimbal motion."""
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace
os.environ.setdefault('ROS_DOMAIN_ID', '213')
os.environ.setdefault('ROS_LOG_DIR', '/tmp/rov-test-logs')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros_nodes/modular_architecture'))
import rclpy
import camera_viewer_rtsp as viewer


def test_health_transitions(monkeypatch):
    monitor = SimpleNamespace(sample=(81., time.monotonic(), ''))
    monkeypatch.setattr(viewer, 'TemperatureMonitor', lambda host: monitor)
    frames = SimpleNamespace(health=lambda: (5., .1))
    rclpy.init()
    node = viewer.ViewerNode()
    try:
        node.start_health(frames)
        assert not node.vision_allowed
        assert '5 fps' in node.health_message
        monitor.sample = (77., time.monotonic(), '')
        frames.health = lambda: (24., .01)
        node.check_health(frames)
        assert not node.vision_allowed
        monitor.sample = (74., time.monotonic(), '')
        node.check_health(frames)
        assert node.vision_allowed
        monitor.sample = (None, 0., 'Camera disconnected')
        node.check_health(frames)
        assert not node.vision_allowed
    finally:
        node.destroy_node()
        rclpy.shutdown()
