"""Exercise viewer thermal transitions without SSH, video, or gimbal motion."""
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
os.environ.setdefault('ROS_DOMAIN_ID', '213')
os.environ.setdefault('ROS_LOG_DIR', '/tmp/rov-test-logs')
import rclpy
import camera_viewer_rtsp as viewer


def test_health_transitions(monkeypatch):
    monitor = SimpleNamespace(sample=(81., time.monotonic(), ''))
    monkeypatch.setattr(viewer, 'TemperatureMonitor', lambda host: monitor)
    connected = threading.Event()
    connected.set()
    frames = SimpleNamespace(health=lambda: (5., .1), connected=connected)
    rclpy.init()
    node = viewer.ViewerNode()
    try:
        node.start_health(frames)
        node._poll_health(frames)
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


def test_health_waits_for_camera_connection(monkeypatch):
    """Temperature (SSH) and fps reads must not start until the stream connects."""
    created = []
    monkeypatch.setattr(viewer, 'TemperatureMonitor',
                         lambda host: created.append(host) or SimpleNamespace(sample=(70., time.monotonic(), '')))
    connected = threading.Event()
    frames = SimpleNamespace(health=lambda: (30., .01), connected=connected)
    rclpy.init()
    node = viewer.ViewerNode()
    try:
        node.start_health(frames)
        node._poll_health(frames)
        assert not created  # not connected yet -> no temperature session, no fps check
        assert not node.vision_allowed
        connected.set()
        node._poll_health(frames)
        assert created  # connects -> checks now run
        assert node.vision_allowed
    finally:
        node.destroy_node()
        rclpy.shutdown()
