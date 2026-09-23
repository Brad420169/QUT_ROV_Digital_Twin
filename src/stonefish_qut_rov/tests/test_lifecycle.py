"""Lifecycle and launch supervision without starting the ROV stack."""
import os
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

os.environ['ROS_DOMAIN_ID'] = '213'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/rov-test-logs')

import pytest
import rclpy
import node_lifecycle
from process_utils import start_process, stop_process
from rov_control import StackSupervisor


def test_lifecycle_cleans_up_once_with_ros_context_live(monkeypatch):
    events = []
    node = SimpleNamespace(_shutdown_requested=False)
    node.stop = lambda: events.append(('stop', rclpy.ok()))
    node.destroy_node = lambda: events.append(('destroy', rclpy.ok()))
    monkeypatch.setattr(rclpy, 'spin_once', lambda *a, **k: setattr(node, '_shutdown_requested', True))
    previous = signal.getsignal(signal.SIGTERM)
    node_lifecycle.run_node(lambda: node, args=[])
    assert events == [('stop', True), ('destroy', True)]
    assert not rclpy.ok()
    assert signal.getsignal(signal.SIGTERM) == previous


@pytest.mark.parametrize("new_session", [True, False])
def test_owned_process_stops_without_killing_another_session(new_session):
    owned = start_process([sys.executable, '-c', 'import time; time.sleep(30)'], new_session=new_session)
    other = start_process([sys.executable, '-c', 'import time; time.sleep(30)'])
    try:
        stop_process(owned, process_group=new_session)
        assert owned.poll() is not None
        assert other.poll() is None
    finally:
        stop_process(owned, process_group=new_session)
        stop_process(other)


@pytest.mark.parametrize('exited', ['Stonefish', 'joy_node', 'interface', 'teleop'])
def test_supervisor_stops_on_any_essential_exit(exited):
    process = Mock()
    process.poll.return_value = 0
    supervisor = SimpleNamespace(processes=[(exited, process)], exit_code=None,
                                 _shutdown_requested=False, get_logger=lambda: Mock())
    StackSupervisor.monitor(supervisor)
    assert supervisor._shutdown_requested
    assert supervisor.exit_code == (0 if exited == 'teleop' else 1)


def test_supervisor_timeout_names_missing_data():
    logger = Mock()
    supervisor = SimpleNamespace(mode="real", processes=[], teleop=None, received={},
        started=time.monotonic()-100, get_logger=lambda: logger, _shutdown_requested=False)
    StackSupervisor.monitor(supervisor)
    assert supervisor._shutdown_requested
    assert supervisor.exit_code == 1
    assert 'joy, depth, imu' in logger.error.call_args.args[0]


@pytest.mark.parametrize("mode", ["sim", "real"])
def test_startup_without_gamepad(mode):
    now = time.monotonic()
    supervisor = SimpleNamespace(
        mode=mode, processes=[], teleop=None, received={"depth": now, "imu": now},
        started=now-100, get_logger=lambda: Mock(), _shutdown_requested=False,
        start=Mock(return_value=Mock()), maybe_print_controls=Mock(), exit_code=0,
    )
    StackSupervisor.monitor(supervisor)
    if mode == "sim":
        supervisor.start.assert_called_once()
        assert supervisor.teleop is not None
        assert not supervisor._shutdown_requested
    else:
        supervisor.start.assert_not_called()
        assert supervisor._shutdown_requested
        assert supervisor.exit_code == 1
