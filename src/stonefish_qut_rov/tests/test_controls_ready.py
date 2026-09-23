"""Readiness banner checks without starting any processes or hardware."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

from rov_control import StackSupervisor


def test_real_banner_waits_for_camera_arm_and_live_manual_connection():
    node = SimpleNamespace(mode='real', controls_printed=False, received={},
                           fcu_ready_at=float('-inf'), arm_client=Mock())
    with patch('rov_control.time.monotonic', return_value=10.), \
         patch('rov_control.print_controls') as output:
        StackSupervisor.maybe_print_controls(node)
        node.received = {'camera': 10., 'arm': 10.}
        StackSupervisor.maybe_print_controls(node)
        output.assert_not_called()
        node.fcu_ready_at = 10.
        node.arm_client.service_is_ready.return_value = False
        StackSupervisor.maybe_print_controls(node)
        output.assert_not_called()
        node.arm_client.service_is_ready.return_value = True
        node.received['camera'] = 8.
        StackSupervisor.maybe_print_controls(node)
        output.assert_not_called()
        node.received['camera'] = 10.
        StackSupervisor.maybe_print_controls(node)
        StackSupervisor.maybe_print_controls(node)
        output.assert_called_once_with('real')


def test_sim_banner_does_not_wait_for_physical_camera():
    node = SimpleNamespace(mode='sim', controls_printed=False)
    with patch('rov_control.print_controls') as output:
        StackSupervisor.maybe_print_controls(node)
        output.assert_called_once_with('sim')


def test_pixhawk_warning_on_startup_connection_failure():
    # Connection is independent of flight mode; stale or disconnected state
    # must not hide the Ethernet hint, while simulation must not show it.
    for mode, connected, age, expected in (
        ('real', None, 0., True),
        ('real', False, 0., True),
        ('real', True, 4., True),
        ('real', True, 0., False),
        ('sim', None, 0., False),
    ):
        logger = Mock()
        node = SimpleNamespace(mode=mode, processes=[], teleop=None, received={},
                               started=0., get_logger=lambda: logger,
                               _shutdown_requested=False)
        if connected is not None:
            with patch('rov_control.time.monotonic', return_value=100.-age):
                StackSupervisor.fcu_ready(node, SimpleNamespace(connected=connected, mode='STABILIZE'))
        with patch('rov_control.time.monotonic', return_value=100.):
            StackSupervisor.monitor(node)
        if expected:
            logger.warning.assert_called_once_with(
                'Failed to connect to the Pixhawk, check your ethernet connection')
        else:
            logger.warning.assert_not_called()
        assert node._shutdown_requested
        assert node.exit_code == 1
