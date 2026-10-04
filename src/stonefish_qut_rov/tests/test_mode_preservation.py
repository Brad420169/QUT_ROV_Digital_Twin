"""Keep the twin's behavior independent of the updated physical controller."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from teleop_controller import GamepadTeleop


@pytest.mark.parametrize('mode,expected', [
    ('sim', (1., 1/3, 1/12, 1/6, 1/6, 0., .5)),
    ('real', (.3, 4/3, 0., 80/600, .4, .2, .5)),
])
def test_effective_mode_gains(mode, expected):
    node = NS(rov_mode=mode)
    node._scaled = lambda value, scale: GamepadTeleop._scaled(node, value, scale)
    GamepadTeleop._resolve_scaling(node)
    assert (node.scale_manual_heave, node.k_depth_kp, node.k_depth_ki,
            node.k_depth_kd, node.k_depth_traj_kd, node.k_yaw_kp,
            node.k_traj_forward) == pytest.approx(expected)


def test_sim_depth_hold_keeps_manual_yaw_without_imu():
    node = NS(rov_mode='sim', fish_follow_mode=False, trajectory_mode=False,
              depth_keeping=False, current_depth=.4, current_yaw=None,
              target_yaw=None, yaw_cmd=.12, inputs={'depth': Mock(), 'imu': Mock()},
              depth_pid=Mock(), yaw_pid=Mock(), get_logger=Mock())
    node.inputs['depth'].fresh.return_value = True
    node.inputs['imu'].fresh.return_value = False
    GamepadTeleop.toggle_depth_keeping(node)
    assert node.depth_keeping and node.target_depth == .4
    assert node.target_yaw is None and node.yaw_cmd == .12
    GamepadTeleop.toggle_depth_keeping(node)
    assert not node.depth_keeping and node.yaw_cmd == .12
    node.yaw_pid.reset.assert_not_called()


@pytest.mark.parametrize('mode,cancelled', [('sim', False), ('real', True)])
def test_depth_hold_imu_freshness_is_mode_specific(mode, cancelled):
    node = NS(rov_mode=mode, fish_follow_mode=False, depth_keeping=True,
              trajectory_mode=False, inputs={k: Mock() for k in ('joy', 'depth', 'imu')},
              _manual_rearm=False, _stopped=False, surge_cmd=.1, heave_cmd=.2,
              yaw_cmd=.0, command_pub=Mock(), follow_enable_pub=Mock(),
              _cancel_for_input_loss=Mock())
    for name, value in node.inputs.items():
        value.fresh.return_value = name != 'imu'
    GamepadTeleop.publish_command(node)
    assert node._cancel_for_input_loss.called is cancelled
    if cancelled:
        node._cancel_for_input_loss.assert_called_once_with('imu')
