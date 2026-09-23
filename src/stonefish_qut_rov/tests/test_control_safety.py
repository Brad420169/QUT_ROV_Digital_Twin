"""Regression tests in an isolated ROS domain; no simulator or vehicle required."""
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

os.environ['ROS_DOMAIN_ID'] = '213'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/rov-test-logs')

import pytest
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Imu, FluidPressure
from std_msgs.msg import Float64

from command_watchdog import Freshness
from sim_interface import SimInterface
from real_interface import RealInterface
from teleop_controller import GamepadTeleop
import teleop_controller
import real_interface


def test_pressure_calibration_average_and_depth(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(real_interface.time, 'monotonic', lambda: now[0])
    node = RealInterface()
    node.depth_pub = Mock()
    try:
        for i in range(30):
            now[0] += .12
            node.pressure_callback(FluidPressure(fluid_pressure=100000.0 + i))
            if i < 29:
                node.depth_pub.publish.assert_not_called()
        assert node.surface_pressure_pa == pytest.approx(100014.5)
        node.pressure_callback(FluidPressure(fluid_pressure=100014.5 + 998 * 9.81))
        assert node.depth_pub.publish.call_args.args[0].data == pytest.approx(1.)
        assert node.surface_pressure_pa == pytest.approx(100014.5)
    finally:
        node.destroy_node()


def test_pressure_calibration_rejects_invalid_and_interrupted_samples(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(real_interface.time, 'monotonic', lambda: now[0])
    node = RealInterface()
    node.depth_pub = Mock()
    try:
        sample = FluidPressure(fluid_pressure=100000.)
        for _ in range(30):
            node.pressure_callback(sample)
        assert len(node._pressure_samples) == 1  # Burst cannot complete calibration.
        now[0] += 2.
        node.pressure_callback(sample)
        assert len(node._pressure_samples) == 1
        for invalid in (float('nan'), float('inf'), -1., 0.):
            node.pressure_callback(FluidPressure(fluid_pressure=invalid))
            assert not node._pressure_samples
        assert node.surface_pressure_pa is None
        node.depth_pub.publish.assert_not_called()
    finally:
        node.destroy_node()


def test_camera_dpad_overrides_shortcuts_until_release(teleop):
    teleop.rov_mode = 'real'
    teleop.viewers.visible = True
    teleop.viewers.camera_visible = True
    teleop.toggle_plotter = Mock()
    teleop._handle_shutdown_hold = Mock()
    for value in (1., -1., -1.):
        teleop._handle_dpad([0.] * 7 + [value])
    teleop.toggle_plotter.assert_not_called()
    teleop._handle_shutdown_hold.assert_not_called()
    teleop.viewers.visible = teleop.viewers.camera_visible = False
    teleop._handle_dpad([0.] * 7 + [-1.])
    teleop._handle_shutdown_hold.assert_not_called()
    teleop._handle_dpad([0.] * 8)
    teleop._handle_dpad([0.] * 7 + [1.])
    teleop.toggle_plotter.assert_called_once()
    teleop._handle_dpad([0.] * 7 + [-1.])
    teleop._handle_shutdown_hold.assert_called_with(True)


@pytest.fixture(autouse=True)
def context():
    rclpy.init(args=[])
    yield
    rclpy.shutdown()


@pytest.fixture
def teleop(monkeypatch):
    monkeypatch.setattr(teleop_controller, 'ViewerManager', Mock())
    node = GamepadTeleop()
    node.command_pub = Mock()
    node.follow_enable_pub = Mock()
    now = [10.0]
    for freshness in node.inputs.values():
        freshness.clock = lambda: now[0]
    node.test_time = now
    yield node
    node.stop()
    node.destroy_node()


def joystick(surge=0.0, button=None):
    msg = Joy()
    msg.axes = [0.0] * 8
    msg.axes[1] = surge
    msg.axes[4] = msg.axes[5] = 1.0
    msg.buttons = [0] * 8
    if button is not None:
        msg.buttons[button] = 1
    return msg


def last_command(node):
    msg = node.command_pub.publish.call_args.args[0]
    return msg.linear.x, msg.linear.z, msg.angular.z


def test_watchdog_clock_reset_and_timeout():
    now = [10.0]
    watchdog = Freshness(0.5, lambda: now[0])
    assert not watchdog.fresh()
    watchdog.touch()
    assert watchdog.fresh()
    now[0] += 0.51
    assert not watchdog.fresh()
    now[0] = 9.0
    assert not watchdog.fresh()


def test_real_ball_mode_routes_yaw_heave_and_heartbeat(teleop):
    teleop.rov_mode = 'real'
    teleop.vision_permission.update(True)
    teleop.joy_callback(joystick())
    teleop.toggle_fish_follow()
    assert teleop.fish_follow_mode
    command = Twist()
    command.linear.x = .7
    command.linear.z = .1
    command.angular.z = -.12
    teleop.fish_command_callback(command)
    teleop.publish_command()
    assert last_command(teleop) == (0, .1, -.12)
    assert teleop.follow_enable_pub.publish.call_args.args[0].data
    teleop.toggle_fish_follow()
    teleop.publish_command()
    assert last_command(teleop) == (0, 0, 0)
    assert not teleop.follow_enable_pub.publish.call_args.args[0].data


def test_joystick_loss_requires_neutral_before_resuming(teleop):
    teleop.joy_callback(joystick())
    teleop.joy_callback(joystick(0.7))
    teleop.publish_command()
    assert last_command(teleop)[0] > 0
    teleop.test_time[0] += 0.6
    teleop.publish_command()
    assert last_command(teleop) == (0, 0, 0)
    teleop.joy_callback(joystick(0.7))
    teleop.publish_command()
    assert last_command(teleop) == (0, 0, 0)
    teleop.joy_callback(joystick())
    teleop.joy_callback(joystick(0.7))
    teleop.publish_command()
    assert last_command(teleop)[0] > 0


@pytest.mark.parametrize('mode,missing', [('depth', 'depth'), ('trajectory', 'imu'), ('fish', 'fish')])
def test_stale_mode_input_cancels_mode_without_auto_restart(teleop, mode, missing):
    teleop.joy_callback(joystick())
    teleop.depth_callback(Float64(data=1.0))
    imu = Imu()
    imu.orientation.w = 1.0
    teleop.imu_callback(imu)
    if mode == 'depth':
        teleop.toggle_depth_keeping()
    elif mode == 'trajectory':
        teleop.toggle_trajectory()
    else:
        teleop._set_fish_follow(True)
        command = Twist()
        command.linear.x = 0.7
        teleop.fish_command_callback(command)
    teleop.test_time[0] += 0.6
    for name, freshness in teleop.inputs.items():
        if name != missing:
            freshness.touch()
    teleop.publish_command()
    assert last_command(teleop) == (0, 0, 0)
    assert not any((teleop.depth_keeping, teleop.trajectory_mode, teleop.fish_follow_mode))
    teleop.inputs[missing].touch()
    teleop.publish_command()
    assert last_command(teleop) == (0, 0, 0)
    assert teleop._manual_rearm


def test_invalid_joy_cannot_toggle_mode(teleop):
    teleop.joy_callback(joystick())
    teleop.depth_callback(Float64(data=1.0))
    bad = joystick(button=6)
    bad.axes = [float('nan')]
    teleop.joy_callback(bad)
    teleop.publish_command()
    assert not teleop.depth_keeping
    assert last_command(teleop) == (0, 0, 0)


def test_stale_depth_cannot_enable_hold(teleop):
    teleop.depth_callback(Float64(data=1.0))
    teleop.inputs['depth'].invalidate()
    teleop.toggle_depth_keeping()
    assert not teleop.depth_keeping


@pytest.mark.parametrize('factory,publisher,neutral', [
    (SimInterface, 'thruster_pub', [0.0]*4),
    (RealInterface, 'rc_pub', [1500]*18),
])
def test_interface_timeout_and_nan_force_neutral(factory, publisher, neutral):
    node = factory()
    node._disarm = Mock()
    output = Mock()
    setattr(node, publisher, output)
    now = [10.0]
    node.command_freshness.clock = lambda: now[0]
    try:
        cmd = Twist()
        cmd.linear.z = 0.6
        node.command_callback(cmd)
        node.publish_thrusters()
        message = output.publish.call_args.args[0]
        assert list(getattr(message, 'data' if publisher == 'thruster_pub' else 'channels')) != neutral
        now[0] += 0.6
        node.publish_thrusters()
        message = output.publish.call_args.args[0]
        assert list(getattr(message, 'data' if publisher == 'thruster_pub' else 'channels')) == neutral
        cmd.linear.z = float('nan')
        node.command_callback(cmd)
        node.publish_thrusters()
        message = output.publish.call_args.args[0]
        assert list(getattr(message, 'data' if publisher == 'thruster_pub' else 'channels')) == neutral
    finally:
        node.stop()
        node.destroy_node()


def test_teleop_stop_is_idempotent(teleop):
    teleop.stop()
    teleop.stop()
    assert last_command(teleop) == (0, 0, 0)
    teleop.viewers.close.assert_called_once()


def test_late_fish_result_cannot_resume_after_gap(teleop):
    teleop.joy_callback(joystick())
    teleop._set_fish_follow(True)
    teleop.test_time[0] += 0.6
    teleop.inputs['joy'].touch()
    command = Twist()
    command.linear.x = 0.8
    teleop.fish_command_callback(command)
    teleop.publish_command()
    assert not teleop.fish_follow_mode
    assert last_command(teleop) == (0, 0, 0)


@pytest.fixture
def follower_class(monkeypatch):
    # Exercise follower logic without loading a GPU/model or opening a GUI.
    import importlib.util
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=Mock()))
    monkeypatch.setitem(sys.modules, 'ultralytics.utils', SimpleNamespace(LOGGER=Mock()))
    spec = importlib.util.spec_from_file_location('follower_under_test',
        Path(__file__).resolve().parents[1] / 'ros_nodes/sim/fish_detector_follower.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_follower_shutdown_sends_one_zero_and_stops_worker(follower_class, monkeypatch):
    from threading import RLock
    from types import MethodType
    monkeypatch.setattr(follower_class.cv2, 'destroyAllWindows', Mock())
    follower = SimpleNamespace(_lock=RLock(), _running=True, _enabled=True,
        _generation=0, _command_publisher=Mock(), _inference_thread=Mock())
    cls = follower_class.FishDetectorFollower
    follower._publish_command = MethodType(cls._publish_command, follower)
    cls.shutdown(follower)
    cls.shutdown(follower)
    follower._command_publisher.publish.assert_called_once()
    message = follower._command_publisher.publish.call_args.args[0]
    assert (message.linear.x, message.linear.z, message.angular.z) == (0, 0, 0)
    assert not follower._running
    follower._inference_thread.join.assert_called_once()


@pytest.mark.parametrize('age,generation,accepted', [(0.1, 3, True), (1.0, 3, False), (0.1, 2, False)])
def test_inference_discards_old_frames_and_previous_sessions(follower_class, monkeypatch, age, generation, accepted):
    from threading import RLock
    monkeypatch.setattr(follower_class.time, 'monotonic', lambda: 10.0)
    monkeypatch.setattr(follower_class.rclpy, 'ok', Mock(side_effect=[True, False]))
    follower = SimpleNamespace(
        _lock=RLock(), _running=True, _pending_frame=SimpleNamespace(shape=(360, 480, 3)),
        _pending_header=None, _pending_time=10.0-age, _pending_generation=generation,
        _generation=3, _model=Mock(), _confidence=0.25, _image_size=480, _device='cpu',
        _use_half=False, _select_target_detection=Mock(return_value=None), _update_follower=Mock())
    follower._model.predict.return_value = [Mock()]
    follower_class.FishDetectorFollower._inference_loop(follower)
    assert follower._update_follower.called == accepted
