import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import tennis_ball_tracker as tracker


def frame_at(x=480, y=120):
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(frame, (x, y), 25, (0, 255, 255), -1)
    return frame


def test_detection_and_pixel_commands():
    target = tracker.detect_ball(frame_at())
    assert target[:2] == pytest.approx((480, 120), abs=2)
    assert tracker.centering_commands(target, 640, 480) == pytest.approx((-.09, .09), abs=.002)
    assert tracker.centering_commands((320, 240), 640, 480) == (0, 0)
    assert tracker.centering_commands((640, 480), 640, 480) == (-.15, -.15)
    assert tracker.detect_ball(np.zeros((480, 640, 3), dtype=np.uint8)) is None


@pytest.mark.parametrize('colour', [(17, 255, 255), (52, 255, 255), (30, 65, 255), (30, 255, 65)])
def test_more_sensitive_colour_defaults(colour):
    hsv = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(hsv, (320, 240), 25, colour, -1)
    frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    target = tracker.detect_ball(frame)
    assert target is not None
    assert target[:2] == pytest.approx((320, 240), abs=2)
    assert tracker.detect_ball(frame, (20, 90, 80), (45, 255, 255)) is None


@pytest.mark.parametrize('width', [640, 960, 1920, 2560])
def test_detection_width_preserves_original_coordinates(width):
    frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
    cv2.circle(frame, (1200, 600), 60, (0, 255, 255), -1)
    target = tracker.detect_ball(frame, detection_width=width)
    assert target is not None
    assert target[:2] == pytest.approx((1200, 600), abs=3)


@pytest.mark.parametrize('width', [0, -1, 960.5, True])
def test_invalid_detection_width(width):
    with pytest.raises(ValueError, match='positive integer'):
        tracker.detect_ball(frame_at(), detection_width=width)


@pytest.mark.parametrize('yaw_gain,heave_gain', [(.18, .18), (.18, .10), (.10, .18)])
def test_tracker_acquisition_loss_and_enable_timeout(monkeypatch, yaw_gain, heave_gain):
    now = [10.0]
    monkeypatch.setattr(tracker.time, 'monotonic', lambda: now[0])
    node = Mock(vision_allowed=True, show=True)
    params = {'ball_yaw_gain': yaw_gain, 'ball_heave_gain': heave_gain}
    node.declare_parameter.side_effect = lambda k, v: params.setdefault(k, v)
    node.get_parameter.side_effect = lambda k: SimpleNamespace(value=params[k])
    node.create_publisher.side_effect = [Mock(), Mock()]
    grabber = Mock()
    ball = tracker.BallTracker(node, grabber)
    assert params['ball_follow_distance_m'] == tracker.BALL_FOLLOW_DISTANCE_M == .2
    assert params['ball_detection_width'] == 960
    assert tuple(params[k] for k in ('ball_h_min', 'ball_h_max', 'ball_s_min', 'ball_v_min')) == (15, 55, 50, 50)
    ball.enable_callback(SimpleNamespace(data=True))

    def tick(frame, age=0):
        now[0] += .05
        grabber.latest_sample.return_value = (frame, now[0]-age)
        ball.tick()
        return ball.pub.publish.call_args.args[0]

    assert tick(frame_at()).angular.z == 0
    assert tick(frame_at()).angular.z == 0
    cmd = tick(frame_at())
    assert cmd.angular.z == pytest.approx(-yaw_gain * .5, abs=.002)
    assert cmd.linear.z == pytest.approx(heave_gain * .5, abs=.002)
    assert cmd.linear.x == params['ball_max_surge']
    node.show = False
    grabber.latest_sample.reset_mock()
    with pytest.MonkeyPatch.context() as hidden_patch:
        detector = Mock(side_effect=AssertionError('Detection ran while hidden'))
        hidden_patch.setattr(tracker, 'detect_ball', detector)
        cmd = tick(frame_at())
        detector.assert_not_called()
    grabber.latest_sample.assert_not_called()
    assert cmd.angular.z == cmd.linear.z == 0
    assert ball.target is None and ball.count == 0
    assert not ball.valid_pub.publish.call_args.args[0].data
    node.show = True
    assert tick(frame_at()).angular.z == 0
    assert tick(frame_at()).angular.z == 0
    assert tick(frame_at()).angular.z < 0
    assert tick(np.zeros((480, 640, 3), dtype=np.uint8)).angular.z == 0
    for _ in range(3):
        tick(frame_at())
    assert tick(frame_at(), age=.3).angular.z == 0
    now[0] += 1
    for _ in range(3):
        cmd = tick(frame_at())
    assert cmd.angular.z == cmd.linear.z == 0
    ball.enable_callback(SimpleNamespace(data=True))
    node.vision_allowed = False
    assert tick(frame_at()).angular.z == 0


@pytest.mark.parametrize('name', ['ball_yaw_gain', 'ball_heave_gain'])
@pytest.mark.parametrize('value', [0., -.1, 1.01, float('nan'), float('inf')])
def test_invalid_axis_gain(name, value):
    node = Mock()
    params = {name: value}
    node.declare_parameter.side_effect = lambda k, v: params.setdefault(k, v)
    node.get_parameter.side_effect = lambda k: SimpleNamespace(value=params[k])
    with pytest.raises(ValueError, match='Invalid tennis-ball tracker tuning'):
        tracker.BallTracker(node, Mock())


@pytest.mark.parametrize('width,radius_px', [(1920, 88.8223858), (1280, 59.2149239)])
def test_distance_estimate_scales_with_frame_resolution(width, radius_px):
    assert tracker.estimate_distance(radius_px, width) == pytest.approx(.7)
    assert tracker.estimate_distance(radius_px / 2, width) == pytest.approx(1.4)
    assert tracker.estimate_distance(radius_px, width, ball_radius_m=.067) == pytest.approx(1.4)
    assert tracker.estimate_distance(32, 1280, hfov_deg=90) == pytest.approx(.67)


@pytest.mark.parametrize('name,value', [
    ('ball_radius_m', 0.), ('ball_radius_m', -.01), ('ball_radius_m', float('nan')),
    ('ball_hfov_deg', 0.), ('ball_hfov_deg', 180.), ('ball_hfov_deg', float('inf')),
])
def test_invalid_distance_tuning(name, value):
    node = Mock()
    params = {name: value}
    node.declare_parameter.side_effect = lambda k, v: params.setdefault(k, v)
    node.get_parameter.side_effect = lambda k: SimpleNamespace(value=params[k])
    with pytest.raises(ValueError, match='Invalid tennis-ball tracker tuning'):
        tracker.BallTracker(node, Mock())


def test_distance_overlay(monkeypatch):
    ball = tracker.BallTracker.__new__(tracker.BallTracker)
    ball.params = {'ball_radius_m': .0335, 'ball_hfov_deg': 54.7}
    ball.overlay = ((960., 540., 88.8223858), 'Tennis ball: detection only')
    put_text = Mock()
    monkeypatch.setattr(tracker.cv2, 'putText', put_text)
    frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
    ball.draw(frame)
    assert 'est. 0.70 m' in put_text.call_args_list[0].args[1]
    put_text.reset_mock()
    ball.overlay = (None, 'Tennis ball: no valid target / motion OFF')
    ball.draw(frame)
    assert put_text.call_count == 1
    assert 'est.' not in put_text.call_args.args[1]


def test_debug_mask_matches_detector():
    frame = frame_at()
    target, mask = tracker.detect_ball(frame, return_mask=True)
    assert target == tracker.detect_ball(frame)
    assert mask.shape == frame.shape[:2]
    assert mask[120, 480] == 255 and mask[0, 0] == 0
    assert set(np.unique(mask)) == {0, 255}
    # White regions can fail the contour-size/shape checks.
    target, mask = tracker.detect_ball(np.full_like(frame, (0, 255, 255)), return_mask=True)
    assert target is None and np.all(mask == 255)
    target, mask = tracker.detect_ball(np.zeros_like(frame), return_mask=True)
    assert target is None and not mask.any()
    _, mask = tracker.detect_ball(frame, detection_width=320, return_mask=True)
    assert mask.shape == (240, 320)


@pytest.mark.parametrize('fresh', [True, False])
def test_debug_viewer_windows_and_cleanup(monkeypatch, fresh):
    import tennis_ball_debug as debug
    grabber = Mock()
    grabber.latest_sample.return_value = (frame_at(), 10. if fresh else 9.)
    monkeypatch.setattr(debug, 'FrameGrabber', Mock(return_value=grabber))
    monkeypatch.setattr(debug.time, 'monotonic', lambda: 10.)
    monkeypatch.setattr(sys, 'argv', ['tennis_ball_debug.py'])
    for name in ('namedWindow', 'resizeWindow', 'destroyAllWindows'):
        monkeypatch.setattr(debug.cv2, name, Mock())
    shown = Mock()
    monkeypatch.setattr(debug.cv2, 'imshow', shown)
    monkeypatch.setattr(debug.cv2, 'waitKey', Mock(return_value=ord('q')))
    debug.main()
    assert shown.call_count == 2
    mask = shown.call_args_list[1].args[1]
    assert mask.ndim == 2
    assert bool(mask.any()) is fresh
    grabber.stop.assert_called_once()
    debug.cv2.destroyAllWindows.assert_called_once()


@pytest.mark.parametrize('distance,expected', [
    (.1, 0), (.2, 0), (.2001, .08), (.5, .08), (3., .08),
    (float('nan'), 0), (float('inf'), 0),
])
def test_following_surge(distance, expected):
    assert tracker.following_surge(distance) == pytest.approx(expected)
    assert tracker.following_surge(distance, maximum=0) == 0
    assert tracker.following_surge(distance, maximum=.12) == pytest.approx(.12 if expected else 0)
    assert tracker.following_surge(.5, follow_distance=.6) == 0


@pytest.mark.parametrize('gate', ['lost', 'stale', 'hidden', 'health', 'heartbeat', 'disabled'])
def test_following_stops_all_axes(monkeypatch, gate):
    now = [10.]
    monkeypatch.setattr(tracker.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(tracker, 'detect_ball', lambda *a: (384, 288, 12))
    node = Mock(vision_allowed=True, show=True)
    params = {}
    node.declare_parameter.side_effect = lambda k, v: params.setdefault(k, v)
    node.get_parameter.side_effect = lambda k: SimpleNamespace(value=params[k])
    node.create_publisher.side_effect = [Mock(), Mock()]
    grabber = Mock()
    ball = tracker.BallTracker(node, grabber)
    ball.enable_callback(SimpleNamespace(data=True))
    for _ in range(3):
        now[0] += .05
        grabber.latest_sample.return_value = (frame_at(), now[0])
        ball.tick()
    cmd = ball.pub.publish.call_args.args[0]
    assert cmd.linear.x == params['ball_max_surge']
    assert cmd.linear.z != 0 and cmd.angular.z != 0
    now[0] += .05
    grabber.latest_sample.return_value = (frame_at(), now[0])
    if gate == 'lost':
        monkeypatch.setattr(tracker, 'detect_ball', lambda *a: None)
    elif gate == 'stale':
        grabber.latest_sample.return_value = (frame_at(), now[0] - .3)
    elif gate == 'hidden':
        node.show = False
    elif gate == 'health':
        node.vision_allowed = False
    elif gate == 'heartbeat':
        ball.enable_time = now[0] - .6
    else:
        ball.enable_callback(SimpleNamespace(data=False))
    ball.tick()
    cmd = ball.pub.publish.call_args.args[0]
    assert cmd.linear.x == cmd.linear.z == cmd.angular.z == 0


@pytest.mark.parametrize('name,value', [
    ('ball_follow_distance_m', 0.), ('ball_follow_distance_m', float('nan')),
    ('ball_max_surge', -.1), ('ball_max_surge', .31),
])
def test_invalid_following_tuning(name, value):
    node = Mock()
    params = {name: value}
    node.declare_parameter.side_effect = lambda k, v: params.setdefault(k, v)
    node.get_parameter.side_effect = lambda k: SimpleNamespace(value=params[k])
    with pytest.raises(ValueError, match='Invalid tennis-ball tracker tuning'):
        tracker.BallTracker(node, Mock())


@pytest.mark.parametrize('mode,surge,expected', [
    ('real', .08, .08), ('real', 1., .3), ('real', -.1, 0.),
    ('sim', -.5, -.5), ('sim', 2., 1.),
])
def test_teleop_routes_following_surge(mode, surge, expected):
    from teleop_controller import GamepadTeleop
    node = Mock(rov_mode=mode, fish_follow_mode=True)
    node.inputs = {'fish': Mock()}
    node.inputs['fish'].fresh.return_value = True
    node.vision_permission.valid.return_value = True
    msg = tracker.Twist()
    msg.linear.x, msg.linear.z, msg.angular.z = surge, .04, -.05
    GamepadTeleop.fish_command_callback(node, msg)
    assert node.fish_surge_cmd == expected
    assert node.fish_heave_cmd == .04 and node.fish_yaw_cmd == -.05


@pytest.mark.parametrize('colour', [(14, 40, 180), (58, 80, 180), (30, 80, 40)])
def test_original_filter_rejects_weak_colour(colour):
    hsv = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(hsv, (320, 240), 6, colour, -1)
    frame = cv2.GaussianBlur(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), (5, 5), 1.)
    assert tracker.detect_ball(frame) is None
    assert (tracker.BALL_MIN_AREA, tracker.BALL_MIN_RADIUS,
            tracker.BALL_MIN_CIRCULARITY, tracker.BALL_MIN_FILL) == (80, 5, .65, .65)


def test_pd_frame_timing_and_reset(monkeypatch):
    now = [10.]
    target = [480., 120., 25.]
    monkeypatch.setattr(tracker.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(tracker, 'detect_ball', lambda *a: tuple(target))
    node = Mock(vision_allowed=True, show=True)
    params = {'ball_yaw_gain': .1, 'ball_heave_gain': .2,
              'ball_yaw_kd': .02, 'ball_heave_kd': .04}
    node.declare_parameter.side_effect = lambda k, v: params.setdefault(k, v)
    node.get_parameter.side_effect = lambda k: SimpleNamespace(value=params[k])
    node.create_publisher.side_effect = [Mock(), Mock()]
    grabber = Mock()
    ball = tracker.BallTracker(node, grabber)
    def tick(new=True):
        now[0] += .05
        ball.enable_callback(SimpleNamespace(data=True))
        if new:
            grabber.latest_sample.return_value = (frame_at(), now[0])
        ball.tick()
        return ball.pub.publish.call_args.args[0]
    for _ in range(3):
        cmd = tick()
    assert (cmd.angular.z, cmd.linear.z) == pytest.approx((-.05, .1))
    target[:] = [448., 144., 25.]  # Errors shrink by .1 in .05 seconds.
    cmd = tick()
    assert (cmd.angular.z, cmd.linear.z) == pytest.approx((0., 0.))
    cmd = tick(new=False)  # Same frame must retain D, not recompute it as zero.
    assert (cmd.angular.z, cmd.linear.z) == pytest.approx((0., 0.))
    node.vision_allowed = False
    tick()
    assert ball.control_stamp is None
    assert all(c._prev_measurement is None for c in ball.controllers)
    node.vision_allowed = True
    cmd = tick()
    assert (cmd.angular.z, cmd.linear.z) == pytest.approx((-.04, .08))
    target[:] = [320., 240., 25.]
    cmd = tick()
    assert cmd.angular.z == cmd.linear.z == 0
    assert all(c._prev_measurement is None for c in ball.controllers)


@pytest.mark.parametrize('name', ['ball_yaw_kd', 'ball_heave_kd'])
@pytest.mark.parametrize('value', [-.01, 1.1, float('nan')])
def test_invalid_derivative_gain(name, value):
    node = Mock()
    params = {name: value}
    node.declare_parameter.side_effect = lambda k, v: params.setdefault(k, v)
    node.get_parameter.side_effect = lambda k: SimpleNamespace(value=params[k])
    with pytest.raises(ValueError, match='Invalid tennis-ball tracker tuning'):
        tracker.BallTracker(node, Mock())



def test_trajectory_surge_and_independent_deadbands():
    from rov_config import TRAJECTORY_FORWARD, REAL_SCALE_TRAJ_FORWARD
    expected = .5 * max(0., min(1., TRAJECTORY_FORWARD * REAL_SCALE_TRAJ_FORWARD))
    assert tracker.BALL_MAX_SURGE == expected
    node = Mock()
    params = {}
    node.declare_parameter.side_effect = lambda k, v: params.setdefault(k, v)
    node.get_parameter.side_effect = lambda k: SimpleNamespace(value=params[k])
    ball = tracker.BallTracker(node, Mock())
    assert ball.params['ball_max_surge'] == expected
    assert params['ball_yaw_deadband'] == .16
    assert params['ball_heave_deadband'] == .08
    # Both axes have 12% normalized error: only heave should correct.
    yaw, heave = tracker.centering_commands((358.4, 268.8), 640, 480,
                                            controllers=ball.controllers)
    assert yaw == 0 and heave != 0
    assert tracker.centering_commands((384, 288), 640, 480)[0] != 0


@pytest.mark.parametrize('name', ['ball_yaw_deadband', 'ball_heave_deadband'])
@pytest.mark.parametrize('value', [-.01, 1., float('nan')])
def test_invalid_axis_deadband(name, value):
    node = Mock()
    params = {name: value}
    node.declare_parameter.side_effect = lambda k, v: params.setdefault(k, v)
    node.get_parameter.side_effect = lambda k: SimpleNamespace(value=params[k])
    with pytest.raises(ValueError, match='Invalid tennis-ball tracker tuning'):
        tracker.BallTracker(node, Mock())
