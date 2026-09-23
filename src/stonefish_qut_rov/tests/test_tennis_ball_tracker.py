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


def test_tracker_acquisition_loss_and_enable_timeout(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(tracker.time, 'monotonic', lambda: now[0])
    node = Mock(vision_allowed=True, show=True)
    params = {}
    node.declare_parameter.side_effect = lambda k, v: params.setdefault(k, v)
    node.get_parameter.side_effect = lambda k: SimpleNamespace(value=params[k])
    node.create_publisher.side_effect = [Mock(), Mock()]
    grabber = Mock()
    ball = tracker.BallTracker(node, grabber)
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
    assert cmd.angular.z < 0 and cmd.linear.z > 0 and cmd.linear.x == 0
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
