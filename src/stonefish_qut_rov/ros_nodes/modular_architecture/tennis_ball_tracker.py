"""HSV tennis-ball centering. Vehicle commands are normalized, not radians/metres."""
import math
import time
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

ENABLE_TOPIC = '/qut_rov/tennis_ball_enabled'
COMMAND_TOPIC = '/qut_rov/tennis_ball_cmd'
TARGET_TOPIC = '/qut_rov/tennis_ball_target_valid'


def detect_ball(frame, hsv_low=(20, 90, 80), hsv_high=(45, 255, 255)):
    """Return (cx, cy, radius) in original pixels, or None.

    Round yellow objects can be false positives; inspect the overlay before use.
    """
    h, w = frame.shape[:2]
    scale = min(1., 640. / w)
    small = cv2.resize(frame, (round(w*scale), round(h*scale))) if scale < 1 else frame
    hsv = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_low, dtype=np.uint8), np.array(hsv_high, dtype=np.uint8))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if area < 80 or area > small.shape[0]*small.shape[1]*.20 or perimeter == 0:
            continue
        if 4*math.pi*area/(perimeter*perimeter) < .65:
            continue
        (x, y), radius = cv2.minEnclosingCircle(contour)
        if radius < 5 or area/(math.pi*radius*radius) < .65:
            continue
        # Reject clipped targets at image edges.
        if x-radius < 1 or y-radius < 1 or x+radius >= small.shape[1]-1 or y+radius >= small.shape[0]-1:
            continue
        candidates.append((area, x/scale, y/scale, radius/scale))
    if not candidates:
        return None
    return max(candidates)[1:]


def centering_commands(target, width, height, gain=.18, maximum=.15, deadband=.08,
                       yaw_sign=-1., heave_sign=-1.):
    if target is None:
        return 0., 0.
    ex = (target[0]-width/2)/(width/2)
    ey = (target[1]-height/2)/(height/2)
    def output(error, sign):
        if abs(error) <= deadband:
            return 0.
        return sign * max(-maximum, min(maximum, gain*error))
    return output(ex, yaw_sign), output(ey, heave_sign)


class BallTracker:
    """Shares the viewer's latest frame; never publishes directly to thrusters."""
    def __init__(self, node, grabber):
        self.node, self.grabber = node, grabber
        defaults = {'ball_gain': .18, 'ball_max_command': .15, 'ball_deadband': .08,
                    'ball_yaw_sign': -1., 'ball_heave_sign': -1.,
                    'ball_h_min': 20, 'ball_h_max': 45, 'ball_s_min': 90, 'ball_v_min': 80}
        for name, value in defaults.items():
            node.declare_parameter(name, value)
        self.params = {name: node.get_parameter(name).value for name in defaults}
        p = self.params
        if (not all(math.isfinite(v) for v in p.values()) or
            not 0 < p['ball_gain'] <= 1 or not 0 < p['ball_max_command'] <= .3 or
            not 0 <= p['ball_deadband'] < 1 or
            p['ball_yaw_sign'] not in (-1, 1) or p['ball_heave_sign'] not in (-1, 1) or
            not 0 <= p['ball_h_min'] <= p['ball_h_max'] <= 179 or
            not 0 <= p['ball_s_min'] <= 255 or not 0 <= p['ball_v_min'] <= 255):
            raise ValueError('Invalid tennis-ball tracker tuning')
        self.enabled = False
        self.enable_time = float('-inf')
        self.frame_stamp = 0.
        self.count = 0
        self.target = None
        self.previous = None
        self.overlay = (None, 'Tennis ball: detection only')
        self.pub = node.create_publisher(Twist, COMMAND_TOPIC, 1)
        self.valid_pub = node.create_publisher(Bool, TARGET_TOPIC, 1)
        self.sub = node.create_subscription(Bool, ENABLE_TOPIC, self.enable_callback, 1)
        self.timer = node.create_timer(1/15, self.tick)

    def enable_callback(self, msg):
        self.enable_time = time.monotonic()
        self.enabled = bool(msg.data)

    def tick(self):
        if not self.node.show:
            self.frame_stamp = 0.
            self.count = 0
            self.target = self.previous = None
            self.overlay = (None, 'Tennis ball: camera hidden / motion OFF')
            self.pub.publish(Twist())
            self.valid_pub.publish(Bool(data=False))
            return
        now = time.monotonic()
        frame, stamp = self.grabber.latest_sample()
        fresh = frame is not None and now-stamp < .25
        if fresh and stamp != self.frame_stamp:
            self.frame_stamp = stamp
            p = self.params
            target = detect_ball(frame, (p['ball_h_min'], p['ball_s_min'], p['ball_v_min']),
                                 (p['ball_h_max'], 255, 255))
            continuous = (target is not None and self.previous is not None and
                          math.hypot(target[0]-self.previous[0], target[1]-self.previous[1]) < frame.shape[1]*.12)
            self.count = self.count+1 if continuous else (1 if target is not None else 0)
            self.previous = self.target = target
        if not fresh:
            self.count = 0
            self.target = self.previous = None
        valid = fresh and self.count >= 3 and self.node.vision_allowed
        self.valid_pub.publish(Bool(data=bool(valid)))
        active = valid and self.enabled and now-self.enable_time < .5
        cmd = Twist()  # No surge, ever.
        if active:
            p = self.params
            cmd.angular.z, cmd.linear.z = centering_commands(
                self.target, frame.shape[1], frame.shape[0], p['ball_gain'], p['ball_max_command'],
                p['ball_deadband'], p['ball_yaw_sign'], p['ball_heave_sign'])
        self.pub.publish(cmd)
        self.overlay = (self.target if fresh else None,
                        'Tennis ball: ACTIVE (yaw/heave)' if active else
                        'Tennis ball: ready / motion OFF' if valid else 'Tennis ball: no valid target / motion OFF')

    def draw(self, frame):
        target, label = self.overlay
        h, w = frame.shape[:2]
        cv2.drawMarker(frame, (w//2, h//2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        if target is not None:
            x, y, radius = (round(v) for v in target)
            cv2.circle(frame, (x, y), radius, (0, 255, 255), 2)
            cv2.putText(frame, f'({x}, {y}) px', (max(0, x-60), max(20, y-radius-10)),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
        cv2.putText(frame, label, (12, h-18), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)

    def stop(self):
        self.timer.cancel()
        self.pub.publish(Twist())
        self.valid_pub.publish(Bool(data=False))
