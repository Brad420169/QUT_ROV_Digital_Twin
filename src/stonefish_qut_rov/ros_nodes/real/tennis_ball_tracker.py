"""HSV tennis-ball centering and distance following. Vehicle commands are normalized, not radians/metres."""
import math
import time
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pid_controller import PIDController
from rov_config import TRAJECTORY_FORWARD, REAL_SCALE_TRAJ_FORWARD

ENABLE_TOPIC = '/qut_rov/tennis_ball_enabled'
COMMAND_TOPIC = '/qut_rov/tennis_ball_cmd'
TARGET_TOPIC = '/qut_rov/tennis_ball_target_valid'

# Following distance in metres: edit this, then restart the camera viewer.
BALL_FOLLOW_DISTANCE_M = 0.2
BALL_SURGE_TRAJECTORY_RATIO = .5
BALL_MAX_SURGE = BALL_SURGE_TRAJECTORY_RATIO * max(0., min(1., TRAJECTORY_FORWARD * REAL_SCALE_TRAJ_FORWARD))

# Alter these when testing; restart camera/debug viewer after edits.
BALL_YAW_KP = .1
BALL_YAW_KD = .015  # Larger D damps motion but amplifies pixel jitter.
BALL_HEAVE_KP = .2
BALL_HEAVE_KD = .015
BALL_MAX_COMMAND = .15
BALL_YAW_DEADBAND = .16  # Fraction of half image width.
BALL_HEAVE_DEADBAND = .08  # Fraction of half image height.
BALL_YAW_SIGN = -1.
BALL_HEAVE_SIGN = -1.

# Original yellow/green filter (OpenCV hue: 0..179).
BALL_H_MIN = 15
BALL_H_MAX = 55
BALL_S_MIN = 50  # Lower admits washed-out colour; raise to reject pale glare.
BALL_V_MIN = 50  # Lower admits darker targets.
BALL_DETECTION_WIDTH = 960
BALL_MIN_AREA = 80  # Detection-image pixels; lower admits smaller blobs/noise.
BALL_MIN_RADIUS = 5
BALL_MIN_CIRCULARITY = .65
BALL_MIN_FILL = .65


def detect_ball(frame, hsv_low=(BALL_H_MIN, BALL_S_MIN, BALL_V_MIN),
                hsv_high=(BALL_H_MAX, 255, 255), detection_width=BALL_DETECTION_WIDTH, *, return_mask=False):
    """Return (cx, cy, radius) in original pixels, or None.

    Round yellow objects can be false positives; inspect the overlay before use.
    detection_width caps processing width without upscaling smaller inputs.
    Minimum area/radius thresholds remain in detection-image pixels.
    return_mask=True returns (target, cleaned HSV mask) for debugging.
    """
    h, w = frame.shape[:2]
    if type(detection_width) is not int or detection_width <= 0:
        raise ValueError('Detection width must be a positive integer')
    scale = min(1., detection_width / w)
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
        if area < BALL_MIN_AREA or area > small.shape[0]*small.shape[1]*.20 or perimeter == 0:
            continue
        if 4*math.pi*area/(perimeter*perimeter) < BALL_MIN_CIRCULARITY:
            continue
        (x, y), radius = cv2.minEnclosingCircle(contour)
        if radius < BALL_MIN_RADIUS or area/(math.pi*radius*radius) < BALL_MIN_FILL:
            continue
        # Reject clipped targets at image edges.
        if x-radius < 1 or y-radius < 1 or x+radius >= small.shape[1]-1 or y+radius >= small.shape[0]-1:
            continue
        candidates.append((area, x/scale, y/scale, radius/scale))
    target = max(candidates)[1:] if candidates else None
    return (target, mask) if return_mask else target


def estimate_distance(radius_px, frame_width, ball_radius_m=.0335, hfov_deg=54.7):
    """Approximate camera-to-ball-centre depth using original-frame pixels."""
    # ponytail: pinhole estimate at fixed zoom; calibrate HFOV in the housing underwater.
    focal_length_px = frame_width / (2 * math.tan(math.radians(hfov_deg / 2)))
    return focal_length_px * ball_radius_m / radius_px


def centering_commands(target, width, height, yaw_gain=.18, heave_gain=.18, maximum=.15,
                       yaw_deadband=BALL_YAW_DEADBAND, heave_deadband=BALL_HEAVE_DEADBAND,
                       yaw_sign=-1., heave_sign=-1., controllers=None, dt=1/15):
    if target is None:
        return 0., 0.
    ex = (target[0]-width/2)/(width/2)
    ey = (target[1]-height/2)/(height/2)
    def output(error, sign, gain, controller, deadband):
        if abs(error) <= deadband:
            if controller is not None:
                controller.reset()
            return 0.
        value = gain * error if controller is None else controller.compute(0., -error, dt)
        return sign * max(-maximum, min(maximum, value))
    yaw, heave = controllers if controllers is not None else (None, None)
    return (output(ex, yaw_sign, yaw_gain, yaw, yaw_deadband),
            output(ey, heave_sign, heave_gain, heave, heave_deadband))


def following_surge(distance, follow_distance=BALL_FOLLOW_DISTANCE_M, maximum=.08):
    """Constant forward command above the stopping distance, otherwise zero."""
    if not math.isfinite(distance) or distance <= follow_distance:
        return 0.
    return maximum


class BallTracker:
    """Shares the viewer's latest frame; never publishes directly to thrusters."""
    def __init__(self, node, grabber):
        self.node, self.grabber = node, grabber

        # Alter these when testing
        defaults = {'ball_yaw_gain': BALL_YAW_KP, 'ball_heave_gain': BALL_HEAVE_KP,
                    'ball_yaw_kd': BALL_YAW_KD, 'ball_heave_kd': BALL_HEAVE_KD,
                    'ball_max_command': BALL_MAX_COMMAND, 'ball_yaw_deadband': BALL_YAW_DEADBAND,
                    'ball_heave_deadband': BALL_HEAVE_DEADBAND,
                    'ball_yaw_sign': BALL_YAW_SIGN, 'ball_heave_sign': BALL_HEAVE_SIGN,
                    'ball_h_min': BALL_H_MIN, 'ball_h_max': BALL_H_MAX,
                    'ball_s_min': BALL_S_MIN, 'ball_v_min': BALL_V_MIN,
                    'ball_follow_distance_m': BALL_FOLLOW_DISTANCE_M, 'ball_max_surge': BALL_MAX_SURGE,
                    'ball_detection_width': BALL_DETECTION_WIDTH, 'ball_radius_m': .0335, 'ball_hfov_deg': 54.7}


        for name, value in defaults.items():
            node.declare_parameter(name, value)
        self.params = {name: node.get_parameter(name).value for name in defaults}
        p = self.params
        if (type(p['ball_detection_width']) is not int or p['ball_detection_width'] <= 0 or
            not all(math.isfinite(v) for v in p.values()) or
            not 0 < p['ball_yaw_gain'] <= 1 or not 0 < p['ball_heave_gain'] <= 1 or
            not 0 <= p['ball_yaw_kd'] <= 1 or not 0 <= p['ball_heave_kd'] <= 1 or
            not 0 < p['ball_max_command'] <= .3 or
            not p['ball_follow_distance_m'] > 0 or
            not 0 <= p['ball_max_surge'] <= .3 or
            not p['ball_radius_m'] > 0 or not 0 < p['ball_hfov_deg'] < 180 or
            not 0 <= p['ball_yaw_deadband'] < 1 or not 0 <= p['ball_heave_deadband'] < 1 or
            p['ball_yaw_sign'] not in (-1, 1) or p['ball_heave_sign'] not in (-1, 1) or
            not 0 <= p['ball_h_min'] <= p['ball_h_max'] <= 179 or
            not 0 <= p['ball_s_min'] <= 255 or not 0 <= p['ball_v_min'] <= 255):
            raise ValueError('Invalid tennis-ball tracker tuning')
        self.controllers = tuple(PIDController(p[f'ball_{axis}_gain'], 0., p[f'ball_{axis}_kd'],
                                               -p['ball_max_command'], p['ball_max_command'])
                                 for axis in ('yaw', 'heave'))
        self.control_stamp = None
        self.center_commands = (0., 0.)
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

    def reset_pd(self):
        for controller in self.controllers:
            controller.reset()
        self.control_stamp = None
        self.center_commands = (0., 0.)

    def enable_callback(self, msg):
        if bool(msg.data) != self.enabled:
            self.reset_pd()
        self.enable_time = time.monotonic()
        self.enabled = bool(msg.data)

    def tick(self):
        if not self.node.show:
            self.reset_pd()
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
                                 (p['ball_h_max'], 255, 255), p['ball_detection_width'])
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
        cmd = Twist()
        if active:
            p = self.params
            if stamp != self.control_stamp:
                dt = stamp - self.control_stamp if self.control_stamp is not None else 1/15
                if dt <= 0 or dt >= .25:
                    self.reset_pd()
                    dt = 1/15
                self.center_commands = centering_commands(
                    self.target, frame.shape[1], frame.shape[0],
                    p['ball_yaw_gain'], p['ball_heave_gain'], p['ball_max_command'],
                    p['ball_yaw_deadband'], p['ball_heave_deadband'], p['ball_yaw_sign'], p['ball_heave_sign'],
                    self.controllers, dt)
                self.control_stamp = stamp
            cmd.angular.z, cmd.linear.z = self.center_commands
            distance = estimate_distance(self.target[2], frame.shape[1], p['ball_radius_m'], p['ball_hfov_deg'])
            cmd.linear.x = following_surge(
                distance, p['ball_follow_distance_m'], p['ball_max_surge'])
        else:
            self.reset_pd()
        self.pub.publish(cmd)
        self.overlay = (self.target if fresh else None,
                        'Tennis ball: ACTIVE (surge/yaw/heave)' if active else
                        'Tennis ball: ready / motion OFF' if valid else 'Tennis ball: no valid target / motion OFF')

    def draw(self, frame):
        target, label = self.overlay
        h, w = frame.shape[:2]
        cv2.drawMarker(frame, (w//2, h//2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        if target is not None:
            x, y, radius = (round(v) for v in target)
            cv2.circle(frame, (x, y), radius, (0, 255, 255), 2)
            distance = estimate_distance(target[2], w, self.params['ball_radius_m'], self.params['ball_hfov_deg'])
            cv2.putText(frame, f'({x}, {y}) px  est. {distance:.2f} m', (max(0, x-60), max(20, y-radius-10)),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
        cv2.putText(frame, label, (12, h-18), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)

    def stop(self):
        self.timer.cancel()
        self.pub.publish(Twist())
        self.valid_pub.publish(Bool(data=False))
