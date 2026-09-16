#!/usr/bin/env python3
"""
Real ROV camera viewer (Z-1Mini RTSP).

Started once alongside teleop_controller in real mode and left running
for the whole session. The RTSP connection stays open the entire time;
the Y button only shows or hides the window, so there is no reconnect
or first-keyframe delay after the initial launch.

Subscribes:
    /qut_rov/camera_show   (std_msgs/Bool)  — window visible or not

OpenCV yellow-ball detection runs only while the camera window is shown.
When enabled by teleop,
the tracker publishes normalized yaw/heave commands to /qut_rov/tennis_ball_cmd.

Standalone (window opens immediately):
    ros2 run stonefish_qut_rov camera_viewer_rtsp.py \
        --ros-args -p start_visible:=true
"""

import os
import signal
import threading
import time
from collections import deque
import numpy as np

# Must be set before cv2 opens the stream.
# TCP transport is far more reliable than UDP over the Fathom tether;
# Let FFmpeg discover the stream normally; tiny probes/nobuffer caused
# startup frame loss with the camera's 720p stream. The grabber drains it.
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
    "rtsp_transport;tcp|flags;low_delay",
)

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float32, String
from gimbal_fpv import FPVHold
from camera_thermal import TemperatureMonitor, VisionGate
from tennis_ball_tracker import BallTracker

from rov_config import (
    CAMERA_SHOW_TOPIC,
    REAL_CAMERA_DISPLAY_WIDTH,
    REAL_RTSP_URL,
    REAL_CAMERA_IP,
)

WINDOW = "SubbyROV - Camera"
RECONNECT_DELAY = 2.0


class FrameGrabber:
    """
    Continuously drains the stream, keeping only the latest frame.

    RTSP buffers aggressively — without this the picture ends up
    seconds behind the ROV.
    """

    def __init__(self, url: str):
        self.url = url
        self.lock = threading.Lock()
        self.frame = None
        self.frame_times = deque(maxlen=240)
        self.running = True

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        cap = None

        while self.running:
            if cap is None or not cap.isOpened():
                if cap is not None:
                    cap.release()

                print(f"Connecting to {self.url} ...", flush=True)
                # Bound reconnect delays and avoid frame-thread decoding queues.
                params = [
                    cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 10000,
                    cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000,
                ]
                if hasattr(cv2, "CAP_PROP_N_THREADS"):
                    params.extend([cv2.CAP_PROP_N_THREADS, 1])
                cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG, params)

                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    pass

                if not cap.isOpened():
                    print("  failed, retrying...", flush=True)
                    time.sleep(RECONNECT_DELAY)
                    continue

                print("  connected.", flush=True)

            ok, frame = cap.read()

            if not ok:
                print("  stream dropped, reconnecting...", flush=True)
                with self.lock:
                    self.frame = None
                    self.frame_times.clear()
                cap.release()
                cap = None
                continue

            with self.lock:
                self.frame = frame
                self.frame_times.append(time.monotonic())

        if cap is not None:
            cap.release()

    def latest(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def latest_sample(self):
        with self.lock:
            return (None if self.frame is None else self.frame.copy(),
                    self.frame_times[-1] if self.frame_times else 0.)

    def stop(self):
        self.running = False
        self.thread.join(timeout=2.0)

    def health(self):
        now = time.monotonic()
        with self.lock:
            recent = [t for t in self.frame_times if now-t <= 2]
        age = now-recent[-1] if recent else float('inf')
        span = recent[-1]-recent[0] if len(recent) > 1 else 0
        fps = (len(recent)-1)/span if span >= 1 else 0
        return fps, age


class ViewerNode(Node):
    """Holds the requested visibility; the main thread owns the GUI."""

    def __init__(self):
        super().__init__("camera_viewer")

        self.declare_parameter("start_visible", False)
        self.declare_parameter("gimbal_hold_enabled", True)
        self.declare_parameter("gimbal_pitch_deg", 90.0)
        self.declare_parameter("gimbal_yaw_deg", -90.0)
        self.declare_parameter("gimbal_host", REAL_CAMERA_IP)
        self.vision_allowed = False
        self.health_message = 'Waiting for camera health; all vision control disabled.'
        self._health_state = None
        self.show = bool(
            self.get_parameter("start_visible")
            .get_parameter_value()
            .bool_value
        )

        self.create_subscription(
            Bool,
            CAMERA_SHOW_TOPIC,
            self.show_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self.get_logger().info(
            f"Camera viewer ready ({REAL_RTSP_URL}) — "
            f"waiting on {CAMERA_SHOW_TOPIC}"
        )

    def show_callback(self, msg: Bool):
        self.show = bool(msg.data)
        self.get_logger().info(
            "Camera window shown." if self.show else "Camera window hidden."
        )
        if self.show and not self.vision_allowed:
            self.get_logger().warning(self.health_message)

    def start_health(self, grabber):
        self.temperature = TemperatureMonitor(self.get_parameter('gimbal_host').value)
        self.gate = VisionGate()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.allowed_pub = self.create_publisher(Bool, '/qut_rov/vision_control_allowed', qos)
        self.health_pub = self.create_publisher(String, '/qut_rov/camera_health', qos)
        self.temp_pub = self.create_publisher(Float32, '/qut_rov/camera_temperature', qos)
        self.health_timer = self.create_timer(.5, lambda: self.check_health(grabber))
        self.check_health(grabber)

    def check_health(self, grabber):
        value, stamp, error = self.temperature.sample
        fps, frame_age = grabber.health()
        self.vision_allowed, self.health_message = self.gate.evaluate(
            value, time.monotonic()-stamp, fps, frame_age)
        if error:
            self.health_message += ' ' + error
        self.allowed_pub.publish(Bool(data=self.vision_allowed))
        self.health_pub.publish(String(data=self.health_message))
        self.temp_pub.publish(Float32(data=float(value) if value is not None else float('nan')))
        state = (self.vision_allowed, self.gate.hot, value is None, fps < 10 or frame_age > 1)
        if state != self._health_state:
            if self.vision_allowed:
                self.get_logger().info(self.health_message)
            else:
                self.get_logger().warning(self.health_message)
            self._health_state = state


def main(args=None):
    rclpy.init(args=args)
    node = ViewerNode()

    gimbal = None
    if node.get_parameter("gimbal_hold_enabled").value:
        gimbal = FPVHold(
            node.get_parameter("gimbal_host").value,
            pitch=node.get_parameter("gimbal_pitch_deg").value,
            yaw=node.get_parameter("gimbal_yaw_deg").value,
            log=node.get_logger().info,
        )

    grabber = FrameGrabber(REAL_RTSP_URL)
    node.start_health(grabber)
    tracker = BallTracker(node, grabber)

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    spin_thread.start()

    requested = threading.Event()

    def shutdown(signum, frame):
        requested.set()

    signal.signal(signal.SIGTERM, shutdown)   # kill / pkill
    signal.signal(signal.SIGHUP, shutdown)    # terminal closed

    window_open = False

    try:
        while rclpy.ok() and not requested.is_set():
            if node.show:
                if not window_open:
                    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
                    window_open = True

                frame = grabber.latest()
                if frame is None:
                    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

                if frame is not None:
                    tracker.draw(frame)
                    height, width = frame.shape[:2]

                    if width > REAL_CAMERA_DISPLAY_WIDTH:
                        scale = REAL_CAMERA_DISPLAY_WIDTH / float(width)
                        frame = cv2.resize(
                            frame,
                            (
                                REAL_CAMERA_DISPLAY_WIDTH,
                                int(height * scale),
                            ),
                        )

                    if not node.vision_allowed:
                        import textwrap
                        lines = textwrap.wrap(node.health_message, width=85)
                        cv2.rectangle(frame, (0, 0), (frame.shape[1], 35 * len(lines) + 15), (0, 0, 100), -1)
                        for i, line in enumerate(lines):
                            cv2.putText(frame, line, (12, 30 + i * 35), cv2.FONT_HERSHEY_SIMPLEX,
                                        .65, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.imshow(WINDOW, frame)

                key = cv2.waitKey(5) & 0xFF

                # q / ESC / closing the window is the same as pressing Y
                if key in (ord("q"), 27):
                    node.show = False

                elif cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    node.show = False

            else:
                if window_open:
                    cv2.destroyWindow(WINDOW)
                    cv2.waitKey(1)
                    window_open = False

                # Idle without burning CPU. The grabber keeps the RTSP
                # connection alive in the background.
                time.sleep(0.1)

    except KeyboardInterrupt:
        pass

    finally:
        tracker.stop()
        node.health_timer.cancel()
        node.allowed_pub.publish(Bool(data=False))
        node.temperature.stop()
        if gimbal is not None:
            gimbal.stop()
        grabber.stop()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
