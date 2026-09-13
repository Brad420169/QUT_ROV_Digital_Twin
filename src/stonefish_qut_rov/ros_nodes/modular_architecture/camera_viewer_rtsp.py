#!/usr/bin/env python3
"""
Real ROV camera viewer (Z-1Mini RTSP).

Started once alongside teleop_controller in real mode and left running
for the whole session. The RTSP connection stays open the entire time;
the Y button only shows or hides the window, so there is no reconnect
or first-keyframe delay after the initial launch.

Subscribes:
    /qut_rov/camera_show   (std_msgs/Bool)  — window visible or not

No detection, no following — just the picture.

Standalone (window opens immediately):
    ros2 run stonefish_qut_rov camera_viewer_rtsp.py \
        --ros-args -p start_visible:=true
"""

import os
import signal
import threading
import time

# Must be set before cv2 opens the stream.
# TCP transport is far more reliable than UDP over the Fathom tether;
# the small probe size keeps connection setup short.
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
    "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay"
    "|probesize;32|analyzeduration;0",
)

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool

from rov_config import (
    CAMERA_SHOW_TOPIC,
    REAL_CAMERA_DISPLAY_WIDTH,
    REAL_RTSP_URL,
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
                cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)

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
                cap.release()
                cap = None
                continue

            with self.lock:
                self.frame = frame

        if cap is not None:
            cap.release()

    def latest(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        self.running = False
        self.thread.join(timeout=2.0)


class ViewerNode(Node):
    """Holds the requested visibility; the main thread owns the GUI."""

    def __init__(self):
        super().__init__("camera_viewer")

        self.declare_parameter("start_visible", False)
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


def main(args=None):
    rclpy.init(args=args)
    node = ViewerNode()

    grabber = FrameGrabber(REAL_RTSP_URL)

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

                if frame is not None:
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

                    cv2.imshow(WINDOW, frame)

                key = cv2.waitKey(20) & 0xFF

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
        grabber.stop()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
