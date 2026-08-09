#!/usr/bin/env python3

import collections
import os
import select
import sys
import threading
import time
from pathlib import Path
from threading import Lock, Thread
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64MultiArray
from ultralytics import YOLO
from ultralytics.utils import LOGGER

LOGGER.setLevel("ERROR")

# Allows pid_controller.py to remain beside this script.
sys.path.insert(0, os.path.dirname(__file__))
from pid_controller import PIDController


BUOYANCY_OFFSET = 185.0
REFERENCE_DIAGONAL_PX = 100.0
REFERENCE_DISTANCE_M = 1.0
GRAPH_LEN = 150

TARGET_DISTANCE_M = 1.0
DISTANCE_DEADBAND_M = 0.1

FOLLOW_FORWARD_PWM = 300

Detection = Tuple[float, float, float, float, float]

def apply_deadband(error, deadband):
    if abs(error) <= deadband:
        return 0.0

    return np.sign(error) * (abs(error) - deadband)

class FishDetectorFollower(Node):
    """Detect a yellow tang and command the ROV to follow it."""

    def __init__(self) -> None:
        super().__init__("fish_detector_follower")

        package_share = Path(get_package_share_directory("stonefish_qut_rov"))
        default_model_path = package_share / "models" / "yellow_tang_best.pt"

        # Detector parameters
        self.declare_parameter("model_path", str(default_model_path))
        self.declare_parameter("input_topic", "/qut_rov/left/image_color")
        self.declare_parameter("output_topic", "/qut_rov/left/fish_annotated")
        self.declare_parameter("detection_topic", "/qut_rov/left/fish_detections")
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("image_size", 480)
        self.declare_parameter("device", "0")
        self.declare_parameter("infer_every_n", 3)
        self.declare_parameter("use_half", True)

        # Target-lock parameters
        # Once Button[3] enables following, the first selected fish is locked.
        # Subsequent detections must overlap it or remain close to its last
        # position. If the target is lost, the ROV stops instead of changing fish.
        self.declare_parameter("lock_min_iou", 0.10)
        self.declare_parameter("lock_max_center_distance_px", 120.0)
        self.declare_parameter("lock_max_size_ratio", 2.5)

        # Follower parameters
        self.declare_parameter("thruster_topic", "/qut_rov/setpoint/thrusters")
        self.declare_parameter("enable_topic", "/qut_rov/fish_follow_enabled")
        self.declare_parameter("max_pwm", 600.0)
        self.declare_parameter("forward_pwm", 300.0)
        self.declare_parameter("deadband_px", 5.0)

        self.declare_parameter("yaw.kp", 1.9)
        self.declare_parameter("yaw.ki", 0.4)
        self.declare_parameter("yaw.kd", 0.4)
        self.declare_parameter("pitch.kp", 15.0)
        self.declare_parameter("pitch.ki", 5.0)
        self.declare_parameter("pitch.kd", 3.0)

        self._model_path = self.get_parameter("model_path").value
        self._input_topic = self.get_parameter("input_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._detection_topic = self.get_parameter("detection_topic").value
        self._thruster_topic = self.get_parameter("thruster_topic").value
        self._enable_topic = self.get_parameter("enable_topic").value

        self._confidence = float(self.get_parameter("confidence").value)
        self._image_size = int(self.get_parameter("image_size").value)
        self._device = str(self.get_parameter("device").value)
        self._infer_every_n = max(1, int(self.get_parameter("infer_every_n").value))
        self._use_half = bool(self.get_parameter("use_half").value)

        self._lock_min_iou = float(self.get_parameter("lock_min_iou").value)
        self._lock_max_center_distance = float(
            self.get_parameter("lock_max_center_distance_px").value
        )
        self._lock_max_size_ratio = float(
            self.get_parameter("lock_max_size_ratio").value
        )

        self._max_pwm = float(self.get_parameter("max_pwm").value)
        self._forward_pwm = float(self.get_parameter("forward_pwm").value)
        self._deadband = float(self.get_parameter("deadband_px").value)

        if not Path(self._model_path).is_file():
            raise FileNotFoundError(f"Model not found: {self._model_path}")

        self.get_logger().info(f"Loading model: {self._model_path}")
        self._model = YOLO(self._model_path)
        self._model.fuse()
        self.get_logger().info(f"Model classes: {self._model.names}")

        self._yaw_pid = PIDController(
            kp=self.get_parameter("yaw.kp").value,
            ki=self.get_parameter("yaw.ki").value,
            kd=self.get_parameter("yaw.kd").value,
            output_min=-self._max_pwm,
            output_max=self._max_pwm,
            wrap_angle=False,
        )
        self._pitch_pid = PIDController(
            kp=self.get_parameter("pitch.kp").value,
            ki=self.get_parameter("pitch.ki").value,
            kd=self.get_parameter("pitch.kd").value,
            output_min=-self._max_pwm,
            output_max=self._max_pwm,
            wrap_angle=False,
        )

        self._bridge = CvBridge()
        self._lock = Lock()
        self._enabled = False
        self._running = True
        self._frame_count = 0
        self._prev_control_time: Optional[float] = None
        self._log_counter = 0

        self._pending_frame = None
        self._pending_header = None
        self._last_annotated = None
        self._last_annotated_header = None
        self._last_detection: Optional[Detection] = None

        # Persistent target state. None means no fish has been locked yet.
        # The lock is deliberately not reassigned while following is enabled.
        self._locked_detection: Optional[Detection] = None
        self._target_was_lost = False

        self._last_frame_width = 640
        self._last_frame_height = 480

        empty = lambda: collections.deque([0.0] * GRAPH_LEN, maxlen=GRAPH_LEN)
        self._hist_xerr = empty()
        self._hist_yerr = empty()
        self._hist_bl = empty()
        self._hist_br = empty()
        self._hist_tl = empty()
        self._hist_tr = empty()

        self._annotated_publisher = self.create_publisher(
            Image, self._output_topic, qos_profile_sensor_data
        )
        self._detection_publisher = self.create_publisher(
            Float64MultiArray, self._detection_topic, qos_profile_sensor_data
        )
        self._thruster_publisher = self.create_publisher(
            Float64MultiArray, self._thruster_topic, 10
        )

        self.create_subscription(
            Image,
            self._input_topic,
            self._camera_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            self._enable_topic,
            self._follow_enable_callback,
            10,
        )

        self._inference_thread = Thread(target=self._inference_loop, daemon=True)
        self._inference_thread.start()
        threading.Thread(target=self._keyboard_listener, daemon=True).start()

        self.get_logger().info(
            "\n========================================\n"
            "  Fish Detector and Follower READY\n"
            "  Button[3] toggles following from teleop\n"
            "========================================\n"
            f"Camera:      {self._input_topic}\n"
            f"Annotated:   {self._output_topic}\n"
            f"Detections:  {self._detection_topic}\n"
            f"Thrusters:   {self._thruster_topic}\n"
            f"Enable:      {self._enable_topic}\n"
            f"imgsz={self._image_size} conf={self._confidence} "
            f"every_n={self._infer_every_n}"
        )

    # ------------------------------------------------------------------
    # Camera and inference
    # ------------------------------------------------------------------

    def _camera_callback(self, message: Image) -> None:
        # The process may receive SIGTERM while a camera callback is queued.
        # Do not decode or publish after shutdown has started.
        if not self._running or not rclpy.ok():
            return

        self._frame_count += 1

        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image decode failed: {exc}")
            return

        height, width = frame.shape[:2]
        with self._lock:
            self._last_frame_width = width
            self._last_frame_height = height

            if self._frame_count % self._infer_every_n == 0:
                # Keep only the newest frame so inference cannot build a backlog.
                self._pending_frame = frame.copy()
                self._pending_header = message.header

            annotated = (
                None if self._last_annotated is None else self._last_annotated.copy()
            )
            annotated_header = self._last_annotated_header
            detection = self._last_detection

        if not self._running or not rclpy.ok():
            return

        if annotated is not None:
            try:
                annotated_msg = self._bridge.cv2_to_imgmsg(
                    annotated, encoding="bgr8"
                )
                annotated_msg.header = annotated_header or message.header

                if self._running and rclpy.ok():
                    self._annotated_publisher.publish(annotated_msg)
                    self._show_debug(annotated, detection)

            except Exception as exc:
                # During process termination the ROS context can become invalid
                # between the rclpy.ok() check and publish(). Ignore that expected
                # shutdown race, but report genuine runtime errors.
                if self._running and rclpy.ok():
                    self.get_logger().error(
                        f"Annotated image publish failed: {exc}"
                    )
                return

        if not self._running or not rclpy.ok():
            return

        # Continue publishing the detection topic for inspection or other nodes.
        detection_msg = Float64MultiArray()
        detection_msg.data = list(detection) if detection is not None else []

        try:
            self._detection_publisher.publish(detection_msg)
        except Exception as exc:
            if self._running and rclpy.ok():
                self.get_logger().error(
                    f"Detection publish failed: {exc}"
                )

    def _inference_loop(self) -> None:
        while rclpy.ok() and self._running:
            with self._lock:
                frame = self._pending_frame
                header = self._pending_header
                if frame is not None:
                    self._pending_frame = None
                    self._pending_header = None

            if frame is None:
                time.sleep(0.005)
                continue

            try:
                results = self._model.predict(
                    source=frame,
                    conf=self._confidence,
                    imgsz=self._image_size,
                    device=self._device,
                    verbose=False,
                    half=self._use_half,
                )

                result = results[0]
                annotated = result.plot()
                detection = self._select_target_detection(result)

                with self._lock:
                    self._last_annotated = annotated
                    self._last_annotated_header = header
                    self._last_detection = detection
                    frame_height, frame_width = frame.shape[:2]

                # Direct detector-to-controller path: no ROS subscription needed.
                self._update_follower(
                    detection=detection,
                    frame_width=frame_width,
                    frame_height=frame_height,
                )

            except Exception as exc:
                self.get_logger().error(f"Inference error: {exc}")
                with self._lock:
                    self._last_detection = None
                self._stop_for_lost_target()

    @staticmethod
    def _box_to_detection(box) -> Detection:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        return (
            float((x1 + x2) / 2.0),
            float((y1 + y2) / 2.0),
            float(x2 - x1),
            float(y2 - y1),
            float(box.conf.item()),
        )

    @staticmethod
    def _detection_iou(a: Detection, b: Detection) -> float:
        """Calculate intersection-over-union between two centre-format boxes."""

        ax, ay, aw, ah, _ = a
        bx, by, bw, bh, _ = b

        ax1, ay1 = ax - aw / 2.0, ay - ah / 2.0
        ax2, ay2 = ax + aw / 2.0, ay + ah / 2.0
        bx1, by1 = bx - bw / 2.0, by - bh / 2.0
        bx2, by2 = bx + bw / 2.0, by + bh / 2.0

        intersection_w = max(0.0, min(ax2, bx2) - max(ax1, bx1))
        intersection_h = max(0.0, min(ay2, by2) - max(ay1, by1))
        intersection = intersection_w * intersection_h

        union = aw * ah + bw * bh - intersection
        return intersection / union if union > 0.0 else 0.0

    def _candidate_matches_lock(
        self,
        candidate: Detection,
        locked: Detection,
    ) -> tuple[bool, float]:
        """Return whether a detection could be the locked fish and its score."""

        cx, cy, width, height, confidence = candidate
        lx, ly, locked_width, locked_height, _ = locked

        centre_distance = ((cx - lx) ** 2 + (cy - ly) ** 2) ** 0.5
        iou = self._detection_iou(candidate, locked)

        candidate_area = max(width * height, 1.0)
        locked_area = max(locked_width * locked_height, 1.0)
        size_ratio = max(
            candidate_area / locked_area,
            locked_area / candidate_area,
        )

        matches = (
            size_ratio <= self._lock_max_size_ratio
            and (
                iou >= self._lock_min_iou
                or centre_distance <= self._lock_max_center_distance
            )
        )

        # IoU is weighted most strongly, followed by proximity and confidence.
        proximity_score = max(
            0.0,
            1.0 - centre_distance / max(self._lock_max_center_distance, 1.0),
        )
        score = 3.0 * iou + proximity_score + 0.25 * confidence
        return matches, score

    def _select_target_detection(self, result) -> Optional[Detection]:
        """
        Select or maintain a persistent fish target.

        When following is disabled, this returns the highest-confidence fish
        for display only. When following is enabled:
          * the first available fish becomes the locked target;
          * later frames may update only from a spatially matching detection;
          * if no match exists, None is returned and the ROV stops;
          * it never jumps to a different fish until following is toggled off.
        """

        if result.boxes is None or len(result.boxes) == 0:
            if self._enabled and self._locked_detection is not None:
                self._target_was_lost = True
            return None

        candidates = [
            self._box_to_detection(box)
            for box in result.boxes
        ]

        # While disabled, show the best detection but do not establish a lock.
        if not self._enabled:
            return max(candidates, key=lambda detection: detection[4])

        # Button[3] has just enabled following: lock the most confident fish
        # currently visible.
        if self._locked_detection is None:
            selected = max(candidates, key=lambda detection: detection[4])
            self._locked_detection = selected
            self._target_was_lost = False
            self.get_logger().info(
                "TARGET LOCKED: "
                f"centre=({selected[0]:.0f},{selected[1]:.0f}) "
                f"conf={selected[4]:.2f}"
            )
            return selected

        # Match only against the previous location/size of the locked target.
        valid_matches = []
        for candidate in candidates:
            matches, score = self._candidate_matches_lock(
                candidate,
                self._locked_detection,
            )
            if matches:
                valid_matches.append((score, candidate))

        if not valid_matches:
            if not self._target_was_lost:
                self.get_logger().warn(
                    "LOCKED TARGET LOST - stopping; "
                    "will not switch to another fish"
                )
            self._target_was_lost = True
            return None

        _, selected = max(valid_matches, key=lambda item: item[0])
        self._locked_detection = selected

        if self._target_was_lost:
            self.get_logger().info("LOCKED TARGET REACQUIRED")
        self._target_was_lost = False
        return selected

    # ------------------------------------------------------------------
    # Follower control
    # ------------------------------------------------------------------

    def _update_follower(
        self,
        detection: Optional[Detection],
        frame_width: int,
        frame_height: int,
    ) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9

        # When disabled, teleop owns the thruster topic. Do not publish zeros here,
        # because that would overwrite manual commands from the teleop node.
        if not self._enabled:
            self._prev_control_time = now
            return

        if detection is None:
            self._prev_control_time = now
            self._stop_for_lost_target()
            return

        if self._prev_control_time is None:
            self._prev_control_time = now
            return

        dt = now - self._prev_control_time
        self._prev_control_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        cx_fish, cy_fish, w_fish, h_fish, confidence = detection

        x_error = float(cx_fish - frame_width / 2.0)
        y_error = float(cy_fish - frame_height / 2.0)

        x_error = apply_deadband(x_error, 5.0)  # Yaw deadband is smaller to allow more responsive turning
        y_error = apply_deadband(y_error, 20.0)  # Pitch deadband is larger to avoid oscillation

        bbox_diagonal = (w_fish**2 + h_fish**2) ** 0.5
        estimated_distance = (
            REFERENCE_DIAGONAL_PX * REFERENCE_DISTANCE_M / bbox_diagonal
            if bbox_diagonal > 0.0
            else float("inf")
        )

        yaw_command = -self._yaw_pid.compute(
            setpoint=0.0, measurement=x_error, dt=dt
        )
        pitch_command = self._pitch_pid.compute(
            setpoint=0.0, measurement=y_error, dt=dt
        )

        # Correct physical thruster naming:
        # TL/TR = vertical-force thrusters
        # BL/BR = horizontal drive/yaw thrusters
        tl = float(
            np.clip(
                pitch_command - BUOYANCY_OFFSET,
                -self._max_pwm,
                self._max_pwm,
            )
        )
        tr = float(
            np.clip(
                pitch_command - BUOYANCY_OFFSET,
                -self._max_pwm,
                self._max_pwm,
            )
        )
        bl = float(
            np.clip(
                self._forward_pwm + yaw_command,
                -self._max_pwm,
                self._max_pwm,
            )
        )
        br = float(
            np.clip(
                self._forward_pwm - yaw_command,
                -self._max_pwm,
                self._max_pwm,
            )
        )

        self._update_history(x_error, y_error, bl, br, tl, tr)

        self._log_counter += 1
        if self._log_counter % 15 == 0:
            self.get_logger().info(
                f"fish=({cx_fish:.0f},{cy_fish:.0f}) "
                f"conf={confidence:.2f} "
                f"x_err={x_error:+.0f} y_err={y_error:+.0f} "
                f"dist={estimated_distance:.2f}m | "
                f"BL={bl:+.0f} BR={br:+.0f} "
                f"TL={tl:+.0f} TR={tr:+.0f}"
            )

        if self._enabled:
            self._publish_pwm(bl, br, tl, tr)
        else:
            self._publish_pwm(0.0, 0.0, 0.0, 0.0)

    def _stop_for_lost_target(self) -> None:
        self._yaw_pid.reset()
        self._pitch_pid.reset()
        self._update_history(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        if self._enabled:
            self._publish_pwm(0.0, 0.0, 0.0, 0.0)

    def _follow_enable_callback(self, message: Bool) -> None:
        """Receive follow-mode authority from the gamepad teleop node."""

        requested = bool(message.data)
        if requested == self._enabled:
            return

        self._enabled = requested
        self._yaw_pid.reset()
        self._pitch_pid.reset()
        self._prev_control_time = None

        # Each new Button[3] activation starts a fresh lock. Disabling releases
        # the old target, so the next activation may deliberately choose another.
        with self._lock:
            self._locked_detection = None
            self._target_was_lost = False

        if not self._enabled:
            self._publish_pwm(0.0, 0.0, 0.0, 0.0)

        self.get_logger().info(
            f"Following {'ENABLED' if self._enabled else 'DISABLED'} by teleop"
        )

    def _keyboard_listener(self) -> None:
        while rclpy.ok() and self._running:
            try:
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            except (ValueError, OSError):
                return

            if not readable:
                continue

            key = sys.stdin.readline().strip().lower()
            if key != "f":
                continue

            self._enabled = not self._enabled
            with self._lock:
                self._locked_detection = None
                self._target_was_lost = False

            if not self._enabled:
                self._yaw_pid.reset()
                self._pitch_pid.reset()
                self._publish_pwm(0.0, 0.0, 0.0, 0.0)

            self.get_logger().info(
                f"Following {'ENABLED' if self._enabled else 'DISABLED'}"
            )

    # ------------------------------------------------------------------
    # Debug display and graphs
    # ------------------------------------------------------------------

    def _show_debug(
        self,
        annotated_frame: np.ndarray,
        detection: Optional[Detection],
    ) -> None:
        debug = annotated_frame.copy()
        height, width = debug.shape[:2]
        centre_x, centre_y = width // 2, height // 2

        cv2.drawMarker(
            debug,
            (centre_x, centre_y),
            (255, 255, 255),
            cv2.MARKER_CROSS,
            30,
            2,
        )

        if detection is None:
            cv2.putText(
                debug,
                "NO FISH DETECTED",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
        else:
            cx_fish, cy_fish, w_fish, h_fish, confidence = detection

            cv2.circle(
                debug,
                (int(cx_fish), int(cy_fish)),
                5,
                (0, 255, 0),
                -1,
            )
            cv2.line(
                debug,
                (centre_x, centre_y),
                (int(cx_fish), int(cy_fish)),
                (0, 255, 255),
                1,
            )

            bbox_diagonal = (w_fish**2 + h_fish**2) ** 0.5
            estimated_distance = (
                REFERENCE_DIAGONAL_PX
                * REFERENCE_DISTANCE_M
                / bbox_diagonal
                if bbox_diagonal > 0.0
                else float("inf")
            )

            # x_error = cx_fish - centre_x
            # y_error = cy_fish - centre_y

            # bl = self._hist_bl[-1]
            # br = self._hist_br[-1]
            # tl = self._hist_tl[-1]
            # tr = self._hist_tr[-1]

            lines = [
                (f"Distance: {estimated_distance:.2f} m", (0, 255, 255)),
            ]

            for index, (text, colour) in enumerate(lines):
                cv2.putText(
                    debug,
                    text,
                    (10, 30 + index * 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    colour,
                    2,
                )

        if self._enabled and self._target_was_lost:
            status = "TARGET LOST - HOLDING LOCK"
            status_colour = (0, 0, 255)
        elif self._enabled:
            status = "FOLLOWING - TARGET LOCKED"
            status_colour = (0, 255, 0)
        else:
            status = "DISABLED - press Button 3"
            status_colour = (0, 165, 255)
        cv2.putText(
            debug,
            status,
            (10, height - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            status_colour,
            2,
        )

        display = cv2.resize(
        debug,
        (640, 480),
        interpolation=cv2.INTER_LINEAR
    )

        cv2.imshow("Fish Detector and Follower", display)
        self._show_graph(width)
        cv2.waitKey(1)

    def _update_history(self, xe, ye, bl, br, tl, tr) -> None:
        self._hist_xerr.append(xe)
        self._hist_yerr.append(ye)
        self._hist_bl.append(bl)
        self._hist_br.append(br)
        self._hist_tl.append(tl)
        self._hist_tr.append(tr)

    @staticmethod
    def _draw_graph(
        canvas,
        data: Sequence[float],
        colour,
        y_min: float,
        y_max: float,
        row_y: int,
        row_height: int,
        label: str,
    ) -> None:
        graph_width = canvas.shape[1]
        values = np.asarray(data, dtype=float)

        def to_pixel(value: float) -> int:
            fraction = (value - y_min) / (y_max - y_min + 1e-9)
            return int(row_y + row_height - fraction * row_height)

        points = [
            (int(index * graph_width / GRAPH_LEN), to_pixel(values[index]))
            for index in range(len(values))
        ]
        for index in range(1, len(points)):
            cv2.line(canvas, points[index - 1], points[index], colour, 1)

        zero_y = to_pixel(0.0)
        cv2.line(
            canvas,
            (0, zero_y),
            (graph_width, zero_y),
            (60, 60, 60),
            1,
        )
        if label:
            cv2.putText(
                canvas,
                label,
                (4, row_y + 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                colour,
                1,
            )

    def _show_graph(self, width: int) -> None:
        graph_height = 300
        graph = np.zeros((graph_height, width, 3), dtype=np.uint8)
        row_height = graph_height // 3

        self._draw_graph(
            graph,
            self._hist_xerr,
            (0, 255, 255),
            -width / 2,
            width / 2,
            0,
            row_height,
            "x_err (px)",
        )
        self._draw_graph(
            graph,
            self._hist_yerr,
            (0, 200, 255),
            -self._last_frame_height / 2,
            self._last_frame_height / 2,
            row_height,
            row_height,
            "y_err (px)",
        )

        for history, colour, label in [
            (self._hist_bl, (180, 255, 100), "BL/BR/TL/TR PWM"),
            (self._hist_br, (100, 255, 180), ""),
            (self._hist_tl, (255, 180, 100), ""),
            (self._hist_tr, (255, 100, 180), ""),
        ]:
            self._draw_graph(
                graph,
                history,
                colour,
                -self._max_pwm,
                self._max_pwm,
                row_height * 2,
                row_height,
                label,
            )

        cv2.imshow("Signal Graph", graph)

    # ------------------------------------------------------------------
    # Thruster publishing and shutdown
    # ------------------------------------------------------------------

    def _publish_pwm(
        self,
        bl: float,
        br: float,
        tl: float,
        tr: float,
        *,
        allow_during_shutdown: bool = False,
    ) -> None:
        # Normal callbacks must never publish after shutdown begins. The optional
        # flag is used only for one best-effort final zero command.
        if not rclpy.ok():
            return
        if not self._running and not allow_during_shutdown:
            return

        message = Float64MultiArray()

        # Corrected physical naming for the existing actuator topic slots:
        # [TL, TR, BL, BR]
        # TL/TR = vertical-force thrusters; BL/BR = horizontal drive/yaw.
        message.data = [
            float(np.clip(tl, -self._max_pwm, self._max_pwm)),
            float(np.clip(tr, -self._max_pwm, self._max_pwm)),
            float(np.clip(bl, -self._max_pwm, self._max_pwm)),
            float(np.clip(br, -self._max_pwm, self._max_pwm)),
        ]

        try:
            self._thruster_publisher.publish(message)
        except Exception as exc:
            if self._running and rclpy.ok():
                self.get_logger().error(f"Thruster publish failed: {exc}")

    def shutdown(self) -> None:
        # Send one best-effort zero command before disabling all publishers.
        self._enabled = False
        with self._lock:
            self._locked_detection = None
            self._target_was_lost = False

        if rclpy.ok():
            self._publish_pwm(
                0.0,
                0.0,
                0.0,
                0.0,
                allow_during_shutdown=True,
            )

        # From this point onward, callbacks and worker threads are not allowed
        # to publish.
        self._running = False

        with self._lock:
            self._pending_frame = None
            self._pending_header = None

        cv2.destroyAllWindows()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FishDetectorFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        # Expected when the subprocess is terminated by the teleop node.
        pass
    except Exception as exc:
        # A callback may already be in flight when SIGTERM invalidates the
        # context. Suppress only the expected shutdown-context error.
        if "context is invalid" not in str(exc):
            raise
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()