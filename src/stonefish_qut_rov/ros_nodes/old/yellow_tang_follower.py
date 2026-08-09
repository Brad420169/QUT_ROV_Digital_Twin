#!/usr/bin/env python3

import os
import sys
import select
import threading
import collections

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge

sys.path.insert(0, os.path.dirname(__file__))
from pid_controller import PIDController

BUOYANCY_OFFSET       = 185.0
REFERENCE_DIAGONAL_PX = 100.0
REFERENCE_DISTANCE_M  = 1.0
GRAPH_LEN             = 150


class YellowTangFollower(Node):

    def __init__(self) -> None:
        super().__init__("yellow_tang_follower")

        self.declare_parameter("max_pwm",          600.0)
        self.declare_parameter("forward_pwm",      150.0)
        self.declare_parameter("deadband_px",      20)
        self.declare_parameter("image_topic",      "/qut_rov/left/fish_annotated")
        self.declare_parameter("detection_topic",  "/qut_rov/left/fish_detections")
        self.declare_parameter("thruster_topic",   "/qut_rov/setpoint/thrusters")
        self.declare_parameter("frame_width",      640)
        self.declare_parameter("frame_height",     480)

        self.declare_parameter("yaw.kp",    1.5)
        self.declare_parameter("yaw.ki",    0.2)
        self.declare_parameter("yaw.kd",    0.3)
        self.declare_parameter("pitch.kp", 15.0)
        self.declare_parameter("pitch.ki",  5.0)
        self.declare_parameter("pitch.kd",  3.0)

        self._max_pwm     = self.get_parameter("max_pwm").value
        self._forward_pwm = self.get_parameter("forward_pwm").value
        self._deadband    = self.get_parameter("deadband_px").value
        self._fw          = self.get_parameter("frame_width").value
        self._fh          = self.get_parameter("frame_height").value

        self._yaw_pid = PIDController(
            kp=self.get_parameter("yaw.kp").value,
            ki=self.get_parameter("yaw.ki").value,
            kd=self.get_parameter("yaw.kd").value,
            output_min=-self._max_pwm, output_max=self._max_pwm, wrap_angle=False,
        )
        self._pitch_pid = PIDController(
            kp=self.get_parameter("pitch.kp").value,
            ki=self.get_parameter("pitch.ki").value,
            kd=self.get_parameter("pitch.kd").value,
            output_min=-self._max_pwm, output_max=self._max_pwm, wrap_angle=False,
        )

        self._enabled        = False
        self._prev_stamp     = None
        self._log_counter    = 0
        self._bridge         = CvBridge()
        self._last_detection = None   # (cx, cy, w, h, conf) or None

        empty = lambda: collections.deque([0.0] * GRAPH_LEN, maxlen=GRAPH_LEN)
        self._hist_xerr = empty()
        self._hist_yerr = empty()
        self._hist_tl   = empty()
        self._hist_tr   = empty()
        self._hist_bl   = empty()
        self._hist_br   = empty()

        self._pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("thruster_topic").value, 10,
        )

        # detection data from detector — drives all control logic
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("detection_topic").value,
            self._detection_callback,
            qos_profile_sensor_data,
        )

        # annotated image — debug display only, no control logic
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._image_callback,
            qos_profile_sensor_data,
        )

        threading.Thread(target=self._keyboard_listener, daemon=True).start()

        self.get_logger().info(
            "\n=================================\n"
            "  Yellow Tang Follower READY\n"
            "  Press 'f' + ENTER to toggle\n"
            "================================="
        )

    # ── Keyboard ──────────────────────────────────────────────────────

    def _keyboard_listener(self) -> None:
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.readline().strip().lower()
                if key == "f":
                    self._enabled = not self._enabled
                    if not self._enabled:
                        self._yaw_pid.reset()
                        self._pitch_pid.reset()
                        self._publish_pwm(0.0, 0.0, 0.0, 0.0)
                    self.get_logger().info(
                        f"Following {'ENABLED' if self._enabled else 'DISABLED'}"
                    )

    # ── Detection callback — all control logic lives here ─────────────

    def _detection_callback(self, msg: Float64MultiArray) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._prev_stamp is None:
            self._prev_stamp = now
            return
        dt = now - self._prev_stamp
        self._prev_stamp = now
        if dt <= 0.0 or dt > 1.0:
            return

        # empty = no fish in frame
        if len(msg.data) == 0:
            self._last_detection = None
            self._update_history(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self._publish_pwm(0.0, 0.0, 0.0, 0.0)
            return

        cx_fish, cy_fish, w_fish, h_fish, conf = msg.data

        self._last_detection = (cx_fish, cy_fish, w_fish, h_fish, conf)

        # frame centre from declared parameters
        cx = self._fw / 2.0
        cy = self._fh / 2.0

        x_err = float(cx_fish - cx)
        y_err = float(cy_fish - cy)
        if abs(x_err) < self._deadband:
            x_err = 0.0
        if abs(y_err) < self._deadband:
            y_err = 0.0

        bbox_diag    = (w_fish**2 + h_fish**2) ** 0.5
        est_distance = (REFERENCE_DIAGONAL_PX * REFERENCE_DISTANCE_M
                        / bbox_diag) if bbox_diag > 0 else float("inf")

        # PID
        yaw_cmd   = self._yaw_pid.compute(setpoint=0.0, measurement=x_err, dt=dt)
        pitch_cmd = self._pitch_pid.compute(setpoint=0.0, measurement=y_err, dt=dt)

        # thruster mixing
        bl = float(np.clip(pitch_cmd - BUOYANCY_OFFSET, -self._max_pwm, self._max_pwm))
        br = float(np.clip(pitch_cmd - BUOYANCY_OFFSET, -self._max_pwm, self._max_pwm))
        tl = float(np.clip(self._forward_pwm + yaw_cmd,  -self._max_pwm, self._max_pwm))
        tr = float(np.clip(self._forward_pwm - yaw_cmd,  -self._max_pwm, self._max_pwm))

        self._update_history(x_err, y_err, tl, tr, bl, br)

        self._log_counter += 1
        if self._log_counter % 15 == 0:
            self.get_logger().info(
                f"fish=({cx_fish:.0f},{cy_fish:.0f}) conf={conf:.2f} "
                f"x_err={x_err:+.0f} y_err={y_err:+.0f} dist={est_distance:.2f}m | "
                f"TL={tl:+.0f} TR={tr:+.0f} BL={bl:+.0f} BR={br:+.0f}"
            )

        if not self._enabled:
            self._publish_pwm(0.0, 0.0, 0.0, 0.0)
            return

        self._publish_pwm(tl, tr, bl, br)

    # ── Image callback — debug display only ───────────────────────────

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Image decode failed: {e}")
            return

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        debug = frame.copy()

        cv2.drawMarker(debug, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 30, 2)

        det = self._last_detection
        if det is None:
            cv2.putText(debug, "NO FISH DETECTED", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            cx_fish, cy_fish, w_fish, h_fish, conf = det

            # draw bbox from detection data — no pixel hunting
            x1 = int(cx_fish - w_fish / 2)
            y1 = int(cy_fish - h_fish / 2)
            x2 = int(cx_fish + w_fish / 2)
            y2 = int(cy_fish + h_fish / 2)
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(debug, (int(cx_fish), int(cy_fish)), 5, (0, 255, 0), -1)
            cv2.line(debug, (cx, cy), (int(cx_fish), int(cy_fish)), (0, 255, 255), 1)

            bbox_diag    = (w_fish**2 + h_fish**2) ** 0.5
            est_distance = (REFERENCE_DIAGONAL_PX * REFERENCE_DISTANCE_M
                            / bbox_diag) if bbox_diag > 0 else float("inf")

            x_err = cx_fish - cx
            y_err = cy_fish - cy

            # read latest PWM values from history for overlay
            tl = self._hist_tl[-1] if self._hist_tl else 0.0
            tr = self._hist_tr[-1] if self._hist_tr else 0.0
            bl = self._hist_bl[-1] if self._hist_bl else 0.0
            br = self._hist_br[-1] if self._hist_br else 0.0

            lines = [
                (f"conf  : {conf:.2f}",              (200, 200, 200)),
                (f"x_err : {x_err:+.0f} px",         (0, 255, 255)),
                (f"y_err : {y_err:+.0f} px",         (0, 255, 255)),
                (f"dist  : {est_distance:.2f} m",    (0, 200, 255)),
                (f"TL={tl:+.0f}  TR={tr:+.0f}",     (180, 255, 100)),
                (f"BL={bl:+.0f}  BR={br:+.0f}",     (180, 255, 100)),
            ]
            for i, (text, colour) in enumerate(lines):
                cv2.putText(debug, text, (10, 30 + i * 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)

        status        = "FOLLOWING" if self._enabled else "DISABLED — press f"
        status_colour = (0, 255, 0) if self._enabled else (0, 165, 255)
        cv2.putText(debug, status, (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_colour, 2)

        self._show(debug)

    # ── Signal graph ──────────────────────────────────────────────────

    def _update_history(self, xe, ye, tl, tr, bl, br):
        self._hist_xerr.append(xe)
        self._hist_yerr.append(ye)
        self._hist_tl.append(tl)
        self._hist_tr.append(tr)
        self._hist_bl.append(bl)
        self._hist_br.append(br)

    def _draw_graph(self, canvas, data, colour, y_min, y_max, row_y, row_h, label):
        W   = canvas.shape[1]
        arr = np.array(data, dtype=float)

        def to_px(v):
            frac = (v - y_min) / (y_max - y_min + 1e-9)
            return int(row_y + row_h - frac * row_h)

        pts = [(int(i * W / GRAPH_LEN), to_px(arr[i])) for i in range(len(arr))]
        for i in range(1, len(pts)):
            cv2.line(canvas, pts[i-1], pts[i], colour, 1)

        zero_y = to_px(0.0)
        cv2.line(canvas, (0, zero_y), (W, zero_y), (60, 60, 60), 1)
        if label:
            cv2.putText(canvas, label, (4, row_y + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1)

    def _show(self, debug_frame):
        cv2.imshow("Yellow Tang Follower", debug_frame)

        GH, GW = 300, debug_frame.shape[1]
        graph  = np.zeros((GH, GW, 3), dtype=np.uint8)
        row_h  = GH // 3

        self._draw_graph(graph, self._hist_xerr, (0, 255, 255),
                         -self._fw / 2, self._fw / 2,
                         0, row_h, "x_err (px)")
        self._draw_graph(graph, self._hist_yerr, (0, 200, 255),
                         -self._fh / 2, self._fh / 2,
                         row_h, row_h, "y_err (px)")

        for hist, col, lbl in [
            (self._hist_tl, (180, 255, 100), "TL/TR/BL/BR PWM"),
            (self._hist_tr, (100, 255, 180), ""),
            (self._hist_bl, (255, 180, 100), ""),
            (self._hist_br, (255, 100, 180), ""),
        ]:
            self._draw_graph(graph, hist, col,
                             -self._max_pwm, self._max_pwm,
                             row_h * 2, row_h, lbl)

        cv2.imshow("Signal Graph", graph)
        cv2.waitKey(1)

    # ── Publish ───────────────────────────────────────────────────────

    def _publish_pwm(self, tl, tr, bl, br):
        msg = Float64MultiArray()
        msg.data = [
            float(np.clip(bl, -self._max_pwm, self._max_pwm)),
            float(np.clip(br, -self._max_pwm, self._max_pwm)),
            float(np.clip(tl, -self._max_pwm, self._max_pwm)),
            float(np.clip(tr, -self._max_pwm, self._max_pwm)),
        ]
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YellowTangFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_pwm(0.0, 0.0, 0.0, 0.0)
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()