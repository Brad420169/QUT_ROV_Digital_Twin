#!/usr/bin/env python3

from pathlib import Path
from threading import Lock, Thread

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from ultralytics import YOLO


class FishDetector(Node):
    def __init__(self) -> None:
        super().__init__("fish_detector")

        package_share = Path(get_package_share_directory("stonefish_qut_rov"))
        default_model_path = package_share / "models" / "yellow_tang_best.pt"

        self.declare_parameter("model_path",       str(default_model_path))
        self.declare_parameter("input_topic",      "/qut_rov/left/image_color")
        self.declare_parameter("output_topic",     "/qut_rov/left/fish_annotated")
        self.declare_parameter("detection_topic",  "/qut_rov/left/fish_detections")
        self.declare_parameter("confidence",       0.25)
        self.declare_parameter("image_size",       320)
        self.declare_parameter("device",           "0")
        self.declare_parameter("infer_every_n",    3)

        model_path        = self.get_parameter("model_path").get_parameter_value().string_value
        input_topic       = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic      = self.get_parameter("output_topic").get_parameter_value().string_value
        self.det_topic    = self.get_parameter("detection_topic").get_parameter_value().string_value
        self.confidence   = self.get_parameter("confidence").get_parameter_value().double_value
        self.image_size   = self.get_parameter("image_size").get_parameter_value().integer_value
        self.device       = self.get_parameter("device").get_parameter_value().string_value
        self.infer_every_n = self.get_parameter("infer_every_n").get_parameter_value().integer_value

        if not Path(model_path).is_file():
            raise FileNotFoundError(f"Model not found: {model_path}")

        self.get_logger().info(f"Loading model: {model_path}")
        self.model = YOLO(model_path)
        self.model.fuse()
        self.get_logger().info(f"Model classes: {self.model.names}")

        self.bridge = CvBridge()
        self.lock   = Lock()

        self.pending_frame   = None
        self.pending_header  = None
        self.last_annotated  = None
        self.last_detection  = []   # [cx, cy, w, h, conf] or []
        self.frame_count     = 0

        self.annotated_publisher = self.create_publisher(
            Image, output_topic, qos_profile_sensor_data
        )
        # publishes [cx, cy, w, h, conf] — empty when no fish detected
        self.detection_publisher = self.create_publisher(
            Float64MultiArray, self.det_topic, qos_profile_sensor_data
        )

        self.create_subscription(
            Image, input_topic, self.camera_callback, qos_profile_sensor_data
        )

        self.infer_thread = Thread(target=self._inference_loop, daemon=True)
        self.infer_thread.start()

        self.get_logger().info(
            f"Listening on {input_topic}\n"
            f"Annotated  → {output_topic}\n"
            f"Detections → {self.det_topic}\n"
            f"imgsz={self.image_size}  conf={self.confidence}  every_n={self.infer_every_n}"
        )

    def camera_callback(self, message: Image) -> None:
        self.frame_count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Bridge error: {e}")
            return

        if self.frame_count % self.infer_every_n == 0:
            with self.lock:
                self.pending_frame  = frame
                self.pending_header = message.header

        with self.lock:
            out       = self.last_annotated
            detection = list(self.last_detection)

        # publish annotated image
        if out is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(out, encoding="bgr8")
                img_msg.header = message.header
                self.annotated_publisher.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Publish error: {e}")

        # publish detection data every frame so follower stays in sync
        det_msg = Float64MultiArray()
        det_msg.data = detection   # [] = no fish, [cx,cy,w,h,conf] = fish found
        self.detection_publisher.publish(det_msg)

    def _inference_loop(self) -> None:
        import time
        while rclpy.ok():
            with self.lock:
                frame  = self.pending_frame
                if frame is not None:
                    self.pending_frame = None

            if frame is None:
                time.sleep(0.005)
                continue

            try:
                results = self.model.predict(
                    source=frame,
                    conf=self.confidence,
                    imgsz=self.image_size,
                    device=self.device,
                    verbose=False,
                    half=True,
                )

                result    = results[0]
                annotated = result.plot()

                # pick highest-confidence detection
                if len(result.boxes) > 0:
                    box      = max(result.boxes, key=lambda b: float(b.conf))
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    detection = [
                        (x1 + x2) / 2.0,   # cx
                        (y1 + y2) / 2.0,   # cy
                        x2 - x1,            # w
                        y2 - y1,            # h
                        float(box.conf),    # confidence
                    ]
                    self.get_logger().debug(
                        f"Detection: cx={detection[0]:.1f} cy={detection[1]:.1f} "
                        f"w={detection[2]:.1f} h={detection[3]:.1f} conf={detection[4]:.2f}"
                    )
                else:
                    detection = []

                with self.lock:
                    self.last_annotated = annotated
                    self.last_detection = detection

            except Exception as e:
                self.get_logger().error(f"Inference error: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FishDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()