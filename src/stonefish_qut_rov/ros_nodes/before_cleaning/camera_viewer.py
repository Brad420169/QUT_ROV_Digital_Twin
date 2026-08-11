#!/usr/bin/env python3
import time
from collections import deque

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraViewer(Node):

    def __init__(self):
        super().__init__("camera_viewer")
        self.bridge = CvBridge()
        self.win_name = "ROV Left Camera"
        self.frame_times = deque(maxlen=30)

        self.create_subscription(
            Image,
            "/qut_rov/left/fish_annotated",
            self.callback,
            qos_profile_sensor_data,
        )
        # self.create_subscription(
        #     Image,
        #     "/qut_rov/left/image_color",
        #     self.callback,
        #     qos_profile_sensor_data,
        # )

        self.get_logger().info("Subscribed to /qut_rov/left/fish_annotated")
        # self.get_logger().info("Subscribed to /qut_rov/left/image_color")

    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Rolling average FPS
            now = time.time()
            self.frame_times.append(now)
            if len(self.frame_times) >= 2:
                fps = (len(self.frame_times) - 1) / (self.frame_times[-1] - self.frame_times[0])
            else:
                fps = 0.0

            cv2.putText(
                img,
                f"FPS: {fps:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
            )

            cv2.imshow(self.win_name, img)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                self.get_logger().info("Exit key pressed. Closing viewer...")
                rclpy.shutdown()

            if cv2.getWindowProperty(self.win_name, cv2.WND_PROP_VISIBLE) < 1:
                self.get_logger().info("Window closed. Exiting...")
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main():
    rclpy.init()
    node = CameraViewer()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()