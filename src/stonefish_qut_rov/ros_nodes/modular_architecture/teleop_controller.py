#!/usr/bin/env python3
"""
High-level ROV controller.

Outputs only vehicle-level surge/heave/yaw on /qut_rov/cmd_vel.

SIM mode:
    Manual control
    Depth keeping
    Trajectory mode
    Camera/fish detector process
    Fish following

REAL mode:
    Manual/depth/trajectory remain available.
    Fish following is blocked.
"""

import math
import os
import signal
import subprocess

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Bool, Float64

from control_utils import (
    apply_deadzone,
    clamp,
    quaternion_to_yaw,
    trigger_to_command,
)
from pid_controller import PIDController
from rov_config import (
    AXIS_LEFT_STICK_Y,
    AXIS_LEFT_TRIGGER,
    AXIS_RIGHT_STICK_X,
    AXIS_RIGHT_TRIGGER,
    CAMERA_BUTTON,
    CMD_VEL_TOPIC,
    DEADZONE,
    DEPTH_FEEDFORWARD,
    DEPTH_INTEGRAL_LIMIT,
    DEPTH_KD,
    DEPTH_KI,
    DEPTH_KP,
    DEPTH_TOPIC,
    FISH_FOLLOW_BUTTON,
    FISH_FOLLOW_ENABLE_TOPIC,
    FISH_FOLLOW_EXECUTABLE,
    FISH_FOLLOW_CMD_TOPIC,
    IMU_TOPIC,
    JOY_TOPIC,
    STATION_KEEPING_BUTTON,
    TRAJECTORY_FORWARD,
    TRAJECTORY_MAX_YAW,
    TRAJECTORY_SET_BUTTON,
    YAW_INTEGRAL_LIMIT,
    YAW_KD,
    YAW_KI,
    YAW_KP,
    YAW_MANUAL_SCALE,
)


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__("gamepad_teleop")

        self.declare_parameter("rov_mode", "sim")
        self.declare_parameter("package_name", "stonefish_qut_rov")

        self.rov_mode = str(
            self.get_parameter("rov_mode").value
        ).strip().lower()

        self.package_name = str(
            self.get_parameter("package_name").value
        )

        # VEHICLE COMMAND
        self.surge_cmd = 0.0
        self.heave_cmd = 0.0
        self.yaw_cmd = 0.0

        # Most recent fish-follow command.
        self.fish_surge_cmd = 0.0
        self.fish_heave_cmd = 0.0
        self.fish_yaw_cmd = 0.0

        # STATE
        self.current_depth = None
        self.target_depth = None

        self.current_yaw = None
        self.target_yaw = None

        self.prev_depth_time = None
        self.prev_yaw_time = None

        self.depth_keeping = False
        self.trajectory_mode = False
        self.fish_follow_mode = False

        # BUTTON EDGES
        self._station_pressed_last = False
        self._trajectory_pressed_last = False
        self._camera_pressed_last = False
        self._fish_pressed_last = False

        # CAMERA/FISH PROCESS
        self._camera_process = None

        # PID
        self.depth_pid = PIDController(
            kp=DEPTH_KP,
            ki=DEPTH_KI,
            kd=DEPTH_KD,
            integral_limit=DEPTH_INTEGRAL_LIMIT,
            output_min=-1.0,
            output_max=1.0,
            wrap_angle=False,
        )

        self.yaw_pid = PIDController(
            kp=YAW_KP,
            ki=YAW_KI,
            kd=YAW_KD,
            integral_limit=YAW_INTEGRAL_LIMIT,
            output_min=-TRAJECTORY_MAX_YAW,
            output_max=TRAJECTORY_MAX_YAW,
            wrap_angle=True,
        )

        # ROS
        self.command_pub = self.create_publisher(
            Twist,
            CMD_VEL_TOPIC,
            10,
        )

        self.follow_enable_pub = self.create_publisher(
            Bool,
            FISH_FOLLOW_ENABLE_TOPIC,
            10,
        )

        self.joy_sub = self.create_subscription(
            Joy,
            JOY_TOPIC,
            self.joy_callback,
            10,
        )

        self.depth_sub = self.create_subscription(
            Float64,
            DEPTH_TOPIC,
            self.depth_callback,
            10,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            IMU_TOPIC,
            self.imu_callback,
            10,
        )

        # Fish follower publishes Twist here rather than direct thrusters.
        self.fish_cmd_sub = self.create_subscription(
            Twist,
            FISH_FOLLOW_CMD_TOPIC,
            self.fish_command_callback,
            10,
        )

        self.timer = self.create_timer(
            0.05,
            self.publish_command,
        )

        self._print_controls()

    # SENSOR CALLBACKS
    def depth_callback(self, msg: Float64):
        self.current_depth = float(msg.data)

        if self.fish_follow_mode:
            return

        if not (self.depth_keeping or self.trajectory_mode):
            self.prev_depth_time = self.get_clock().now()
            return

        if self.target_depth is None:
            return

        now = self.get_clock().now()

        if self.prev_depth_time is None:
            self.prev_depth_time = now
            return

        dt = (now - self.prev_depth_time).nanoseconds * 1e-9
        self.prev_depth_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        pid_output = self.depth_pid.compute(
            setpoint=self.target_depth,
            measurement=self.current_depth,
            dt=dt,
        )

        self.heave_cmd = clamp(
            -pid_output + DEPTH_FEEDFORWARD
        )

    def imu_callback(self, msg: Imu):
        q = msg.orientation

        self.current_yaw = quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w,
        )

        if self.fish_follow_mode:
            return

        if not self.trajectory_mode or self.target_yaw is None:
            self.prev_yaw_time = self.get_clock().now()
            return

        now = self.get_clock().now()

        if self.prev_yaw_time is None:
            self.prev_yaw_time = now
            return

        dt = (now - self.prev_yaw_time).nanoseconds * 1e-9
        self.prev_yaw_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        self.yaw_cmd = self.yaw_pid.compute(
            setpoint=self.target_yaw,
            measurement=self.current_yaw,
            dt=dt,
        )

    # FISH FOLLOW COMMAND
    def fish_command_callback(self, msg: Twist):
        if not self.fish_follow_mode:
            return

        self.fish_surge_cmd = clamp(float(msg.linear.x))
        self.fish_heave_cmd = clamp(float(msg.linear.z))
        self.fish_yaw_cmd = clamp(float(msg.angular.z))

    # GAMEPAD
    def joy_callback(self, msg: Joy):
        axes = msg.axes
        buttons = msg.buttons

        self._handle_buttons(buttons)

        if self.fish_follow_mode:
            return

        max_axis = max(
            AXIS_LEFT_STICK_Y,
            AXIS_RIGHT_STICK_X,
            AXIS_LEFT_TRIGGER,
            AXIS_RIGHT_TRIGGER,
        )

        if len(axes) <= max_axis:
            self.get_logger().warn(
                f"Only {len(axes)} joystick axes detected. "
                "Check rov_config.py."
            )
            return

        if self.trajectory_mode:
            self.surge_cmd = TRAJECTORY_FORWARD
            return

        self.surge_cmd = apply_deadzone(
            axes[AXIS_LEFT_STICK_Y],
            DEADZONE,
        )

        self.yaw_cmd = (
            apply_deadzone(
                axes[AXIS_RIGHT_STICK_X],
                DEADZONE,
            )
            * YAW_MANUAL_SCALE
        )

        if not self.depth_keeping:
            ascend = trigger_to_command(
                axes[AXIS_RIGHT_TRIGGER]
            )

            descend = trigger_to_command(
                axes[AXIS_LEFT_TRIGGER]
            )

            self.heave_cmd = clamp(
                ascend - descend
            )

    def _handle_buttons(self, buttons):
        station_pressed = (
            len(buttons) > STATION_KEEPING_BUTTON
            and buttons[STATION_KEEPING_BUTTON] == 1
        )

        trajectory_pressed = (
            len(buttons) > TRAJECTORY_SET_BUTTON
            and buttons[TRAJECTORY_SET_BUTTON] == 1
        )

        camera_pressed = (
            len(buttons) > CAMERA_BUTTON
            and buttons[CAMERA_BUTTON] == 1
        )

        fish_pressed = (
            len(buttons) > FISH_FOLLOW_BUTTON
            and buttons[FISH_FOLLOW_BUTTON] == 1
        )

        if station_pressed and not self._station_pressed_last:
            self.toggle_depth_keeping()

        if trajectory_pressed and not self._trajectory_pressed_last:
            self.toggle_trajectory()

        if camera_pressed and not self._camera_pressed_last:
            self.toggle_camera()

        if fish_pressed and not self._fish_pressed_last:
            self.toggle_fish_follow()

        self._station_pressed_last = station_pressed
        self._trajectory_pressed_last = trajectory_pressed
        self._camera_pressed_last = camera_pressed
        self._fish_pressed_last = fish_pressed

    # DEPTH KEEPING
    def toggle_depth_keeping(self):
        if self.fish_follow_mode:
            self.get_logger().warn(
                "Disable fish following first."
            )
            return

        if self.trajectory_mode:
            self.get_logger().warn(
                "Disable trajectory mode first."
            )
            return

        requested = not self.depth_keeping

        if requested:
            if self.current_depth is None:
                self.get_logger().warn(
                    "Depth keeping NOT enabled: no depth data yet."
                )
                return

            self.depth_keeping = True
            self.target_depth = self.current_depth
            self.prev_depth_time = None
            self.depth_pid.reset()

            self.get_logger().info(
                f"Depth keeping ENABLED | {self.target_depth:.2f} m"
            )

        else:
            self.depth_keeping = False
            self.target_depth = None
            self.heave_cmd = 0.0
            self.prev_depth_time = None
            self.depth_pid.reset()

            self.get_logger().info(
                "Depth keeping DISABLED"
            )

    # TRAJECTORY
    def toggle_trajectory(self):
        if self.fish_follow_mode:
            self.get_logger().warn(
                "Disable fish following first."
            )
            return

        requested = not self.trajectory_mode

        if requested:
            if self.current_depth is None:
                self.get_logger().warn(
                    "Trajectory NOT enabled: no depth data yet."
                )
                return

            if self.current_yaw is None:
                self.get_logger().warn(
                    "Trajectory NOT enabled: no IMU data yet."
                )
                return

            self.trajectory_mode = True
            self.depth_keeping = False

            self.target_depth = self.current_depth
            self.target_yaw = self.current_yaw

            self.surge_cmd = TRAJECTORY_FORWARD
            self.heave_cmd = 0.0
            self.yaw_cmd = 0.0

            self.prev_depth_time = None
            self.prev_yaw_time = None

            self.depth_pid.reset()
            self.yaw_pid.reset()

            self.get_logger().info(
                "Trajectory ENABLED | "
                f"depth {self.target_depth:.2f} m | "
                f"heading {math.degrees(self.target_yaw):.1f} deg"
            )

        else:
            self.trajectory_mode = False
            self.target_depth = None
            self.target_yaw = None

            self.surge_cmd = 0.0
            self.heave_cmd = 0.0
            self.yaw_cmd = 0.0

            self.prev_depth_time = None
            self.prev_yaw_time = None

            self.depth_pid.reset()
            self.yaw_pid.reset()

            self.get_logger().info(
                "Trajectory DISABLED"
            )

    # CAMERA / DETECTOR
    def toggle_camera(self):
        if self.rov_mode != "sim":
            self.get_logger().warn(
                "The current camera/fish detector launcher is SIM-only."
            )
            return

        running = (
            self._camera_process is not None
            and self._camera_process.poll() is None
        )

        if not running:
            self.get_logger().info(
                "Opening fish detector and camera viewer..."
            )

            try:
                self._camera_process = subprocess.Popen(
                    [
                        "ros2",
                        "run",
                        self.package_name,
                        FISH_FOLLOW_EXECUTABLE,
                    ],
                    start_new_session=True,
                )

            except Exception as error:
                self._camera_process = None
                self.get_logger().error(
                    f"Failed to start detector: {error}"
                )

        else:
            self.close_camera_viewer()

    def close_camera_viewer(self):
        if self.fish_follow_mode:
            self._set_fish_follow(False)

        if self._camera_process is None:
            return

        if self._camera_process.poll() is None:
            try:
                os.killpg(
                    os.getpgid(self._camera_process.pid),
                    signal.SIGTERM,
                )
                self._camera_process.wait(timeout=2.0)

            except subprocess.TimeoutExpired:
                os.killpg(
                    os.getpgid(self._camera_process.pid),
                    signal.SIGKILL,
                )
                self._camera_process.wait()

            except ProcessLookupError:
                pass

        self._camera_process = None
        self.get_logger().info(
            "Fish detector/camera closed"
        )

    # FISH FOLLOW
    def toggle_fish_follow(self):
        if self.rov_mode != "sim":
            self.get_logger().warn(
                "Fish following is disabled in REAL mode."
            )
            return

        detector_running = (
            self._camera_process is not None
            and self._camera_process.poll() is None
        )

        if not detector_running:
            self.get_logger().warn(
                "Fish following requires the detector. "
                "Press the camera button first."
            )
            return

        self._set_fish_follow(
            not self.fish_follow_mode
        )

    def _set_fish_follow(self, enabled: bool):
        if enabled:
            self.depth_keeping = False
            self.trajectory_mode = False

            self.depth_pid.reset()
            self.yaw_pid.reset()

            self.surge_cmd = 0.0
            self.heave_cmd = 0.0
            self.yaw_cmd = 0.0

            self.fish_surge_cmd = 0.0
            self.fish_heave_cmd = 0.0
            self.fish_yaw_cmd = 0.0

        self.fish_follow_mode = enabled

        enable_msg = Bool()
        enable_msg.data = enabled
        self.follow_enable_pub.publish(enable_msg)

        if not enabled:
            self.fish_surge_cmd = 0.0
            self.fish_heave_cmd = 0.0
            self.fish_yaw_cmd = 0.0

        self.get_logger().info(
            f"Fish following {'ENABLED' if enabled else 'DISABLED'}"
        )

    # OUTPUT
    def publish_command(self):
        msg = Twist()

        if self.fish_follow_mode:
            msg.linear.x = float(self.fish_surge_cmd)
            msg.linear.z = float(self.fish_heave_cmd)
            msg.angular.z = float(self.fish_yaw_cmd)

        else:
            msg.linear.x = float(clamp(self.surge_cmd))
            msg.linear.z = float(clamp(self.heave_cmd))
            msg.angular.z = float(clamp(self.yaw_cmd))

        self.command_pub.publish(msg)

    def stop(self):
        self._set_fish_follow(False)

        self.surge_cmd = 0.0
        self.heave_cmd = 0.0
        self.yaw_cmd = 0.0

        self.publish_command()

    # DISPLAY
    def _print_controls(self):
        fish_text = (
            "available"
            if self.rov_mode == "sim"
            else "disabled"
        )

        lines = [
            "",
            "╔══════════════════════════════════════════════╗",
            "║           SubbyROV Gamepad Teleop            ║",
            "╠══════════════════════════════════════════════╣",
            "║ Left stick ↑↓     Surge                      ║",
            "║ Right stick ←→    Yaw                        ║",
            "║ RT / LT           Heave up / down            ║",
            "║ L bumper          Depth keeping              ║",
            "║ R bumper          Trajectory mode            ║",
            "║ Y button          Camera/detector             ║",
            "║ X button          Fish follow                ║",
            "╠══════════════════════════════════════════════╣",
            f"║ Mode: {self.rov_mode.upper():<39}║",
            f"║ Fish follow: {fish_text:<32}║",
            "╚══════════════════════════════════════════════╝",
            "",
        ]

        for line in lines:
            self.get_logger().info(line)


def main(args=None):
    rclpy.init(args=args)
    node = GamepadTeleop()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        node.close_camera_viewer()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
