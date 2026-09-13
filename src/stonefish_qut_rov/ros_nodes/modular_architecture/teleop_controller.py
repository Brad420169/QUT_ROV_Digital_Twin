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

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, Joy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float64

from control_utils import (
    apply_deadzone,
    clamp,
    quaternion_to_yaw,
    trigger_to_command,
)
from pid_controller import PIDController
from command_watchdog import Freshness
from node_lifecycle import run_node
from viewer_manager import ViewerManager
from battery_status import report_battery
from rov_config import (
    ARM_BUTTON,
    BATTERY_BUTTON,
    BATTERY_TOPIC,
    ARM_TOGGLE_TOPIC,
    ARMED_STATE_TOPIC,
    AXIS_DPAD_Y,
    AXIS_LEFT_STICK_Y,
    AXIS_LEFT_TRIGGER,
    AXIS_RIGHT_STICK_X,
    AXIS_RIGHT_TRIGGER,
    CAMERA_BUTTON,
    CMD_VEL_TOPIC,
    DEADZONE,
    DPAD_PRESS_LEVEL,
    DPAD_SHUTDOWN_HOLD_S,
    DEPTH_FEEDFORWARD,
    DEPTH_INTEGRAL_LIMIT,
    DEPTH_KD,
    DEPTH_KI,
    DEPTH_KP,
    DEPTH_TOPIC,
    FISH_FOLLOW_BUTTON,
    FISH_FOLLOW_ENABLE_TOPIC,
    FISH_FOLLOW_CMD_TOPIC,
    IMU_TOPIC,
    JOY_TOPIC,
    JOY_TIMEOUT_S,
    SENSOR_TIMEOUT_S,
    FISH_COMMAND_TIMEOUT_S,
    REAL_SCALE_DEPTH_FF,
    REAL_SCALE_DEPTH_KD,
    REAL_SCALE_DEPTH_KI,
    REAL_SCALE_DEPTH_KP,
    REAL_SCALE_MANUAL_HEAVE,
    REAL_SCALE_MANUAL_SURGE,
    REAL_SCALE_MANUAL_YAW,
    REAL_SCALE_TRAJ_FORWARD,
    REAL_SCALE_TRAJ_YAW,
    REAL_SCALE_YAW_KD,
    REAL_SCALE_YAW_KI,
    REAL_SCALE_YAW_KP,
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

        if self.rov_mode not in ("sim", "real"):
            raise ValueError("rov_mode must be sim or real")
        self.inputs = {
            "joy": Freshness(JOY_TIMEOUT_S),
            "depth": Freshness(SENSOR_TIMEOUT_S),
            "imu": Freshness(SENSOR_TIMEOUT_S),
            "fish": Freshness(FISH_COMMAND_TIMEOUT_S),
        }
        self._manual_rearm = True
        self._input_fault = None
        self._stopped = False

        # SIM-TO-REAL SCALING
        # Sim constants are canonical; real mode multiplies them by the
        # transfer scalers in rov_config.py. Resolved once here so no
        # control path below has to branch on mode.
        self._resolve_scaling()

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
        self._plot_pressed_last = False
        self._arm_pressed_last = False
        self._battery_pressed_last = False

        # D-pad down must be HELD to shut down; this is when the current
        # hold started (None = not held).
        self._shutdown_hold_start = None
        self._shutdown_requested = False


        # ARMING (real mode only)
        # teleop only ever asks for a toggle; real_interface owns the
        # actual state and reports it back on ARMED_STATE_TOPIC.
        self.arm_toggle_pub = self.create_publisher(
            Bool,
            ARM_TOGGLE_TOPIC,
            10,
        )

        self.create_subscription(
            Bool,
            ARMED_STATE_TOPIC,
            self.armed_state_callback,
            10,
        )

        # BATTERY (real mode only)
        self.battery = None

        self.create_subscription(
            BatteryState,
            BATTERY_TOPIC,
            self.battery_callback,
            10,
        )

        # PID
        self.depth_pid = PIDController(
            kp=self.k_depth_kp,
            ki=self.k_depth_ki,
            kd=self.k_depth_kd,
            integral_limit=DEPTH_INTEGRAL_LIMIT,
            output_min=-1.0,
            output_max=1.0,
            wrap_angle=False,
        )

        self.yaw_pid = PIDController(
            kp=self.k_yaw_kp,
            ki=self.k_yaw_ki,
            kd=self.k_yaw_kd,
            integral_limit=YAW_INTEGRAL_LIMIT,
            output_min=-self.k_traj_max_yaw,
            output_max=self.k_traj_max_yaw,
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

        self.viewers = ViewerManager(self, self.package_name, self.rov_mode)
        if self.rov_mode == "real":
            self.viewers.start_camera()

    # SENSOR CALLBACKS
    def depth_callback(self, msg: Float64):
        if not math.isfinite(msg.data):
            self.inputs["depth"].invalidate()
            return
        if (self.depth_keeping or self.trajectory_mode) and not self.inputs["depth"].fresh():
            self._cancel_for_input_loss("depth")
        self.inputs["depth"].touch()
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
            -pid_output + self.k_depth_ff
        )

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values) or sum(v*v for v in values) < 1e-12 or msg.orientation_covariance[0] == -1:
            self.inputs["imu"].invalidate()
            return
        if self.trajectory_mode and not self.inputs["imu"].fresh():
            self._cancel_for_input_loss("imu")
        self.inputs["imu"].touch()

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

        if not all(math.isfinite(v) for v in (msg.linear.x, msg.linear.z, msg.angular.z)):
            self.inputs["fish"].invalidate()
            return
        if not self.inputs["fish"].fresh():
            self._cancel_for_input_loss("fish")
            return
        self.inputs["fish"].touch()
        self.fish_surge_cmd = clamp(float(msg.linear.x))
        self.fish_heave_cmd = clamp(float(msg.linear.z))
        self.fish_yaw_cmd = clamp(float(msg.angular.z))

    # GAMEPAD
    def joy_callback(self, msg: Joy):
        axes = msg.axes
        buttons = msg.buttons

        max_axis = max(AXIS_LEFT_STICK_Y, AXIS_RIGHT_STICK_X,
                       AXIS_LEFT_TRIGGER, AXIS_RIGHT_TRIGGER)
        if len(axes) <= max_axis or not all(math.isfinite(v) for v in axes):
            self.inputs["joy"].invalidate()
            return
        # Detect a gap even when this callback runs before the output timer.
        if not self.inputs["joy"].fresh():
            self._cancel_for_input_loss("joy")
        self.inputs["joy"].touch()
        if self._manual_rearm:
            neutral = (abs(axes[AXIS_LEFT_STICK_Y]) <= DEADZONE
                       and abs(axes[AXIS_RIGHT_STICK_X]) <= DEADZONE
                       and trigger_to_command(axes[AXIS_LEFT_TRIGGER]) == 0
                       and trigger_to_command(axes[AXIS_RIGHT_TRIGGER]) == 0
                       and not any(buttons)
                       and (len(axes) <= AXIS_DPAD_Y or abs(axes[AXIS_DPAD_Y]) < DPAD_PRESS_LEVEL))
            if neutral:
                self._manual_rearm = False
                self._input_fault = None
                self.get_logger().info("Controls neutral; manual control ready.")
            return

        self._handle_buttons(buttons)
        self._handle_dpad(axes)
        if self.fish_follow_mode or self._shutdown_requested:
            return

        if self.trajectory_mode:
            self.surge_cmd = self.k_traj_forward
            return

        self.surge_cmd = (
            apply_deadzone(
                axes[AXIS_LEFT_STICK_Y],
                DEADZONE,
            )
            * self.scale_manual_surge
        )

        self.yaw_cmd = (
            apply_deadzone(
                axes[AXIS_RIGHT_STICK_X],
                DEADZONE,
            )
            * YAW_MANUAL_SCALE
            * self.scale_manual_yaw
        )

        if not self.depth_keeping:
            ascend = trigger_to_command(
                axes[AXIS_RIGHT_TRIGGER]
            )

            descend = trigger_to_command(
                axes[AXIS_LEFT_TRIGGER]
            )

            self.heave_cmd = clamp(
                (ascend - descend) * self.scale_manual_heave
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

        arm_pressed = (
            len(buttons) > ARM_BUTTON
            and buttons[ARM_BUTTON] == 1
        )

        battery_pressed = (
            len(buttons) > BATTERY_BUTTON
            and buttons[BATTERY_BUTTON] == 1
        )

        if station_pressed and not self._station_pressed_last:
            self.toggle_depth_keeping()

        if trajectory_pressed and not self._trajectory_pressed_last:
            self.toggle_trajectory()

        if camera_pressed and not self._camera_pressed_last:
            self.toggle_camera()

        if fish_pressed and not self._fish_pressed_last:
            self.toggle_fish_follow()

        if arm_pressed and not self._arm_pressed_last:
            self.toggle_arming()

        if battery_pressed and not self._battery_pressed_last:
            self.report_battery()

        self._station_pressed_last = station_pressed
        self._trajectory_pressed_last = trajectory_pressed
        self._camera_pressed_last = camera_pressed
        self._fish_pressed_last = fish_pressed
        self._arm_pressed_last = arm_pressed
        self._battery_pressed_last = battery_pressed

    def _handle_dpad(self, axes):
        """
        The D-pad is reported as an axis on this controller
        (axis 7: up = +1.0, down = -1.0, neutral = 0.0).

        Up   — rising edge toggles the depth/IMU plotter.
        Down — must be HELD for DPAD_SHUTDOWN_HOLD_S to shut the stack
               down, so a stray thumb cannot kill teleop mid-dive.
        """
        if len(axes) <= AXIS_DPAD_Y:
            return

        value = axes[AXIS_DPAD_Y]

        plot_pressed = value > DPAD_PRESS_LEVEL
        stop_held    = value < -DPAD_PRESS_LEVEL

        if plot_pressed and not self._plot_pressed_last:
            self.toggle_plotter()

        self._plot_pressed_last = plot_pressed

        self._handle_shutdown_hold(stop_held)

    def _handle_shutdown_hold(self, stop_held: bool):
        """Track how long D-pad down has been held, and act at the limit."""
        if self._shutdown_requested:
            return

        now = self.get_clock().now()

        if not stop_held:
            if self._shutdown_hold_start is not None:
                self.get_logger().info("Shutdown cancelled.")

            self._shutdown_hold_start = None
            return

        if self._shutdown_hold_start is None:
            self._shutdown_hold_start = now
            self.get_logger().warn(
                f"Hold D-pad down for {DPAD_SHUTDOWN_HOLD_S:.0f} s "
                "to shut down..."
            )
            return

        held = (
            now - self._shutdown_hold_start
        ).nanoseconds * 1e-9

        if held >= DPAD_SHUTDOWN_HOLD_S:
            self._shutdown_requested = True
            self.request_shutdown()

    def request_shutdown(self):
        """
        Shut the stack down from the gamepad, exactly as Ctrl+C does.

        The lifecycle loop exits after this callback and publishes neutral
        before closing viewers. The supervisor then stops the remaining nodes.
        """
        self.get_logger().warn("D-pad down held — shutting down.")

        self._shutdown_requested = True

    def toggle_plotter(self):
        self.viewers.toggle_plotter()

    # BATTERY
    def battery_callback(self, msg: BatteryState):
        self.battery = msg

    def report_battery(self):
        report_battery(self, self.battery, self.rov_mode)

    # ARMING
    def toggle_arming(self):
        """
        Ask real_interface to flip the arm state. Nothing is assumed here
        about what that state currently is — the interface node holds the
        truth and reports the result back.
        """
        if self.rov_mode == "sim":
            self.get_logger().info("Arming is real-mode only.")
            return

        self.arm_toggle_pub.publish(Bool())

    def armed_state_callback(self, msg: Bool):
        """Print the vehicle's armed state as reported by the FCU."""
        if msg.data:
            self.get_logger().warn("ARMED — thrusters are live.")
        else:
            self.get_logger().info("DISARMED — thrusters inactive.")

    # SIM-TO-REAL SCALING
    def _scaled(self, value, real_scale):
        """
        Sim values are canonical. In real mode they are multiplied by the
        matching transfer scaler from rov_config.py.
        """
        if self.rov_mode == "sim":
            return value

        return value * real_scale

    def _resolve_scaling(self):
        """Resolve every scaled constant once, at startup."""
        is_sim = self.rov_mode == "sim"

        # Manual stick scaling is applied at the command, not the gain.
        self.scale_manual_surge = 1.0 if is_sim else REAL_SCALE_MANUAL_SURGE
        self.scale_manual_yaw   = 1.0 if is_sim else REAL_SCALE_MANUAL_YAW
        self.scale_manual_heave = 1.0 if is_sim else REAL_SCALE_MANUAL_HEAVE

        # Trajectory mode
        self.k_traj_forward = self._scaled(
            TRAJECTORY_FORWARD, REAL_SCALE_TRAJ_FORWARD
        )
        self.k_traj_max_yaw = self._scaled(
            TRAJECTORY_MAX_YAW, REAL_SCALE_TRAJ_YAW
        )

        # Depth hold
        self.k_depth_kp = self._scaled(DEPTH_KP, REAL_SCALE_DEPTH_KP)
        self.k_depth_ki = self._scaled(DEPTH_KI, REAL_SCALE_DEPTH_KI)
        self.k_depth_kd = self._scaled(DEPTH_KD, REAL_SCALE_DEPTH_KD)
        self.k_depth_ff = self._scaled(
            DEPTH_FEEDFORWARD, REAL_SCALE_DEPTH_FF
        )

        # Heading hold
        self.k_yaw_kp = self._scaled(YAW_KP, REAL_SCALE_YAW_KP)
        self.k_yaw_ki = self._scaled(YAW_KI, REAL_SCALE_YAW_KI)
        self.k_yaw_kd = self._scaled(YAW_KD, REAL_SCALE_YAW_KD)

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
            if self.current_depth is None or not self.inputs["depth"].fresh():
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
            if self.current_depth is None or not self.inputs["depth"].fresh():
                self.get_logger().warn(
                    "Trajectory NOT enabled: no depth data yet."
                )
                return

            if self.current_yaw is None or not self.inputs["imu"].fresh():
                self.get_logger().warn(
                    "Trajectory NOT enabled: no IMU data yet."
                )
                return

            self.trajectory_mode = True
            self.depth_keeping = False

            self.target_depth = self.current_depth
            self.target_yaw = self.current_yaw

            self.surge_cmd = self.k_traj_forward
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

    def toggle_camera(self):
        if self.rov_mode == "sim" and self.viewers.running(self.viewers.camera):
            self._set_fish_follow(False)
        self.viewers.toggle_camera()

    # FISH FOLLOW
    def toggle_fish_follow(self):
        if self.rov_mode != "sim":
            self.get_logger().warn(
                "Fish following is disabled in REAL mode."
            )
            return

        detector_running = (
            self.viewers.running(self.viewers.camera)
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
            self.inputs["fish"].touch()  # Allow one timeout interval for the first result.
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
        if rclpy.ok():
            self.follow_enable_pub.publish(enable_msg)

        if not enabled:
            self.fish_surge_cmd = 0.0
            self.fish_heave_cmd = 0.0
            self.fish_yaw_cmd = 0.0

        self.get_logger().info(
            f"Fish following {'ENABLED' if enabled else 'DISABLED'}"
        )

    def _cancel_for_input_loss(self, source):
        if self._input_fault != source:
            self.get_logger().warn(f"Missing/stale {source} input: neutral output; release controls to resume manually.")
        self._input_fault = source
        self._manual_rearm = True
        if self.fish_follow_mode:
            self._set_fish_follow(False)
        self.depth_keeping = self.trajectory_mode = False
        self.target_depth = self.target_yaw = None
        self.prev_depth_time = self.prev_yaw_time = None
        self.depth_pid.reset()
        self.yaw_pid.reset()
        self.surge_cmd = self.heave_cmd = self.yaw_cmd = 0.0
        self.fish_surge_cmd = self.fish_heave_cmd = self.fish_yaw_cmd = 0.0
        self._shutdown_hold_start = None
        for name in ("station", "trajectory", "camera", "fish", "plot", "arm", "battery"):
            setattr(self, f"_{name}_pressed_last", False)

    # OUTPUT
    def publish_command(self):
        required = ["joy"]
        if self.depth_keeping or self.trajectory_mode:
            required.append("depth")
        if self.trajectory_mode:
            required.append("imu")
        if self.fish_follow_mode:
            required.append("fish")
        for source in required:
            if not self.inputs[source].fresh():
                self._cancel_for_input_loss(source)
                break
        msg = Twist()
        if self._manual_rearm or self._stopped:
            self.command_pub.publish(msg)
            return

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
        if self._stopped:
            return
        self._stopped = True
        self.timer.cancel()
        if rclpy.ok():
            self._set_fish_follow(False)
            self.command_pub.publish(Twist())
        self.viewers.close()


def main(args=None):
    run_node(GamepadTeleop, args)


if __name__ == "__main__":
    main()
