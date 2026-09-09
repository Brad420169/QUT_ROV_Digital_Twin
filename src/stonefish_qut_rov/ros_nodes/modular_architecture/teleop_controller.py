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
import sys
import subprocess

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
from rov_config import (
    ARM_BUTTON,
    BATTERY_BUTTON,
    BATTERY_CELL_CURVE,
    BATTERY_CELLS,
    BATTERY_MIN_PLAUSIBLE_V,
    BATTERY_TOPIC,
    ARM_TOGGLE_TOPIC,
    ARMED_STATE_TOPIC,
    CAMERA_SHOW_TOPIC,
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
    FISH_FOLLOW_EXECUTABLE,
    FISH_FOLLOW_CMD_TOPIC,
    IMU_TOPIC,
    JOY_TOPIC,
    PLOTTER_EXECUTABLE,
    REAL_CAMERA_EXECUTABLE,
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

        # CAMERA/FISH PROCESS
        self._camera_process = None

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

        # REAL CAMERA WINDOW
        # The viewer process runs for the whole session; Y only toggles
        # window visibility over this topic, so there is no reconnect wait.
        self._camera_visible = False

        self.camera_show_pub = self.create_publisher(
            Bool,
            CAMERA_SHOW_TOPIC,
            10,
        )

        # PLOTTER PROCESS
        self._plot_process = None

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

        self._kill_leftover_viewers()

        if self.rov_mode != "sim":
            self.start_real_camera_viewer()

        #self._print_controls()

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
            -pid_output + self.k_depth_ff
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
        self._handle_dpad(axes)

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

        Thrusters are zeroed first, then SIGINT is raised on this process
        so rclpy.spin() unwinds through the normal shutdown path. The
        launcher's watchdog sees teleop exit and stops everything else.
        """
        self.get_logger().warn("D-pad down held — shutting down.")

        self.stop()

        os.kill(os.getpid(), signal.SIGINT)

    # PLOTTER
    def toggle_plotter(self):
        """
        Start/stop imu_depth_plotter.py. It subscribes to the common
        /qut_rov/depth and /qut_rov/imu topics, so it is valid in both
        sim and real mode.
        """
        running = (
            self._plot_process is not None
            and self._plot_process.poll() is None
        )

        if not running:
            self.get_logger().info("Opening depth/IMU plotter...")

            try:
                self._plot_process = subprocess.Popen(
                    [
                        "ros2",
                        "run",
                        self.package_name,
                        PLOTTER_EXECUTABLE,
                    ],
                    start_new_session=True,
                )

            except Exception as error:
                self._plot_process = None
                self.get_logger().error(
                    f"Failed to start plotter: {error}"
                )

        else:
            self.close_plotter()

    def close_plotter(self):
        if self._plot_process is None:
            return

        if self._plot_process.poll() is None:
            try:
                os.killpg(
                    os.getpgid(self._plot_process.pid),
                    signal.SIGTERM,
                )
                self._plot_process.wait(timeout=2.0)

            except subprocess.TimeoutExpired:
                os.killpg(
                    os.getpgid(self._plot_process.pid),
                    signal.SIGKILL,
                )
                self._plot_process.wait()

            except ProcessLookupError:
                pass

        self._plot_process = None
        self.get_logger().info("Plotter closed")

    # BATTERY
    def battery_callback(self, msg: BatteryState):
        self.battery = msg

    @staticmethod
    def _percent_from_voltage(voltage: float):
        """
        Estimate charge from resting pack voltage using a per-cell curve.

        Returns None when the voltage is too low to be a healthy 4S LiPo —
        a bench supply or a flat pack — rather than reporting a
        misleading 0%.
        """
        if voltage < BATTERY_MIN_PLAUSIBLE_V:
            return None

        cell = voltage / BATTERY_CELLS

        if cell <= BATTERY_CELL_CURVE[0][0]:
            return 0.0

        if cell >= BATTERY_CELL_CURVE[-1][0]:
            return 100.0

        for index in range(1, len(BATTERY_CELL_CURVE)):
            low_v, low_pct = BATTERY_CELL_CURVE[index - 1]
            high_v, high_pct = BATTERY_CELL_CURVE[index]

            if cell <= high_v:
                span = high_v - low_v
                fraction = 0.0 if span == 0 else (cell - low_v) / span
                return low_pct + fraction * (high_pct - low_pct)

        return 100.0

    def report_battery(self):
        """Print the pack state. B button."""
        if self.rov_mode == "sim":
            self.get_logger().info("Battery reporting is real-mode only.")
            return

        if self.battery is None:
            self.get_logger().warn(
                "No battery data yet — is MAVROS connected?"
            )
            return

        voltage = float(self.battery.voltage)
        current = float(self.battery.current)
        cell = voltage / BATTERY_CELLS

        percent = self._percent_from_voltage(voltage)

        if percent is None:
            level = "n/a (not a 4S pack?)"
        else:
            level = f"{percent:.0f}%"

        self.get_logger().info(
            f"Battery: {voltage:.2f} V  ({cell:.2f} V/cell)  "
            f"{level}   draw {abs(current):.1f} A"
        )

        if percent is not None and percent <= 20.0:
            self.get_logger().warn("Battery low — surface soon.")

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

    def _scaling_summary(self):
        """Scalers that differ from 1.0, for the startup banner."""
        if self.rov_mode == "sim":
            return []

        applied = [
            ("manual surge", REAL_SCALE_MANUAL_SURGE),
            ("manual yaw",   REAL_SCALE_MANUAL_YAW),
            ("manual heave", REAL_SCALE_MANUAL_HEAVE),
            ("traj forward", REAL_SCALE_TRAJ_FORWARD),
            ("traj yaw",     REAL_SCALE_TRAJ_YAW),
            ("depth Kp",     REAL_SCALE_DEPTH_KP),
            ("depth Ki",     REAL_SCALE_DEPTH_KI),
            ("depth Kd",     REAL_SCALE_DEPTH_KD),
            ("depth FF",     REAL_SCALE_DEPTH_FF),
            ("yaw Kp",       REAL_SCALE_YAW_KP),
            ("yaw Ki",       REAL_SCALE_YAW_KI),
            ("yaw Kd",       REAL_SCALE_YAW_KD),
        ]

        return [
            f"{name} x{scale:g}"
            for name, scale in applied
            if scale != 1.0
        ]

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

    # CAMERA / DETECTOR
    def _kill_leftover_viewers(self):
        """
        Kill viewers left over from a previous run. teleop can be
        SIGKILLed by the launcher, which skips the shutdown handler and
        leaves the camera viewer and plotter running — they then stack
        up across sessions, each holding its own RTSP connection.
        """
        for executable in (REAL_CAMERA_EXECUTABLE, PLOTTER_EXECUTABLE):
            try:
                subprocess.run(
                    ["pkill", "-f", executable],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception as error:
                self.get_logger().warn(
                    f"Could not kill leftover {executable}: {error}"
                )

    def start_real_camera_viewer(self):
        """
        Launch the RTSP viewer once, hidden. It holds the stream open for
        the whole session so the Y button is instant instead of paying
        connect + first-keyframe cost every time.
        """
        self.get_logger().info("Starting camera viewer (hidden)...")

        try:
            self._camera_process = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    self.package_name,
                    REAL_CAMERA_EXECUTABLE,
                ],
                start_new_session=True,
            )

        except Exception as error:
            self._camera_process = None
            self.get_logger().error(
                f"Failed to start camera viewer: {error}"
            )

    def toggle_camera(self):
        """
        SIM  — launches/kills the fish detector + camera viewer.
        REAL — shows/hides the already-running RTSP viewer window.
        """
        if self.rov_mode != "sim":
            # Viewer gone (crashed, or killed externally)? Bring it back.
            if (
                self._camera_process is None
                or self._camera_process.poll() is not None
            ):
                self.start_real_camera_viewer()

            self._camera_visible = not self._camera_visible

            msg = Bool()
            msg.data = self._camera_visible
            self.camera_show_pub.publish(msg)

            self.get_logger().info(
                "Camera shown" if self._camera_visible else "Camera hidden"
            )
            return

        running = (
            self._camera_process is not None
            and self._camera_process.poll() is None
        )

        if running:
            self.close_camera_viewer()
            return

        self.get_logger().info("Opening fish detector and camera viewer...")

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
                f"Failed to start fish detector: {error}"
            )

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

def main(args=None):
    rclpy.init(args=args)
    node = GamepadTeleop()

    def shutdown(signum, frame):
        node.stop()
        node.close_camera_viewer()
        node.close_plotter()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGHUP, shutdown)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        node.close_camera_viewer()
        node.close_plotter()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()