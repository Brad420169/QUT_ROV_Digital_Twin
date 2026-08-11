#!/usr/bin/env python3
"""
GameSir Gamepad Teleop for SubbyROV — with Depth Keeping and Trajectory Mode
-----------------------------------------------------------------------------
Requires: ros2 run joy joy_node

Thruster array: [BL, BR, TL, TR]
  index 0 = BL  (vertical left)
  index 1 = BR  (vertical right)
  index 2 = TL  (horizontal left)
  index 3 = TR  (horizontal right)

Controls:
  Left stick  UP/DOWN    → Forward / Backward      (TL, TR)
  Right stick LEFT/RIGHT → Turn (half power)        (TL, TR differential)
  Hold RT                → Ascend                  (BL, BR)
  Hold LT                → Descend                 (BL, BR)
  Button[6] (L bumper)   → Toggle depth keeping ON/OFF
  Button[7] (R bumper)   → Toggle trajectory mode ON/OFF
                           Locks current heading + depth, drives forward
"""

import os
import sys
import math
import subprocess
import signal
sys.path.insert(0, os.path.dirname(__file__))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import Joy, FluidPressure, Imu

from pid_controller import PIDController


# --- Axis indices ---
AXIS_LEFT_STICK_Y  = 1
AXIS_RIGHT_STICK_X = 2
AXIS_LEFT_TRIGGER  = 5   # Descend
AXIS_RIGHT_TRIGGER = 4   # Ascend

# --- Button indices ---
STATION_KEEPING_BUTTON = 6   # LEFT BUTTON ABOVE LT
TRAJECTORY_SET_BUTTON  = 7   # RIGHT BUTTON ABOVE RT
CAMERA_BUTTON          = 4   # The Y button on the GameSir
FISH_FOLLOW_BUTTON     = 3   # The X button on the GameSir 

# --- Deadzone ---
DEADZONE = 0.08

# --- Depth keeping PID ---
DEPTH_KP             = 200.0
DEPTH_KI             = 5.0
DEPTH_KD             = 80.0
DEPTH_INTEGRAL_LIMIT = 400.0
DEPTH_FEEDFORWARD    = -160.0   # PWM to hover at neutral buoyancy

# --- Pitch feedforward ---
# Applied to vertical thrusters (BL/BR) to counteract pitch caused by horizontal thrust
# Negative = nose-up correction when going forward, flips sign when going backward
PITCH_FEEDFORWARD = 200.0

# --- Yaw keeping PID ---
YAW_KP             = 0.0
YAW_KI             = 0.0
YAW_KD             = 0.0
YAW_INTEGRAL_LIMIT = 150.0

# --- Trajectory forward speed ---
TRAJECTORY_FORWARD_PWM = 300.0

MAX_PWM       = 600.0
WATER_DENSITY = 1031.0
GRAVITY       = 9.81

def apply_deadzone(value, deadzone):
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def trigger_to_thrust(raw):
    """Triggers rest at +1.0, fully pressed = -1.0. Returns 0.0–1.0."""
    if raw > 0.0:
        return 0.0
    return abs(raw)


def wrap_to_pi(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def clamp(value, limit):
    return max(-limit, min(limit, value))


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__('gamepad_teleop')

        self.max_thrust = MAX_PWM

        # [TL, TR, BL, BR]
        self.thrust = [0.0, 0.0, 0.0, 0.0]

        # --- Depth keeping state ---
        self.depth_keeping    = False
        self.target_depth     = None
        self.current_depth    = None
        self.prev_stamp       = None
        self._log_counter     = 0

        # --- Trajectory mode state ---
        self.trajectory_mode  = False
        self.target_yaw       = None
        self.current_yaw      = None

        # --- Store forward value for pressure_callback to access ---
        self.current_forward  = 0.0

        # --- Button edge detection ---
        self._sk_pressed_last   = False
        self._traj_pressed_last = False
        self._follow_pressed_last = False

        # --- PIDs ---
        self.depth_pid = PIDController(
            kp=DEPTH_KP, ki=DEPTH_KI, kd=DEPTH_KD,
            integral_limit=DEPTH_INTEGRAL_LIMIT,
            output_min=-MAX_PWM, output_max=MAX_PWM,
            wrap_angle=False,
        )
        self.yaw_pid = PIDController(
            kp=YAW_KP, ki=YAW_KI, kd=YAW_KD,
            integral_limit=YAW_INTEGRAL_LIMIT,
            output_min=-MAX_PWM, output_max=MAX_PWM,
            wrap_angle=True,
        )

        # --- Publishers / Subscribers ---
        self.pub = self.create_publisher(
            Float64MultiArray, '/qut_rov/setpoint/thrusters', 10
        )
        self.follow_enable_pub = self.create_publisher(
            Bool, '/qut_rov/fish_follow_enabled', 10
        )
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )
        self.pressure_sub = self.create_subscription(
            FluidPressure, '/qut_rov/pressure', self.pressure_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/qut_rov/imu', self.imu_callback, 10
        )

        # --- Camera viewer state ---
        self._cam_pressed_last = False
        self._camera_process = None
        self.fish_follow_mode = False

        self.timer = self.create_timer(0.05, self.publish_thrust)

        self._print_controls()
        self.ensure_joy_node()


    def ensure_joy_node(self):
        """Start joy_node unless one is already running."""

        result = subprocess.run(
            ["pgrep", "-f", "ros2 run joy joy_node"],
            capture_output=True,
            text=True,
            check=False,
        )

        if result.returncode != 0:
            self.get_logger().info("Starting joy_node...")

            try:
                self._joy_process = subprocess.Popen([
                    "ros2",
                    "run",
                    "joy",
                    "joy_node",
                ])
            except Exception as error:
                self._joy_process = None
                self.get_logger().error(
                    f"Failed to start joy_node: {error}"
                )
        else:
            self._joy_process = None
            self.get_logger().info("joy_node is already running.")

    def _print_controls(self):
        lines = [
            "",
            "╔══════════════════════════════════════════════╗",
            "║           SubbyROV Gamepad Teleop            ║",
            "╠══════════════════════════════════════════════╣",
            "║  MOVEMENT                                    ║",
            "║   Left stick  ↑↓   Forward / Backward        ║",
            "║   Right stick ←→   Turn (half power)         ║",
            "║   Hold RT          Ascend                    ║",
            "║   Hold LT          Descend                   ║",
            "╠══════════════════════════════════════════════╣",
            "║  MODES                                       ║",
            "║   L Bumper [6]     Toggle depth keeping      ║",
            "║   R Bumper [7]     Toggle trajectory mode    ║",
            "╠══════════════════════════════════════════════╣",
            "║  THRUSTERS  [BL, BR, TL, TR]                 ║",
            "║   BL / BR          Vertical (depth)          ║",
            "║   TL / TR          Horizontal (drive + turn) ║",
            "║   Y Button [4]     Toggle camera viewer      ║",
            "║   X Button [3]     Track detected fish       ║",
            "╚══════════════════════════════════════════════╝",
            "",
        ]
        for line in lines:
            self.get_logger().info(line)

    # ------------------------------------------------------------------
    # IMU callback — track yaw
    # ------------------------------------------------------------------

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ------------------------------------------------------------------
    # Pressure callback — depth PID (used by both modes)
    # ------------------------------------------------------------------

    def pressure_callback(self, msg: FluidPressure):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.current_depth = msg.fluid_pressure / (WATER_DENSITY * GRAVITY)

        if not self.depth_keeping and not self.trajectory_mode:
            self.prev_stamp = stamp
            return

        if self.prev_stamp is None:
            self.prev_stamp = stamp
            return

        dt = stamp - self.prev_stamp
        self.prev_stamp = stamp

        if dt <= 0.0 or dt > 1.0:
            return

        pid_out   = self.depth_pid.compute(
            setpoint=self.target_depth,
            measurement=self.current_depth,
            dt=dt,
        )
        heave_cmd = clamp(-pid_out + DEPTH_FEEDFORWARD, self.max_thrust)

        # Vertical thrusters: BL=index0, BR=index1
        self.thrust[0] = heave_cmd
        self.thrust[1] = heave_cmd

        # Log at ~1 Hz
        self._log_counter += 1
        if self._log_counter % 50 == 0:
            err  = self.target_depth - self.current_depth
            mode = "TRAJ+DEPTH" if self.trajectory_mode else "DEPTH"
            self.get_logger().info(
                f"[{mode}] depth {self.current_depth:5.2f} m | "
                f"target {self.target_depth:.2f} m | "
                f"err {err:+5.2f} m | "
                f"PWM {heave_cmd:+7.1f}"
            )

    # ------------------------------------------------------------------
    # Camera viewer
    # ------------------------------------------------------------------

    def toggle_camera(self):
        """Open/close the fish detector and annotated camera viewer."""

        running = (
            self._camera_process is not None
            and self._camera_process.poll() is None
        )

        if not running:
            self.get_logger().info("Opening fish detector and camera viewer...")

            try:
                self._camera_process = subprocess.Popen(
                    [
                        "ros2",
                        "run",
                        "stonefish_qut_rov",
                        "fish_detector_follower.py",
                    ],
                    start_new_session=True,
                )
            except Exception as e:
                self.get_logger().error(f"Failed to start camera viewer: {e}")
                self._camera_process = None

        else:
            self.get_logger().info("Closing fish detector and camera viewer...")
            self.close_camera_viewer()


    def close_camera_viewer(self):
        """Close the camera viewer and all child processes."""

        if self._camera_process is None:
            return

        if self.fish_follow_mode:
            self.fish_follow_mode = False
            enable_msg = Bool()
            enable_msg.data = False
            self.follow_enable_pub.publish(enable_msg)
            self.thrust = [0.0, 0.0, 0.0, 0.0]
            self.publish_thrust()

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
        self.get_logger().info("Fish detector and camera viewer closed")

    def toggle_fish_follow(self):
        """Toggle autonomous fish-follow mode using Button[3]."""

        detector_running = (
            self._camera_process is not None
            and self._camera_process.poll() is None
        )

        if not detector_running:
            self.get_logger().warn(
                "Fish following requires the detector. Press Button[4] first."
            )
            return

        requested_mode = not self.fish_follow_mode

        # Disable other automatic modes so only one controller has authority.
        if requested_mode:
            self.depth_keeping = False
            self.trajectory_mode = False
            self.depth_pid.reset()
            self.yaw_pid.reset()
            self.thrust = [0.0, 0.0, 0.0, 0.0]

            # Publish one final zero command before handing control to the follower.
            zero_msg = Float64MultiArray()
            zero_msg.data = [0.0, 0.0, 0.0, 0.0]
            self.pub.publish(zero_msg)

        self.fish_follow_mode = requested_mode

        enable_msg = Bool()
        enable_msg.data = self.fish_follow_mode
        self.follow_enable_pub.publish(enable_msg)

        self.get_logger().info(
            f"Fish following {'ENABLED' if self.fish_follow_mode else 'DISABLED'}"
        )

        if not self.fish_follow_mode:
            # Resume manual control from zero to avoid carrying old commands.
            self.thrust = [0.0, 0.0, 0.0, 0.0]
            self.publish_thrust()


    # ------------------------------------------------------------------
    # Joystick callback
    # ------------------------------------------------------------------

    def joy_callback(self, msg: Joy):
        axes    = msg.axes
        buttons = msg.buttons

        # Button edge detection
        sk_pressed   = len(buttons) > STATION_KEEPING_BUTTON and buttons[STATION_KEEPING_BUTTON] == 1
        traj_pressed = len(buttons) > TRAJECTORY_SET_BUTTON  and buttons[TRAJECTORY_SET_BUTTON]  == 1
        cam_pressed    = len(buttons) > CAMERA_BUTTON      and buttons[CAMERA_BUTTON]      == 1
        follow_pressed = len(buttons) > FISH_FOLLOW_BUTTON and buttons[FISH_FOLLOW_BUTTON] == 1

        if sk_pressed and not self._sk_pressed_last:
            self.toggle_depth_keeping()
        if traj_pressed and not self._traj_pressed_last:
            self.toggle_trajectory()
        if cam_pressed and not self._cam_pressed_last:
            self.toggle_camera()
        if follow_pressed and not self._follow_pressed_last:
            self.toggle_fish_follow()

        self._sk_pressed_last     = sk_pressed
        self._traj_pressed_last   = traj_pressed
        self._cam_pressed_last    = cam_pressed
        self._follow_pressed_last = follow_pressed

        if self.fish_follow_mode:
            return

        max_needed = max(AXIS_LEFT_STICK_Y, AXIS_RIGHT_STICK_X,
                         AXIS_LEFT_TRIGGER, AXIS_RIGHT_TRIGGER)
        if len(axes) <= max_needed:
            self.get_logger().warn(
                f"Only {len(axes)} axes detected. Update AXIS_* constants."
            )
            return

        forward = apply_deadzone(axes[AXIS_LEFT_STICK_Y],  DEADZONE)
        turn    = apply_deadzone(axes[AXIS_RIGHT_STICK_X], DEADZONE) * 0.5

        # Store forward for pitch feedforward access
        self.current_forward = forward

        # -------------------------------------------------------
        # TRAJECTORY MODE — fixed forward + yaw hold
        # -------------------------------------------------------
        if self.trajectory_mode:
            yaw_correction = 0.0
            if self.current_yaw is not None and self.target_yaw is not None:
                yaw_err        = wrap_to_pi(self.target_yaw - self.current_yaw)
                yaw_correction = clamp((YAW_KP * yaw_err) / self.max_thrust, 0.3)

            # Horizontal thrusters: BL=index2, BR=index3
            base = TRAJECTORY_FORWARD_PWM / self.max_thrust
            self.thrust[2] = clamp((base - yaw_correction) * self.max_thrust, self.max_thrust)
            self.thrust[3] = clamp((base + yaw_correction) * self.max_thrust, self.max_thrust)
            # BL/BR (indices 0,1) handled by pressure_callback
            return

        # -------------------------------------------------------
        # NORMAL / DEPTH KEEPING MODE
        # -------------------------------------------------------

        # Horizontal thrusters: BL=index2, BR=index3
        bl = (forward - turn) * self.max_thrust
        br = (forward + turn) * self.max_thrust
        self.thrust[2] = clamp(bl, self.max_thrust)
        self.thrust[3] = clamp(br, self.max_thrust)

        # Pitch feedforward on vertical thrusters (BL/BR)
        # Forward → pitch_ff negative (pushes nose down to counteract nose-up)
        # Backward → pitch_ff positive (flips to counteract nose-down)
        # Stationary → 0 (no correction needed)
        pitch_ff = 0 #PITCH_FEEDFORWARD if abs(forward) > DEADZONE else 0.0

        # Vertical thrusters: BL=index0, BR=index1 (manual when depth keeping OFF)
        if not self.depth_keeping:
            ascend   = trigger_to_thrust(axes[AXIS_RIGHT_TRIGGER])
            descend  = trigger_to_thrust(axes[AXIS_LEFT_TRIGGER])
            vertical = clamp((ascend - descend) * self.max_thrust - pitch_ff, self.max_thrust)
            self.thrust[0] = vertical
            self.thrust[1] = vertical
        # else: pressure_callback handles indices 0 and 1
        # Note: pitch feedforward not applied in depth keeping mode —
        # the depth PID will naturally compensate

    # ------------------------------------------------------------------
    # Toggle depth keeping
    # ------------------------------------------------------------------

    def toggle_depth_keeping(self):
        if self.trajectory_mode:
            self.get_logger().warn("Disable trajectory mode first.")
            return

        self.depth_keeping = not self.depth_keeping

        if self.depth_keeping:
            if self.current_depth is not None:
                self.target_depth = self.current_depth
                self.depth_pid.reset()
                self.get_logger().info(
                    f"Depth keeping ENABLED — holding at {self.target_depth:.2f} m"
                )
            else:
                self.depth_keeping = False
                self.get_logger().warn("Depth keeping NOT enabled — no pressure data yet.")
        else:
            self.depth_pid.reset()
            self.thrust[0] = 0.0
            self.thrust[1] = 0.0
            self.get_logger().info("Depth keeping DISABLED — manual control resumed")

    # ------------------------------------------------------------------
    # Toggle trajectory mode
    # ------------------------------------------------------------------

    def toggle_trajectory(self):
        self.trajectory_mode = not self.trajectory_mode

        if self.trajectory_mode:
            if self.current_depth is None:
                self.trajectory_mode = False
                self.get_logger().warn("Trajectory NOT enabled — no pressure data yet.")
                return
            if self.current_yaw is None:
                self.trajectory_mode = False
                self.get_logger().warn("Trajectory NOT enabled — no IMU data yet.")
                return

            self.target_depth  = self.current_depth
            self.target_yaw    = self.current_yaw
            self.depth_keeping = False
            self.depth_pid.reset()
            self.yaw_pid.reset()

            self.get_logger().info(
                f"Trajectory ENABLED — heading {math.degrees(self.target_yaw):.1f}° | "
                f"depth {self.target_depth:.2f} m | "
                f"forward PWM {TRAJECTORY_FORWARD_PWM:.0f}"
            )
        else:
            self.depth_pid.reset()
            self.yaw_pid.reset()
            self.thrust = [0.0, 0.0, 0.0, 0.0]
            self.get_logger().info("Trajectory DISABLED — manual control resumed")

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def publish_thrust(self):
        # The follower node owns the thruster topic while fish-follow mode is active.
        if self.fish_follow_mode:
            return

        msg = Float64MultiArray()
        msg.data = self.thrust
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GamepadTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.fish_follow_mode = False
        follow_msg = Bool()
        follow_msg.data = False
        node.follow_enable_pub.publish(follow_msg)
        node.thrust = [0.0, 0.0, 0.0, 0.0]
        node.publish_thrust()
        node.close_camera_viewer()
        node.destroy_node()
        if node._joy_process is not None:
            node._joy_process.terminate()
            node._joy_process.wait()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()