#!/usr/bin/env python3
"""
Real ROV interface.

Mirrors sim_interface.py — same subscriptions, same published common
topics — but translates to MAVROS instead of Stonefish.

Receives common vehicle-level commands:
    /qut_rov/cmd_vel             (geometry_msgs/Twist)

Publishes to MAVROS:
    /mavros/rc/override          (mavros_msgs/OverrideRCIn)

Bridges MAVROS sensors onto the common topics teleop_controller uses,
so the controller cannot tell sim from real:
    /mavros/imu/static_pressure  ->  /qut_rov/depth  (Float64, metres)
    /mavros/imu/data             ->  /qut_rov/imu    (Imu)

NOTE ON THRUST ALLOCATION
    This node does NOT use thruster_mixer. Vehicle-level surge/heave/yaw
    are sent straight to ArduSub's RC channels and ArduSub's SimpleROV-4
    mixer performs the thrust allocation on the Pixhawk.

    RC channel mapping (SimpleROV-4, confirmed on the bench):
        ch3 = heave    (throttle)
        ch4 = yaw
        ch5 = surge    (forward)
        all others neutral

    thruster_mixer.mix_normalised() is therefore the SIM-side model of
    ArduSub's mixer, not a shared code path. Allocation fidelity is a
    known sim-to-real gap.

PWM conversion:
    RC neutral = 1500 us, range +-400 us -> 1100-1900 us.
    rc_us = 1500 + normalised * 400

Prerequisites:
    ros2 launch mavros apm.launch fcu_url:=udp://:14550@<ROV_IP>:14555
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, MessageInterval, SetMode
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, FluidPressure, Imu
from sim_to_real_scales import REAL_INVERT_HEAVE_CMD, REAL_INVERT_SURGE_CMD, REAL_INVERT_YAW_CMD
from std_msgs.msg import Bool, Float64
from control_utils import quaternion_to_rpy, rpy_to_quaternion

from rov_config import (
    ARM_TOGGLE_TOPIC,
    BATTERY_TOPIC,
    REAL_BATTERY_TOPIC,
    ARMED_STATE_TOPIC,
    CMD_VEL_TOPIC,
    COMMAND_TIMEOUT_S,
    DEPTH_TOPIC,
    GRAVITY,
    IMU_TOPIC,
    REAL_ARMING_SERVICE,
    REAL_FAST_STREAM_MESSAGE_IDS,
    REAL_IMU_TOPIC,
    REAL_MESSAGE_INTERVAL_SERVICE,
    REAL_PRESSURE_TOPIC,
    REAL_SENSOR_RATE_HZ,
    REAL_RC_NEUTRAL_US,
    REAL_RC_OVERRIDE_TOPIC,
    REAL_RC_RANGE_US,
    REAL_SET_MODE_SERVICE,
    REAL_YAW_INVERT,
    REAL_PITCH_INVERT,
    FRESH_WATER_DENSITY,
)
from command_watchdog import Freshness
from node_lifecycle import run_node
from control_utils import clamp
from vehicle_command import VehicleCommand

# OverrideRCIn expects 18 channels on ROS 2 / MAVROS 2.x
RC_CHANNEL_COUNT = 18

SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


def normalised_to_rc(value: float) -> int:
    """
    Convert a normalised +-1.0 command to ArduSub RC microseconds.

    +1.0  ->  1900 us  (full forward / up)
     0.0  ->  1500 us  (stopped)
    -1.0  ->  1100 us  (full reverse / down)
    """
    clamped = max(-1.0, min(1.0, value))
    return int(REAL_RC_NEUTRAL_US + clamped * REAL_RC_RANGE_US)


class RealInterface(Node):

    def __init__(self):
        super().__init__("real_interface")

        self.command = VehicleCommand()
        self.surface_pressure_pa = None
        self._pressure_samples = []
        self._pressure_sample_time = None
        self.get_logger().info(
            'Measuring Pa for depth calibration: keep the pressure sensor in air '
            'and the vehicle disarmed. Averaging 30 samples over at least 3 seconds; '
            'depth output is withheld until ready.'
        )

        # Watchdog state
        self.command_freshness = Freshness(COMMAND_TIMEOUT_S)
        self._stopped = False
        self._command_stale = True   # start stale: no command yet -> neutral

        # Publishers
        self.rc_pub = self.create_publisher(
            OverrideRCIn,
            REAL_RC_OVERRIDE_TOPIC,
            10,
        )

        self.depth_pub = self.create_publisher(
            Float64,
            DEPTH_TOPIC,
            10,
        )

        self.imu_pub = self.create_publisher(
            Imu,
            IMU_TOPIC,
            10,
        )

        # Reports the vehicle's actual armed state back to teleop, which
        # prints it. This node owns the truth — teleop only ever asks for
        # a toggle, so the two can never disagree.
        self.armed_pub = self.create_publisher(
            Bool,
            ARMED_STATE_TOPIC,
            10,
        )

        self.battery_pub = self.create_publisher(
            BatteryState,
            BATTERY_TOPIC,
            10,
        )

        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist,
            CMD_VEL_TOPIC,
            self.command_callback,
            10,
        )

        self.pressure_sub = self.create_subscription(
            FluidPressure,
            REAL_PRESSURE_TOPIC,
            self.pressure_callback,
            SENSOR_QOS,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            REAL_IMU_TOPIC,
            self.imu_callback,
            SENSOR_QOS,
        )

        self.state_sub = self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            SENSOR_QOS,
        )

        self.battery_sub = self.create_subscription(
            BatteryState,
            REAL_BATTERY_TOPIC,
            self.battery_callback,
            SENSOR_QOS,
        )

        self.arm_toggle_sub = self.create_subscription(
            Bool,
            ARM_TOGGLE_TOPIC,
            self.arm_toggle_callback,
            10,
        )

        # Latest armed state from the FCU (None until the first message)
        self.armed = None

        # 20 Hz publish timer (same rate as sim_interface)
        self.timer = self.create_timer(0.05, self.publish_thrusters)

        # Request MANUAL mode from ArduSub after a short delay (one-shot)
        self._mode_timer = self.create_timer(6.0, self._set_manual_mode)

        # Request faster IMU/depth streaming once MAVROS is up (one-shot).
        # ArduSub does not honour this from SRx_* params alone on a fresh
        # connection, so it has to be asked for every run.
        self._rate_timer = self.create_timer(3.0, self._set_sensor_rates)

        self.get_logger().info(
            "REAL interface ready: surge/heave/yaw -> "
            f"{REAL_RC_OVERRIDE_TOPIC} | bridging depth + IMU"
        )

    # ------------------------------------------------------------------
    # Command in (identical shape to sim_interface) + watchdog stamp
    # ------------------------------------------------------------------
    def command_callback(self, msg: Twist):
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.linear.z, msg.angular.z)):
            self.command.zero()
            self.command_freshness.invalidate()
            return
        self.command.surge = float(msg.linear.x)
        self.command.heave = float(msg.linear.z)
        self.command.yaw   = float(msg.angular.z)

        self.command_freshness.touch()

        if self._command_stale:
            self._command_stale = False
            self.get_logger().info("cmd_vel stream live.")

    # ------------------------------------------------------------------
    # Arming
    # ------------------------------------------------------------------
    def state_callback(self, msg: State):
        """Track the FCU's armed state and republish it on change."""
        armed = bool(msg.armed)

        if armed == self.armed:
            return

        self.armed = armed

        out = Bool()
        out.data = armed
        self.armed_pub.publish(out)

        self.get_logger().info("ARMED" if armed else "DISARMED")

    def arm_toggle_callback(self, msg: Bool):
        """
        Gamepad asked to flip the arm state. This node holds the truth, so
        the request carries no desired value — we invert what the FCU
        actually reports.
        """
        if self.armed is None:
            self.get_logger().warn(
                "No /mavros/state yet — cannot toggle arming."
            )
            return

        self._set_arming(not self.armed)

    def _set_arming(self, arm: bool):
        action = "Arm" if arm else "Disarm"

        client = self.create_client(CommandBool, REAL_ARMING_SERVICE)

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f"{action} failed — arming service unavailable."
            )
            return

        req = CommandBool.Request()
        req.value = arm

        future = client.call_async(req)
        future.add_done_callback(
            lambda f: self._arming_result(f, action)
        )

    def _arming_result(self, future, action: str):
        if future.result() and future.result().success:
            self.get_logger().info(f"{action} request accepted.")
        else:
            self.get_logger().warn(
                f"{action} request rejected by the FCU."
            )

    # ------------------------------------------------------------------
    # Sensor bridges -> common topics
    # ------------------------------------------------------------------
    def pressure_callback(self, msg: FluidPressure):
        pressure = float(msg.fluid_pressure)
        if not math.isfinite(pressure) or pressure <= 0:
            if self.surface_pressure_pa is None:
                self._pressure_samples.clear()
                self._pressure_sample_time = None
            return
        if self.surface_pressure_pa is None:
            now = time.monotonic()
            if self._pressure_sample_time is not None:
                gap = now - self._pressure_sample_time
                if gap > 1.0 or gap < 0:
                    self._pressure_samples.clear()
                    self.get_logger().warning('Pressure sampling interrupted; restarting depth calibration.')
                elif gap < .11:
                    return  # Spread samples across time, not a queued burst.
            self._pressure_sample_time = now
            self._pressure_samples.append(pressure)
            count = len(self._pressure_samples)
            if count < 30:
                if count % 10 == 0:
                    self.get_logger().info(f'Measuring Pa for depth calibration: {count}/30 samples.')
                return
            self.surface_pressure_pa = math.fsum(self._pressure_samples) / count
            self.get_logger().info(
                f'Depth calibration ready: average surface pressure = '
                f'{self.surface_pressure_pa:.2f} Pa ({count} samples). '
                'Real depth readings enabled (metres, positive down).'
            )
        gauge_pressure = pressure - self.surface_pressure_pa
        depth = gauge_pressure / (FRESH_WATER_DENSITY * GRAVITY)

        depth_msg = Float64()
        depth_msg.data = depth
        self.depth_pub.publish(depth_msg)

    def battery_callback(self, msg: BatteryState):
        """Republish the MAVROS battery state on the common topic."""
        self.battery_pub.publish(msg)

    def imu_callback(self, msg: Imu):
        if not (REAL_PITCH_INVERT or REAL_YAW_INVERT):
            self.imu_pub.publish(msg)
            return

        q = msg.orientation
        roll, pitch, yaw = quaternion_to_rpy(q.x, q.y, q.z, q.w)

        if REAL_PITCH_INVERT:
            pitch = -pitch

        if REAL_YAW_INVERT:
            yaw = -yaw

        x, y, z, w = rpy_to_quaternion(roll, pitch, yaw)

        out = Imu()
        out.header = msg.header
        out.orientation.x = x
        out.orientation.y = y
        out.orientation.z = z
        out.orientation.w = w
        out.orientation_covariance = msg.orientation_covariance

        out.angular_velocity = msg.angular_velocity
        if REAL_PITCH_INVERT:
            out.angular_velocity.y = -msg.angular_velocity.y
        if REAL_YAW_INVERT:
            out.angular_velocity.z = -msg.angular_velocity.z
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.imu_pub.publish(out)

    # ------------------------------------------------------------------
    # Thruster publish
    # ------------------------------------------------------------------
    def _command_is_stale(self) -> bool:
        return not self.command_freshness.fresh()

    @staticmethod
    def _neutral_channels() -> list[int]:
        return [REAL_RC_NEUTRAL_US] * RC_CHANNEL_COUNT

    def publish_thrusters(self):
        msg = OverrideRCIn()

        if self._command_is_stale():
            # Controller has stopped publishing (crashed, killed, tether
            # dropped). Latch neutral rather than holding the last stick
            # position, and zero the stored command so recovery starts
            # from a known state.
            if not self._command_stale:
                self._command_stale = True
                self.get_logger().warn(
                    f"No cmd_vel for >{COMMAND_TIMEOUT_S:.2f} s "
                    "— forcing neutral."
                )

            self.command.zero()
            msg.channels = self._neutral_channels()
            self.rc_pub.publish(msg)
            return

        surge = clamp(self.command.surge)
        heave = clamp(self.command.heave)
        yaw   = clamp(self.command.yaw)

        if REAL_INVERT_SURGE_CMD:
            surge = -surge
        if REAL_INVERT_HEAVE_CMD:
            heave = -heave
        if REAL_INVERT_YAW_CMD:
            yaw = -yaw

        channels = self._neutral_channels()
        channels[2] = normalised_to_rc(heave)   # ch3 — throttle / heave
        channels[3] = normalised_to_rc(yaw)     # ch4 — yaw
        channels[4] = normalised_to_rc(surge)   # ch5 — forward / surge

        msg.channels = channels
        self.rc_pub.publish(msg)

    # Stop — neutral on all channels (called on shutdown)
    def stop(self):
        if self._stopped:
            return
        self._stopped = True
        self.timer.cancel()
        self._mode_timer.cancel()
        self.command.zero()
        if not rclpy.ok():
            return

        msg = OverrideRCIn()
        msg.channels = self._neutral_channels()
        self.rc_pub.publish(msg)

        self.get_logger().info("Thrusters zeroed.")

        self._disarm()

    def _disarm(self):
        """
        Disarm on shutdown.

        Neutral RC alone leaves the vehicle armed, so without this the
        ROV stays live between sessions — thrusters hot on the bench with
        nothing running, and the startup splash skips straight past its
        arming stage on the next launch.
        """
        client = self.create_client(CommandBool, REAL_ARMING_SERVICE)

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "Arming service unavailable — vehicle may still be ARMED. "
                "Disarm manually in QGC."
            )
            return

        req = CommandBool.Request()
        req.value = False

        future = client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result() and future.result().success:
                self.get_logger().info("Disarm acknowledged.")
            else:
                self.get_logger().warn("Disarm not confirmed; check vehicle state in QGC.")
        finally:
            self.destroy_client(client)

    # ------------------------------------------------------------------
    # Mode / arming
    # ------------------------------------------------------------------
    def _set_sensor_rates(self):
        self._rate_timer.cancel()

        self._rate_client = self.create_client(MessageInterval, REAL_MESSAGE_INTERVAL_SERVICE)

        if not self._rate_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "MAVROS set_message_interval service not available — "
                "IMU/depth staying at their ArduSub default rate."
            )
            return

        # MAVROS's cmd plugin is not safe for concurrent SET_MESSAGE_INTERVAL
        # calls (confirmed on the bench: firing them concurrently crashed
        # mavros_node with "Promise already satisfied") — request one at a
        # time, moving to the next only once each response lands.
        self._rate_ids_remaining = list(REAL_FAST_STREAM_MESSAGE_IDS)
        self._request_next_sensor_rate()

    def _request_next_sensor_rate(self):
        if not self._rate_ids_remaining:
            return
        message_id = self._rate_ids_remaining.pop(0)
        req = MessageInterval.Request()
        req.message_id = message_id
        req.message_rate = REAL_SENSOR_RATE_HZ
        self._rate_client.call_async(req).add_done_callback(self._rate_response_callback)

    def _rate_response_callback(self, future):
        result = future.result()
        if not result or not result.success:
            self.get_logger().warn("MAVROS declined a set_message_interval request.")
        self._request_next_sensor_rate()

    def _set_manual_mode(self):
        self._mode_timer.cancel()

        client = self.create_client(SetMode, REAL_SET_MODE_SERVICE)

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "MAVROS set_mode service not available — "
                "is MAVROS running?  Set flight mode to MANUAL in QGC manually."
            )
            return

        req = SetMode.Request()
        req.custom_mode = "MANUAL"

        future = client.call_async(req)
        future.add_done_callback(self._mode_response_callback)

    def _mode_response_callback(self, future):
        if future.result() and future.result().mode_sent:
            self.get_logger().info("ArduSub flight mode set to MANUAL.")

            if self.armed:
                self.get_logger().warn(
                    "Vehicle was ARMED at startup — disarming for a known state."
                )
                self._set_arming(False)
            else:
                self.get_logger().info("Press A to arm — thrusters are NOT live.")
        else:
            self.get_logger().warn(
                "set_mode call failed — set MANUAL mode in QGC manually."
            )

# Entry point
def main(args=None):
    run_node(RealInterface, args)


if __name__ == "__main__":
    main()
