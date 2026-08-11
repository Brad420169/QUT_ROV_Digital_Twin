#!/usr/bin/env python3
"""
Real ROV interface.

Mirrors sim_interface.py exactly — same subscriptions, same depth
topic output — but translates to MAVROS instead of Stonefish.

Receives common vehicle-level commands:
    /qut_rov/cmd_vel          (geometry_msgs/Twist)

Publishes to MAVROS:
    /mavros/rc/override        (mavros_msgs/OverrideRCIn)

Receives depth from MAVROS Bar30:
    /mavros/imu/static_pressure  (sensor_msgs/FluidPressure)

Publishes common depth topic (same as sim_interface):
    /qut_rov/depth             (std_msgs/Float64)

RC channel mapping (verify with QGC Motor Test before first wet run):
    Channel 1 → TL  (vertical left)
    Channel 2 → TR  (vertical right)
    Channel 3 → BL  (horizontal left)
    Channel 4 → BR  (horizontal right)
    Channels 5–8 → RC_PASSTHROUGH (no override)

PWM conversion:
    mix_normalised() outputs ±1.0.
    RC neutral = 1500 µs, range ±400 µs → 1100–1900 µs.
    rc_us = 1500 + normalised * 400

Thruster order from thruster_mixer.mix_normalised():
    index 0 = TL
    index 1 = TR
    index 2 = BL
    index 3 = BR

Prerequisites:
    ros2 launch mavros apm.launch fcu_url:=udp://:14550@<ROV_IP>:14555
"""

import threading

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64

from rov_config import (
    CMD_VEL_TOPIC,
    DEPTH_TOPIC,
    GRAVITY,
    SURFACE_PRESSURE_PA,
    WATER_DENSITY,
)
from thruster_mixer import VehicleCommand, mix_normalised

# MAVROS topics
MAVROS_RC_OVERRIDE_TOPIC    = "/mavros/rc/override"
MAVROS_PRESSURE_TOPIC       = "/mavros/imu/static_pressure"
MAVROS_SET_MODE_SERVICE     = "/mavros/set_mode"

# RC constants
RC_NEUTRAL_US    = 1500     # µs — ArduSub neutral / stopped
RC_RANGE_US      = 400      # ±400 µs either side of neutral
RC_PASSTHROUGH   = 0        # tells ArduSub to ignore this channel override


def normalised_to_rc(value: float) -> int:
    """
    Convert a normalised ±1.0 thruster value to ArduSub RC microseconds.

    +1.0  →  1900 µs  (full forward / up)
     0.0  →  1500 µs  (stopped)
    -1.0  →  1100 µs  (full reverse / down)
    """
    clamped = max(-1.0, min(1.0, value))
    return int(RC_NEUTRAL_US + clamped * RC_RANGE_US)


class RealInterface(Node):

    def __init__(self):
        super().__init__("real_interface")

        self.command = VehicleCommand()

        # Publishers
        self.rc_pub = self.create_publisher(
            OverrideRCIn,
            MAVROS_RC_OVERRIDE_TOPIC,
            10,
        )

        self.depth_pub = self.create_publisher(
            Float64,
            DEPTH_TOPIC,
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
            MAVROS_PRESSURE_TOPIC,
            self.pressure_callback,
            10,
        )

        # 20 Hz publish timer (same rate as sim_interface)
        self.timer = self.create_timer(0.05, self.publish_thrusters)

        # Request MANUAL mode from ArduSub so RC override is accepted
        threading.Thread(target=self._set_manual_mode, daemon=True).start()

        self.get_logger().info(
            "REAL interface ready: surge/heave/yaw -> /mavros/rc/override"
        )

    # Command callback — identical shape to sim_interface
    def command_callback(self, msg: Twist):
        self.command.surge = float(msg.linear.x)
        self.command.heave = float(msg.linear.z)
        self.command.yaw   = float(msg.angular.z)

    # Pressure callback — convert Bar30 reading to depth metres and
    # re-publish on /qut_rov/depth so teleop_controller sees the same
    # topic regardless of mode (identical conversion to sim_interface)

    def pressure_callback(self, msg: FluidPressure):
        gauge_pressure = float(msg.fluid_pressure) - SURFACE_PRESSURE_PA
        depth = gauge_pressure / (WATER_DENSITY * GRAVITY)

        depth_msg = Float64()
        depth_msg.data = depth
        self.depth_pub.publish(depth_msg)

    # --------------------------------------------------------------------
    # Thruster publish — mix then convert to RC microseconds
    #
    # mix_normalised() output order: [TL, TR, BL, BR]
    # RC channel mapping:
    #   channels[0]  ch1 → TL   (vertical left)
    #   channels[1]  ch2 → TR   (vertical right)
    #   channels[2]  ch3 → BL   (horizontal left)
    #   channels[3]  ch4 → BR   (horizontal right)
    #   channels[4–7]    → RC_PASSTHROUGH
    #
    # *** Confirm mapping with QGC Motor Test before first wet run! ***
    # --------------------------------------------------------------------

    def publish_thrusters(self):
        tl, tr, bl, br = mix_normalised(self.command)

        msg = OverrideRCIn()
        msg.channels = [
            normalised_to_rc(tl),   # ch1 → TL
            normalised_to_rc(tr),   # ch2 → TR
            normalised_to_rc(bl),   # ch3 → BL
            normalised_to_rc(br),   # ch4 → BR
            RC_PASSTHROUGH,          # ch5 — unused
            RC_PASSTHROUGH,          # ch6 — unused
            RC_PASSTHROUGH,          # ch7 — unused
            RC_PASSTHROUGH,          # ch8 — unused
        ]
        self.rc_pub.publish(msg)

    # Stop — send neutral on all channels (called on shutdown)
    def stop(self):
        msg = OverrideRCIn()
        msg.channels = [RC_NEUTRAL_US] * 8
        self.rc_pub.publish(msg)
        self.get_logger().info("Thrusters zeroed.")

    # Request ArduSub MANUAL mode so RC override commands are obeyed.
    # Runs in a daemon thread so it doesn't block __init__.
    def _set_manual_mode(self):
        client = self.create_client(SetMode, MAVROS_SET_MODE_SERVICE)

        if not client.wait_for_service(timeout_sec=8.0):
            self.get_logger().warn(
                "MAVROS set_mode service not available — "
                "is MAVROS running?  Set flight mode to MANUAL in QGC manually."
            )
            return

        req = SetMode.Request()
        req.custom_mode = "MANUAL"

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().mode_sent:
            self.get_logger().info("ArduSub flight mode set to MANUAL.")
        else:
            self.get_logger().warn(
                "set_mode call failed — set MANUAL mode in QGC manually."
            )


# Entry point
def main(args=None):
    rclpy.init(args=args)
    node = RealInterface()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
