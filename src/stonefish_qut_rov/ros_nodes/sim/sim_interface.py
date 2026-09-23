#!/usr/bin/env python3
"""
Simulation interface.

Receives common vehicle-level commands:
    /qut_rov/cmd_vel

Converts them to Stonefish actuator commands:
    /qut_rov/setpoint/thrusters
    [TL, TR, BL, BR]

Also converts Stonefish pressure into:
    /qut_rov/depth
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64, Float64MultiArray

from rov_config import (
    CMD_VEL_TOPIC,
    COMMAND_TIMEOUT_S,
    DEPTH_TOPIC,
    GRAVITY,
    SIM_MAX_SETPOINT,
    SIM_PRESSURE_TOPIC,
    SIM_THRUSTER_TOPIC,
    SIM_SURFACE_PRESSURE_PA,
    WATER_DENSITY,
)
from command_watchdog import Freshness
from node_lifecycle import run_node
from thruster_mixer import mix_to_setpoints
from vehicle_command import VehicleCommand


class SimInterface(Node):
    def __init__(self):
        super().__init__("sim_interface")

        self.command = VehicleCommand()
        self.command_freshness = Freshness(COMMAND_TIMEOUT_S)

        self.thruster_pub = self.create_publisher(
            Float64MultiArray,
            SIM_THRUSTER_TOPIC,
            10,
        )

        self.depth_pub = self.create_publisher(
            Float64,
            DEPTH_TOPIC,
            10,
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            CMD_VEL_TOPIC,
            self.command_callback,
            10,
        )

        self.pressure_sub = self.create_subscription(
            FluidPressure,
            SIM_PRESSURE_TOPIC,
            self.pressure_callback,
            10,
        )

        self.timer = self.create_timer(
            0.05,
            self.publish_thrusters,
        )

        self.get_logger().info(
            "SIM interface ready: surge/heave/yaw -> [TL, TR, BL, BR]"
        )

    def command_callback(self, msg: Twist):
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.linear.z, msg.angular.z)):
            self.command.zero()
            self.command_freshness.invalidate()
            return
        self.command_freshness.touch()
        self.command.surge = float(msg.linear.x)
        self.command.heave = float(msg.linear.z)
        self.command.yaw = float(msg.angular.z)

    def pressure_callback(self, msg: FluidPressure):
        if not math.isfinite(msg.fluid_pressure):
            return
        gauge_pressure = float(msg.fluid_pressure) - SIM_SURFACE_PRESSURE_PA
        depth = gauge_pressure / (WATER_DENSITY * GRAVITY)

        depth_msg = Float64()
        depth_msg.data = depth
        self.depth_pub.publish(depth_msg)

    def publish_thrusters(self):
        if not self.command_freshness.fresh():
            self.command.zero()
        msg = Float64MultiArray()
        msg.data = mix_to_setpoints(
            self.command,
            SIM_MAX_SETPOINT,
        )
        self.thruster_pub.publish(msg)

    def stop(self):
        self.timer.cancel()
        self.command.zero()
        if not rclpy.ok():
            return
        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.thruster_pub.publish(zero_msg)


def main(args=None):
    run_node(SimInterface, args)


if __name__ == "__main__":
    main()
