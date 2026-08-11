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

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64, Float64MultiArray

from rov_config import (
    CMD_VEL_TOPIC,
    DEPTH_TOPIC,
    GRAVITY,
    SIM_MAX_SETPOINT,
    SIM_PRESSURE_TOPIC,
    SIM_THRUSTER_TOPIC,
    SURFACE_PRESSURE_PA,
    WATER_DENSITY,
)
from thruster_mixer import VehicleCommand, mix_to_setpoints


class SimInterface(Node):
    def __init__(self):
        super().__init__("sim_interface")

        self.command = VehicleCommand()

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
        self.command.surge = float(msg.linear.x)
        self.command.heave = float(msg.linear.z)
        self.command.yaw = float(msg.angular.z)

    def pressure_callback(self, msg: FluidPressure):
        gauge_pressure = float(msg.fluid_pressure) - SURFACE_PRESSURE_PA
        depth = gauge_pressure / (WATER_DENSITY * GRAVITY)

        depth_msg = Float64()
        depth_msg.data = depth
        self.depth_pub.publish(depth_msg)

    def publish_thrusters(self):
        msg = Float64MultiArray()
        msg.data = mix_to_setpoints(
            self.command,
            SIM_MAX_SETPOINT,
        )
        self.thruster_pub.publish(msg)

    def stop(self):
        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.thruster_pub.publish(zero_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimInterface()

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
