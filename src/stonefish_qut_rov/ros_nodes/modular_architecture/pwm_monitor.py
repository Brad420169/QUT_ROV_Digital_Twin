#!/usr/bin/env python3
"""Read-only terminal monitor for the physical ROV's existing MAVROS stack.

Displays RC override requests and FCU-reported servo PWM, not measured RPM.
No publishers, services, arming commands or direct MAVLink connections are used.
"""

import time

import rclpy
from mavros_msgs.msg import OverrideRCIn, RCOut, State
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def channel_text(channels, indices, prefix, override=False):
    values = []
    for index in indices:
        value = channels[index] if index < len(channels) else None
        if value is None:
            label = "--"
        elif override and value == 0:
            label = "RELEASE"
        elif override and value == 65535:
            label = "IGNORE"
        else:
            label = str(value)
        name = f"[{index}]" if override else f"{index + 1}"
        values.append(f"{prefix}{name}={label}")
    return " ".join(values)


class PWMMonitor(Node):
    def __init__(self):
        super().__init__("rov_pwm_monitor")
        self.declare_parameter("mavros_namespace", "/mavros")
        namespace = self.get_parameter("mavros_namespace").value.rstrip("/")
        self.samples = {}
        self.subscriptions_owned = [
            self.create_subscription(
                message_type, f"{namespace}/{topic}",
                lambda message, key=key: self.receive(key, message),
                qos_profile_sensor_data,
            )
            for message_type, topic, key in (
                (OverrideRCIn, "rc/override", "request"),
                (RCOut, "rc/out", "output"),
                (State, "state", "state"),
            )
        ]
        self.create_timer(0.5, self.display)
        print(
            f"Read-only PWM monitor: {namespace}/rc/override and {namespace}/rc/out\n"
            "PWM units: microseconds. M1..M4 = FCU output channels 1..4.\n"
            "Output is FCU-reported PWM, NOT measured motor RPM or thrust.\n"
            "Requests and outputs are latest independent samples, not matched acknowledgements.\n"
            "STALE means no new sample for >2 seconds. Ctrl+C stops only this monitor.",
            flush=True,
        )

    def receive(self, key, message):
        self.samples[key] = (message, time.monotonic())

    def describe(self, key, now):
        if key not in self.samples:
            return "WAITING"
        message, received = self.samples[key]
        age = now - received
        freshness = f"{'STALE ' if age > 2.0 else ''}age={age:.1f}s"
        if key == "request":
            body = channel_text(message.channels, (2, 3, 4), "ch", override=True)
        elif key == "output":
            body = channel_text(message.channels, range(4), "M")
        else:
            body = (
                f"{'CONNECTED' if message.connected else 'DISCONNECTED'} "
                f"{'ARMED' if message.armed else 'DISARMED'} mode={message.mode}"
            )
        return f"{body} ({freshness})"

    def display(self):
        now = time.monotonic()
        print(
            f"{time.strftime('%H:%M:%S')} | {self.describe('state', now)} | "
            f"TX: {self.describe('request', now)} | "
            f"FCU OUT: {self.describe('output', now)}",
            flush=True,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PWMMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
