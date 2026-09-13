#!/usr/bin/env python3
"""Launch and supervise the sim/real stack; wait for telemetry before teleop."""
import argparse
import math
import sys
import time

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float64

from node_lifecycle import run_node
from process_utils import start_process, stop_process
from rov_config import (DEPTH_TOPIC, IMU_TOPIC, JOY_TOPIC, REAL_FCU_URL,
                        REAL_INTERFACE_EXECUTABLE, STARTUP_TIMEOUT_S,
                        JOY_TIMEOUT_S, SENSOR_TIMEOUT_S)

PACKAGE = "stonefish_qut_rov"
MAVROS_LOG_FILE = "/tmp/mavros.log"


def print_controls(rov_mode: str):
    fish_text = "AVAILABLE" if rov_mode == "sim" else "DISABLED"

    print()
    print("╔══════════════════════════════════════════════╗")
    print("║           SubbyROV Gamepad Teleop            ║")
    print("╠══════════════════════════════════════════════╣")
    print("║ Left stick ↑↓     Surge                      ║")
    print("║ Right stick ←→    Yaw                        ║")
    print("║ RT / LT           Heave up / down            ║")
    print("║ L bumper          Depth keeping              ║")
    print("║ R bumper          Trajectory mode            ║")
    print("║ Y button          Camera / detector          ║")
    print("║ X button          Fish follow (Cam Window)   ║")
    print("║ A button          Arm / disarm               ║")
    print("║ B button          Battery level              ║")
    print("║ D-pad ↑           Depth / IMU plotter        ║")
    print("║ D-pad ↓ (hold 1s) Shut down                  ║")
    print("╠══════════════════════════════════════════════╣")
    print(f"║ Mode: {rov_mode.upper():<39}║")
    print(f"║ Fish follow: {fish_text:<32}║")
    print("║ Plotter: available (sim + real)              ║")
    print("╚══════════════════════════════════════════════╝")
    print()

class StackSupervisor(Node):
    def __init__(self, mode):
        super().__init__("rov_stack_supervisor")
        self.mode = mode
        self.processes = []
        self.teleop = None
        self.exit_code = 0
        self._shutdown_requested = False
        self.received = {}
        self.started = time.monotonic()
        self.subscriptions_ready = [
            self.create_subscription(Float64, DEPTH_TOPIC, self.depth_ready, qos_profile_sensor_data),
            self.create_subscription(Imu, IMU_TOPIC, self.imu_ready, qos_profile_sensor_data),
            self.create_subscription(Joy, JOY_TOPIC, self.joy_ready, qos_profile_sensor_data),
        ]
        try:
            if mode == "sim":
                self.start("Stonefish", ["ros2", "launch", PACKAGE, "launch_rov.py"])
            else:
                self.start("MAVROS", ["ros2", "launch", "mavros", "apm.launch",
                                      f"fcu_url:={REAL_FCU_URL}"], MAVROS_LOG_FILE)
            # A held stick must still provide fresh messages to the watchdog.
            self.start("joy_node", ["ros2", "run", "joy", "joy_node", "--ros-args",
                                    "-p", "autorepeat_rate:=20.0"])
            interface = "sim_interface.py" if mode == "sim" else REAL_INTERFACE_EXECUTABLE
            self.start("interface", ["ros2", "run", PACKAGE, interface])
            self.timer = self.create_timer(0.1, self.monitor)
            self.get_logger().info("Waiting for joystick, depth and IMU data before starting teleop...")
        except BaseException:
            self.stop()
            self.destroy_node()
            raise

    def start(self, name, command, log_file=None):
        process = start_process(command, log_file)
        self.processes.append((name, process))
        return process

    def depth_ready(self, msg):
        if math.isfinite(msg.data):
            self.received["depth"] = time.monotonic()

    def imu_ready(self, msg):
        q = msg.orientation
        if (all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w))
                and sum(v*v for v in (q.x, q.y, q.z, q.w)) > 1e-12
                and msg.orientation_covariance[0] != -1):
            self.received["imu"] = time.monotonic()

    def joy_ready(self, msg):
        if msg.axes and all(math.isfinite(v) for v in msg.axes):
            self.received["joy"] = time.monotonic()

    def monitor(self):
        for name, process in self.processes:
            code = process.poll()
            if code is not None:
                self.exit_code = code if name == "teleop" else (code or 1)
                self.get_logger().info(f"{name} exited ({code}); shutting down stack.")
                self._shutdown_requested = True
                return
        if self.teleop is not None:
            return
        now = time.monotonic()
        missing = [name for name, timeout in (("joy", JOY_TIMEOUT_S), ("depth", SENSOR_TIMEOUT_S),
                                              ("imu", SENSOR_TIMEOUT_S))
                   if now - self.received.get(name, -math.inf) > timeout]
        if not missing:
            self.teleop = self.start("teleop", ["ros2", "run", PACKAGE, "teleop_controller.py",
                "--ros-args", "-p", f"rov_mode:={self.mode}", "-p", f"package_name:={PACKAGE}"])
            print_controls(self.mode)
            self.get_logger().info("Stack started. Release controls to neutral to enable manual control.")
        elif now - self.started > STARTUP_TIMEOUT_S:
            self.get_logger().error(f"Startup timed out waiting for: {', '.join(missing)}")
            self.exit_code = 1
            self._shutdown_requested = True

    def stop(self):
        if hasattr(self, "timer"):
            self.timer.cancel()
        for name, process in reversed(self.processes):
            stop_process(process, name)
        self.processes.clear()


def main():
    parser = argparse.ArgumentParser(description="QUT ROV control launcher")
    parser.add_argument("--mode", choices=["sim", "real", "s", "r"])
    args = parser.parse_args()
    mode = args.mode
    while mode is None:
        choice = input("Select mode [s] simulation / [r] real ROV: ").strip().lower()
        if choice in ("s", "r", "sim", "real"):
            mode = choice
    mode = "sim" if mode in ("s", "sim") else "real"
    holder = []

    def create():
        node = StackSupervisor(mode)
        holder.append(node)
        return node

    run_node(create, args=[])
    return holder[0].exit_code if holder else 1


if __name__ == "__main__":
    sys.exit(main())
