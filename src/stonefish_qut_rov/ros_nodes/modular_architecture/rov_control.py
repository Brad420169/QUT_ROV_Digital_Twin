#!/usr/bin/env python3
"""
Single entry point for the QUT ROV control stack.

Usage:
    ros2 run stonefish_qut_rov rov_control.py

Then select:
    [s] Simulation
    [r] Real ROV

SIM mode automatically starts:
    1. ros2 launch stonefish_qut_rov launch_rov.py
    2. joy_node
    3. sim_interface.py
    4. teleop_controller.py  (rov_mode:=sim)

REAL mode automatically starts:
    1. MAVROS  (apm.launch, FCU URL from rov_config.REAL_FCU_URL)
    2. joy_node
    3. real_interface.py
    4. teleop_controller.py  (rov_mode:=real)
"""

import argparse
import os
import signal
import subprocess
import sys
import time

from rov_config import (
    REAL_FCU_URL,
    REAL_INTERFACE_EXECUTABLE,
    REAL_MAVROS_STARTUP_DELAY,
)


PACKAGE         = "stonefish_qut_rov"

# MAVROS prints a long plugin banner on startup. Send it to a file so the
# launcher terminal stays readable; tail the file when debugging.
MAVROS_LOG_FILE = "/tmp/mavros.log"
SIM_LAUNCH_FILE = "launch_rov.py"

# Startup delays let each part of the ROS stack initialise before
# the next process starts.
STONEFISH_STARTUP_DELAY = 5.0
JOY_STARTUP_DELAY       = 1.0
INTERFACE_STARTUP_DELAY = 0.5


# ---------------------------------------------------------------------------
# Process helpers — unchanged from original
# ---------------------------------------------------------------------------

def start_process(
    command: list[str],
    log_file: str | None = None,
) -> subprocess.Popen:
    """
    Start a process in its own process group.

    Using a separate process group is important for ros2 launch because
    launch_rov.py starts Stonefish and other child processes. It lets this
    launcher shut down the entire group cleanly with Ctrl+C.

    If log_file is given, stdout and stderr go there instead of this
    terminal — used for MAVROS, whose plugin banner otherwise buries
    everything else during startup. Tail it when you need the detail:
        tail -f /tmp/mavros.log
    """
    print("$", " ".join(command), flush=True)

    if log_file is not None:
        handle = open(log_file, "w")

        return subprocess.Popen(
            command,
            stdout=handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    return subprocess.Popen(
        command,
        start_new_session=True,
    )


def stop_process(process: subprocess.Popen | None, name: str = "process"):
    """Stop a process and its child process group."""
    if process is None or process.poll() is not None:
        return

    print(f"Stopping {name}...", flush=True)

    try:
        os.killpg(
            os.getpgid(process.pid),
            signal.SIGINT,
        )

        process.wait(timeout=5.0)

    except subprocess.TimeoutExpired:
        try:
            os.killpg(
                os.getpgid(process.pid),
                signal.SIGTERM,
            )

            process.wait(timeout=3.0)

        except subprocess.TimeoutExpired:
            try:
                os.killpg(
                    os.getpgid(process.pid),
                    signal.SIGKILL,
                )
            except ProcessLookupError:
                pass

            process.wait()

    except ProcessLookupError:
        pass


# ---------------------------------------------------------------------------
# Mode selection
# ---------------------------------------------------------------------------

def choose_mode() -> str:
    print()
    print("================================")
    print("         QUT ROV CONTROL")
    print("================================")
    print("[s] Simulation")
    print("[r] Real ROV")
    print()

    while True:
        choice = input("Select mode: ").strip().lower()

        if choice in ("s", "r"):
            return choice

        print("Please enter 's' or 'r'.")


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

# ---------------------------------------------------------------------------
# Simulation mode — unchanged from original
# ---------------------------------------------------------------------------

def run_simulation() -> int:
    print()
    print("================================")
    print("        SIMULATION MODE")
    print("================================")
    print("Launching Stonefish and ROS control nodes...")
    print()

    processes: list[tuple[str, subprocess.Popen]] = []

    try:
        # ================================================================
        # 1. STONEFISH
        # ================================================================

        stonefish = start_process([
            "ros2",
            "launch",
            PACKAGE,
            SIM_LAUNCH_FILE,
        ])
        processes.append(("Stonefish simulation", stonefish))

        print(
            f"Waiting {STONEFISH_STARTUP_DELAY:.0f} s for Stonefish to start...",
            flush=True,
        )
        time.sleep(STONEFISH_STARTUP_DELAY)

        if stonefish.poll() is not None:
            print(
                "ERROR: Stonefish launch exited before control nodes started.",
                file=sys.stderr,
            )
            return stonefish.returncode or 1

        # ================================================================
        # 2. GAMEPAD DRIVER
        # ================================================================

        joy = start_process([
            "ros2",
            "run",
            "joy",
            "joy_node",
        ])
        processes.append(("joy_node", joy))

        time.sleep(JOY_STARTUP_DELAY)

        # ================================================================
        # 3. SIM INTERFACE
        #
        # /qut_rov/cmd_vel
        #       ->
        # surge/heave/yaw mixer
        #       ->
        # /qut_rov/setpoint/thrusters
        # ================================================================

        sim_interface = start_process([
            "ros2",
            "run",
            PACKAGE,
            "sim_interface.py",
        ])

        processes.append(("sim_interface", sim_interface))

        time.sleep(1.0)

        print()
        print("Waiting for Stonefish to finish initialising...")
        time.sleep(5.0)


        # ================================================================
        # 4. HIGH-LEVEL TELEOP / AUTONOMY CONTROLLER
        # ================================================================

        teleop = start_process([
            "ros2",
            "run",
            PACKAGE,
            "teleop_controller.py",
            "--ros-args",
            "-p",
            "rov_mode:=sim",
            "-p",
            f"package_name:={PACKAGE}",
        ])
        processes.append(("teleop_controller", teleop))

        print()
        print("================================")
        print("         SIMULATION READY")
        print("================================")
        print("Stonefish:       RUNNING")
        print("Gamepad:         RUNNING")
        print("Sim interface:   RUNNING")
        print("Teleop control:  RUNNING")

        print_controls("sim")

        print("Press Ctrl+C here to stop everything.")
        print()

        # ================================================================
        # 5. WATCHDOG
        #
        # Keep rov_control.py alive while both Stonefish and teleop are
        # running. If either one closes, leave this loop so the finally
        # block shuts down the rest of the stack automatically.
        # ================================================================

        while True:
            if stonefish.poll() is not None:
                print()
                print("Stonefish has closed.")
                print("Stopping the remaining simulation nodes...")
                return stonefish.returncode or 0

            if teleop.poll() is not None:
                return_code = teleop.returncode or 0

                if return_code != 0:
                    print(
                        f"teleop_controller exited with code {return_code}",
                        file=sys.stderr,
                    )

                print()
                print("Teleop controller has closed.")
                print("Stopping the remaining simulation nodes...")
                return return_code

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nCtrl+C received.")
        return 0

    finally:
        print()
        print("Shutting down simulation...")

        for name, process in reversed(processes):
            stop_process(process, name)

        print("Simulation stopped.")


# ---------------------------------------------------------------------------
# Real ROV mode
# ---------------------------------------------------------------------------

def run_real() -> int:
    print()
    print("================================")
    print("          REAL ROV MODE")
    print("================================")
    print(f"FCU URL: {REAL_FCU_URL}")
    print("Launching MAVROS and ROS control nodes...")
    print()

    processes: list[tuple[str, subprocess.Popen]] = []

    try:
        # ================================================================
        # 1. MAVROS
        #
        # Bridges MAVLink (Pixhawk / ArduSub) to ROS2 topics.
        # Publishes:
        #   /mavros/imu/static_pressure  (Bar30 depth)
        #   /mavros/imu/data             (IMU orientation)
        # Subscribes:
        #   /mavros/rc/override          (thruster RC commands)
        # ================================================================

        mavros = start_process(
            [
                "ros2",
                "launch",
                "mavros",
                "apm.launch",
                f"fcu_url:={REAL_FCU_URL}",
            ],
            log_file=MAVROS_LOG_FILE,
        )
        processes.append(("MAVROS", mavros))

        print(f"MAVROS output -> {MAVROS_LOG_FILE}", flush=True)

        print(
            f"Waiting {REAL_MAVROS_STARTUP_DELAY:.0f} s for MAVROS to connect...",
            flush=True,
        )
        time.sleep(REAL_MAVROS_STARTUP_DELAY)

        if mavros.poll() is not None:
            print(
                "ERROR: MAVROS exited before control nodes started.\n"
                "       Check the FCU URL and tether connection.",
                file=sys.stderr,
            )
            return mavros.returncode or 1

        # ================================================================
        # 2. GAMEPAD DRIVER
        # ================================================================

        joy = start_process([
            "ros2",
            "run",
            "joy",
            "joy_node",
        ])
        processes.append(("joy_node", joy))

        time.sleep(JOY_STARTUP_DELAY)

        # ================================================================
        # 3. REAL INTERFACE
        #
        # /qut_rov/cmd_vel
        #       ->
        # surge/heave/yaw mixer  +  normalised_to_rc()
        #       ->
        # /mavros/rc/override  (1100–1900 µs)
        #
        # /mavros/imu/static_pressure
        #       ->
        # gauge pressure / (rho * g)
        #       ->
        # /qut_rov/depth  (Float64, metres)
        # ================================================================

        real_interface = start_process([
            "ros2",
            "run",
            PACKAGE,
            REAL_INTERFACE_EXECUTABLE,
        ])
        processes.append(("real_interface", real_interface))

        time.sleep(INTERFACE_STARTUP_DELAY)

        # ================================================================
        # 4. HIGH-LEVEL TELEOP / AUTONOMY CONTROLLER
        #
        # rov_mode:=real disables fish following and the sim camera launcher.
        # All other modes (manual, depth keeping, trajectory) work as normal.
        # ================================================================

        teleop = start_process([
            "ros2",
            "run",
            PACKAGE,
            "teleop_controller.py",
            "--ros-args",
            "-p",
            "rov_mode:=real",
            "-p",
            f"package_name:={PACKAGE}",
        ])
        processes.append(("teleop_controller", teleop))

        print()
        print("================================")
        print("         REAL ROV READY")
        print("================================")
        print("MAVROS:          RUNNING")
        print("Gamepad:         RUNNING")
        print("Real interface:  RUNNING")
        print("Teleop control:  RUNNING")

        print_controls("real")

        print("Press Ctrl+C here to stop everything.")
        print()

        return_code = teleop.wait()

        if return_code != 0:
            print(
                f"teleop_controller exited with code {return_code}",
                file=sys.stderr,
            )

        return return_code

    except KeyboardInterrupt:
        print("\nCtrl+C received.")
        return 0

    finally:
        print()
        print("Shutting down real ROV stack...")

        # Shut down in reverse order:
        # controller -> interface -> joystick -> MAVROS
        for name, process in reversed(processes):
            stop_process(process, name)

        print("Real ROV stack stopped.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="QUT ROV control launcher"
    )
    parser.add_argument(
        "--mode",
        choices=["sim", "real", "s", "r"],
        help="Start directly in simulation or real ROV mode.",
    )

    args = parser.parse_args()

    if args.mode is None:
        mode = choose_mode()
    else:
        mode = "s" if args.mode in ("sim", "s") else "r"

    if mode == "s":
        return run_simulation()

    return run_real()


if __name__ == "__main__":
    sys.exit(main())