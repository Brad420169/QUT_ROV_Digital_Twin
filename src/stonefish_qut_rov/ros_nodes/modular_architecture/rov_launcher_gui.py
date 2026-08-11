#!/usr/bin/env python3
"""
QUT ROV graphical launcher.

Buttons launch the existing ROS 2 rov_control.py in a terminal:
    Stonefish Simulation -> --mode sim
    Real ROV             -> --mode real
"""

import os
import shlex
import shutil
import subprocess
import tkinter as tk
from tkinter import messagebox


WORKSPACE = os.path.expanduser("~/Desktop/EGH490/ros2_ws")
ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WORKSPACE_SETUP = os.path.join(WORKSPACE, "install", "setup.bash")

PACKAGE = "stonefish_qut_rov"
EXECUTABLE = "rov_control.py"


def find_optional_venv():
    """
    Fish detection used the venv-yellow-tang environment previously.
    Try a few sensible locations. If none exists, continue without it.
    """
    candidates = [
        os.path.expanduser("~/Desktop/EGH490/ros2_ws/venv-yellow-tang/bin/activate"),
        os.path.expanduser("~/Desktop/EGH490/venv-yellow-tang/bin/activate"),
        os.path.expanduser("~/.venvs/venv-yellow-tang/bin/activate"),
        os.path.expanduser("~/venv-yellow-tang/bin/activate"),
    ]

    for path in candidates:
        if os.path.isfile(path):
            return path

    return None


def build_shell_command(mode: str) -> str:
    parts = [
        f"source {shlex.quote(ROS_SETUP)}",
        f"source {shlex.quote(WORKSPACE_SETUP)}",
    ]

    venv = find_optional_venv()
    if venv:
        parts.append(f"source {shlex.quote(venv)}")

    parts.append(
        f"ros2 run {shlex.quote(PACKAGE)} "
        f"{shlex.quote(EXECUTABLE)} --mode {shlex.quote(mode)}"
    )

    # Keep the terminal visible after the ROS stack exits.
    parts.append('echo')
    parts.append('echo "ROV control process ended. Press Enter to close this terminal."')
    parts.append('read')

    return " && ".join(parts)


def open_terminal(command: str):
    """Launch command in an available Linux terminal emulator."""
    if shutil.which("gnome-terminal"):
        subprocess.Popen([
            "gnome-terminal",
            "--",
            "bash",
            "-lc",
            command,
        ])
        return

    if shutil.which("x-terminal-emulator"):
        subprocess.Popen([
            "x-terminal-emulator",
            "-e",
            "bash",
            "-lc",
            command,
        ])
        return

    if shutil.which("konsole"):
        subprocess.Popen([
            "konsole",
            "-e",
            "bash",
            "-lc",
            command,
        ])
        return

    messagebox.showerror(
        "Terminal not found",
        "Could not find gnome-terminal, x-terminal-emulator, or konsole.",
    )


def launch_mode(mode: str):
    if not os.path.isfile(ROS_SETUP):
        messagebox.showerror(
            "ROS 2 not found",
            f"Could not find:\n{ROS_SETUP}",
        )
        return

    if not os.path.isfile(WORKSPACE_SETUP):
        messagebox.showerror(
            "Workspace not built",
            "Could not find the workspace setup file:\n\n"
            f"{WORKSPACE_SETUP}\n\n"
            "Run colcon build first.",
        )
        return

    if mode == "real":
        proceed = messagebox.askyesno(
            "Real ROV",
            "Start REAL ROV mode?\n\n"
            "This mode is intended to communicate with the physical ROV.",
            icon="warning",
        )
        if not proceed:
            return

    command = build_shell_command(mode)
    open_terminal(command)
    # Close the launcher GUI after starting the selected ROV mode
    root.destroy()


root = tk.Tk()
root.title("QUT ROV Control")
root.geometry("520x330")
root.resizable(False, False)

title = tk.Label(
    root,
    text="QUT ROV Control",
    font=("Arial", 24, "bold"),
)
title.pack(pady=(28, 5))

subtitle = tk.Label(
    root,
    text="Select the platform to control",
    font=("Arial", 12),
)
subtitle.pack(pady=(0, 25))

sim_button = tk.Button(
    root,
    text="Stonefish Simulation",
    font=("Arial", 16, "bold"),
    width=27,
    height=2,
    command=lambda: launch_mode("sim"),
)
sim_button.pack(pady=8)

real_button = tk.Button(
    root,
    text="Real ROV",
    font=("Arial", 16, "bold"),
    width=27,
    height=2,
    command=lambda: launch_mode("real"),
)
real_button.pack(pady=8)

footer = tk.Label(
    root,
    text="ROS 2 Jazzy • stonefish_qut_rov",
    font=("Arial", 9),
)
footer.pack(side="bottom", pady=14)

root.mainloop()
