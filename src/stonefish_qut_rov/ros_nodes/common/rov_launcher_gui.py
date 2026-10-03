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
from pathlib import Path
import tkinter as tk
from tkinter import messagebox


WORKSPACE = os.environ.get("ROV_WORKSPACE") or next(
    (str(parent) for parent in Path(__file__).resolve().parents
     if (parent / "install/setup.bash").is_file()), "")
ROS_SETUP = f"/opt/ros/{os.environ.get('ROS_DISTRO', 'jazzy')}/setup.bash"
WORKSPACE_SETUP = os.path.join(WORKSPACE, "install", "setup.bash")

PACKAGE = "stonefish_qut_rov"
EXECUTABLE = "rov_control.py"


def find_optional_venv():
    """
    Fish detection used the venv-yellow-tang environment previously.
    Try a few sensible locations. If none exists, continue without it.
    """
    candidates = [
        os.path.join(WORKSPACE, "venv-yellow-tang/bin/activate"),
        str(Path(WORKSPACE).parent / "venv-yellow-tang/bin/activate"),
        os.path.expanduser("~/.venvs/venv-yellow-tang/bin/activate"),
        os.path.expanduser("~/venv-yellow-tang/bin/activate"),
    ]

    for path in candidates:
        if os.path.isfile(path):
            return path

    return None


def build_shell_command(mode: str, rov_scenario: str = "real_rov.scn") -> str:
    parts = [
        f"source {shlex.quote(ROS_SETUP)}",
        f"source {shlex.quote(WORKSPACE_SETUP)}",
    ]

    venv = find_optional_venv()
    if venv:
        parts.append(f"source {shlex.quote(venv)}")

    scenario_arg = f" --rov-scenario {shlex.quote(rov_scenario)}" if mode == "sim" else ""
    parts.append(
        f"ros2 run {shlex.quote(PACKAGE)} "
        f"{shlex.quote(EXECUTABLE)} --mode {shlex.quote(mode)}{scenario_arg}"
    )

    # Setup failures and nonzero stack exits must also leave diagnostics visible.
    return (
        " && ".join(parts)
        + '; rov_exit_code=$?; printf "\\nROV control process ended (exit code %s). '
          'Press Enter to close this terminal.\\n" "$rov_exit_code"; '
          'read -r; exit "$rov_exit_code"'
    )


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


def launch_mode(mode: str, root, rov_scenario: str = "real_rov.scn"):
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

    command = build_shell_command(mode, rov_scenario)
    open_terminal(command)
    # Close the launcher GUI after starting the selected ROV mode
    root.destroy()


def build_window(root):
    """Build the launcher without starting a vehicle or entering the event loop."""
    from tkinter import ttk

    bg, panel, border = "#071d2b", "#102e40", "#24495d"
    text, muted, accent = "#edf8fc", "#adc8d6", "#38d6cb"
    root.title("QUT ROV • Mission launcher")
    root.geometry("700x610")
    root.minsize(680, 600)
    root.configure(bg=bg)
    root.option_add("*Font", ("DejaVu Sans", 10))
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("Marine.TCombobox", fieldbackground=bg, background=border,
                    foreground=text, arrowcolor=accent, padding=8)
    style.map("Marine.TCombobox", fieldbackground=[("readonly", bg)],
              foreground=[("readonly", text)], selectbackground=[("readonly", bg)],
              selectforeground=[("readonly", text)])
    root.option_add("*TCombobox*Listbox.background", panel)
    root.option_add("*TCombobox*Listbox.foreground", text)
    root.option_add("*TCombobox*Listbox.selectBackground", "#166a80")

    body = tk.Frame(root, bg=bg)
    body.pack(fill="both", expand=True, padx=28, pady=24)
    tk.Label(body, text="QUT  /  UNDERWATER ROBOTICS", bg=bg, fg=accent,
             font=("DejaVu Sans", 10, "bold")).pack(anchor="w")
    tk.Label(body, text="ROV mission control", bg=bg, fg=text,
             font=("DejaVu Sans", 25, "bold")).pack(anchor="w", pady=(8, 4))
    tk.Label(body, text="Choose your platform and start exploring.",
             bg=bg, fg=muted).pack(anchor="w", pady=(0, 22))
    scenario = tk.StringVar(root, value="real_rov.scn")

    # Resolve installed assets first, with a source-tree fallback for direct runs.
    try:
        from ament_index_python.packages import get_package_share_directory
        icons = Path(get_package_share_directory(PACKAGE)) / "icons"
    except (ImportError, LookupError):
        icons = Path(__file__).resolve().parents[2] / "icons"
    root.launcher_images = []

    def card(mode, title, description, button_text, filename):
        frame = tk.Frame(body, bg=panel, highlightbackground=border, highlightthickness=1)
        frame.pack(fill="x", pady=(0, 14))
        photo_frame = tk.Frame(frame, bg=panel, width=176, height=132)
        photo_frame.pack(side="left", padx=16, pady=18)
        photo_frame.pack_propagate(False)
        try:
            photo = tk.PhotoImage(master=root, file=str(icons / filename))
            root.launcher_images.append(photo)
            tk.Label(photo_frame, image=photo, bg=panel).pack(expand=True)
        except tk.TclError:
            tk.Label(photo_frame, text="SIM" if mode == "sim" else "ROV",
                     font=("DejaVu Sans", 24, "bold"), bg=panel, fg=accent).pack(expand=True)
        content = tk.Frame(frame, bg=panel)
        content.pack(side="left", fill="both", expand=True, padx=(0, 18), pady=16)
        tk.Label(content, text=title, bg=panel, fg=text,
                 font=("DejaVu Sans", 16, "bold")).pack(anchor="w")
        tk.Label(content, text=description, bg=panel, fg=muted,
                 justify="left", wraplength=370).pack(anchor="w", pady=(4, 10))
        if mode == "sim":
            row = tk.Frame(content, bg=panel)
            row.pack(fill="x", pady=(0, 10))
            tk.Label(row, text="Model", bg=panel, fg=muted).pack(side="left", padx=(0, 10))
            ttk.Combobox(row, textvariable=scenario, state="readonly", width=20,
                         style="Marine.TCombobox", values=("real_rov.scn", "rov_v1.scn",
                         "rov_v2.scn", "rov_v3.scn", "rov_v4.scn")).pack(side="left", fill="x", expand=True)
        tk.Button(content, text=button_text, command=lambda: launch_mode(mode, root, scenario.get()),
                  bg="#11718b" if mode == "sim" else "#16556c", fg=text,
                  activebackground="#2096a8", activeforeground="white",
                  relief="flat", borderwidth=0, padx=16, pady=10, cursor="hand2",
                  highlightthickness=2, highlightbackground=panel, highlightcolor=accent,
                  font=("DejaVu Sans", 11, "bold")).pack(anchor="w", fill="x")
        return frame

    card("sim", "Stonefish simulation", "Coral reef • Virtual vehicle & sensors",
         "Launch simulation  →", "launcher_sim.png")
    card("real", "Real ROV", "Connect to the physical vehicle.\nConfirmation required before launch.",
         "Connect to real ROV  →", "launcher_real.png")
    tk.Label(body, text="ROS 2 JAZZY     /     QUT ROV", bg=bg, fg=muted,
             font=("DejaVu Sans", 9)).pack(side="bottom", anchor="w", pady=(8, 0))
    return scenario


def main():
    root = tk.Tk()
    build_window(root)
    root.mainloop()


if __name__ == "__main__":
    main()
