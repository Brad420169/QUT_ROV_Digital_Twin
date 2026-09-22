#!/usr/bin/env bash
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source ~/Desktop/EGH490/ros2_ws/install/setup.bash

exec python3 "$SCRIPT_DIR/rov_launcher_gui.py"
