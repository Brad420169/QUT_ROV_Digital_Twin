#!/usr/bin/env bash
set -e

GUI="$HOME/Desktop/EGH490/ros2_ws/src/stonefish_qut_rov/ros_nodes/modular_architecture/rov_launcher_gui.py"

if [ ! -f "$GUI" ]; then
    echo "ROV GUI not found:"
    echo "$GUI"
    read -p "Press Enter to close..."
    exit 1
fi

python3 "$GUI"
