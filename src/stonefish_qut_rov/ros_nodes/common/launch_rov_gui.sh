#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ROV_WORKSPACE="${ROV_WORKSPACE:-$(cd -- "$SCRIPT_DIR/../../../.." && pwd)}"

for setup in "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" "$ROV_WORKSPACE/install/setup.bash"; do
    if [[ ! -f "$setup" ]]; then
        echo "Missing $setup. Follow the workspace README.md installation steps." >&2
        exit 1
    fi
    source "$setup"
done

exec /usr/bin/python3 "$SCRIPT_DIR/rov_launcher_gui.py"
