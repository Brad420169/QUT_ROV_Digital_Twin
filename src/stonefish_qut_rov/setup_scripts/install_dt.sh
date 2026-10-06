#!/usr/bin/env bash
# Add Stonefish and the DT to the workspace prepared by install_software.sh.
set -eo pipefail
PACKAGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd -- "$PACKAGE_DIR/../.." && pwd)"
if [[ "${1:-}" == --help ]]; then
    echo "Usage: bash src/stonefish_qut_rov/setup_scripts/install_dt.sh"
    echo "Install Stonefish, stonefish_ros2, DT dependencies and the ROV shortcut."
    echo "Run install_software.sh first. Optional: ROV_BUILD_JOBS=1 for low-memory PCs."
    exit 0
fi
[[ $# == 0 ]] || { echo "Unexpected argument; use --help." >&2; exit 1; }
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 || "$EUID" == 0 ]]; then
    echo "Run as your normal desktop user on Ubuntu 24.04, without sudo." >&2
    exit 1
fi
if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$ROOT/install/stonefish_qut_rov/share/stonefish_qut_rov/package.xml" ]]; then
    echo "Run src/stonefish_qut_rov/setup_scripts/install_software.sh first." >&2
    exit 1
fi
if [[ -n "${VIRTUAL_ENV:-}" || -n "${CONDA_PREFIX:-}" ]]; then
    echo "Open a fresh terminal outside any Python virtual environment and rerun." >&2
    exit 1
fi
if [[ ! "${ROV_BUILD_JOBS:-2}" =~ ^[1-9][0-9]*$ ]]; then
    echo "ROV_BUILD_JOBS must be a positive integer." >&2
    exit 1
fi

# Both pinned upstream revisions declare version 1.6.0.
STONEFISH_REV=b21eb8e194c570ff2f61e91aeffb38d73dc25f42
ROS2_REV=6646e7ac25eed982f37f807abc7b545dbd2d6648
checkout_dependency() {
    local url="$1" directory="$2" revision="$3"
    if [[ ! -e "$directory" ]]; then
        git clone "$url" "$directory"
        git -C "$directory" checkout --detach "$revision"
    fi
    if [[ "$(git -C "$directory" rev-parse HEAD)" != "$revision" ]] || \
       [[ -n "$(git -C "$directory" status --porcelain --untracked-files=no)" ]]; then
        echo "Existing checkout at $directory differs from required revision $revision." >&2
        echo "Preserve it and move it aside before rerunning; no existing checkout was changed." >&2
        exit 1
    fi
}

sudo apt-get update
sudo apt-get install -y git build-essential cmake pkg-config \
    libglm-dev libsdl2-dev libfreetype6-dev libgl1-mesa-dev libpcl-dev mesa-utils \
    python3-venv python3-pip python3-rosdep python3-colcon-common-extensions
mkdir -p "$ROOT/.dependencies" "$ROOT/src"
touch "$ROOT/.dependencies/COLCON_IGNORE"
checkout_dependency https://github.com/patrykcieslak/stonefish.git \
    "$ROOT/.dependencies/stonefish" "$STONEFISH_REV"
checkout_dependency https://github.com/patrykcieslak/stonefish_ros2.git \
    "$ROOT/src/stonefish_ros2" "$ROS2_REV"

export CMAKE_BUILD_PARALLEL_LEVEL="${ROV_BUILD_JOBS:-2}"
export MAKEFLAGS="-j${ROV_BUILD_JOBS:-2}"
cmake -S "$ROOT/.dependencies/stonefish" -B "$ROOT/.dependencies/stonefish/build" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build "$ROOT/.dependencies/stonefish/build" --parallel "$CMAKE_BUILD_PARALLEL_LEVEL"
sudo cmake --install "$ROOT/.dependencies/stonefish/build"
sudo ldconfig

source /opt/ros/jazzy/setup.bash
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update --rosdistro jazzy
# Upstream declares "pcl", which has no rosdep key; libpcl-dev is installed above.
rosdep install --from-paths "$PACKAGE_DIR" "$ROOT/src/stonefish_ros2" \
    --ignore-src --rosdistro jazzy --skip-keys pcl -y
cd "$ROOT"
colcon build --symlink-install --packages-up-to stonefish_qut_rov \
    --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Keep ROS system packages visible, with a NumPy ABI compatible with Jazzy.
/usr/bin/python3 -m venv --system-site-packages "$ROOT/venv-yellow-tang"
touch "$ROOT/venv-yellow-tang/COLCON_IGNORE"
"$ROOT/venv-yellow-tang/bin/python" -m pip install \
    'numpy<2' 'opencv-python<4.12' -r "$PACKAGE_DIR/requirements-detector.txt"
source "$ROOT/install/setup.bash"
"$ROOT/venv-yellow-tang/bin/python" - "$PACKAGE_DIR/models/stonefish_yolo.pt" <<'PY'
import sys
import rclpy, tkinter, cv2, cv_bridge, tf_transformations
import numpy as np
from ament_index_python.packages import get_package_prefix
from ultralytics import YOLO
print('Simulator package:', get_package_prefix('stonefish_ros2'))
YOLO(sys.argv[1]).predict(np.zeros((320, 320, 3), dtype=np.uint8), device='cpu', verbose=False)
print('ROS, GUI, camera bridge and fish detector checks passed.')
PY
/usr/bin/python3 "$PACKAGE_DIR/setup_scripts/create_shortcut.py"
echo "DT installation complete. Open ROV and select Stonefish simulation."
echo "Check glxinfo -B for OpenGL 4.3 or newer before launching."
echo "No simulator or vehicle control was started."
