#!/usr/bin/env bash
set -eo pipefail
PACKAGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd -- "$PACKAGE_DIR/../.." && pwd)"
if [[ "${1:-}" == --help ]]; then
    echo "Usage: bash src/stonefish_qut_rov/setup_scripts/install_software.sh"
    exit 0
fi
if [[ $# != 0 ]]; then
    echo "Usage: bash src/stonefish_qut_rov/setup_scripts/install_software.sh" >&2
    exit 1
fi
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
    echo "Real ROV requires Ubuntu 24.04 Desktop." >&2
    exit 1
fi
if [[ "$EUID" == 0 ]]; then
    echo "Run as your desktop user, without sudo. Administrator access is requested when needed." >&2
    exit 1
fi

install_software() {
    if [[ -n "${VIRTUAL_ENV:-}" || -n "${CONDA_PREFIX:-}" ]]; then
        echo "Open a fresh terminal outside any Python virtual environment and rerun." >&2
        exit 1
    fi
    sudo -v
    sudo apt-get update
    sudo apt-get install -y software-properties-common curl ca-certificates python3 locales
    sudo add-apt-repository -y universe
    export LANG=C.UTF-8
    export LC_ALL=C.UTF-8
    if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
        ROV_INSTALL_TMP="$(mktemp -d)"
        trap 'rm -rf -- "$ROV_INSTALL_TMP"' EXIT
        curl --fail --silent --show-error --location \
          https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
          -o "$ROV_INSTALL_TMP/release.json"
        ROV_APT_VERSION="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tag_name"])' "$ROV_INSTALL_TMP/release.json")"
        if [[ ! "$ROV_APT_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
            echo "Unexpected ROS apt source version: $ROV_APT_VERSION" >&2
            exit 1
        fi
        curl --fail --show-error --location \
          "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROV_APT_VERSION}/ros2-apt-source_${ROV_APT_VERSION}.noble_all.deb" \
          -o "$ROV_INSTALL_TMP/ros2-apt-source.deb"
        sudo dpkg -i "$ROV_INSTALL_TMP/ros2-apt-source.deb"
    fi
    sudo apt-get update
    # The ROV has its own GUI; RViz and simulation demos are not required.
    sudo apt-get install -y \
      ros-jazzy-ros-base build-essential cmake git \
      python3-colcon-common-extensions python3-pytest \
      ros-jazzy-ament-cmake ros-jazzy-ament-cmake-pytest \
      ros-jazzy-launch ros-jazzy-launch-ros ros-jazzy-ament-index-python \
      ros-jazzy-tf-transformations ros-jazzy-rclpy ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs ros-jazzy-std-msgs \
      ros-jazzy-mavros ros-jazzy-mavros-msgs ros-jazzy-joy ros-jazzy-cv-bridge \
      python3-numpy python3-opencv python3-pexpect python3-matplotlib python3-tk \
      network-manager xdg-user-dirs libglib2.0-bin gnome-terminal \
      openssh-client gcc-arm-linux-gnueabihf libc6-dev-armhf-cross
    DATASET_INSTALLER=/opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
    if [[ -f "$DATASET_INSTALLER" ]]; then
        sudo bash "$DATASET_INSTALLER"
    else
        echo "MAVROS GeographicLib dataset installer missing: $DATASET_INSTALLER" >&2
        exit 1
    fi

    export ROS_DISTRO=jazzy
    source /opt/ros/jazzy/setup.bash
    cd "$ROOT"
    # This package can build without Stonefish; real mode never launches it.
    colcon build --symlink-install --packages-select stonefish_qut_rov \
        --cmake-args -DBUILD_TESTING=OFF
}

printf '\nInstalling Real ROV software. Keep internet connected.\n'
install_software
/usr/bin/python3 "$PACKAGE_DIR/setup_scripts/create_shortcut.py"
source "/opt/ros/jazzy/setup.bash"
source "$ROOT/install/setup.bash"
PYTHONDONTWRITEBYTECODE=1 python3 -c 'import rclpy, tkinter, cv2, tf_transformations; import teleop_controller, real_interface, camera_viewer_rtsp'
printf '\nInstallation complete. Open ROV from the desktop or applications menu.\n'
printf 'After connecting and powering the ROV, run: bash "%s/src/stonefish_qut_rov/setup_scripts/setuphardware.sh"\n' "$ROOT"
echo "No vehicle control was started."
