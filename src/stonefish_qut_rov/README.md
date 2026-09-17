# QUT ROV control stack

The active Python code is in `ros_nodes/modular_architecture/`. The shared
controller publishes normalized surge/heave/yaw effort on `/qut_rov/cmd_vel`;
these values are not velocities in metres/second. Simulation and MAVROS
interfaces translate this command into their respective actuator outputs.

## Physical ROV: copy-and-paste setup

Target: **Ubuntu 24.04, ROS 2 Jazzy**, a graphical desktop, and the existing
configured Pixhawk/ArduSub vehicle. These instructions do not flash or calibrate
a new Pixhawk. A Git clone copies code, **not PC network profiles, SSH keys,
camera settings, or autopilot configuration**.

Keep the vehicle disarmed, clear the gimbal's travel, and disconnect thruster
power for bench setup where practical. Do not run thrusters dry. Do not disable
arming checks to get past an error. Only arm in a suitable test environment.

### 1. One time on each PC: install ROS and tools

If `/opt/ros/jazzy/setup.bash` already exists, skip the ROS repository bootstrap
and proceed to the dependency installation. These commands require internet.
See the [official Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

```bash
# Check this reports Ubuntu 24.04 and that locale reports a UTF-8 locale.
cat /etc/os-release
locale

sudo apt update
sudo apt install -y software-properties-common curl ca-certificates
sudo add-apt-repository -y universe
ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

Install physical-stack dependencies, including the camera's ARM helper compiler:

```bash
sudo apt update
sudo apt install -y git openssh-client network-manager netcat-openbsd ffmpeg tcpdump \
  ros-jazzy-mavros ros-jazzy-mavros-msgs ros-jazzy-joy ros-jazzy-cv-bridge \
  python3-numpy python3-opencv python3-matplotlib python3-tk python3-pexpect \
  python3-pytest gcc-arm-linux-gnueabihf libc6-dev-armhf-cross
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
source /opt/ros/jazzy/setup.bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
```

The GeographicLib geoid data is required by MAVROS even for this underwater
vehicle; missing data can terminate MAVROS. See the
[MAVROS installation documentation](https://github.com/mavlink/mavros/blob/ros2/mavros/README.md).

### 2. Clone and build

Use this location on every PC for the commands below. If the clone already
exists, skip `git clone`; do not clone into an existing checkout or overwrite it.

```bash
mkdir -p "$HOME/Desktop/EGH490"
git clone --recurse-submodules https://github.com/Brad420169/QUT_ROV_Digital_Twin.git "$HOME/Desktop/EGH490/ros2_ws"
cd "$HOME/Desktop/EGH490/ros2_ws"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/stonefish_qut_rov --ignore-src -r -y --skip-keys stonefish_ros2
colcon build --symlink-install --packages-select stonefish_qut_rov
source install/setup.bash
ros2 pkg prefix stonefish_qut_rov
ls install/stonefish_qut_rov/lib/stonefish_qut_rov/gimbal_motor.*
```

The selected physical build does not need the Stonefish simulator. The
`stonefish_ros2` dependency is intentionally skipped here, not for simulation.
Use the system Python/ROS environment initially; an unrelated Conda/venv can
hide ROS or OpenCV packages. After source changes, rebuild and source again.

### 3. Wire and power the ROV

Connect the PC Ethernet adapter to the topside Fathom-X, both Fathom boards
through the tether, and the onboard Fathom-X to the M80 Ethernet switch.
The camera and configured Pixhawk Ethernet connection share the onboard switch.
Power both Fathom boards and the ROV electronics from their intended supplies;
wait for link LEDs and camera boot. Connect the gamepad to the PC.

Fathom-X transports Ethernet across the tether; it does not assign camera IP
addresses. The M80 switches Ethernet. Do not assume either device owns the
MAVLink endpoint address below, or that DHCP will configure this link.

| Device / service | Project configuration |
| --- | --- |
| PC tether adapter | `192.168.2.1/24` and `192.168.144.10/24` |
| Vehicle MAVLink endpoint | `192.168.2.2`, UDP destination port `14555` |
| PC MAVROS listener | UDP `14550` |
| Camera | `192.168.144.108` |
| Camera video | `rtsp://192.168.144.108/`, TCP `554` |
| Camera angle commands | TCP `2332` |
| Camera motor helper / temperature | SSH TCP `22`, user `root` |

### 4. One time: save the tether network profile

Identify the **tether USB Ethernet adapter**, not Wi-Fi or your internet LAN.
Adapter names differ between PCs. This PC used `enx00e04c680725`; do not blindly
reuse that name. Activating the profile below replaces the adapter's active
connection, so do not select the adapter carrying your remote login.

```bash
ip -br address
nmcli device status
nmcli connection show --active
read -rp "Tether Ethernet interface name: " ROV_IFACE

if nmcli connection show ROV-Tether >/dev/null 2>&1; then
  sudo nmcli connection modify ROV-Tether connection.interface-name "$ROV_IFACE" \
    ipv4.method manual ipv4.addresses "192.168.2.1/24,192.168.144.10/24" \
    ipv4.gateway "" ipv4.dns "" ipv4.never-default yes ipv6.method disabled
else
  sudo nmcli connection add type ethernet ifname "$ROV_IFACE" con-name ROV-Tether \
    ipv4.method manual ipv4.addresses "192.168.2.1/24,192.168.144.10/24" \
    ipv4.never-default yes ipv6.method disabled
fi
sudo nmcli connection modify ROV-Tether connection.autoconnect yes connection.autoconnect-priority 100
sudo nmcli connection up ROV-Tether

ip route get 192.168.144.108
ip route get 192.168.2.2
ping -c 3 -W 2 192.168.144.108
ping -c 3 -W 2 192.168.2.2
```

Camera routing should use the tether device and source `192.168.144.10`, vehicle
routing source `192.168.2.1`, with no home-router gateway. ICMP failure alone does
not prove MAVLink is unavailable. Keep Wi-Fi/internet on a separate subnet.
If another PC shares the ROV network simultaneously, give it different unused
host addresses and check the vehicle's telemetry destination configuration.

The profile persists across reboot/unplugging; normally no IP command is needed
again. If another wired profile wins, activate `ROV-Tether` explicitly. Replacing
the USB adapter may require updating the profile's interface name.

### 5. One time per PC: camera SSH access

Video does not require SSH, but this implementation uses SSH for motor
enable/disable and temperature. A working video feed does not prove the gimbal
is ready. First connect interactively and verify the camera's host-key identity
against a trusted record before accepting it:

```bash
ssh root@192.168.144.108
# Enter the camera password at the prompt. In the remote shell, run:
exit
```

Create this PC's own key only if one does not already exist. Never overwrite an
existing key or put passwords/private keys in Git. Enter the camera password
when `ssh-copy-id` prompts; this adds the public key without replacing other keys.

```bash
if [ ! -f "$HOME/.ssh/id_ed25519" ]; then
  ssh-keygen -t ed25519 -f "$HOME/.ssh/id_ed25519" -C rov-camera
fi
ssh-copy-id -i "$HOME/.ssh/id_ed25519.pub" root@192.168.144.108
ssh -o BatchMode=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=5 \
  root@192.168.144.108 'cat /sys/class/thermal/thermal_zone0/temp'
```

A value such as `62352` means 62.352 °C. The final command must succeed **without
a password prompt**. For a passphrase-protected key, unlock it in an SSH agent
in the terminal that will launch the stack:

```bash
# Only start an agent if this terminal does not already have a working one.
if [ -z "${SSH_AUTH_SOCK:-}" ]; then eval "$(ssh-agent -s)"; fi
ssh-add "$HOME/.ssh/id_ed25519"
```

Repeat the BatchMode test after unlocking. Do not run the stack with `sudo`:
that changes the SSH identity/home environment. Camera firmware updates or a
factory reset may require restoring authorized keys and verifying a new host
key. Do not disable host-key checking to conceal an identity mismatch.

### 6. Verify the video before starting the vehicle controller

```bash
nc -vz -w 3 192.168.144.108 22
nc -vz -w 3 192.168.144.108 554
nc -vz -w 3 192.168.144.108 2332
ffprobe -v error -rtsp_transport tcp -select_streams v:0 \
  -show_entries stream=codec_name,width,height,r_frame_rate -of default=noprint_wrappers=1 \
  rtsp://192.168.144.108/
ffplay -rtsp_transport tcp rtsp://192.168.144.108/
```

Press `q` to close ffplay before starting the stack. Port checks only establish
TCP reachability, not protocol health. `ffprobe` reports advertised stream
properties, not a reliable measurement of delivered FPS under thermal throttling.

The existing camera was configured for **1280×720, 24 FPS, H.264, about 2048 kbps,
OSD off and onboard AI tracking off**. Those are camera-side saved settings, not
settings installed by cloning or launching ROS. Verify a replacement/reset
camera separately. Do not change the HDMI output to 720p/24: that previously
crashed the video process. To inspect the camera configuration without changing it:

```bash
curl --fail --max-time 10 'http://192.168.144.108/get_config?chn=-1' | python3 -m json.tool
```

### 7. Every session: start the physical ROV

Power/connect the system, connect the gamepad, keep controls neutral, then:

```bash
cd "$HOME/Desktop/EGH490/ros2_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 run stonefish_qut_rov rov_control.py --mode real
```

Alternatively, from the same sourced environment, run the GUI and select real
mode. **Use one launcher, not both**:

```bash
ros2 run stonefish_qut_rov rov_launcher_gui.py
```

The launcher starts MAVROS, joystick, the real interface and control processes.
Its MAVROS URL is `udp://:14550@192.168.2.2:14555`. It waits up to 45 seconds for
joystick/depth/IMU, FCU connection, MANUAL mode and the arming service. The real
interface requests MANUAL and 30 Hz ATTITUDE/RAW_IMU/SCALED_PRESSURE telemetry.
It does not replace autopilot calibration, thruster mapping or failsafe setup.

Current controls: **A** arm/disarm; **Y** camera show/hide; left stick surge;
right stick yaw; triggers heave. Release all controls before startup/recovery.
Use Ctrl+C in the main terminal for normal shutdown; verify disarm rather than
assuming a cleanup request succeeded. Physical power isolation remains the
fallback if communication/control is lost.

Opening the camera window can move the gimbal: current defaults are **pitch
−90°, yaw +90°** in FPV mode. While visible, release the D-pad first, then use
up/down for pitch and left/right for roll; yaw stays fixed. Hiding the viewer
requests motor-off. Do not run a second gimbal-control program at the same time.
The helper is reuploaded after camera power cycles. Its firmware checksum guard
must not be bypassed after firmware changes.

Camera protection blocks vision control for missing temperature data, excessive
temperature or a slow/stale feed. The configured thermal threshold is 80 °C
(5 FPS thermal mode), with recovery below 75 °C and healthy frames. Allowing
vision does not automatically enable an autonomous mode. A successful camera
test is not authorization to test autonomous vehicle movement.

### 8. Troubleshooting: copy into a second terminal

First source the same workspace/domain in every diagnostic terminal:

```bash
cd "$HOME/Desktop/EGH490/ros2_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
```

**Pixhawk not connected / startup timed out:** while the main stack is running:

```bash
ip route get 192.168.2.2
ros2 topic echo /mavros/state --once
ros2 topic hz /mavros/imu/data
# Ctrl+C ends only this rate check; then run the next one.
ros2 topic hz /mavros/imu/static_pressure
ros2 topic hz /joy
tail -n 100 /tmp/mavros.log
```

Look for `connected: true`, MANUAL mode, and fresh sensor/gamepad messages.
If the launcher has already shut down after timeout, these topics will be absent.
For a standalone MAVROS connection test, stop the full stack first, then run:

```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@192.168.2.2:14555
```

Inspect `/mavros/state` in another sourced terminal; stop standalone MAVROS before
restarting the full stack. Never run two vehicle controllers/MAVROS instances.
This diagnostic alone does not request the full stack's sensor rates.
For wire-level inspection:

```bash
read -rp "Tether Ethernet interface name: " ROV_IFACE
sudo tcpdump -ni "$ROV_IFACE" 'udp port 14550 or udp port 14555'
```

If there is no return traffic, check wiring, the actual vehicle endpoint,
autopilot Ethernet/bridge configuration, and its PC telemetry destination. Check
`sudo ufw status verbose`; if a firewall is active, allow only the required
vehicle-to-PC UDP 14550 traffic on the tether adapter, rather than disabling the
firewall. A UDP `nc` probe is not proof of a MAVLink handshake. The current stack
uses Ethernet MAVLink, not a Pixhawk USB serial device; plugging in USB alone
does not satisfy this URL. Do not guess new firmware/network parameters.

**Video works but gimbal does not:**

```bash
ssh -o BatchMode=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=5 \
  root@192.168.144.108 'cat /sys/class/thermal/thermal_zone0/temp'
command -v arm-linux-gnueabihf-gcc
ls install/stonefish_qut_rov/lib/stonefish_qut_rov/gimbal_motor.*
nc -vz -w 3 192.168.144.108 2332
```

Fix key authorization/agent errors using step 5; install the compiler using step
1; rebuild missing installed helpers using step 2. Check the camera window is
visible and D-pad has been released. Unsupported firmware requires helper
revalidation, not bypassing the guard. These were the key new-PC issues found
on this system: missing SSH authorization and stale installed helper files.

**Camera unreachable / reconnecting:** rerun step 4's route checks and step 6's
port tests. Confirm both Fathom boards and the M80/camera have power/link. Try
`sudo nmcli connection up ROV-Tether` if the wrong profile is active. Check for
duplicate IPs, another interface/VPN using the same subnet, or a damaged cable.
An SSH problem does not explain a failed RTSP port, and vice versa.

**Camera hot / 5 FPS / vision disabled:**

```bash
ssh -o BatchMode=yes -o ConnectTimeout=5 root@192.168.144.108 \
  'cat /sys/class/thermal/thermal_zone0/temp'
ros2 topic echo /qut_rov/camera_health --once
ros2 topic echo /qut_rov/vision_control_allowed --once
```

Let it cool and check enclosure heat dissipation; do not defeat the guard.
Health topics require the camera node to be running. If telemetry is unknown,
fix SSH access. Low FPS can also be a network/decoder issue, not only heat.

**Gamepad / package not found:**

```bash
ls /dev/input/js*
ros2 topic echo /joy --once
ros2 pkg prefix stonefish_qut_rov
```

Reconnect/pair the controller if missing. Re-source `install/setup.bash` if the
package is missing or comes from the wrong workspace. Do not use `sudo` as a
workaround for GUI, ROS or SSH problems.

For read-only output monitoring with the normal stack running:

```bash
ros2 run stonefish_qut_rov pwm_monitor.py
```

For an explicit disarm request while MAVROS is connected:

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: false}'
ros2 topic echo /mavros/state --once
```

Verify `armed: false`; a disconnected service cannot make the vehicle safe.

## Simulation build and run

From the ROS workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/stonefish_qut_rov --ignore-src -r -y
colcon build --symlink-install --packages-select stonefish_qut_rov
source install/setup.bash
ros2 run stonefish_qut_rov rov_control.py --mode sim
# Or: ros2 run stonefish_qut_rov rov_control.py --mode real
```

Fish detection additionally requires `requirements-detector.txt` in the Python
environment used by the ROS executable (including access to ROS Python packages).
Its default device is GPU `0`; use `device:=cpu use_half:=false` as ROS parameters
when launching the detector directly on a machine without CUDA.

The optional GUI is available as `ros2 run stonefish_qut_rov rov_launcher_gui.py`.
The source-tree `launch_rov_gui.sh` also works. The GUI discovers the workspace
from its location; set `ROV_WORKSPACE` if using a relocated installation.

To launch only Stonefish with the pool environment:

```bash
ros2 launch stonefish_qut_rov launch_rov.py environment:=pool_environment
```

## Input loss and recovery

- The launcher waits up to 45 seconds for joystick, depth and IMU telemetry before
  starting teleop. It supervises the simulator/MAVROS, joystick, interface and
  teleop processes. An essential process exit stops the remaining stack.
- The joystick driver publishes at 20 Hz even when a stick is held still.
- Startup and input-loss recovery require neutral sticks/triggers and released
  buttons/D-pad. Then manual control is enabled. Autonomous modes must be
  explicitly selected again after an input-loss event.
- Joystick freshness is required in every mode. Depth hold additionally requires
  fresh depth; trajectory mode requires depth and IMU; fish following requires
  fresh follower commands. A missing required input cancels the mode and commands
  neutral; neutral means zero commanded thrust, not active station keeping.
- Both actuator interfaces independently force neutral after a command timeout.
  Non-finite commands are rejected. A stopped camera/inference stream therefore
  cannot leave its last movement command latched through teleop.
- Timeouts are monotonic, with 0.5-second defaults in `rov_config.py`. Detector
  results older than the fish command timeout, or belonging to an earlier enable
  session, are discarded. Slow inference may require measured timeout tuning.
- SIGINT, SIGTERM, SIGHUP and gamepad shutdown use one cleanup path before the ROS
  context closes. The real interface requests disarm and waits briefly for its
  acknowledgement; lack of acknowledgement is reported explicitly.

## Code ownership

- `teleop_controller.py`: joystick input, mode selection and high-level control.
- `sim_interface.py`, `real_interface.py`: actuator and sensor adapters.
- `command_watchdog.py`: shared input freshness checks.
- `node_lifecycle.py`: control-node signal handling and cleanup.
- `rov_control.py`, `process_utils.py`: stack supervision and owned process groups.
- `viewer_manager.py`: optional camera/plotter lifecycle, outside control callbacks.
- `battery_status.py`: operator battery formatting.
- `fish_detector_follower.py`: perception and following, enabled only by teleop.

Legacy scripts are preserved in `archive/legacy_ros_nodes/` and are not installed.
Generated build/install/log directories belong at the workspace root, not inside
Python source directories.

## Regression tests

### Physical ROV PWM monitor

With the regular physical ROV stack running, open another terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
python3 /home/brad/Desktop/EGH490/ros2_ws/src/stonefish_qut_rov/ros_nodes/modular_architecture/pwm_monitor.py
```

Use the same ROS_DOMAIN_ID as the control stack. The monitor displays the latest
requested `channels[2]`, `[3]`, `[4]` alongside FCU-reported output PWM for motors
1–4, in microseconds, twice per second. Samples are independently received, not
paired acknowledgements. These outputs are not measured RPM or thrust. Missing
data shows WAITING; samples older than two seconds show STALE. If output stays
WAITING, check that MAVROS publishes `/mavros/rc/out` with its `rc_io` plugin enabled
and that the FCU is streaming SERVO_OUTPUT_RAW. The monitor sends no commands and
does not change telemetry rates. Ctrl+C closes only the monitor.

After rebuilding the package, `ros2 run stonefish_qut_rov pwm_monitor.py` also works.
For a different MAVROS namespace, append
`--ros-args -p mavros_namespace:=/your_mavros_namespace`.

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest src/stonefish_qut_rov/tests
# After building:
colcon test --packages-select stonefish_qut_rov
colcon test-result --verbose
```

Tests use ROS domain 213, mocked actuator publishers and mocked detector inference;
they do not start Stonefish, open viewers or connect to the ROV. They exercise
input expiry/recovery, invalid inputs, follower shutdown, stale inference results,
process supervision and node cleanup. These do not replace a live simulation
check or physical-vehicle validation.
