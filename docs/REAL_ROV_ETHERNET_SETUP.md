# Physical ROV Ethernet setup

Persistent project setup notes, recorded 24 September 2026 for reuse in a public
repository README. This documents the current QUT ROV configuration, not factory
defaults for every Pixhawk or camera.

## Network layout

The laptop reaches the vehicle MAVLink endpoint and camera through the tether
Ethernet adapter. Put two IPv4 addresses on that same adapter:

| Purpose | Laptop address | Device address | Protocol / port |
| --- | --- | --- | --- |
| Pixhawk-side MAVLink connection | `192.168.2.1/24` | `192.168.2.2` | Laptop UDP 14550; remote UDP 14555 |
| Camera video | `192.168.144.10/24` | `192.168.144.108` | RTSP TCP 554 |
| Camera gimbal angle control | same camera-subnet address | same camera address | TCP 2332 |
| Camera motor helper and temperature | same camera-subnet address | same camera address | SSH TCP 22 |

`192.168.2.2` is the configured vehicle-side network endpoint for MAVLink. Do not
assume it is an IP address configured directly on the Pixhawk: the exact onboard
Ethernet/serial bridge and its configuration have not been inspected here.

Use mask `255.255.255.0` for both laptop addresses. Leave Ethernet gateway and DNS
blank and prevent this connection becoming the default route. Wi-Fi can retain
internet access. No routing between these two subnets is needed on the laptop.
Use unique laptop addresses if more than one computer is attached; changing the
MAVLink host address may also require changing onboard forwarding.

## Set up Ubuntu NetworkManager

Identify the USB Ethernet adapter rather than copying its name blindly:

```bash
ip -br address
nmcli device status
nmcli connection show
```

On the tested laptop it is `enx00e04c680725`. The initial DHCP-based connection
never acquired an IPv4 address, and both device routes went through Wi-Fi.
A dedicated static profile fixed device reachability.

Create the following profile once (replace the adapter name on another PC):

```bash
sudo nmcli connection add type ethernet \
  con-name Real_ROV_Tether ifname enx00e04c680725 \
  ipv4.method manual \
  ipv4.addresses '192.168.2.1/24,192.168.144.10/24' \
  ipv4.never-default yes \
  ipv6.method link-local \
  connection.autoconnect no
sudo nmcli connection up Real_ROV_Tether
```

If `Real_ROV_Tether` already exists, do not create another copy. Inspect it with
`nmcli connection show Real_ROV_Tether`, then activate it. This profile is already
saved on the tested laptop. It leaves other saved Ethernet profiles intact.

Daily activation after plugging in the tether:

```bash
nmcli connection up Real_ROV_Tether
```

The saved profile has autoconnect disabled. For an adapter dedicated to the ROV,
optional automatic activation can be enabled with:

```bash
sudo nmcli connection modify Real_ROV_Tether \
  connection.autoconnect yes connection.autoconnect-priority 100
```

An unplug/replug or reboot may otherwise leave Ethernet without these addresses.
The ROV launcher does not currently activate the NetworkManager profile itself.

## Check routes and physical connectivity

```bash
ip -br address
ip route get 192.168.2.2
ip route get 192.168.144.108
ping -c 3 -W 2 192.168.2.2
ping -c 3 -W 2 192.168.144.108
```

Both routes should name the tether adapter. The respective source addresses
should be `192.168.2.1` and `192.168.144.10`, with no Wi-Fi gateway hop.
If Ethernet has no carrier, check vehicle power, USB adapter, tether and physical
links. If it has carrier but no IPv4 address, activate the static profile.

If a firewall is enabled, its rules must permit the MAVLink traffic to local UDP
14550 and connections to the camera ports above. Scope any rules to the tether
interface/device addresses; do not disable the firewall as a blanket fix.

## Pixhawk-side MAVLink setup and verification

The application uses this URL in `ros_nodes/common/rov_config.py`:

```text
udp://:14550@192.168.2.2:14555
```

MAVROS binds local UDP port 14550 and uses remote endpoint `192.168.2.2:14555`.
The vehicle-side bridge must forward Pixhawk MAVLink telemetry to laptop
`192.168.2.1:14550` and accept the return traffic at its configured remote port.
The bridge model, UART settings and onboard forwarding UI remain unverified;
inspect the actual vehicle before documenting specific bridge configuration steps.
A successful ping alone does not confirm this forwarding or Pixhawk telemetry.

After installing ROS Jazzy, MAVROS and its GeographicLib datasets, use a separate
terminal to test the connection without starting the driving controller:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@192.168.2.2:14555
```

In another ROS-sourced terminal:

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/imu/data --once --qos-reliability best_effort
ros2 topic echo /mavros/imu/static_pressure --once --qos-reliability best_effort
```

Look for `connected: true` and current sensor samples. Sensor stream rates depend
on the vehicle configuration; the project's real interface requests its required
streams during normal startup. Do not run a second MAVROS instance on UDP 14550.
Stop the diagnostic MAVROS instance with Ctrl+C before starting the normal stack.
These checks require no arming or movement commands.

## Camera video and SSH access

The configured stream URL is `rtsp://192.168.144.108:554/`. Reachable port 554 is
only a connectivity check; decoding frames is needed to verify video operation.
Video can work while gimbal control fails: the current motor start/stop helper
and temperature polling require SSH, in addition to TCP angle control on 2332.

Each operating PC needs trusted, noninteractive SSH access to the camera:

```bash
ssh root@192.168.144.108
```

Verify the camera host fingerprint against a trusted record/device before
accepting it, then exit the remote shell. Do not bypass host-key checking.
Use an existing appropriate SSH key, or create one if the path is unused:

```bash
# Run only if this key does not already exist; never overwrite an existing key.
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -C rov-camera
ssh-copy-id -i ~/.ssh/id_ed25519.pub root@192.168.144.108
ssh -o BatchMode=yes -o StrictHostKeyChecking=yes root@192.168.144.108 \
  'cat /sys/class/thermal/thermal_zone0/temp'
```

Enter the camera password interactively when requested. Passphrase-protected keys
must be unlocked in an SSH agent available to the desktop launcher. Private keys,
passwords and a user's SSH directory must not be distributed with the repository.
The returned temperature is in millidegrees Celsius.

Host dependencies for the camera helper include:

```bash
sudo apt install openssh-client gcc-arm-linux-gnueabihf libc6-dev-armhf-cross \
  python3-pexpect python3-opencv python3-numpy
```

Keep `gimbal_motor.py` and `gimbal_motor.c` in the installation. The helper builds
and uploads a temporary camera executable automatically and checks a supported
firmware checksum. Firmware changes require revalidation; do not bypass the guard.
See the existing camera/gimbal guide for full operation details.

## Verified observations and outstanding checks

On 23 September 2026, after activating the static profile:

- Both device addresses answered ping (2/2 replies each).
- Camera TCP ports 22, 554 and 2332 accepted connections.
- Camera temperature SSH failed because this PC had no trusted ED25519 host key
  for the camera. Noninteractive authentication and temperature retrieval were
  therefore not verified.
- ROS dependencies installed and all seven real ROV node imports passed after
  adding `ros-jazzy-tf-transformations`.
- No retained telemetry result establishes Pixhawk heartbeat/IMU/pressure success.
  Live video decoding and gimbal operation are also not yet verified.

On 24 September 2026 the profile remained saved but inactive, and the Ethernet
adapter had no IP addresses. Activate it again before attempting the above checks.

These observations establish Ethernet reachability, not end-to-end readiness or
permission to arm the vehicle. Existing thermal-threshold and D-pad direction test
mismatches are tracked separately in the standalone workspace VALIDATION.md.
