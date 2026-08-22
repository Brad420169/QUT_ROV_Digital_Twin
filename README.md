# SubbyROV — Digital Twin to Real Vehicle Control Stack

ROS 2 control stack for the QUT SubbyROV underwater vehicle, built so that
the **same controller code drives both a Stonefish digital twin and the real
ROV**. Only the interface layer changes between them.

Part of EGH490 Research Project, QUT.

---

## Architecture

Everything hinges on one abstraction: the high-level controller only ever
publishes vehicle-level commands to `/qut_rov/cmd_vel`, and only ever reads
sensors from `/qut_rov/depth` and `/qut_rov/imu`. It has no idea whether it
is driving a simulation or a real vehicle.

<p align="center">
  <img src="src/stonefish_qut_rov/icons/sim_to_real_interface.png" 
       alt="SubbyROV Digital Twin to Real Vehicle System Architecture" 
       width="100%">
</p>

### Nodes

| Node | Role |
|---|---|
| `rov_control.py` | Launcher. Prompts for sim or real, starts the stack, tears it down cleanly |
| `rov_launcher_gui.py` | Optional GUI wrapper around the launcher |
| `teleop_controller.py` | Gamepad input, depth hold, trajectory mode. Mode-agnostic |
| `sim_interface.py` | Translates `cmd_vel` to Stonefish thruster setpoints |
| `real_interface.py` | Translates `cmd_vel` to MAVROS RC override; bridges Bar30 and IMU |
| `thruster_mixer.py` | Surge/heave/yaw to four-thruster allocation (**sim only**) |
| `pid_controller.py` | PID with derivative-on-measurement and integral clamping |
| `imu_depth_plotter.py` | Live depth and roll/pitch/yaw plot. Works in both modes |
| `camera_viewer_rtsp.py` | RTSP viewer for the real camera (real mode) |
| `fish_detector_follower.py` | YOLO fish detection and visual servoing (**sim only**) |
| `rov_config.py` | All topics, gains, button mappings, and sim-to-real scalers |

### Thrust allocation is not shared

`thruster_mixer.py` runs in **simulation only**. On the real vehicle,
vehicle-level commands go straight to ArduSub's RC channels and the
Pixhawk's SimpleROV-4 mixer does the allocation.

This is deliberate but worth stating plainly: our mixer is a *model* of
ArduSub's mixer, not the same code. Allocation fidelity is a known
sim-to-real gap.

---

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Stonefish + `stonefish_ros2` (simulation only)
- MAVROS (real vehicle only)
- A gamepad (developed against a Zikway HID / GameSir-style controller)

### System packages

```bash
sudo apt install \
    ros-jazzy-mavros ros-jazzy-mavros-extras \
    ros-jazzy-joy \
    ffmpeg python3-tk
```

MAVROS needs the GeographicLib datasets installed once:

```bash
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

### Python packages

```bash
pip install opencv-python matplotlib ultralytics
```

`ultralytics` is only needed for `fish_detector_follower.py` (sim mode).

---

## Build

```bash
cd ~/ros2_ws/src
git clone <this-repo> stonefish_qut_rov

cd ~/ros2_ws/src/stonefish_qut_rov/ros_nodes/modular_architecture
chmod +x *.py

cd ~/ros2_ws
colcon build --packages-select stonefish_qut_rov
source install/setup.bash
```

**The `chmod +x` matters.** `ros2 run` silently reports *"No executable
found"* for a script without the executable bit, which looks like a missing
file rather than a permissions problem. If you add a new node, add it to the
`install(PROGRAMS ...)` block in `CMakeLists.txt` — not `install(FILES ...)`,
which strips the exec bit.

---

## Host network setup (real vehicle only)

**This is not in the repo and must be done on every new machine.** Two
subnets run over the single Fathom tether link:

| Device | Address |
|---|---|
| Your computer (ROV subnet) | `192.168.2.1` |
| Pixhawk 6X | `192.168.2.2` |
| Your computer (camera subnet) | `192.168.144.1` |
| Z-1Mini camera | `192.168.144.108` |

Find your tether interface — the name differs per machine, so don't copy
one from elsewhere:

```bash
ip -br addr
```

Then add both addresses:

```bash
sudo ip addr add 192.168.2.1/24   dev <interface>
sudo ip addr add 192.168.144.1/24 dev <interface>
```

These are lost on reboot. To make them stick:

```bash
nmcli con show
nmcli con mod "<connection>" +ipv4.addresses 192.168.144.1/24
nmcli con up "<connection>"
```

**Symptom of a missing camera alias:** MAVROS connects fine and the vehicle
drives, but the camera viewer loops on `No route to host`. The camera is on
the subnet you forgot.

---

## Running

```bash
ros2 run stonefish_qut_rov rov_control.py
```

Then choose `s` for simulation or `r` for the real vehicle. The launcher
handles startup ordering and shuts the whole stack down on Ctrl+C.

MAVROS output goes to `/tmp/mavros.log` rather than the terminal, since its
plugin banner otherwise buries everything else during startup:

```bash
tail -f /tmp/mavros.log
```

### Controls

| Input | Action |
|---|---|
| Left stick up/down | Surge |
| Right stick left/right | Yaw |
| RT / LT | Heave up / down |
| L bumper | Depth keeping |
| R bumper | Trajectory mode |
| Y button | Camera / detector |
| X button | Fish follow (sim only) |
| D-pad up | Depth / IMU plotter |

---

## Sim-to-real transfer

Every constant in `rov_config.py` is the **simulation** value and stays
canonical — the digital twin is the reference. A separate block of scalers
adapts those values for the real vehicle:

```python
REAL_SCALE_TRAJ_FORWARD = 0.5    # real vehicle is faster than the DT predicts
REAL_SCALE_DEPTH_KP     = 1.0    # transfers directly
```

`1.0` means the value transfers unchanged. **Anything that is not 1.0 is a
measured DT fidelity gap**, and this block is intended as the record of
where simulation and reality diverge.

Scalers apply in real mode only; simulation always runs unscaled. They are
resolved once at startup in `teleop_controller._resolve_scaling()`, so no
control path branches on mode. Changing one requires a restart, not just a
rebuild — the PID objects are constructed with the scaled gains.

One caveat worth understanding: scaling a *command* (like trajectory surge)
is a clean operation. Scaling a *gain* changes loop dynamics, not just
magnitude, and is only strictly valid if the plant differs by a pure gain —
which it will not, given different thrust curves, added mass, and drag.
Expect the depth gains to need individual tuning rather than one shared
factor.

---

## Vehicle setup that lives outside this repo

### ArduSub parameters

- Frame set to **SimpleROV-4**, so channels map as ch3 = heave,
  ch4 = yaw, ch5 = surge
- `AHRS_ORIENTATION` must match how the Pixhawk is physically mounted.
  A mismatch shows up as a large constant roll or pitch offset with the
  vehicle level, and corrupts yaw as well
- Compass calibrated **with the battery and thrusters in place**. An
  uncalibrated or rejected compass leaves yaw on gyro dead-reckoning, which
  drifts linearly and silently — heading hold will look like a slow turn

### Camera (Z-1Mini)

The pod is a small BusyBox Linux system reachable over SSH. Its config lives
at `/opt/bin/gcu/`, not in this repo — reference copies are in
`config/camera/`.

Changes made for underwater use:

| Setting | Value | Why |
|---|---|---|
| `Ai` | `false` | Shipped running a drone-surveillance YOLO model. Useless for fish, pure waste heat |
| `Detect` / `Track` | `false` | Same |
| `Resolutionx/y` | `1920 × 1080` | Was 4K. Less encode load, less tether bandwidth |
| `Dvr` (ch0) | `false` | 10 Mbps H.265 recording to SD that nothing used |
| `out_wait_key_frame_flag` | `false` | Cuts several seconds off stream startup |

The RTSP stream is served at the **bare IP with no path**:

```
rtsp://192.168.144.108:554/
```

Thermals: the SoC throttles at **80 °C** (first trip point; 105 and 120
follow). Above `TempHigh: 80` the pod drops from 30 fps to 5 fps to cool
itself. It runs hot on the bench in still air — this is much less of a
problem submerged, where water carries the heat away. Don't leave it running
dry for long stretches.

---

## Before every dive

1. **Set `REAL_SURFACE_PRESSURE_PA`** in `rov_config.py` to the atmospheric
   pressure measured at the surface *on the day*. Barometric pressure drifts,
   and an error here is a constant depth offset. Sanity check: depth should
   read near zero floating at the surface
2. **Confirm the vehicle is disarmed** before launching. The stack disarms on
   shutdown, but a crash can leave it armed
3. **Check roll and pitch read near zero** with the ROV level
4. Dry-run the stack with thrusters disconnected to confirm MAVROS reports
   ArduSub capabilities properly

---

## Safety features

- **Command watchdog** — `real_interface.py` forces neutral RC if no
  `cmd_vel` arrives for 0.5 s. Without this, a teleop crash leaves the last
  stick position latched and the ROV driving indefinitely
- **Disarm on shutdown** — neutral RC alone leaves the vehicle armed
- **Orphan cleanup** — teleop clears leftover viewer processes at startup,
  since the launcher can SIGKILL it and skip the shutdown handler

---

## Known issues

- **Yaw drifts** (~4°/s stationary) until the compass is calibrated.
  `YAW_KP/KI/KD` are currently `0.0`, so trajectory mode holds depth and
  drives forward but does not yet hold heading
- **`REAL_PITCH_INVERT = True`** compensates for a pitch sign difference
  between the DT and the real vehicle. Worth confirming whether the root
  cause is a genuine convention difference or `AHRS_ORIENTATION`
- **`DEPTH_FEEDFORWARD`** is fitted to the digital twin's buoyancy. The real
  vehicle has unmodelled foam and tether trim, so
  `REAL_SCALE_DEPTH_FF` is unlikely to stay at `1.0`
- **Flat module imports** — nodes use `from rov_config import ...` rather
  than a proper Python package, which relies on the install directory being
  on `PYTHONPATH`
