# Real-camera FPV hold

The real RTSP viewer starts a worker that first disables the gimbal motors.
Opening the camera window selects XF FPV mode (0x1C), applies the configured
body-relative angles at 40 Hz, and enables the motors. Hiding/closing the window
(Y, q, Escape, or the window close button) disables the motors again. Viewer
shutdown also requests motor stop. The worker retries failed operations every
two seconds and reports failures instead of claiming the gimbal is limp.

While the physical camera window is shown, D-pad up/down changes pitch and
left/right changes roll continuously while held, at 20 degrees/second.
Release stops movement; joystick data older than 0.5 seconds also stops movement
and requires releasing the D-pad before moving again.
Yaw stays fixed. Targets are bounded to roll ±45° and pitch -145° to +90° and retained
when reopening the window. Release the D-pad after opening before aiming.
The camera publishes actual visibility on `/qut_rov/camera_visible`; teleop
suppresses the plot and shutdown shortcuts while shown. After closing, release
the D-pad before using those shortcuts again. This does not alter stick controls.

Motor start/stop uses SSH to the camera's internal UART; TCP angle commands alone
do not disable holding torque. Requires trusted, noninteractive root SSH key
access and `arm-linux-gnueabihf-gcc` on the host (already installed on Brad's PC).
The small C helper is compiled and uploaded to camera `/tmp` automatically.
It briefly pauses `gb_control`, preserves its current command fields, sends the
documented motor command, checks the CRC/status acknowledgement, and resumes
the controller. An independent two-second guard resumes it if the helper dies.
No firmware or persistent settings are changed. Power cycling enables motors
again. This helper is restricted to the verified controller SHA256 in
`gimbal_motor.py`; firmware updates require re-verifying its RAM layout.

Avoid other gimbal controllers during use. A lost Ethernet connection or forced
process kill can prevent shutdown stop from reaching the camera; consult logs.
Setting `gimbal_hold_enabled:=false` disables ALL motor/hold management, for a
video-only viewer; it does not itself send a limp command.

Startup ROS parameters (restart the viewer to change):

```bash
ros2 run stonefish_qut_rov camera_viewer_rtsp.py --ros-args \
  -p start_visible:=true -p gimbal_pitch_deg:=90.0 -p gimbal_yaw_deg:=-90.0
```

Video only:

```bash
ros2 run stonefish_qut_rov camera_viewer_rtsp.py --ros-args \
  -p start_visible:=true -p gimbal_hold_enabled:=false
```

`gimbal_host` defaults to REAL_CAMERA_IP in rov_config.py. Targets are
restricted to roll/yaw ±90° and pitch -145° to +90° in the packet encoder.
The lower pitch limit follows the Z-1 Mini manual's -145° value; the existing
upper command range is preserved. Actual travel also depends on camera firmware
and mounting; these are command limits, not measured travel.
Control errors are logged and do not stop video. Mode changes from another
controller cause reconnection and FPV reapplication.

## Thermal protection and future vision control

The viewer reads `/sys/class/thermal/thermal_zone0/temp` on the camera via
SSH every two seconds. Use an existing trusted SSH key, or enter the camera
password into the launch environment (not into source or shell history):

```bash
read -rsp 'Camera SSH password: ' CAMERA_SSH_PASSWORD; echo
export CAMERA_SSH_PASSWORD
# Start your usual ROV launcher from this same terminal.
```

Requires `python3-pexpect` and a trusted camera host key in known_hosts.
On first use, connect with `ssh root@192.168.144.108` and verify its identity.
No password is saved by this code. If camera authentication or temperature
telemetry is unavailable, vision permission remains false; manual driving
and video viewing remain available.

At >=80 C, thermal blocking latches until the measured temperature is <75 C.
Starting up in the 75–80 C band is conservatively blocked as possible cooldown.
Actual delivered frames must also be >=10 fps and less than one second old.
The manufacturer thermal setting is 5 fps; the warning distinguishes that
setting from measured frame delivery. No temperature threshold on the camera
is changed and the software does not deliberately heat the camera.

Published twice per second:

- `/qut_rov/vision_control_allowed` (`Bool`): permission, NOT enable command.
- `/qut_rov/camera_health` (`String`): human-readable reason.
- `/qut_rov/camera_temperature` (`Float32`): Celsius, NaN if unavailable.

The viewer displays/logs blocking conditions even when opened after the trip.
Existing real fish-follow remains disabled. Its command path additionally
checks the permission before enabling and before publishing motion.

Every future real vision controller must use `VisionPermission` from
`camera_thermal.py`, subscribe with volatile QoS to the permission topic,
call `update(msg.data)` on each heartbeat, and check `valid()` before enabling
and before issuing every motion command. On false/expired permission, clear
the mode's enable flag and its motion output. Require a new operator enable
after recovery. Missing heartbeats revoke permission after two seconds.
This contract cannot automatically protect future nodes that bypass it.
