# Real-camera FPV hold

The real RTSP viewer starts an independent TCP worker that commands XF FPV
mode (0x1C), pitch +90°, yaw -90°, roll 0°, at 40 Hz. Angles are relative to
the camera mounting body. Hiding the camera window does not stop the hold.
The worker retries connections and reapplies the target after camera restart.
It needs Ethernet access to 192.168.144.108:2332 as well as the RTSP service.

Starting the viewer now automatically moves the gimbal to this pose. Avoid
running another gimbal controller concurrently. Stopping the viewer closes
the control connection without commanding a return to zero. No startup pose
is written to the camera's persistent memory; the viewer must run to apply it.

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
restricted to ±90° in this implementation, not a statement of hardware limits.
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
