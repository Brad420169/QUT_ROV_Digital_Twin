# QUT ROV control stack

The active Python code is in `ros_nodes/modular_architecture/`. The shared
controller publishes normalized surge/heave/yaw effort on `/qut_rov/cmd_vel`;
these values are not velocities in metres/second. Simulation and MAVROS
interfaces translate this command into their respective actuator outputs.

## Build and run

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
