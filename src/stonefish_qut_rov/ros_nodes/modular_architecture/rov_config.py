"""
Central configuration for the QUT ROV control stack.
"""

# COMMON ROS TOPICS
JOY_TOPIC       = "/joy"
CMD_VEL_TOPIC   = "/qut_rov/cmd_vel"
DEPTH_TOPIC     = "/qut_rov/depth"
IMU_TOPIC       = "/qut_rov/imu"

# ARMING (real mode only)
# teleop asks for a toggle; real_interface owns the truth and reports the
# resulting state back so the teleop terminal can print it.
ARM_TOGGLE_TOPIC = "/qut_rov/arm_toggle"
ARMED_STATE_TOPIC = "/qut_rov/armed"

# BATTERY (real mode only)
# real_interface republishes the MAVROS battery state on a common topic;
# teleop prints it on demand.
BATTERY_TOPIC = "/qut_rov/battery"
REAL_BATTERY_TOPIC = "/mavros/battery"

FISH_FOLLOW_ENABLE_TOPIC = "/qut_rov/fish_follow_enabled"
FISH_FOLLOW_CMD_TOPIC    = "/qut_rov/fish_follow_cmd"

# STONEFISH (SIM) TOPICS
SIM_THRUSTER_TOPIC = "/qut_rov/setpoint/thrusters"
SIM_PRESSURE_TOPIC = "/qut_rov/pressure"

# MAVROS (REAL ROV) TOPICS
REAL_RC_OVERRIDE_TOPIC = "/mavros/rc/override"
REAL_PRESSURE_TOPIC    = "/mavros/imu/static_pressure"
REAL_IMU_TOPIC         = "/mavros/imu/data"
REAL_SET_MODE_SERVICE  = "/mavros/set_mode"
REAL_ARMING_SERVICE    = "/mavros/cmd/arming"

# Set True if a bench yaw test shows real-mode heading rotating the
# opposite way to the DT for the same physical rotation.
REAL_YAW_INVERT = False
REAL_PITCH_INVERT = True

# REAL ROV CAMERA (Z-1Mini, RTSP)
# Default pod address is 192.168.144.108. Either reconfigure the pod to
# 192.168.2.3 with GCU_Assistant, or alias the Fathom interface:
#   sudo ip addr add 192.168.144.1/24 dev enx00e04c680725
REAL_CAMERA_IP   = "192.168.144.108"
# lal serves the stream at the bare IP — no path (confirmed by probing
# the pod: gb_control builds "rtsp://%d.%d.%d.%d" and RtspPath is empty).
REAL_CAMERA_PATH = ""
REAL_RTSP_URL    = f"rtsp://{REAL_CAMERA_IP}:554/{REAL_CAMERA_PATH}"

# Window is scaled down from 4K for display.
REAL_CAMERA_DISPLAY_WIDTH = 1280

# COMMAND WATCHDOG
# If no /qut_rov/cmd_vel message arrives within this many seconds the
# interface forces neutral output. Protects against teleop_controller
# dying mid-run and leaving the last stick command latched.
COMMAND_TIMEOUT_S = 0.5

# GAMEPAD AXES
AXIS_LEFT_STICK_Y  = 1
AXIS_RIGHT_STICK_X = 2
AXIS_RIGHT_TRIGGER = 4
AXIS_LEFT_TRIGGER  = 5

# GAMEPAD D-PAD (reported as axes on this controller)
# Axis 7: up = +1.0, down = -1.0, neutral = 0.0
AXIS_DPAD_Y        = 7
DPAD_PRESS_LEVEL   = 0.5

# D-pad down shuts the whole stack down, so it must be held rather than
# tapped — a stray thumb should not kill teleop with the ROV in the water.
DPAD_SHUTDOWN_HOLD_S = 1.0

# GAMEPAD BUTTONS
ARM_BUTTON             = 0
BATTERY_BUTTON         = 1
STATION_KEEPING_BUTTON = 6
TRAJECTORY_SET_BUTTON  = 7
CAMERA_BUTTON          = 4
FISH_FOLLOW_BUTTON     = 3

# GAMEPAD RESPONSE
DEADZONE        = 0.08
YAW_MANUAL_SCALE = 0.5

# DEPTH CONTROLLER
DEPTH_KP = 200.0 / 600.0
DEPTH_KI = 5.0   / 600.0
DEPTH_KD = 80.0  / 600.0

DEPTH_INTEGRAL_LIMIT = 400.0
DEPTH_FEEDFORWARD    = -160.0 / 600.0

# YAW / TRAJECTORY CONTROLLER
YAW_KP             = 0.0
YAW_KI             = 0.0
YAW_KD             = 0.0
YAW_INTEGRAL_LIMIT = 150.0

TRAJECTORY_FORWARD = 0.5
TRAJECTORY_MAX_YAW = 0.3

# ── SIM-TO-REAL TRANSFER ────────────────────────────────────────────
# Every constant above is the SIM value and stays canonical — the DT is
# the reference. These scalers adapt those values for the real vehicle,
# where one normalised unit means different thrust (ArduSub's SimpleROV-4
# mixer, real thrust curves, real drag, real buoyancy).
#
#   1.0  = transfers directly, no correction needed
#   <1.0 = real vehicle is more responsive than the DT predicts
#   >1.0 = real vehicle is less responsive than the DT predicts
#
# Anything that is not 1.0 is a measured DT fidelity gap — keep this
# block as the record of where sim and real diverge.
#
# Scalers are applied ONLY in real mode; sim always runs unscaled.

# Manual stick commands
REAL_SCALE_MANUAL_SURGE = 1.0
REAL_SCALE_MANUAL_YAW   = 1.0
REAL_SCALE_MANUAL_HEAVE = 1.0

# Trajectory mode
REAL_SCALE_TRAJ_FORWARD = 0.5
REAL_SCALE_TRAJ_YAW     = 1.0    # scales the yaw output clamp

# Depth hold. NOTE: scaling a gain changes loop dynamics, not just
# magnitude — a single factor is only strictly valid if the plant
# differs by a pure gain, which it will not. Expect these to need
# individual tuning rather than one shared number.
REAL_SCALE_DEPTH_KP = 1.0
REAL_SCALE_DEPTH_KI = 1.0
REAL_SCALE_DEPTH_KD = 1.0

# Buoyancy trim feedforward. The DT does not model the foam or the
# tether nose-up trim, so this one is very unlikely to stay at 1.0.
REAL_SCALE_DEPTH_FF = 1.0

# Heading hold (gains are currently 0.0, so these do nothing yet)
REAL_SCALE_YAW_KP = 1.0
REAL_SCALE_YAW_KI = 1.0
REAL_SCALE_YAW_KD = 1.0

# BATTERY
# Tattu 2200 mAh 4S LiPo. Percentage is estimated from resting voltage
# using a per-cell curve — ArduPilot's own remaining-percent needs
# BATT_CAPACITY set and coulomb counting, which is unreliable here.
# Under thruster load voltage sags, so a reading taken while driving will
# understate the true charge.
BATTERY_CELLS = 4

BATTERY_CELL_CURVE = [
    (3.00, 0),
    (3.30, 5),
    (3.50, 10),
    (3.60, 20),
    (3.70, 30),
    (3.75, 40),
    (3.80, 50),
    (3.85, 60),
    (3.90, 70),
    (3.95, 80),
    (4.05, 90),
    (4.20, 100),
]

# Below this the pack is either flat or is not a 4S LiPo at all (bench
# supply, for example), so the percentage is reported as unknown.
BATTERY_MIN_PLAUSIBLE_V = 11.0

# WATER / DEPTH
WATER_DENSITY      = 1031.0
GRAVITY            = 9.81
SIM_SURFACE_PRESSURE_PA = 0
REAL_SURFACE_PRESSURE_PA = 102137.0 # NEED TO CHANGE TO WHATEVER THE ATMOSPHERIC PRESSURE IS AT THE SURFACE (IN PASCALS) WHEN THE ROV IS IN USE!

# SIMULATOR THRUSTER OUTPUT
SIM_MAX_SETPOINT = 600.0

# REAL ROV — RC microseconds
REAL_RC_NEUTRAL_US  = 1500   # stopped / neutral
REAL_RC_RANGE_US    = 400    # ±400 µs → 1100–1900 µs full range

# REAL ROV — MAVROS / network
# IP of the ROV-side Fathom-X / M80 on the tether network.
# Update this if your subnet differs from the BlueRobotics default.
REAL_ROV_IP         = "192.168.2.2"
REAL_FCU_URL        = f"udp://:14550@{REAL_ROV_IP}:14555"

# Startup delay to allow MAVROS to connect to the Pixhawk before
# the interface node starts sending RC override messages.
REAL_MAVROS_STARTUP_DELAY = 5.0

# REAL CAMERA WINDOW TOGGLE
# The viewer starts with teleop and holds the RTSP connection open;
# the Y button only shows/hides the window, so there is no reconnect
# delay after the first launch.
CAMERA_SHOW_TOPIC = "/qut_rov/camera_show"

# PLOTTER
PLOT_WINDOW_SECONDS = 60.0
PLOT_REFRESH_MS     = 200

# PROCESS NAMES
FISH_FOLLOW_EXECUTABLE = "fish_detector_follower.py"
PLOTTER_EXECUTABLE     = "imu_depth_plotter.py"
REAL_CAMERA_EXECUTABLE = "camera_viewer_rtsp.py"
REAL_INTERFACE_EXECUTABLE = "real_interface.py"