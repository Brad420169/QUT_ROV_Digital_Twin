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
JOY_TIMEOUT_S = 0.5
SENSOR_TIMEOUT_S = 0.5
FISH_COMMAND_TIMEOUT_S = 0.5
STARTUP_TIMEOUT_S = 45.0

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

# SIM-TO-REAL TRANSFER
# All real-mode tuning lives in sim_to_real_scales.py — that is the only
# file to edit at the pool. Re-exported here so existing imports of
# rov_config keep working unchanged.
from sim_to_real_scales import (          # noqa: E402,F401
    POOL_WATER_DENSITY,
    REAL_PITCH_INVERT,
    REAL_RC_NEUTRAL_US,
    REAL_RC_RANGE_US,
    REAL_SCALE_DEPTH_FF,
    REAL_SCALE_DEPTH_KD,
    REAL_SCALE_DEPTH_KI,
    REAL_SCALE_DEPTH_KP,
    REAL_SCALE_MANUAL_HEAVE,
    REAL_SCALE_MANUAL_SURGE,
    REAL_SCALE_MANUAL_YAW,
    REAL_SCALE_TRAJ_FORWARD,
    REAL_SCALE_TRAJ_YAW,
    REAL_SCALE_YAW_KD,
    REAL_SCALE_YAW_KI,
    REAL_SCALE_YAW_KP,
    REAL_SURFACE_PRESSURE_PA,
    REAL_WATER_DENSITY,
    REAL_YAW_INVERT,
)

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
# Density and real surface pressure are set in sim_to_real_scales.py.
WATER_DENSITY      = REAL_WATER_DENSITY
FRESH_WATER_DENSITY = POOL_WATER_DENSITY
GRAVITY            = 9.81
SIM_SURFACE_PRESSURE_PA = 0

# SIMULATOR THRUSTER OUTPUT
SIM_MAX_SETPOINT = 600.0

# REAL ROV — MAVROS / network
# IP of the ROV-side Fathom-X / M80 on the tether network.
# Update this if your subnet differs from the BlueRobotics default.
REAL_ROV_IP         = "192.168.2.2"
REAL_FCU_URL        = f"udp://:14550@{REAL_ROV_IP}:14555"

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