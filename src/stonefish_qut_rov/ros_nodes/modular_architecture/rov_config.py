"""
Central configuration for the QUT ROV control stack.
"""

# COMMON ROS TOPICS
JOY_TOPIC       = "/joy"
CMD_VEL_TOPIC   = "/qut_rov/cmd_vel"
DEPTH_TOPIC     = "/qut_rov/depth"
IMU_TOPIC       = "/qut_rov/imu"

FISH_FOLLOW_ENABLE_TOPIC = "/qut_rov/fish_follow_enabled"
FISH_FOLLOW_CMD_TOPIC    = "/qut_rov/fish_follow_cmd"

# STONEFISH (SIM) TOPICS
SIM_THRUSTER_TOPIC = "/qut_rov/setpoint/thrusters"
SIM_PRESSURE_TOPIC = "/qut_rov/pressure"

# MAVROS (REAL ROV) TOPICS
REAL_RC_OVERRIDE_TOPIC = "/mavros/rc/override"
REAL_PRESSURE_TOPIC    = "/mavros/imu/static_pressure"

# GAMEPAD AXES
AXIS_LEFT_STICK_Y  = 1
AXIS_RIGHT_STICK_X = 2
AXIS_RIGHT_TRIGGER = 4
AXIS_LEFT_TRIGGER  = 5

# GAMEPAD BUTTONS
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

# WATER / DEPTH
WATER_DENSITY      = 1031.0
GRAVITY            = 9.81
SURFACE_PRESSURE_PA = 0.0

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

# SIM-ONLY PROCESS NAMES
FISH_FOLLOW_EXECUTABLE = "fish_detector_follower.py"
REAL_INTERFACE_EXECUTABLE = "real_interface.py"
