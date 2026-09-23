cmake_minimum_required(VERSION 3.8)

project(stonefish_qut_rov)

find_package(ament_cmake REQUIRED)

# PACKAGE DATA
install(
  DIRECTORY
    launch
    scenarios
    cad_model_data
    models
  DESTINATION share/${PROJECT_NAME}
)


# EXECUTABLE ROS 2 NODES
install(
  PROGRAMS
    ros_nodes/common/rov_launcher_gui.py
    ros_nodes/common/rov_control.py
    ros_nodes/common/teleop_controller.py
    ros_nodes/sim/sim_interface.py
    ros_nodes/sim/fish_detector_follower.py
    ros_nodes/real/real_interface.py
    ros_nodes/common/imu_depth_plotter.py
    ros_nodes/real/pwm_monitor.py
    ros_nodes/real/camera_viewer_rtsp.py
  DESTINATION lib/${PROJECT_NAME}
)


# PYTHON HELPER MODULES
install(
  FILES
    ros_nodes/common/command_watchdog.py
    ros_nodes/common/node_lifecycle.py
    ros_nodes/common/process_utils.py
    ros_nodes/common/viewer_manager.py
    ros_nodes/real/gimbal_fpv.py
    ros_nodes/real/gimbal_motor.py
    ros_nodes/real/gimbal_motor.c
    ros_nodes/real/camera_thermal.py
    ros_nodes/real/tennis_ball_tracker.py
    ros_nodes/real/battery_status.py
    ros_nodes/common/rov_config.py
    ros_nodes/common/control_utils.py
    ros_nodes/common/vehicle_command.py
    ros_nodes/sim/thruster_mixer.py
    ros_nodes/common/pid_controller.py
    ros_nodes/common/sim_to_real_scales.py
  DESTINATION lib/${PROJECT_NAME}
)


# Keep sibling imports available when executables are symlinked to source folders.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/ros_nodes.dsv")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(control_safety tests TIMEOUT 60
    ENV "PYTHONDONTWRITEBYTECODE=1" "ROS_DOMAIN_ID=213")
endif()

ament_package()
