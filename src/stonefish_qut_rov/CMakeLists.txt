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
    ros_nodes/modular_architecture/rov_launcher_gui.py
    ros_nodes/modular_architecture/rov_control.py
    ros_nodes/modular_architecture/teleop_controller.py
    ros_nodes/modular_architecture/sim_interface.py
    ros_nodes/modular_architecture/fish_detector_follower.py
    ros_nodes/modular_architecture/real_interface.py
    ros_nodes/modular_architecture/imu_depth_plotter.py
    ros_nodes/modular_architecture/camera_viewer_rtsp.py
  DESTINATION lib/${PROJECT_NAME}
)


# PYTHON HELPER MODULES
install(
  FILES
    ros_nodes/modular_architecture/command_watchdog.py
    ros_nodes/modular_architecture/node_lifecycle.py
    ros_nodes/modular_architecture/process_utils.py
    ros_nodes/modular_architecture/viewer_manager.py
    ros_nodes/modular_architecture/battery_status.py
    ros_nodes/modular_architecture/rov_config.py
    ros_nodes/modular_architecture/control_utils.py
    ros_nodes/modular_architecture/thruster_mixer.py
    ros_nodes/modular_architecture/pid_controller.py
    ros_nodes/modular_architecture/sim_to_real_scales.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(control_safety tests TIMEOUT 60
    ENV "PYTHONDONTWRITEBYTECODE=1" "ROS_DOMAIN_ID=213")
endif()

ament_package()
