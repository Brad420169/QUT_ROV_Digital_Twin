cmake_minimum_required(VERSION 3.8)

project(stonefish_qut_rov)

find_package(ament_cmake REQUIRED)

# PACKAGE DATA
install(
  DIRECTORY
    launch
    scenarios
    cad_model_data
    config
    models
  DESTINATION share/${PROJECT_NAME}
)


# EXECUTABLE ROS 2 NODES
install(
  PROGRAMS
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
    ros_nodes/modular_architecture/rov_config.py
    ros_nodes/modular_architecture/control_utils.py
    ros_nodes/modular_architecture/thruster_mixer.py
    ros_nodes/modular_architecture/pid_controller.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
