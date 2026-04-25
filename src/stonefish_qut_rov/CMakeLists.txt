cmake_minimum_required(VERSION 3.8)
project(stonefish_qut_rov)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch scenarios cad_model_data
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ros_nodes/teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
