cmake_minimum_required(VERSION 3.8)
project(stonefish_qut_rov)
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY launch scenarios cad_model_data config models
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
     ros_nodes/old/teleop.py
     ros_nodes/station_keeping_node.py
     ros_nodes/old/imu_data_stability_plot.py
     ros_nodes/old/depth_plotter.py
     ros_nodes/camera.py
     ros_nodes/old/ball_centering.py
     ros_nodes/teleop_controller.py
     ros_nodes/camera_viewer.py
     ros_nodes/old/yellow_tang_detector.py
     ros_nodes/old/yellow_tang_follower.py
     ros_nodes/fish_detector_follower.py
     DESTINATION lib/${PROJECT_NAME}
   )
install(FILES
  ros_nodes/pid_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
