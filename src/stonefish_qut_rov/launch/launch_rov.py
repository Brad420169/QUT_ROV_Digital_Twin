"""Stonefish environment selection and simulator settings."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    package = get_package_share_directory("stonefish_qut_rov")
    return LaunchDescription([
        DeclareLaunchArgument("environment", default_value="ocean_environment",
                              choices=["ocean_environment", "pool_environment"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("stonefish_ros2"), "launch", "stonefish_simulator.launch.py")),
            launch_arguments={
                "simulation_data": package,
                "scenario_desc": PathJoinSubstitution([
                    package, "scenarios", [LaunchConfiguration("environment"), ".scn"]]),
                "simulation_rate": "500.0",
                "window_res_x": "920",
                "window_res_y": "1000",
                "rendering_quality": "low",
            }.items(),
        ),
    ])
