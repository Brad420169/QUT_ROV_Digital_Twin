from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

ENVIRONMENT = 'ocean_environment'

def generate_launch_description():
    pkg_dir = get_package_share_directory('stonefish_qut_rov')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('stonefish_ros2') +
                '/launch/stonefish_simulator.launch.py'
            ),
            launch_arguments={
                'simulation_data': pkg_dir,
                'scenario_desc': pkg_dir + f'/scenarios/{ENVIRONMENT}.scn',
                'simulation_rate': '100.0',
                'window_res_x': '1600',
                'window_res_y': '1000',
                'rendering_quality': 'high',
            }.items()
        )
    ])