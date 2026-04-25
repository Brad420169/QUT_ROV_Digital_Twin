from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

ENVIRONMENT = 'open_ocean'

def generate_launch_description():
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('stonefish_ros2') +
            '/launch/stonefish_simulator.launch.py'
        ),
        launch_arguments={
            'simulation_data': get_package_share_directory('stonefish_qut_rov'),
            'scenario_desc': get_package_share_directory('stonefish_qut_rov') + f'/scenarios/{ENVIRONMENT}.scn',
            'simulation_rate': '100.0',
            'window_res_x': '1600',
            'window_res_y': '1000',
            'rendering_quality': 'high',
        }.items()
    )

    return LaunchDescription([
        launch_include
    ])