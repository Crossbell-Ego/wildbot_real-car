import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('wildbot_slam')
    slam_config_path = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    slam_toolbox_launch_dir = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_launch_dir, 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': slam_config_path,
                'use_sim_time': 'false'
            }.items()
        ),
    ])
