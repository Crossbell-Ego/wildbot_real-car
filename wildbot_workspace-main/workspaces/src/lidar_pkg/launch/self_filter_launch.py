import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("lidar_pkg")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(share_dir, "config", "self_filter.yaml"),
                description="Path to LiDAR self-filter parameters.",
            ),
            Node(
                package="lidar_pkg",
                executable="lidar_nan_value_filter_node",
                name="lidar_nan_value_filter_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
