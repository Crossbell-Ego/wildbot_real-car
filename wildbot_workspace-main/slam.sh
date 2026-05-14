#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /workspaces/install/setup.bash
echo "Starting SLAM Toolbox Mapping Mode..."
ros2 launch wildbot_slam slam_launch.py
