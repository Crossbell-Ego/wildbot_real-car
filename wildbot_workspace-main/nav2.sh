#!/bin/bash
# 導航啟動腳本
# 用法: 
# 1. 直接執行: ./nav2.sh (配合 slam.sh 使用，或者單獨啟動)
# 2. 啟動 SLAM + 導航: ./nav2.sh slam
# 3. 使用地圖導航: ./nav2.sh /path/to/map.yaml

source /opt/ros/jazzy/setup.bash
source /workspaces/install/setup.bash

ARG=$1

if [ "$ARG" == "slam" ]; then
    echo "Starting Nav2 Navigation with SLAM..."
    ros2 launch wildbot_nav2 nav2_launch.py slam:=True
elif [ -n "$ARG" ] && [ -f "$ARG" ]; then
    echo "Starting Nav2 Navigation with map: $ARG"
    ros2 launch wildbot_nav2 nav2_launch.py map:="$ARG"
else
    echo "Starting Nav2 Navigation (assuming SLAM is already running)..."
    ros2 launch wildbot_nav2 nav2_launch.py
fi
