#!/bin/bash
# ==========================================
# 💾 Wildbot 容器內一鍵地圖儲存腳本
# ==========================================

# 載入 ROS 2 環境
if [ -f "/workspaces/install/setup.bash" ]; then
    source /workspaces/install/setup.bash
else
    source /opt/ros/jazzy/setup.bash
fi

echo "正在儲存地圖至 /workspaces/maps/my_map..."
ros2 run nav2_map_server map_saver_cli -f /workspaces/maps/my_map --ros-args -p save_map_timeout:=10.0
