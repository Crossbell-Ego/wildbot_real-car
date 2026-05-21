#!/bin/bash
# 導航啟動腳本
# 用法: 
# 1. 直接執行: ./nav2.sh (配合 slam.sh 使用，或者單獨啟動)
# 2. 啟動 SLAM + 導航: ./nav2.sh slam
# 3. 使用地圖導航: ./nav2.sh /path/to/map.yaml

source /opt/ros/jazzy/setup.bash
source /workspaces/install/setup.bash

# 建立清理函式
cleanup() {
    echo -e "\n🛑 正在關閉手把遙控與清理背景進程..."
    kill $TELEOP_PID 2>/dev/null
    pkill -f joy_to_base_camera.py 2>/dev/null
    pkill -f joy_node 2>/dev/null
    exit 0
}

# 捕捉 SIGINT (Ctrl+C) 和 SIGTERM 信號，觸發 cleanup
trap cleanup SIGINT SIGTERM

# 在背景啟動手把遙控 (若檢測到無其他遙控實例執行)
if ! pgrep -f joy_to_base_camera.py > /dev/null; then
    /workspaces/teleop.sh > /tmp/teleop.log 2>&1 &
    TELEOP_PID=$!
    echo "🎮 手把遙控系統已在背景啟動 (PID: $TELEOP_PID)"
    echo "📝 手把日誌記錄在 /tmp/teleop.log"
else
    echo "🎮 手把遙控系統已在運行中，無需重複啟動。"
fi

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

# 正常退出時也進行清理
cleanup
