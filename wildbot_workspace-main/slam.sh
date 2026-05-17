#!/bin/bash
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

# 在背景啟動手把遙控 (日誌輸出到 /tmp，保持畫面乾淨)
/workspaces/teleop.sh > /tmp/teleop.log 2>&1 &
TELEOP_PID=$!
echo "🎮 手把遙控系統已在背景啟動 (PID: $TELEOP_PID)"
echo "📝 手把日誌記錄在 /tmp/teleop.log"

echo "Starting SLAM Toolbox Mapping Mode..."
ros2 launch wildbot_slam slam_launch.py

# 正常退出時也進行清理
cleanup
