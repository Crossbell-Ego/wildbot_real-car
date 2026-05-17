#!/bin/bash
# ============================================
# 手把遙控啟動腳本 (在 kros_car 容器內執行)
# 使用方式：
#   docker exec -it compose-kros_car-1 bash
#   ./teleop.sh
# ============================================

echo "🎮 啟動手把遙控系統..."

# 背景啟動 joy_node (手把驅動)
source /opt/ros/jazzy/setup.bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0 &
JOY_PID=$!
echo "✅ Joy driver 已啟動 (PID: $JOY_PID)"

sleep 1

# 前景啟動手把控制腳本
echo "✅ 啟動手把控制程式..."
PYTHONUNBUFFERED=1 python3 /workspaces/joy_to_base_camera.py

# 結束時清理 joy_node
kill $JOY_PID 2>/dev/null
echo "🛑 手把遙控已停止"
