#!/bin/bash
source /workspaces/install/setup.bash

# 清理舊的行程
echo "正在清理舊的 YOLO 相關行程..."
pkill -f yolo_node || true
pkill -f tracking_node || true
pkill -f debug_node || true
sleep 1

# 啟動 YOLO
# 暫時關閉 tracking 以進行排除法測試
ros2 launch yolo_bringup yolov26.launch.py \
    use_3d:=False \
    use_tracking:=False \
    target_frame:=base_link &

# 等待節點出現並手動激活
wait_for_activation() {
    local node_name=$1
    echo "正在激活 ${node_name} ..."
    # 等待節點出現在列表中
    until ros2 node list | grep -q "${node_name}"; do
        sleep 0.5
    done
    # 執行狀態轉換
    ros2 lifecycle set "${node_name}" configure
    ros2 lifecycle set "${node_name}" activate
}

# 依序激活核心節點 (暫時跳過 tracking_node)
wait_for_activation /yolo/yolo_node
wait_for_activation /yolo/debug_node

echo "---"
echo "✅ YOLO 測試模式 (No Tracking) 已全數激活！"
echo "影像話題：/yolo/dbg_image/compressed"
echo "偵測話題：/yolo/detections"
echo "---"
wait
