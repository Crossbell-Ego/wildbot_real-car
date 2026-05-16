#!/bin/bash

# 確保環境變數已載入
source /opt/ros/jazzy/setup.bash
source /workspaces/install/setup.bash

echo "正在清理舊的 YOLO 相關行程..."
pkill -f yolo_node || true
pkill -f tracking_node || true
pkill -f debug_node || true
pkill -f detect_3d_node || true
sleep 1

echo "🚀 啟動 YOLO 3D 偵測模式..."
echo "注意：此模式需要深度相機發布 /camera/depth/image_raw"

# 啟動 YOLO Launch
# 1. 開啟 3D 功能
# 2. 預設關閉 tracking 以節省資源 (可視需求開啟)
# 3. 確保 target_frame 為 base_link
ros2 launch yolo_bringup yolov26.launch.py \
    use_3d:=True \
    use_tracking:=False \
    target_frame:=base_link \
    threshold:=0.7 \
    input_depth_topic:=/camera/depth/image_raw &

# 等待節點出現並手動激活
wait_for_activation() {
    local node_name=$1
    echo "正在激活 ${node_name} ..."
    until ros2 node list | grep -q "${node_name}"; do
        sleep 0.5
    done
    ros2 lifecycle set "${node_name}" configure
    ros2 lifecycle set "${node_name}" activate
}

# 依序激活 3D 數據鏈路節點
wait_for_activation /yolo/yolo_node
wait_for_activation /yolo/detect_3d_node
wait_for_activation /yolo/debug_node

echo "---"
echo "✅ YOLO 3D 模式已全數激活！"
echo "影像話題：/yolo/dbg_image/compressed"
echo "3D 偵測話題：/yolo/detections_3d"
echo "---"
wait
