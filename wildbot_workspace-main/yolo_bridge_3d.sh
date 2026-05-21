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

echo "🚀 啟動 YOLO 3D 橋樑偵測模式 (使用 bridge.pt)..."
echo "注意：此模式需要深度相機發布 /camera/depth/image_raw"

# 啟動 YOLO 載入 bridge.pt 模型
# 1. model 指定為 bridge.pt
# 2. 開啟 3D 功能 (use_3d:=True)
# 3. 設定較低的信心度閾值 (threshold:=0.4) 確保遠處就能偵測到
# 4. device 設為 cpu (或 cuda:0，如果 GPU 支援的話)
ros2 launch yolo_bringup yolov26.launch.py \
    model:=/workspaces/workspaces/src/yolo_ros-main/model/bridge.pt \
    use_3d:=True \
    use_tracking:=False \
    target_frame:=base_link \
    threshold:=0.4 \
    device:=cpu \
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
echo "✅ YOLO 3D 橋樑偵測模型已激活！"
echo "3D 偵測話題：/yolo/detections_3d"
echo "---"
wait
