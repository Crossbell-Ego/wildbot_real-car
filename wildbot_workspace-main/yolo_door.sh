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

echo "🚀 啟動 YOLO 門把偵測模式 (使用 bearknob_openvino_model)..."

# 1. model 指定為 bearknob_openvino_model (適合 AMD CPU 高效能推論)
# 2. 暫不啟用 3D 轉換以節省資源 (因為開門任務使用 2D 中心對齊 + 定時前進)
# 3. threshold 設為 0.3
# 4. device 設為 cpu
ros2 launch yolo_bringup yolov26.launch.py \
    model:=/workspaces/workspaces/src/yolo_ros-main/model/bearknob_openvino_model \
    use_3d:=False \
    use_tracking:=False \
    target_frame:=base_link \
    threshold:=0.3 \
    device:=cpu &

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

# 依序激活 2D 數據鏈路節點
wait_for_activation /yolo/yolo_node
wait_for_activation /yolo/debug_node

echo "---"
echo "✅ YOLO 門把偵測模型已激活！"
echo "2D 偵測話題：/yolo/detections"
echo "---"
wait
