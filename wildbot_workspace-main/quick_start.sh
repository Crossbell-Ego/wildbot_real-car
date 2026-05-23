#!/bin/bash
# 🐻 Wildbot 一鍵啟動 Nav2 導航、初始定位與 YOLO 3D 辨識系統

MAP_PATH=${1:-"/workspaces/maps/my_map.yaml"}
CONTAINER_NAME="compose-kros_car-1"

# 函數：清理舊行程
cleanup_old_processes() {
    echo "🧹 正在自動清理可能殘留的舊 Nav2 與 YOLO 行程..."
    if [ -f /.dockerenv ]; then
        # 容器內執行
        pkill -9 -f "[n]av2.sh" || true
        pkill -9 -f "[y]olo_3d.sh" || true
        pkill -9 -f "[n]av2_launch.py" || true
        pkill -9 -f "[y]olo_node" || true
        pkill -9 -f "[d]etect_3d_node" || true
        pkill -9 -f "[d]ebug_node" || true
        pkill -9 -f "[c]omponent_container_isolated" || true
        pkill -9 -f "[j]oy_to_base_camera.py" || true
        pkill -9 -f "[t]eleop.sh" || true
    else
        # 主機端執行
        docker exec "$CONTAINER_NAME" bash -c 'pkill -9 -f "[n]av2.sh"; pkill -9 -f "[y]olo_3d.sh"; pkill -9 -f "[n]av2_launch.py"; pkill -9 -f "[y]olo_node"; pkill -9 -f "[d]etect_3d_node"; pkill -9 -f "[d]ebug_node"; pkill -9 -f "[c]omponent_container_isolated"; pkill -9 -f "[j]oy_to_base_camera.py"; pkill -9 -f "[t]eleop.sh"' 2>/dev/null || true
    fi
    sleep 2
}

# 如果傳入第一個參數為 stop，則執行停止並清理
if [ "$1" = "stop" ]; then
    cleanup_old_processes
    echo "============================================="
    echo "🛑 已成功關閉並清理所有 Nav2 與 YOLO 背景服務！"
    echo "============================================="
    exit 0
fi

# 檢查是否在 Docker 容器內執行
if [ -f /.dockerenv ]; then
    echo "============================================="
    echo "🐳 偵測到正在 Docker 容器內執行..."
    echo "============================================="
    
    cleanup_old_processes
    
    # 1. 啟動 Nav2
    echo "⏳ 1. 正在啟動 Nav2 導航 (地圖: $MAP_PATH)..."
    nohup ./nav2.sh "$MAP_PATH" > /tmp/nav2.log 2>&1 &
    NAV2_PID=$!
    echo "✅ Nav2 啟動指令已送出 (背景 PID: $NAV2_PID)"
    
    # 2. 等待並發布定位 (Unblock Deadlock)
    echo "⏳ 等待 5 秒以利訂閱註冊..."
    sleep 5
    echo "📍 2. 正在發布起跑點初始定位 (Initial Pose) 至 (0,0)..."
    source /workspaces/install/setup.bash
    ros2 topic pub -1 -w 0 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
    
    # 3. 啟動 YOLO 3D
    echo "⏳ 3. 正在啟動 YOLO 3D 辨識..."
    nohup ./yolo_3d.sh > /tmp/yolo_3d.log 2>&1 &
    YOLO_PID=$!
    echo "✅ YOLO 3D 啟動指令已送出 (背景 PID: $YOLO_PID)"
    
    # 4. 驗證所有服務是否轉為 Active 狀態
    echo -n "⏳ 正在驗證 Nav2 與 YOLO 3D 是否成功啟動並啟動完畢"
    ALL_READY=false
    for i in {1..25}; do
        if ros2 lifecycle get /planner_server 2>/dev/null | grep -q "active" && \
           ros2 lifecycle get /yolo/yolo_node 2>/dev/null | grep -q "active"; then
            ALL_READY=true
            echo -e "\n✅ 所有服務已成功轉換至 active 狀態，運行正常！"
            break
        fi
        echo -n "."
        sleep 1
    done
    if [ "$ALL_READY" = false ]; then
        echo -e "\n⚠️ 警告：部分服務啟動較慢或超時，請使用 'ros2 node list' 或查看 /tmp 下的日誌進行確認。"
    fi
    
    echo "============================================="
    echo "🎉 容器內背景服務啟動完成！"
    echo "============================================="

else
    # 在 Host 主機端執行
    echo "============================================="
    echo "💻 偵測到正在 Host 主機端執行..."
    echo "============================================="
    
    # 檢查容器是否處於運行狀態
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "❌ 錯誤：找不到運行中的容器 ${CONTAINER_NAME}！"
        echo "💡 請先確認是否已執行：sudo ./scripts/00_start_all.sh"
        exit 1
    fi
    
    cleanup_old_processes
    
    # 1. 啟動 Nav2
    echo "⏳ 1. 正在透過 Docker 啟動 Nav2 導航 (地圖: $MAP_PATH)..."
    docker exec -d "$CONTAINER_NAME" bash -c "cd /workspaces && ./nav2.sh $MAP_PATH > /tmp/nav2.log 2>&1"
    echo "✅ Nav2 啟動指令已送出"
    
    # 2. 等待並發布定位 (Unblock Deadlock)
    echo "⏳ 等待 5 秒以利訂閱註冊..."
    sleep 5
    echo "📍 2. 正在發布起跑點初始定位 (Initial Pose) 至 (0,0)..."
    docker exec "$CONTAINER_NAME" bash -c "source /workspaces/install/setup.bash && ros2 topic pub -1 -w 0 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: \"map\"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'"
    
    # 3. 啟動 YOLO 3D
    echo "⏳ 3. 正在透過 Docker 啟動 YOLO 3D 辨識..."
    docker exec -d "$CONTAINER_NAME" bash -c "cd /workspaces && ./yolo_3d.sh > /tmp/yolo_3d.log 2>&1"
    echo "✅ YOLO 3D 啟動指令已送出"
    
    # 4. 驗證所有服務是否轉為 Active 狀態
    echo -n "⏳ 正在驗證 Nav2 與 YOLO 3D 是否成功啟動並啟動完畢"
    ALL_READY=false
    for i in {1..25}; do
        if docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/jazzy/setup.bash && ros2 lifecycle get /planner_server 2>/dev/null" | grep -q "active" && \
           docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/jazzy/setup.bash && ros2 lifecycle get /yolo/yolo_node 2>/dev/null" | grep -q "active"; then
            ALL_READY=true
            echo -e "\n✅ 所有服務已成功轉換至 active 狀態，運行正常！"
            break
        fi
        echo -n "."
        sleep 1
    done
    if [ "$ALL_READY" = false ]; then
        echo -e "\n⚠️ 警告：部分服務啟動較慢或超時，請使用 Foxglove 檢視或手動確認。"
    fi
    
    echo "============================================="
    echo "🎉 一鍵啟動完成，所有服務已成功運行！"
    echo "👉 提示：如果想查看 YOLO 偵測狀況，可使用 Foxglove 連線監看。"
    echo "============================================="
fi
