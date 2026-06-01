#!/bin/bash
# ==========================================
# 🤖 Wildbot 快捷指令集 (Aliases & Functions)
# ==========================================
# 請將以下這行加入您實體主機的 ~/.bashrc 中：
# source "/home/robot/wildbot_real car/wildbot_workspace-main/scripts/wildbot_aliases.sh"

# 1. 基礎啟動服務
alias start='cd "/home/robot/wildbot_real car/wildbot_workspace-main" && sudo ./scripts/00_start_all.sh'

# 2. 進入容器 bash
alias enter='docker exec -it compose-kros_car-1 bash'
alias cdarm='docker exec -it -w /workspaces/workspaces/src/arm_ik/arm_ik/ compose-kros_car-1 bash'

# 3. 導航與視覺一鍵啟動 (nstart/nstop 避開 start 的衝突)
alias nstart='cd "/home/robot/wildbot_real car/wildbot_workspace-main" && ./quick_start.sh'
alias nstop='cd "/home/robot/wildbot_real car/wildbot_workspace-main" && ./quick_start.sh stop'

# 4. 手把遙控與 SLAM 建圖
alias teleop='docker exec -it -w /workspaces compose-kros_car-1 ./teleop.sh'
alias slam='docker exec -it -w /workspaces compose-kros_car-1 ./slam.sh'
alias savemap='docker exec -it compose-kros_car-1 /workspaces/savemap.sh'

# 5. 初始定位發布
alias initpose="docker exec -it compose-kros_car-1 bash -c \"source /workspaces/install/setup.bash && ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: \\\"map\\\"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'\""

# 6. 發布目標點位測試 (L)
alias L="docker exec -it compose-kros_car-1 bash -c \"source /workspaces/install/setup.bash && ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: \\\"map\\\"}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'\""

# ==========================================
# 7. 動態傳參函數 (可在主機端直接傳遞參數執行)
# ==========================================

# 執行任務協調器 (e.g. coord 或是 coord 0.38 1.0)
coord() {
    docker exec -it -w /workspaces/workspaces/src/arm_ik/arm_ik/ compose-kros_car-1 bash -c "source /workspaces/install/setup.bash && python3 competition_coordinator.py $*"
}

# 執行自動導航測試 (e.g. nav 1 或是 nav 1 2 1)
nav() {
    docker exec -it -w /workspaces/workspaces/src/arm_ik/arm_ik/ compose-kros_car-1 bash -c "source /workspaces/install/setup.bash && python3 nav_execute.py $*"
}

# 手臂動作點位回放 (e.g. arm 1 2 1)
arm() {
    docker exec -it -w /workspaces/workspaces/src/arm_ik/arm_ik/ compose-kros_car-1 bash -c "source /workspaces/install/setup.bash && python3 grab_execute.py $*"
}

# 手臂教導程式
teach() {
    docker exec -it -w /workspaces/workspaces/src/arm_ik/arm_ik/ compose-kros_car-1 bash -c "source /workspaces/install/setup.bash && python3 grab_teach.py"
}

# 相機與自動夾取測試 (e.g. grab 或是 grab 1.0)
grab() {
    docker exec -it -w /workspaces/workspaces/src/arm_ik/arm_ik/ compose-kros_car-1 bash -c "source /workspaces/install/setup.bash && python3 \"camera and grab.py\" $*"
}

# 校正與量測工具
calib() {
    docker exec -it -w /workspaces/workspaces/src/arm_ik/arm_ik/ compose-kros_car-1 bash -c "source /workspaces/install/setup.bash && python3 calibration_tool.py"
}

# 執行光達+里程計任務協調器 (e.g. lcoord 0.50 0.38 0.9615 20.0)
lcoord() {
    docker exec -it -w /workspaces/workspaces/src/arm_ik/arm_ik/ compose-kros_car-1 bash -c "source /workspaces/install/setup.bash && python3 lidar_competition_coordinator.py $*"
}

# 執行導航座標教導/回放工具 (nteach)
nteach() {
    docker exec -it -w /workspaces/workspaces/src/arm_ik/arm_ik/ compose-kros_car-1 bash -c "source /workspaces/install/setup.bash && python3 nav_teach.py"
}

# 執行過橋自動導航任務 (e.g. bnav L 或是 bnav R 20.0)
bnav() {
    docker exec -it -w /workspaces compose-kros_car-1 bash -c "/workspaces/bnav $*"
}

