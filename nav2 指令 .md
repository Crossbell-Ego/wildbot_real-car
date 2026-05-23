# 🤖 Wildbot 實體小車 Nav2 導航操作指南

本指南詳細說明如何使用已建好的地圖啟動 Nav2 導航系統、初始化小車定位、發布目標點，以及常見的參數調校方法。

---

## 📋 準備工作

確保您的實體小車硬體連接正常，且 Docker 服務已啟動。

1. **啟動底盤與感測器 (LiDAR、底盤電機等) 服務**
   請在 Host 終端機執行：
   ```bash
   cd "/home/robot/wildbot_real car/wildbot_workspace-main"
   sudo ./scripts/00_start_all.sh
   ```

---

## 🚀 第一步：啟動 Nav2 導航堆疊

在 Host 開啟一個新的終端機視窗，先進入 Docker 開發容器，再執行工作空間下的 `nav2.sh` 腳本（地圖路徑以容器內路徑為準，預設地圖通常放在 `/workspaces/maps/`）：

```bash
cd "/home/robot/wildbot_real car/wildbot_workspace-main"
# 1. 啟動並進入主容器終端機
sudo ./launch_shell.sh

# 2. 進入容器後，執行導航腳本並指定地圖
./nav2.sh /workspaces/maps/my_map.yaml
```

**⚠️ 啟動後，請靜待終端機輸出以下就緒日誌：**
`[lifecycle_manager_navigation]: Managed nodes are active`
這代表 Nav2 的所有核心生命週期節點（Planner, Controller, Behavior 等）已成功對齊並激活。

---

## 📍 第二步：設定初始位置（Initial Pose）

Nav2 導航節點就緒後，必須給予小車初始位置，AMCL 定位系統與成本地圖才能正常工作。

*   **方法一：在 RViz 中視覺化設定（推薦）**
    1. 在 RViz 的上方工具列，點擊 **「2D Pose Estimate」** 按鈕。
    2. 在地圖上對應小車實際物理位置的點，按住左鍵並**拖曳出車頭的方向箭頭**。
    3. 觀察小車的紅包雷達點雲（`/scan`）是否與地圖黑線重合。若有偏差，請多調整幾次。

*   **方法二：使用終端機指令設定（適用於快速重置/自動化）**
    另開一個終端機，執行 `sudo ./launch_shell.sh` 進入容器內，執行：
    ```bash
    source install/setup.bash
    ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
    ```

---

## 🎯 第三步：發布目標點（Goal Pose）引導移動

當小車完成初始定位後，發布目標點即可命令小車自主移動。

*   **方法一：在 RViz 中發布目標點**
    1. 在 RViz 上方工具列點擊 **「Nav2 Goal」** 按鈕。
    2. 在地圖上的目的地點點擊左鍵，並**拉出小車到達目的地時的朝向角度**。
    3. 小車將會自動規劃出綠色/藍色的全域路徑並開始移動。

*   **方法二：使用終端機指令移動**
    在容器終端機中，發布一個相對於地圖 (map) 座標系目標點（例如：前進 0.5 公尺）：
    ```bash
    ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
    ```

---

## 🛠️ 常見核心參數調整說明

導航設定檔位於：`workspaces/src/wildbot_nav2/config/nav2_params.yaml`。

### 1. 膨脹半徑調整 (`inflation_radius`)
為防止小車因為通道太窄而規劃失敗，可以將膨脹半徑縮小。
目前已設定為 **`0.35`** 公尺（適用於較窄的門寬、橋梁通道）。
```yaml
# 在 local_costmap 與 global_costmap 的 inflation_layer 底下：
inflation_radius: 0.35  # 單位：公尺
```

### 2. 控制指令時間戳支援 (`enable_stamped_cmd_vel`)
因 ROS2 Jazzy 底盤接收 `geometry_msgs/msg/TwistStamped`，Nav2 發送速度的節點均已設定為 `true`：
```yaml
controller_server:
  ros__parameters:
    enable_stamped_cmd_vel: true

velocity_smoother:
  ros__parameters:
    enable_stamped_cmd_vel: true

collision_monitor:
  ros__parameters:
    enable_stamped_cmd_vel: true

behavior_server:
  ros__parameters:
    enable_stamped_cmd_vel: true
```
