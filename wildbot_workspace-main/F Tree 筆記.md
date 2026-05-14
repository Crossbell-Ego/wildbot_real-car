# 🤖 Wildbot 容器內關鍵檔案位置與 TF 修復指南

本指南記錄了 Wildbot 機器人模型與控制器的關鍵檔案路徑（位於 Docker 容器內），以及如何配置 EKF 融合層與修復 SLAM Toolbox 所需的 TF Tree。

---

## 📂 關鍵檔案路徑 (容器內 - Container Paths)

| 檔案類型 | 容器內絕對路徑 | 說明 |
| :--- | :--- | :--- |
| **機器人模型 (URDF/XACRO)** | `/robot_ws/src/wildbot-car-description/urdf/kros_car.xacro` | 定義機器人結構、關節與感測器位置。 |
| **控制器設定 (YAML)** | `/robot_ws/src/wildbot-car/config/controllers.yaml` | 設定底盤里程計 (odom) 是否發布 TF。 |
| **EKF 融合配置 (YAML)** | `/workspaces/workspaces/src/wildbot_localization/config/ekf.yaml` | **(新)** 設定輪速計與 IMU 的融合參數。 |
| **定位啟動檔 (Python)** | `/workspaces/workspaces/src/wildbot_localization/launch/ekf_launch.py` | **(新)** 啟動 EKF 節點並發布融合後的 TF。 |
| **主要啟動檔 (Python)** | `/robot_ws/src/wildbot-car/launch/bringup.launch.py` | 系統核心啟動邏輯，包含載入 URDF 與啟動控制器。 |

---

## 🛰️ 里程計融合層 (Sensor Fusion - EKF)

為了提升定位精度，我們使用 `robot_localization` 將「輪式里程計」與「IMU」數據進行融合。

### 1. 資料流架構
- **輸入數據**：`/base_controller/odom` (輪速計) + `/imu/data` (IMU)
- **處理中心**：`ekf_filter_node`
- **輸出成果**：發布精確的 `/odom` 主題，並提供 `odom -> base_link` 坐標變換。

### 2. 重要設定事項
> [!IMPORTANT]
> **防止 TF 衝突**：當使用 EKF 融合層時，必須確保 `base_controller` **不要**發布 `odom` 變換。
> **檢查點**：在 `/robot_ws/src/wildbot-car/config/controllers.yaml` 中，`enable_odom_tf` 必須設為 **`false`**。

---

## 🛠️ TF 樹修復步驟

### 1. 補齊雷達變換 (`base_link` -> `laser`)
**目標檔案：** `/robot_ws/src/wildbot-car-description/urdf/kros_car.xacro`
請在檔案末尾的 `</robot>` 之前加入固定變換：
```xml
  <link name="laser"/>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser"/>
    <origin xyz="0.1 0 0.15" rpy="0 0 0"/> 
  </joint>
```

---

## 🔄 如何套用與編譯？

修改完成後，必須在容器內重新編譯相關套件：

1. **進入容器**：`docker exec -it compose-kros_car-1 bash`
2. **編譯基礎包**：
   ```bash
   cd /robot_ws && colcon build --packages-select kros_car kros_car_description
   ```
3. **編譯定位包**：
   ```bash
   cd /workspaces && colcon build --packages-select wildbot_localization
   ```
4. **重啟全機**：在宿主機執行 `sudo ./scripts/00_start_all.sh`

---

## 🔍 驗證指令
*   **查看 TF 樹 (確認 odom 由 ekf 提供)**：`ros2 run tf2_tools view_frames`
*   **觀察融合數據輸出**：`ros2 topic echo /odom`
*   **確認雷達 ID**：`ros2 topic echo /scan --once | grep frame_id`

---

## 🔍 搜尋技巧 (萬一檔案路徑變了)
*   **搜尋模型檔**：`find /robot_ws -name "*.xacro" | grep description`
*   **搜尋控制器設定**：`find /robot_ws -name "controllers.yaml"`