# 🧭 Wildbot IMU (Handsfree A9) 設定手冊

本文件紀錄了 Wildbot 實體小車 IMU 的硬體通訊、驅動配置與自動化啟動設定。

## 1. 硬體連接與映射 (udev)
IMU 使用 Handsfree A9 模組，透過 USB 連接，已在宿主機綁定固定 Symlink：
- **設備路徑**: `/dev/imu_a9` (指向 `/dev/ttyUSB1`)
- **通訊參數**: Baudrate `921600`

## 2. 驅動工作空間 (Workspace)
- **宿主機路徑**: `/home/robot/wildbot_real car/wildbot_workspace-main/workspaces/imu_ws`
- **容器內路徑**: `/workspaces/workspaces/imu_ws`

## 3. 核心節點 (ROS 2 Nodes)
我們在 `handsfree_imu_ros2` 包中配置了兩個節點：

| 節點名稱 | 指令 | 輸出 Topic | 說明 |
| :--- | :--- | :--- | :--- |
| **標準數據節點** | `imu_a9_data_node` | `/imu/data` | 提供標準 `sensor_msgs/Imu` (四元數、加速度、角速度)。**用於 EKF 融合。** |
| **尤拉角節點** | `imu_a9_node` | `/imu/rpy_deg` | 僅提供 Roll/Pitch/Yaw 角度數據 (Vector3)。 |

## 4. 自動化啟動設定 (Docker Compose)
IMU 驅動已整合進 `docker-compose_kros_car.yml`，會隨系統啟動自動執行。

## 5. 數據驗證與解讀 (Data Verification)
經 2026-05-15 實測，`/imu/data` 輸出狀態如下：

*   **更新頻率**: 穩定在 **100 Hz**。
*   **座標系 (Frame ID)**: `imu_link`。

### 數值含義說明：
| 數據項 | 實測值範例 | 物理含義與驗證 |
| :--- | :--- | :--- |
| **Orientation** | `z: 0.22, w: 0.97` | 四元數格式，代表水平朝向。靜止時數值應保持穩定。 |
| **Angular Velocity** | `x/y/z ≈ 0.00` | 小車靜止時，三個軸的角度變化率應趨近於零。 |
| **Linear Acceleration** | `z ≈ -9.79` | **重力加速度驗證**。Z 軸數值接近 -9.8 代表感測器平放且運作正常。 |

### 快速檢查指令：
```bash
# 檢查頻率
ros2 topic hz /imu/data

# 檢查單幀數據
ros2 topic echo /imu/data --once
```
**自動執行指令**:
```bash
source /workspaces/workspaces/imu_ws/install/setup.bash && \
ros2 run handsfree_imu_ros2 imu_a9_data_node --ros-args -p port:=/dev/imu_a9 -p baud:=921600
```

## 5. 常見問題與偵錯 (Troubleshooting)

### 發生 `SerialException` (Port busy)
- **原因**: 通常是 `ydlidar` 驅動誤抓了 `/dev/ttyUSB1`。
- **檢查**: 在宿主機執行 `sudo fuser /dev/imu_a9`。
- **修復**: 確保 `ydlidar_4ros.yaml` 中的 `port` 設定為 `/dev/usb_lidar` 而非具體的 `ttyUSB` 編號。

### 驗證數據是否正常
```bash
# 檢查是否有數據輸出
ros2 topic echo /imu/data --once

# 檢查變換是否正確 (應連接至 imu_link)
ros2 run tf2_tools view_frames
```

---
*最後更新日期: 2026-05-14*