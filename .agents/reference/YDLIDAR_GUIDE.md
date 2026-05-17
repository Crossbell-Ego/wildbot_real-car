# YDLIDAR 4ROS (TG30) ROS 2 Jazzy 使用指南 (2026-05-14 更新)

本文件記錄了 YDLIDAR 4ROS 雷達在 Wildbot 小車 (ROS 2 Jazzy / Ubuntu 24.04) 環境下的配置與修正說明。

## 1. 硬體連接與設備別名 (物理路徑綁定)

由於 Wildbot 使用的雷達與 IMU 晶片相同 (CP2102)，傳統的 Vendor/Product ID 綁定會失效。我們採用 **物理插槽路徑 (USB Path)** 進行鎖定。

*   **雷達物理路徑**：`pci-0000:62:00.0-usb-0:2:1.0-port0` (對應 `/dev/usb_lidar`)
*   **IMU 物理路徑**：`pci-0000:68:00.0-usb-0:2:1.0-port0` (對應 `/dev/imu_a9`)

### 修正後的 udev 規則 (/etc/udev/rules.d/10-wildbot.rules)：
```bash
SUBSYSTEM=="tty", ENV{ID_PATH}=="pci-0000:62:00.0-usb-0:2:1.0", MODE="0666", SYMLINK+="usb_lidar"
SUBSYSTEM=="tty", ENV{ID_PATH}=="pci-0000:68:00.0-usb-0:2:1.0", MODE="0666", SYMLINK+="imu_a9"
```

## 2. Docker 環境配置 (關鍵)

在 Docker 中運行高波特率 (512000) 的雷達，必須具備以下條件：

1.  **特權模式 (Privileged)**：必須在 `docker-compose.yml` 加入 `privileged: true`，否則會出現大量的 `Checksum error`。
2.  **硬體映射**：推薦使用物理路徑映射，確保容器內外的設備對應永不漂移。

## 3. ROS 2 Jazzy 相容性修正

1.  **C++ 修正**：`declare_parameter` 必須帶有類型範本，例如 `node->declare_parameter<std::string>(...)`。
2.  **Launch 修正**：`LifecycleNode` 的建構子參數已由 `node_name` 改為 `name`。

## 4. 雷達最佳參數設定 (`ydlidar_4ros.yaml`)

針對 **TG30** 型號，請務必使用以下配置：

| 參數 | 設定值 | 說明 |
| :--- | :--- | :--- |
| `port` | `/dev/usb_lidar` | 鎖定後的序列埠路徑 |
| `baudrate` | `512000` | 標準通訊波特率 |
| `lidar_type` | `0` | TOF 模式 (不要設為 1，否則 SDK 會報錯) |
| `sample_rate` | `20` | 原生採樣率 20kHz |
| `intensity` | `true` | **必填**：TG30 會發送 16-bit 強度數據，不開啟會導致校驗錯誤 |
| `frequency` | `10.0` | 標準掃描頻率 10Hz |

## 5. 自車反射過濾 (`/scan_tmp` -> `/scan`)

Wildbot 的雷達會掃到車體本身時，不要直接讓 SLAM/Nav2 訂閱原始 `/scan`。目前流程設定為：

1. `ydlidar_ros2_driver` 原始資料 remap 到 `/scan_tmp`。
2. `lidar_pkg/lidar_nan_value_filter_node` 訂閱 `/scan_tmp`。
3. filter 節點把 NaN、0、超出範圍、以及自車遮罩區域改成 `+inf`，再發布乾淨的 `/scan`。

自車遮罩設定檔：

```bash
wildbot_workspace-main/workspaces/src/lidar_pkg/config/self_filter.yaml
```

遮罩格式為 `"start_deg:end_deg:max_range_m"`，角度以雷達 frame 為準：

```yaml
self_mask_sectors:
  - "-45:45:0.55"      # 前方 90 度，55cm 內視為車體
  - "135:180:0.45"     # 後方左半
  - "-180:-135:0.45"   # 後方右半
```

調整建議：

*   在 RViz 顯示 `/scan_tmp`，找出固定貼著車體的角度。
*   只增加會掃到車體的角度區間，不要把整圈近距離都濾掉，否則近距離障礙物會被隱藏。
*   修改後重新啟動 LiDAR/filter 服務，確認 `/scan` 中自車殘影消失。

## 6. 常見錯誤診斷

*   **Checksum error**：
    *   檢查是否開啟了 `privileged: true`。
    *   檢查 `intensity` 是否設為 `true`。
    *   物理檢查：確認 USB 補電線是否有插在外部 5V 電源上，電力不足會導致資料毀損。
*   **Cannot bind to serial port**：
    *   檢查 `/dev/usb_lidar` 是否正確指向雷達而非 IMU。
    *   確認沒有其他容器正在運行並佔用同一個埠口。

---
*文件更新日期：2026-05-14 (Debug 成功版本)*
