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

## 5. 常見錯誤診斷

*   **Checksum error**：
    *   檢查是否開啟了 `privileged: true`。
    *   檢查 `intensity` 是否設為 `true`。
    *   物理檢查：確認 USB 補電線是否有插在外部 5V 電源上，電力不足會導致資料毀損。
*   **Cannot bind to serial port**：
    *   檢查 `/dev/usb_lidar` 是否正確指向雷達而非 IMU。
    *   確認沒有其他容器正在運行並佔用同一個埠口。

---
*文件更新日期：2026-05-14 (Debug 成功版本)*
