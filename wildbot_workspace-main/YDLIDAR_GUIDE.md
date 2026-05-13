# YDLIDAR 4ROS (TG30) ROS 2 Jazzy 使用指南

本文件記錄了 YDLIDAR 4ROS 雷達在 Wildbot 小車 (ROS 2 Jazzy / Ubuntu 24.04) 環境下的配置與修正說明。

## 1. 硬體連接與設備別名 (udev)

雷達預設被系統識別為 `/dev/ttyUSBx`，為了穩定性，我們使用了別名 `/dev/usb_lidar`。

*   **確認設備存在**：
    ```bash
    ls -l /dev/usb_lidar
    ```
    *(應顯示指向某個 ttyUSB 設備的連結)*

## 2. ROS 2 Jazzy 相容性修正 (核心改動)

原廠驅動程式是針對較舊的 ROS 2 版本編寫，在 Jazzy 版本中會遇到以下錯誤，我們已完成修正：

### A. 參數宣告修正 (C++)
在 `ydlidar_ros2_driver_node.cpp` 中，Jazzy 嚴格要求參數必須提供 **範本類型 (Template Type)**。
*   **修正前**：`node->declare_parameter("port", "/dev/ydlidar");`
*   **修正後**：`node->declare_parameter<std::string>("port", "/dev/ydlidar");`

### B. Launch 檔案語法修正 (Python)
Jazzy 的 Launch 系統更改了參數名稱。
*   `node_executable` ➔ `executable`
*   `node_name` ➔ `name`
*   `node_namespace` ➔ `namespace`

## 3. 雷達參數調校 (`ydlidar.yaml`)

針對 **YDLIDAR 4ROS (TG30)**，必須使用以下關鍵參數才能成功啟動：

| 參數 | 設定值 | 說明 |
| :--- | :--- | :--- |
| `port` | `/dev/usb_lidar` | 序列埠路徑 |
| `baudrate` | `512000` | 4ROS/TG系列通訊波特率 |
| `lidar_type` | `0` | **重要**：0 代表 TOF 序列埠模式 (2 是網路模式) |
| `intensity` | `false` | 關閉光強解析以避免 Checksum 錯誤 |
| `fixed_resolution` | `false` | 關閉固定解析度，允許點數動態浮動 |
| `sample_rate` | `20` | 20kHz 採樣頻率 |

## 4. 啟動方式

### 方法 A：手動調試啟動 (互動模式)
進入容器並手動執行：
```bash
sudo ./launch_shell.sh
# 進入容器後
source /workspaces/install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

### 方法 B：全機自動啟動 (背景模式)
直接在主機執行：
```bash
sudo ./scripts/00_start_all.sh
```

## 5. 常見問題排除

*   **Checksum error**：如果出現大量校驗錯誤，請檢查 `baudrate` 是否為 `512000`，且 `intensity` 必須與硬體實際輸出相符。
*   **Cannot bind to IP Address**：代表 `lidar_type` 被誤設為 `2` (網路模式)，請改回 `0`。
*   **Real points > fixed points**：代表雷達回傳點數超過預期，請將 `fixed_resolution` 設為 `false`。

---
*文件更新日期：2026-05-13*
