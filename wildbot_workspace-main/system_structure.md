# Wildbot 系統目錄與 Docker 掛載結構指南

為了確保系統穩定，所有 Docker Compose 服務已統一掛載規則。

## 1. 核心路徑定義 (Path Definitions)

*   **主機根目錄**: `/home/robot/wildbot_real car/wildbot_workspace-main`
*   **容器內根目錄**: `/workspaces`
*   **編譯產物路徑**: `/workspaces/install/setup.bash` (這是所有服務的核心依賴)

## 2. 各服務掛載與啟動清單 (Service Inventory)

| 服務名稱 (Service) | Compose 檔案 | 掛載路徑 (Host -> Container) | 啟動指令 (Source Path) |
| :--- | :--- | :--- | :--- |
| **底盤/手臂** | `kros_car.yml` | `.../wildbot_workspace-main:/workspaces` | `/workspaces/install/setup.bash` |
| **Rosbridge** | `rosbridge_server.yml` | `.../wildbot_workspace-main:/workspaces` | `/workspaces/install/setup.bash` |
| **LiDAR 驅動** | `ydlidar.yml` | `.../wildbot_workspace-main:/workspaces` | `/workspaces/install/setup.bash` |
| **IMU 驅動** | `kros_car.yml` | `.../wildbot_workspace-main:/workspaces` | `/workspaces/workspaces/imu_ws/install/setup.bash` |
| **手把控制** | `kros_car.yml` | `.../wildbot_workspace-main:/workspaces` | `/opt/ros/jazzy/setup.bash` |
| **YOLO (手動)** | `yolo.sh` | (繼承 kros_car 容器) | `/workspaces/workspaces/install/setup.bash` |

## 3. 重要注意事項 (Critical Rules)

1.  **路徑一致性**: 
    - 所有的 YAML 檔案中的 `volumes` 必須指向主機的 `wildbot_workspace-main` 目錄。
    - 內部指令必須先 `source /workspaces/install/setup.bash` 才能執行自定義功能。

2.  **YOLO 手動啟動**:
    - YOLO 已從 `docker-compose_kros_car.yml` 移除，改為手動啟動以節省資源。
    - 指令: `docker exec -it compose-kros_car-1 bash` -> `./yolo.sh`

3.  **Foxglove 連接**:
    - Rosbridge 必須掛載 `/workspaces` 才能讀取到 `yolo_msgs` 的訊息定義。
    - 如果 Foxglove 沒畫面，請先檢查 `docker logs compose-rosbridge-1` 是否有路徑錯誤。

4.  **IMU 特殊路徑**:
    - IMU 驅動位於獨立的工作空間，因此其 source 路徑為 `/workspaces/workspaces/imu_ws/install/setup.bash`。

---
*Last Updated: 2026-05-15*
