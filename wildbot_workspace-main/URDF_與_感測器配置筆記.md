# 🏗️ Wildbot URDF 與感測器配置手冊

本手冊專門記錄 Wildbot 機器人的實體結構定義 (URDF/XACRO) 以及如何手動修正感測器坐標變換 (TF)。

---

## 📂 關鍵檔案路徑 (容器內)

*   **模型定義主檔**：`/robot_ws/src/wildbot-car-description/urdf/kros_car.xacro`
*   **感測器 XACRO**：`/robot_ws/src/wildbot-car-description/urdf/sensors/_d435.urdf.xacro` (若有分開定義)
*   **編譯路徑**：`/robot_ws`

---

## 🛠️ 常見修正動作

### 1. 修正/新增雷達坐標 (`laser`)
若 SLAM Toolbox 報錯找不到 `laser` 或 `laser_frame`，請在 `kros_car.xacro` 末尾（`</robot>` 標籤之前）加入以下定義：

```xml
  <!-- LiDAR Link -->
  <link name="laser"/>

  <!-- LiDAR Joint: 定義雷達相對於機器人中心的物理位置 -->
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser"/>
    <!-- xyz: 前後, 左右, 高度 (單位: 公尺) -->
    <!-- rpy: 翻滾, 俯仰, 偏航 (單位: 弧度) -->
    <origin xyz="0.1 0 0.15" rpy="0 0 0"/> 
  </joint>
```

### 2. 調整感測器偏移 (Calibration)
當發現雷達或相機的點雲與實際物體有偏差時，請修改對應 Joint 的 `<origin>` 參數：
- **xyz**: 增加 `x` 會將感測器向前移，增加 `z` 會向上移。
- **rpy**: 調整 `yaw` (第三個數值) 可以修正雷達安裝歪斜的問題。

---

## 🔄 編譯與套用流程

URDF 修改後必須重新編譯 Package 才能生效：

1.  **進入控制容器**：
    ```bash
    docker exec -it compose-kros_car-1 bash
    ```
2.  **執行編譯**：
    ```bash
    cd /robot_ws
    colcon build --packages-select kros_car_description
    ```
3.  **重啟服務**：
    在宿主機執行 `sudo ./scripts/00_start_all.sh` 以載入新的模型。

---

## 🔍 驗證模型
- **檢查 TF Tree**：使用 `ros2 run tf2_tools view_frames` 檢查 `base_link` -> `laser` 是否連通。
- **檢查 Robot Description**：`ros2 topic echo /robot_description` 可查看當前載入的 XML 內容。

> [!TIP]
> 修改 URDF 時，建議先在本地端使用 RViz 預覽，確認坐標軸方向 (紅:X, 綠:Y, 藍:Z) 是否符合機器人前進方向。
