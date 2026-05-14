Edited urdf%20%E8%88%87%20TF%20Tree%20%E7%AD%86%E8%A8%98.md
Viewed urdf%20%E8%88%87%20TF%20Tree%20%E7%AD%86%E8%A8%98.md:3-36

我已經更新了你的筆記，特別強化了**容器內檔案路徑**的詳細說明。這份清單能幫你快速定位並進行修改。

### [urdf 與 TF Tree 筆記.md](file:///home/robot/wildbot_real%20car/wildbot_workspace-main/urdf%20%E8%88%87%20TF%20Tree%20%E7%AD%86%E8%A8%98.md)

```markdown
# 🤖 Wildbot 容器內關鍵檔案位置與 TF 修復指南

本指南記錄了 Wildbot 機器人模型與控制器的關鍵檔案路徑（位於 Docker 容器內），以及如何修復 SLAM Toolbox 所需的 TF Tree。

---

## 📂 關鍵檔案路徑 (容器內 - Container Paths)

| 檔案類型 | 容器內絕對路徑 | 說明 |
| :--- | :--- | :--- |
| **機器人模型 (URDF/XACRO)** | `/robot_ws/src/wildbot-car-description/urdf/kros_car.xacro` | 定義機器人結構、關節與感測器位置。 |
| **控制器設定 (YAML)** | `/robot_ws/src/wildbot-car/config/controllers.yaml` | 設定底盤里程計 (odom) 是否發布 TF。 |
| **硬體參數設定 (YAML)** | `/robot_ws/src/wildbot-car/config/hardware.yaml` | 定義馬達連接埠、齒輪比等硬體資訊。 |
| **主要啟動檔 (Python)** | `/robot_ws/src/wildbot-car/launch/bringup.launch.py` | 系統核心啟動邏輯，包含載入 URDF 與啟動控制器。 |

---

## 🛠️ 修復步驟

### 1. 補齊雷達變換 (`base_link` -> `laser`)
**目標檔案：** `/robot_ws/src/wildbot-car-description/urdf/kros_car.xacro`

請在檔案末尾的 `</robot>` 之前加入：
```xml
  <link name="laser"/>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser"/>
    <origin xyz="0.1 0 0.15" rpy="0 0 0"/> <!-- 請根據實機修改 -->
  </joint>
```

### 2. 開啟里程計變換 (`odom` -> `base_link`)
**目標檔案：** `/robot_ws/src/wildbot-car/config/controllers.yaml`

搜尋 `base_controller` 區塊，將 `enable_odom_tf` 改為 `true`：
```yaml
base_controller:
  ros__parameters:
    enable_odom_tf: true
    base_frame_id: base_link
    odom_frame_id: odom
```

---

## 🔄 如何套用修改？

由於這些檔案位於 `/robot_ws` (容器內的 Workspace)，修改後必須重新編譯：

1. **進入容器：** `docker exec -it compose-kros_car-1 bash`
2. **編譯：** 
   ```bash
   cd /robot_ws
   colcon build --packages-select kros_car kros_car_description
   ```
3. **重新啟動系統：** 在宿主機執行 `sudo ./scripts/00_start_all.sh`

---

## 🔍 驗證指令
*   **查看 TF 樹：** `ros2 run tf2_tools view_frames`
*   **確認雷達 ID：** `ros2 topic echo /scan --once | grep frame_id` (應顯示 `laser`)
```

Edited urdf%20%E8%88%87%20TF%20Tree%20%E7%AD%86%E8%A8%98.md
Viewed urdf%20%E8%88%87%20TF%20Tree%20%E7%AD%86%E8%A8%98.md:1-29

為了確保你下次依然能快速找到這些檔案（萬一路徑發生變動），我在筆記中新增了**搜尋技巧**一節。

### [urdf 與 TF Tree 筆記.md](file:///home/robot/wildbot_real%20car/wildbot_workspace-main/urdf%20%E8%88%87%20TF%20Tree%20%E7%AD%86%E8%A8%98.md)

我已經將以下內容整合進你的筆記末尾：

```markdown
---

## 🔍 萬一檔案路徑變了怎麼辦？ (搜尋技巧)

如果未來工作空間結構調整，可以使用以下指令在容器內快速定位：

1. **搜尋模型檔 (XACRO/URDF):**
   ```bash
   find /robot_ws -name "*.xacro" | grep description
   ```
2. **搜尋控制器設定 (YAML):**
   ```bash
   find /robot_ws -name "controllers.yaml"
   ```
3. **確認 Package 安裝位置:**
   ```bash
   # 記得先 source 環境
   source /robot_ws/install/setup.bash
   ros2 pkg prefix kros_car_description
   ```

> [!TIP]
> 這些路徑僅在 `compose-kros_car-1` 容器內有效。如果你在宿主機 (Host) 尋找，這些檔案通常位於掛載的 Volume 中，或者是 Docker Image 的內部路徑。
```

這樣一來，無論檔案移動到哪裡，你都能透過這些指令找回來。你可以安心開始修改了！