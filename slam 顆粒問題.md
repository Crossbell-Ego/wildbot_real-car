# 🤖 Wildbot SLAM 建圖噪點與穿牆顆粒問題排查與處解決方案

在實體小車建圖 (SLAM) 的過程中，常會遇到雷達訊號在牆面後方產生離散的顆粒雜訊，或是因為反射、玻璃、穿透縫隙而在地圖中形成「穿牆顆粒」及「散落點」，進而干擾導航與建圖品質。

本文件記錄了該問題的根本成因、對應解決方案與參數調整指引。

---

## 🔍 問題成因分析

1. **雷達超大量程接收弱訊號**：
   * 雷達預設最大量程高達 `64.0` 米，在遠距離或面對高反射性材質（如金屬、壓克力、玻璃）時，容易收到不穩定的微弱多次反射訊號，並被解算成無效的離散點。
2. **孤立噪點 (Outliers)**：
   * 雷達射線穿過門縫、玻璃或由鏡面反射（Specular Reflection）折射出去，在空白處或牆後產生單個或少數個孤立的雷達測距點。
3. **過低的建圖判定門檻**：
   * SLAM Toolbox 的 `occupancy_threshold` 預設僅為 `0.1` (10%)，意即只要該柵格有 10% 的射線打中，就會直接判定為障礙物，導致偶爾穿透的雜訊顆粒迅速被固化在牆面後方。

---

## 🛠️ 改善解決方案與設定檔異動

我們透過 **「資料源頭限幅」** ➡️ **「鄰群孤立濾波」** ➡️ **「建圖門檻調校」** 三道關卡進行了全面改善。

### 第一關：雷達驅動限幅與無效值過濾 (`ydlidar.yaml`)
* **修改檔案**：`workspaces/src/ydlidar_ros2_driver/params/ydlidar.yaml`
* **調整參數**：
  * `range_max`: 由原本的 `64.0` 米縮減至 **`10.0`** 米（因室內賽道環境通常不超過 10 米，限制在此範圍可過濾大於 10 米的所有噪點）。
  * `range_min`: 設定為 **`0.15`** 米（避免車體邊緣盲區造成噪點）。
  * `invalid_range_is_inf`: 修改為 **`true`**，將超出或無效的測距資料發布為 `inf`，讓 SLAM 核心直接忽略。

### 第二關：新增半徑鄰群孤立點濾波器 (`lidar_pkg`)
在雷達濾波節點中實作了孤立噪點濾波（類似 PCL 的 Radius Outlier Removal）。
* **修改檔案**：`workspaces/src/lidar_pkg/lidar_pkg/lidar_nan_value_filter_node.py`
* **調整參數 (`self_filter.yaml`)**：
  ```yaml
  # 孤立點濾波器配置
  outlier_filter_enabled: true
  outlier_max_neighbor_diff: 0.2  # 鄰近點的最大距離差限制在 20cm 內
  outlier_window_size: 2          # 檢查前後各 2 個鄰居點（繞 360 度圈）
  outlier_min_neighbors: 1        # 至少需有 1 個鄰近支援點，否則判定為孤立顆粒
  ```
* **原理**：正常的牆面或物體會反射連續的雷達束（相鄰點的距離非常相近）；而穿牆的顆粒通常是單點孤立的（與相鄰點距離差通常達數米）。此過濾器會將沒有鄰居支撐的孤立顆粒直接轉為 `inf`。

### 第三關：調高建圖門檻與更新間距 (`wildbot_slam`)
* **修改檔案**：`workspaces/src/wildbot_slam/config/mapper_params_online_async.yaml`
* **調整參數**：
  * `max_laser_range`: 由 `25.0` 縮小為 **`10.0`**。
  * `minimum_travel_distance` & `minimum_travel_heading`: 從 `0.1` 提高為 **`0.25`**（位移達 25cm 或轉動達 0.25 弧度時才進行地圖圖表更新，避免車子原地微小晃動時寫入噪點）。
  * `occupancy_threshold`: 從 `0.1` 提高至 **`0.25`** (25%)，必須有更穩定的雷達訊號連續擊中，該區域才會在地圖中被標記為黑色障礙物。

---

## 📈 後續維護與參數調校指引

若未來在不同場地建圖仍遇到顆粒噪點，可依序調整以下參數：

1. **若仍有少數穿牆顆粒**：
   * 可將 `self_filter.yaml` 中的 `outlier_max_neighbor_diff` 調得更嚴格（例如 `0.15`），或是將 `outlier_min_neighbors` 提高為 `2`（要求至少有 2 個相鄰點類似）。
2. **若地圖邊緣有零碎的虛幻黑點**：
   * 可繼續調高 `mapper_params_online_async.yaml` 中的 `occupancy_threshold`（例如調整到 `0.3`），以增加判定障礙物的嚴格度。
3. **若牆壁出現斷線或不完整**：
   * 表示過濾太強。可放寬 `outlier_max_neighbor_diff`（例如 `0.3`），或降低 `occupancy_threshold`（回退至 `0.15` 或 `0.2`）。

---
💡 **注意**：每次修改完成後，請確保在容器中執行 `colcon build`，並使用 `docker restart compose-lidar_pkg-1 compose-ydlidar-1` 重啟容器以套用最新的參數配置。
