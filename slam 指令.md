# 🤖 Wildbot 實體小車 SLAM 建圖操作指南

本指南詳細說明如何啟動 SLAM 建圖、使用遊戲手把遙控小車，以及在建圖完成後如何儲存地圖。

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

## 🚀 第一步：啟動 SLAM 建圖與手把遙控

在 Host 開啟一個新的終端機視窗，執行工作空間下的 `slam.sh` 腳本：

```bash
cd "/home/robot/wildbot_real car/wildbot_workspace-main"
./slam.sh
```

### 💡 `slam.sh` 腳本會自動完成以下工作：
1. **背景啟動遊戲手把遙控**：自動執行 `/workspaces/teleop.sh`（日誌記錄在 `/tmp/teleop.log`），此時您可直接用手把控制小車移動以進行掃圖。
2. **前景啟動 SLAM 建圖**：自動運行 `ros2 launch wildbot_slam slam_launch.py`。
3. **自動清理機制**：當您按下 `Ctrl + C` 結束建圖時，腳本會自動清理背景運行的遙控與手把節點，保持系統乾淨。

---

## 🎨 第二步：在 RViz 中觀察建圖

在您的操作電腦上開啟 **RViz2**：
1. 將 **Fixed Frame** 設定為 `map`。
2. 添加 **Map** 顯示項，並將話題 (Topic) 設定為 `/map`。
3. 添加 **LaserScan** 顯示項，並將話題設定為 `/scan`。
4. 使用手把遙控小車在場地內**緩慢移動**（特別是轉彎時要慢，否則雷達特徵容易匹配失敗），直到地圖完整建立。

---

## 💾 第三步：儲存地圖

當您在 RViz 中確認地圖建立完畢後，請**不要關閉建圖的 `slam.sh` 視窗**。請另開一個 Host 終端機視窗，進入容器並執行地圖儲存指令：

### 1. 進入容器內部終端機
```bash
cd "/home/robot/wildbot_real car/wildbot_workspace-main"
sudo ./launch_shell.sh
```

### 2. 執行儲存指令
在容器終端機中，執行以下指令將地圖存檔（地圖將保存在主機的 `maps` 資料夾中）：

*   **方法一：使用標準地圖發布工具（推薦，生成 .yaml 與 .pgm 導航用圖）**
    ```bash
    source install/setup.bash
    ros2 run nav2_map_server map_saver_cli -f /workspaces/maps/my_map --ros-args -p save_map_timeout:=10.0
    ```

*   **方法二：使用 SLAM Toolbox 內建服務儲存（供後續繼續建圖/載入用）**
    ```bash
    source install/setup.bash
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/workspaces/maps/my_map'}}"
    ```

儲存成功後，您會在 `maps/` 資料夾下看到兩個新檔案：
*   `my_map.pgm`（地圖柵格圖檔）
*   `my_map.yaml`（地圖設定檔，供 Nav2 導航載入使用）
