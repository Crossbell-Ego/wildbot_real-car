# Wildbot YOLO 整合與實作指南

## 1. 專案概述 (Overview)
本實作目標是在 **Wildbot** (AMD Ryzen AI 平台) 上整合 YOLO 物體偵測。透過對話題的重新對接與 CPU 效能優化，實現了基於彩色壓縮影像的 3D 目標定位。

## 2. 技術棧與環境 (Tech Stack)
- **核心架構**: ROS 2 Jazzy (Dockerized)
- **AI 框架**: Ultralytics YOLOv12 / v26 (Python 3.12 + ROCm 6.1)
- **硬體平台**: AMD Ryzen AI 7 350 (Radeon 860M GPU 加速模式)
- **輸入源**: 
    - 影像: `/camera/color/image_raw/compressed`
    - 深度: `/camera/depth/image_raw`

## 3. 執行與操作步驟 (How to Run)

比照 SLAM 的模式，YOLO 已設定為手動啟動：

### 步驟 1: 啟動基礎服務
在宿主機執行腳本，這會啟動底盤、相機與雷達：
```bash
sudo ./scripts/00_start_all.sh
```

### 步驟 2: 手動開啟 YOLO
進入小車主容器後，根據任務需求選擇執行模式：

#### 模式 A：2D 追蹤模式 (節省資源)
適用於一般的物體辨識與追蹤，不涉及空間座標。
```bash
docker exec -it compose-kros_car-1 bash
# 進入容器後執行
./yolo.sh
```

#### 模式 B：3D 定位模式 (空間抓取)
適用於機械手臂抓取任務，會將物體投影至 `base_link` 座標系。
```bash
docker exec -it compose-kros_car-1 bash
# 進入容器後執行
./yolo_3d.sh
```

## 4. 關鍵配置與優化 (Optimization)

### A. AMD 平台的 GPU 加速模式 (ROCm)
透過安裝 **AMD ROCm 6.1** 版 PyTorch，系統已成功啟用 **Radeon 860M** 整合顯卡進行硬體加速，大幅提升推論 FPS。
- **裝置代號**: 在 PyTorch 與 YOLO 參數中使用 `device:=cuda:0`。
- **驅動環境**: 容器需掛載 `/dev/kfd` 與 `/dev/dri` 並設定環境變數 `HSA_OVERRIDE_GFX_VERSION=11.0.0` 以確保 RDNA 3.5 架構相容性。

### B. 壓縮影像話題對接
為了節省頻寬，YOLO 節點直接訂閱 `/camera/color/image_raw/compressed`。這已在 `yolo_node.py` 中透過相對話題路徑完成修正。

### C. 3D 投影功能
系統目前支援動態切換：
- **預設模式 (`yolo.sh`)**: `use_3d:=False`，降低 CPU 負擔。
- **3D 模式 (`yolo_3d.sh`)**: `use_3d:=True`，自動訂閱深度圖並激活 `detect_3d_node`。

## 5. 觀察與驗證 (Verification)

### 查看 3D 偵測結果
在容器內執行：
```bash
ros2 topic echo /yolo/detections_3d
```

### 查看標註後的除錯影像
在 Foxglove 中訂閱：
- `/yolo/dbg_image/compressed` (這會顯示帶有邊框與標籤的壓縮影像)

## 6. 下一步建議 (Next Steps)
- **OpenVINO 加速**: 未來可將 `.pt` 模型轉換為 `.xml` (OpenVINO) 格式，以利用 AMD 的 NPU 進行加速。
- **追蹤功能**: 已整合 `bytetrack`，可實現在複雜環境下的物體持續追蹤。

---

## 🛠️ 實體小車部署與除錯紀錄 (2026-05-15)

在實體小車 (Wildbot) 上部署時，我們遇到了以下問題並已完成修復：

### 1. 影像同步逾時 (Synchronization Timeout)
*   **現象**：`yolo_node` 有數據，但 `debug_node` 或 `tracking_node` 無法輸出，Foxglove 沒畫面。
*   **原因**：實體小車 CPU 負載較高，影像處理延遲導致時間戳誤差超過預設的 `0.5s`。
*   **修復**：將 `ApproximateTimeSynchronizer` 的 `slop` (容許誤差) 放寬至 **`2.0s`**。
*   **檔案**：`debug_node.py`, `tracking_node.py`。

### 2. 缺失追蹤依賴 (Missing Dependencies)
*   **現象**：`tracking_node` 卡在 `Configuring...` 階段並報出 `Transitioning failed`。
*   **原因**：容器內缺少 `lap` (Linear Assignment) 與 `filterpy` 庫，導致 ByteTrack 演算法無法初始化。
*   **修復**：執行 `pip install lap filterpy`。

### 3. 生命周期管理衝突 (Lifecycle Race Condition)
*   **現象**：出現 `Unknown transition requested` 或 `Node not found` 警告。
*   **原因**：Python 代碼中的「自我激活」與 `yolo.sh` 的「外部激活」產生競態衝突。
*   **修復**：註解掉 Python 檔案 `main()` 裡的 `trigger_...` 代碼，統一由 `yolo.sh` 管理節點狀態。

### 4. 影像傳輸優化
*   **修復**：將 `debug_node` 的輸出從原始影像改為 **`CompressedImage (jpeg)`**，大幅降低 Foxglove 監看時的網路負載。

### 5. OpenVINO 效能優化 (2026-05-15)
*   **環境**：在 AMD Ryzen AI 平台，使用 OpenVINO 推論速度比 PyTorch 快 2-3 倍。
*   **模型轉換指令**：
    ```bash
    yolo export model=bearknob.pt format=openvino dynamic=True
    ```
*   **關鍵坑洞 - 動態尺寸 (Dynamic Shape)**：
    *   **現象**：報錯 `Can't set the input tensor... because the model input (shape=[1,3,640,640]) and the tensor (shape=(1,3,480,640)) are incompatible`。
    *   **原因**：預設導出為固定尺寸 (Static)，無法處理解析度非 640x640 的相機影像。
    *   **對策**：導出時必須加上 `dynamic=True`。

### 6. 3D 定位黃金準則 (預備開啟)
*   **深度不壓縮**：必須訂閱 `/camera/depth/image_raw`，避免 JPEG 損壞深度數值。
*   **底盤過濾**：已在 `detect_3d_node.py` 加入 `depth > 0.42m` 過濾，防止拍到機器人底盤。
*   **相信 TF 樹**：不手動交換 X/Y 座標，完全依賴 `/tf` 進行空間投射。

### 7. AMD GPU (ROCm) 硬體加速部署 (2026-05-16)
*   **任務目標**：由 CPU 推論切換至 **AMD GPU** 全速運行。
*   **修復與設定重點**：
    *   **鏡像固化**：將 ROCm 版 PyTorch (`--index-url https://download.pytorch.org/whl/rocm6.1`) 寫入 `Dockerfile`，解決每次重啟都要重新安裝的問題。
    *   **模型預載**：在 `Dockerfile` 中加入模型預抓指令 (`YOLO('yolov8n.pt')`)，達成「開箱即用」不再等待下載。
    *   **權限修正**：更新 `launch_shell.sh` 加入 `video` 與 `render` 使用者群組，並自動掛載 GPU 設備文件。
    *   **兼容性修正**：針對最新款 Ryzen AI 處理器，加入 `HSA_OVERRIDE_GFX_VERSION=11.0.0` 強制啟用硬體加速。
