# 🌉 Wildbot 實體小車：競賽任務執行指南

本指南提供執行過橋任務與開門任務所需的完整終端機指令、執行順序以及異常排除步驟。

---

## 🚀 步驟 1：啟動底層硬體與通訊 (終端機 A - 主機端)

請在**主機 (Host) 終端機**執行以下指令，透過 Docker Compose 背景啟動底盤、相機、雷達與 Bridge 服務：

```bash
# 切換至專案根目錄
cd "/home/robot/wildbot_real car/wildbot_workspace-main"

# 啟動所有 Docker 服務
sudo ./scripts/00_start_all.sh
```

> [!NOTE]  
> 啟動完畢後，您可以使用 `docker ps` 確認所有容器（如 `compose-kros_car-1`、`compose-ydlidar-1` 等）皆處於 `Up` 狀態。

---

## 🛣️ 任務一：執行【過橋任務】

### 1. 啟動過橋 YOLO 偵測 (終端機 B - 容器內)
```bash
docker exec -it compose-kros_car-1 bash
./yolo_bridge_3d.sh
```

### 2. 執行過橋對齊與直行任務 (終端機 C - 容器內)
```bash
docker exec -it compose-kros_car-1 bash
python3 bridge_align_imu.py bridge cross 0.15 1.0
```

---

## 🚪 任務二：執行【開門任務】

### 1. 啟動門把 YOLO 偵測 (終端機 B - 容器內)
*注意：本腳本會自動清理舊的過橋 YOLO 行程，並加載專屬的 `bearknob` 開門模型。*
```bash
docker exec -it compose-kros_car-1 bash
./yolo_door.sh
```

### 2. 執行自動開門任務 (終端機 C - 容器內)
```bash
docker exec -it compose-kros_car-1 bash
python3 door_opener.py knob 0.08
```

---

## 🛡️ 安全防護與注意事項 (競賽規範)

> [!IMPORTANT]  
> 1. **機械手臂自動防燒毀**：手臂夾取動作後 **0.5 秒** 會自動退回 **$2^\circ$** 以釋放堵轉壓力，請勿手動修改此保護。
> 2. **馬達過熱停機保護 (Software E-Stop)**：當任一馬達溫度 $\ge 70^\circ\text{C}$ 時，控制程式會自動攔截所有移動與轉向指令。必須待溫度降回 $60^\circ\text{C}$ 以下方可解除鎖定。
> 3. **緊急避障與停車**：若任務執行途中發生異常，可在終端機 C 按下 `Ctrl + C`，程式將發送緊急停車訊號。
