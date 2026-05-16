# Wildbot 深度 USB 設備綁定指南 (udev KERNELS)

## 1. 為什麼需要「深度綁定」？
當系統中存在多個相同型號的感測器（如兩個相同的 LiDAR 或 IMU）時，它們的 `idVendor` 和 `idProduct` 甚至 `Serial Number` 可能完全一致。傳統方式無法區分它們，會導致 `Symlink` 衝突或隨機分配。

## 2. 查詢物理路徑 (KERNELS)
使用以下指令查詢裝置插在主機上的具體位置：
```bash
udevadm info -q path -n /dev/ttyUSBX
```
*範例結果解析：*
`/devices/pci.../usb1/1-2/1-2:1.0/ttyUSB1/...`
- **1-2**：代表 USB 控制器 1 的第 2 個埠位。
- **1-2:1.0**：代表該埠位的第 1 個介面（通常 Serial 設備都是 :1.0）。

## 3. 撰寫 udev 規則範例
檔案位址：`/etc/udev/rules.d/99-wildbot-devices.rules`

```text
# 鎖定插在 USB 1-2 埠位的裝置為 IMU
KERNEL=="ttyUSB*", KERNELS=="1-2:1.0", SYMLINK+="imu_a9", MODE="0666"

# 鎖定插在 USB 5-2 埠位的裝置為 雷達
KERNEL=="ttyUSB*", KERNELS=="5-2:1.0", SYMLINK+="usb_lidar", SYMLINK+="ydlidar", MODE="0666"
```

## 4. 套用指令
```bash
sudo cp scripts/99-wildbot-imu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 5. 常見問題 (Q&A)
**Q: 如果我換了 USB 插孔怎麼辦？**
A: `Symlink` 會失效。您必須重新執行第 2 步獲取新的 `KERNELS` 路徑，並修改規則檔案。

**Q: 我怎麼知道哪個物理路徑對應哪個孔？**
A: 先拔掉所有 USB 裝置，插上其中一個，執行 `dmesg | tail` 查看它是 `ttyUSB0` 還是 `ttyUSB1`，再用 `udevadm info` 查路徑。重複此步驟標定所有孔位。
