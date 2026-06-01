import argparse
import glob
import os
import struct
import sys
import time

import serial

HDR = 0x55

CMD_MOVE_TIME_WRITE      = 0x01
CMD_ID_WRITE             = 0x0D
CMD_LOAD_OR_UNLOAD_WRITE = 0x1F

# 讀取指令（LewanSoul / Hiwonder / HTD-45H 常見協定）
CMD_TEMP_READ            = 0x1A
CMD_POS_READ             = 0x1C

DEFAULT_PORT = "/dev/usb_robot_arm"


def calc_chk(frame: bytes) -> int:
    length = frame[3]
    s = sum(frame[2:2+length]) & 0xFF
    return (~s) & 0xFF


def build_frame(sid: int, cmd: int, params: bytes = b'') -> bytes:
    ln = len(params) + 3
    head = bytes([HDR, HDR, sid & 0xFF, ln & 0xFF, cmd & 0xFF])
    tmp = head + params
    return tmp + bytes([calc_chk(tmp)])


class HTD45H:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, 115200, timeout=0.08)

    def close(self):
        self.ser.close()

    def send(self, sid: int, cmd: int, params: bytes = b''):
        self.ser.write(build_frame(sid, cmd, params))
        self.ser.flush()

    def request(self, sid: int, cmd: int, params: bytes = b'', wait_s: float = 0.03) -> bytes | None:
        """
        送出讀取指令並接收回覆。
        回覆格式通常為：
        55 55 ID LEN CMD DATA... CHK
        """
        self.ser.reset_input_buffer()
        self.send(sid, cmd, params)
        time.sleep(wait_s)

        data = self.ser.read(32)
        if not data:
            return None

        # 找 55 55 開頭
        idx = data.find(bytes([HDR, HDR]))
        if idx < 0:
            return None
        data = data[idx:]

        if len(data) < 6:
            return None

        length = data[3]
        total_len = length + 3  # 55 55 + ID + LEN 後，LEN 含 ID? 此協定完整包長通常為 LEN + 3
        if len(data) < total_len:
            # 再補讀一次
            data += self.ser.read(total_len - len(data))

        if len(data) < total_len:
            return None

        frame = data[:total_len]

        # 驗證 checksum
        chk_calc = calc_chk(frame[:-1])
        chk_recv = frame[-1]
        if chk_calc != chk_recv:
            return None

        return frame

    def torque(self, sid: int, on: bool = True):
        self.send(sid, CMD_LOAD_OR_UNLOAD_WRITE, bytes([1 if on else 0]))

    def move(self, sid: int, deg: float, t_ms: int = 300):
        deg = max(0.0, min(240.0, float(deg)))
        pos = int(round(deg / 240.0 * 1000.0))
        t_ms = int(max(0, min(30000, int(t_ms))))
        self.send(sid, CMD_MOVE_TIME_WRITE, struct.pack("<HH", pos, t_ms))

    def set_id(self, old_id: int, new_id: int):
        self.send(old_id, CMD_ID_WRITE, bytes([new_id & 0xFF]))

    def read_temperature(self, sid: int) -> int | None:
        """
        讀取舵機溫度，回傳 °C。
        DATA 通常為 1 byte。
        """
        frame = self.request(sid, CMD_TEMP_READ)
        if frame is None:
            return None

        # frame: 55 55 ID LEN CMD DATA CHK
        if len(frame) < 7:
            return None

        return int(frame[5])

    def read_position_raw(self, sid: int) -> int | None:
        """
        讀取目前位置 raw 值，通常 0~1000 對應 0~240 度。
        DATA 通常為 little-endian int16 / uint16。
        """
        frame = self.request(sid, CMD_POS_READ)
        if frame is None:
            return None

        # frame: 55 55 ID LEN CMD POS_L POS_H CHK
        if len(frame) < 8:
            return None

        pos = struct.unpack("<h", frame[5:7])[0]

        # 有些舵機回傳可能短暫出現負值，保留 raw 給除錯
        return pos

    def read_position_deg(self, sid: int) -> float | None:
        raw = self.read_position_raw(sid)
        if raw is None:
            return None

        deg = raw / 1000.0 * 240.0
        return deg

    def read_status(self, sid: int) -> tuple[int | None, int | None, float | None]:
        """
        回傳：溫度°C、位置raw、位置角度°
        """
        temp = self.read_temperature(sid)
        time.sleep(0.02)
        raw = self.read_position_raw(sid)
        deg = None if raw is None else raw / 1000.0 * 240.0
        return temp, raw, deg


def ask_int(prompt: str, min_v: int = None, max_v: int = None) -> int:
    while True:
        s = input(prompt).strip()
        try:
            v = int(s)
            if min_v is not None and v < min_v:
                raise ValueError
            if max_v is not None and v > max_v:
                raise ValueError
            return v
        except:
            rng = ""
            if min_v is not None or max_v is not None:
                rng = f"（範圍 {min_v}~{max_v}）"
            print(f"輸入無效 {rng}，再試一次。")


def ask_float(prompt: str, min_v: float = None, max_v: float = None) -> float:
    while True:
        s = input(prompt).strip()
        try:
            v = float(s)
            if min_v is not None and v < min_v:
                raise ValueError
            if max_v is not None and v > max_v:
                raise ValueError
            return v
        except:
            rng = ""
            if min_v is not None or max_v is not None:
                rng = f"（範圍 {min_v}~{max_v}）"
            print(f"輸入無效 {rng}，再試一次。")


def print_status(dev: HTD45H, sid: int):
    temp, raw, deg = dev.read_status(sid)

    temp_s = "讀取失敗" if temp is None else f"{temp} °C"
    pos_s = "讀取失敗" if raw is None else f"{raw} raw / {deg:.1f}°"

    print(f"📟 ID={sid} 狀態：溫度={temp_s}，目前位置={pos_s}")


# ---------- 角度控制模式（連續輸入） ----------
def angle_console(dev: HTD45H, sid: int):
    print("\n🎛️ 角度控制模式")
    print("輸入角度 0~240（例如 120），或輸入：")
    print("  r=讀取目前位置 + 溫度")
    print("  p=只讀取目前位置")
    print("  temp=只讀取溫度")
    print("  t=切換扭力 on/off")
    print("  s=設定時間(ms)")
    print("  q=離開\n")

    torque_on = True
    move_time = 300

    # 先確保扭力開
    dev.torque(sid, True)
    time.sleep(0.03)

    print_status(dev, sid)

    while True:
        cmd = input(f"[ID={sid}] angle/r/p/temp/t/s/q > ").strip().lower()
        if cmd == "q":
            print("離開角度控制模式。\n")
            return

        if cmd == "r":
            print_status(dev, sid)
            continue

        if cmd == "p":
            raw = dev.read_position_raw(sid)
            if raw is None:
                print("❌ 目前位置讀取失敗")
            else:
                deg = raw / 1000.0 * 240.0
                print(f"📍 目前位置：{raw} raw / {deg:.1f}°")
            continue

        if cmd == "temp":
            temp = dev.read_temperature(sid)
            if temp is None:
                print("❌ 溫度讀取失敗")
            else:
                print(f"🌡️ 溫度：{temp} °C")
            continue

        if cmd == "t":
            torque_on = not torque_on
            dev.torque(sid, torque_on)
            print(f"扭力：{'ON' if torque_on else 'OFF'}")
            time.sleep(0.03)
            print_status(dev, sid)
            continue

        if cmd == "s":
            move_time = ask_int("移動時間 ms（0~30000）：", 0, 30000)
            print(f"移動時間：{move_time} ms")
            continue

        # 當作角度
        try:
            deg = float(cmd)
        except:
            print("無效指令/角度，請再輸入。")
            continue

        dev.torque(sid, True)
        time.sleep(0.01)
        dev.move(sid, deg, move_time)
        print(f"已送出：角度 {deg}°，時間 {move_time} ms")

        # 等舵機移動一下，再讀取狀態
        time.sleep(min(max(move_time / 1000.0, 0.08), 0.8))
        print_status(dev, sid)


# ---------- 功能 1：自動掃描 ID ----------
def scan_id(dev: HTD45H) -> int | None:
    print("🔍 開始掃描 ID (1~253)...")
    for sid in range(1, 254):
        try:
            # 先嘗試用讀取位置確認 ID
            raw = dev.read_position_raw(sid)
            if raw is not None:
                print(f"✅ 找到可回覆的 ID：{sid}，目前位置 raw={raw}")
                return sid

            # 若讀不到，再用小動作測試
            dev.torque(sid, True)
            time.sleep(0.02)
            dev.move(sid, 120, 200)
            time.sleep(0.12)
            dev.move(sid, 60, 200)
            print(f"👉 可能有反應的 ID：{sid}")
            return sid
        except:
            pass

    print("❌ 掃描不到舵機（請檢查：外部供電 / 共地 / 單線訊號 / 只接一顆）")
    return None


# ---------- 功能 2：改 ID 精靈 ----------
def id_wizard(dev: HTD45H):
    old_id = scan_id(dev)
    if old_id is None:
        return

    print_status(dev, old_id)

    new_id = ask_int("✏️ 輸入你要的新 ID（1~253）：", 1, 253)

    print(f"\n📤 正在送出改 ID 指令：{old_id} → {new_id}")
    dev.set_id(old_id, new_id)

    print("""
⚠️ 重要（HTD-45H 必做）：
1️⃣ 立刻【斷掉舵機電源】（不是拔 USB，是拔舵機電源）
2️⃣ 等 2～3 秒
3️⃣ 再上電
完成後按 Enter 繼續
""")
    input("👉 我已經重新上電，按 Enter 繼續")

    print("🔎 驗證新 ID 是否成功（會讀取位置/溫度 + 送扭力+移動）...")
    try:
        print_status(dev, new_id)

        dev.torque(new_id, True)
        time.sleep(0.05)
        dev.move(new_id, 120, 300)
        time.sleep(0.35)
        print_status(dev, new_id)

        dev.move(new_id, 60, 300)
        time.sleep(0.35)
        print_status(dev, new_id)

        print(f"✅ 成功！新 ID = {new_id}")

        # 進入角度控制
        go = input("要進入角度控制模式嗎？(y/n)：").strip().lower()
        if go == "y":
            angle_console(dev, new_id)

    except Exception as e:
        print(f"❌ 驗證失敗：{e}")
        print("請確認你真的有『斷電再上電』，且總線上只有這一顆舵機。")


# ---------- 功能 3：指定 ID 直接角度控制 ----------
def angle_mode(dev: HTD45H):
    sid = ask_int("輸入要控制的舵機 ID（1~253）：", 1, 253)
    angle_console(dev, sid)


# ---------- 功能 4：指定 ID 讀取狀態 ----------
def status_mode(dev: HTD45H):
    sid = ask_int("輸入要讀取的舵機 ID（1~253）：", 1, 253)
    print_status(dev, sid)


def serial_candidates() -> list[str]:
    ports = [DEFAULT_PORT]
    ports.extend(glob.glob("/dev/ttyUSB*"))
    ports.extend(glob.glob("/dev/ttyACM*"))
    return sorted(set(ports))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HTD45H servo utility")
    parser.add_argument(
        "-p",
        "--port",
        default=os.environ.get("HTD45H_PORT", DEFAULT_PORT),
        help=f"serial device path (default: %(default)s; env: HTD45H_PORT)",
    )
    return parser.parse_args()


def open_device(port: str) -> HTD45H:
    try:
        return HTD45H(port)
    except serial.SerialException as exc:
        print(f"❌ 無法開啟序列埠 {port}: {exc}")
        candidates = [p for p in serial_candidates() if os.path.exists(p)]
        if candidates:
            print("目前看得到的候選序列埠：")
            for candidate in candidates:
                print(f"  - {candidate}")
            print(f"可改用：python3 {os.path.basename(__file__)} --port <上面的路徑>")
        else:
            print("目前 container 內看不到 /dev/usb_robot_arm、/dev/ttyUSB* 或 /dev/ttyACM*。")
            print("請確認啟動 Docker 時有掛載硬體，例如使用 ./launch_shell.sh，或 docker run 加上 --privileged -v /dev:/dev。")
        sys.exit(1)


# ---------- 主程式 ----------
if __name__ == "__main__":
    # args = parse_args()
    # print(f"使用序列埠：{args.port}")
    # dev = open_device(args.port)
    dev = open_device('/dev/ttyUSB0')

    try:
        while True:
            print("""
選擇功能：
1 = 自動掃描 ID（找到後可直接角度控制）
2 = 改 ID 精靈（含驗證，成功後可角度控制）
3 = 角度控制模式（指定 ID）
4 = 讀取溫度 + 目前位置（指定 ID）
q = 離開
""")
            c = input("輸入 1/2/3/4/q：").strip().lower()

            if c == "1":
                sid = scan_id(dev)
                if sid is not None:
                    print_status(dev, sid)
                    go = input("要進入角度控制模式嗎？(y/n)：").strip().lower()
                    if go == "y":
                        angle_console(dev, sid)

            elif c == "2":
                id_wizard(dev)

            elif c == "3":
                angle_mode(dev)

            elif c == "4":
                status_mode(dev)

            elif c == "q":
                break

            else:
                print("無效選擇。\n")

    finally:
        dev.close()
        print("已關閉序列埠。")
