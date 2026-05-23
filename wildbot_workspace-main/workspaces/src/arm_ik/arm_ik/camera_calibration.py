import os
import sys
import rclpy
from rclpy.node import Node
import json
import time
import threading
import cv2
import numpy as np
import subprocess
from sensor_msgs.msg import Image, CompressedImage
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

# ==============================================================================
# 🔍 沙盒測試 X11 顯示器是否支援 (防止 Qt 致命錯誤 Abort 導致主程序崩潰)
# ==============================================================================
def check_x11_support():
    """在獨立的子程序中測試 OpenCV GUI 是否可用，防止 xcb 連接失敗直接結束主程序。"""
    if 'DISPLAY' not in os.environ:
        return False
    
    test_code = """
import cv2
try:
    cv2.namedWindow('X11 Test', cv2.WINDOW_NORMAL)
    cv2.destroyAllWindows()
    print('OK')
except Exception:
    import sys
    sys.exit(1)
"""
    try:
        res = subprocess.run(
            [sys.executable, "-c", test_code],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            env=os.environ, timeout=2.0
        )
        return res.returncode == 0 and b"OK" in res.stdout
    except Exception:
        return False

# ==============================================================================
# 📷 ROS 2 相機軟體即時調光與校正節點 (自適應 GUI & CLI)
# ==============================================================================
class CameraCalibrationTool(Node):
    def __init__(self):
        super().__init__('camera_calibration_tool')
        
        # 訂閱相機的原生未調光影像話題
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw_raw',
            self.image_callback,
            10
        )
        
        # 發布已調光校正 raw 影像話題給 Foxglove
        self.image_pub = self.create_publisher(
            Image,
            '/camera/color/image_raw',
            10
        )
        
        # 發布已調光校正 compressed 影像話題給 YOLO 及 Foxglove
        self.compressed_pub = self.create_publisher(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            10
        )
        
        # 建立與原生相機通訊的 Client，用於動態禁用相機原生壓縮話題發布
        self.param_client = self.create_client(SetParameters, '/camera/camera/set_parameters')
        
        self.current_frame = None       # 原始影格
        self.calibrated_frame = None    # 調光後影格
        self.avg_gray_val = 0.0         # 原生畫面平均亮度
        self.calibrated_avg_val = 0.0   # 調光後畫面平均亮度
        
        # 校正參數 (預設亮度偏移為 0，預設對比度增益係數為 50，即 alpha=1.0)
        self.brightness = 0
        self.contrast = 50
        self.gui_initialized = False
        
        # 載入現有設定檔 (改為固定在 /workspaces 下以在不同容器間持久共享)
        self.config_file = "/workspaces/camera_config.json"
        self.load_saved_config()

        self.get_logger().info("🚀 相機軟體即時雙通道 (Raw & Compressed) 調光轉發節點已啟動！")
        self.get_logger().info("📡 已接管影像串流 -> 訂閱: /camera/color/image_raw_raw")
        self.get_logger().info("📡 已接管影像發布 -> /camera/color/image_raw & /camera/color/image_raw/compressed")
        
        # 異步執行禁用原生壓縮功能
        threading.Thread(target=self.disable_camera_compressed_plugins, daemon=True).start()

    def disable_camera_compressed_plugins(self):
        """呼叫大白相機的參數設定服務，動態關閉其自動發布的壓縮影像話題。"""
        self.get_logger().info("⏳ 正在等待大白相機的參數設定服務 /camera/camera/set_parameters ...")
        # 最多等待 5 秒
        if not self.param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("⚠️ 無法連線至 /camera/camera 的參數設定服務，原生相機可能仍會發布原生壓縮話題。")
            return
            
        req = SetParameters.Request()
        
        # 建立空字串陣列參數值
        val = ParameterValue(
            type=ParameterType.PARAMETER_STRING_ARRAY,
            string_array_value=[]
        )
        
        # 將原生相機對應的彩色影像壓縮外掛關閉
        p1 = Parameter(name="color.image_raw.enable_pub_plugins", value=val)
        
        req.parameters = [p1]
        
        # 發送異步調用
        future = self.param_client.call_async(req)
        future.add_done_callback(self.param_set_callback)

    def param_set_callback(self, future):
        try:
            res = future.result()
            success = True
            for r in res.results:
                if not r.successful:
                    self.get_logger().warn(f"⚠️ 動態禁用原生相機壓縮外掛部分失敗: {r.reason}")
                    success = False
            if success:
                self.get_logger().info("✅ 原生相機壓縮影像外掛已成功動態禁用！獨佔壓縮發布通道打通。")
        except Exception as e:
            self.get_logger().error(f"⚠️ 調用參數設定服務異常: {e}")

    def load_saved_config(self):
        """載入儲存的設定檔。"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    self.brightness = config.get("brightness", 0)
                    self.contrast = config.get("contrast", 50)
                self.get_logger().info(f"📂 載入歷史設定：亮度偏移={self.brightness:+d}, 對比度參數={self.contrast}")
            except Exception as e:
                self.get_logger().error(f"📂 載入設定檔失敗: {e}，使用預設值。")
        else:
            self.get_logger().info("📂 未找到歷史設定，使用預設設定（亮度=0, 對比=50）。")

    def save_config(self, brightness, contrast):
        """將設定值存檔。"""
        self.brightness = brightness
        self.contrast = contrast
        try:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump({"brightness": brightness, "contrast": contrast}, f, indent=4)
            self.get_logger().info(f"💾 設定已存檔: {self.config_file}")
        except Exception as e:
            self.get_logger().error(f"💾 儲存設定檔失敗: {e}")

    def image_callback(self, msg):
        """影像接收回呼，進行即時軟體調光運算並重新發布至真話題 (Raw & Compressed)。"""
        try:
            if msg.encoding in ['bgr8', 'rgb8']:
                h, w = msg.height, msg.width
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                if msg.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                self.current_frame = img
                
                # 計算原生影像平均灰階亮度
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                self.avg_gray_val = float(np.mean(gray))
                
                # 軟體調光公式： g(x,y) = alpha * f(x,y) + beta
                # 1. 亮度偏移 beta (brightness): -100 ~ 100
                beta = float(self.brightness)
                
                # 2. 對比度增益 alpha (contrast): alpha = 1.0 + (contrast_param - 50) * 0.02
                alpha = 1.0 + (self.contrast - 50) * 0.02
                if alpha < 0.0:
                    alpha = 0.0
                
                # 即時影像校正運算
                calibrated = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
                self.calibrated_frame = calibrated
                
                # 計算調光後影像平均亮度
                calibrated_gray = cv2.cvtColor(calibrated, cv2.COLOR_BGR2GRAY)
                self.calibrated_avg_val = float(np.mean(calibrated_gray))
                
                # 1. 發布 Raw 話題
                out_msg = Image()
                out_msg.header = msg.header
                out_msg.height = h
                out_msg.width = w
                out_msg.encoding = 'bgr8'
                out_msg.is_bigendian = msg.is_bigendian
                out_msg.step = w * 3
                out_msg.data = calibrated.tobytes()
                self.image_pub.publish(out_msg)
                
                # 2. 進行 JPEG 壓縮並發布 Compressed 話題
                ret, encoded_img = cv2.imencode('.jpg', calibrated, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ret:
                    comp_msg = CompressedImage()
                    comp_msg.header = msg.header
                    comp_msg.format = 'bgr8; jpeg'
                    comp_msg.data = encoded_img.tobytes()
                    self.compressed_pub.publish(comp_msg)
                
        except Exception as e:
            self.get_logger().error(f"影像即時調光處理失敗: {e}")

    def on_brightness_change(self, val):
        """亮度 Trackbar 回呼 (GUI)。
        拉吧範圍 0~200 對應 -100~100 (預設在 100 代表 0)
        """
        self.brightness = val - 100

    def on_contrast_change(self, val):
        """對比度 Trackbar 回呼 (GUI)。
        拉吧範圍 0~100 對應對比度參數 (預設在 50)
        """
        self.contrast = val

    def reset_to_auto(self):
        """重設亮度和對比度為預設值。"""
        self.brightness = 0
        self.contrast = 50
        if self.gui_initialized:
            cv2.setTrackbarPos('Brightness', 'Camera Calibration', 100)
            cv2.setTrackbarPos('Contrast', 'Camera Calibration', 50)
        self.get_logger().info("🔄 已重設為預設模式 (亮度=0, 對比=50)")

    def init_gui(self):
        """初始化 OpenCV 本地 GUI 視窗與拉吧。"""
        try:
            cv2.namedWindow('Camera Calibration', cv2.WINDOW_NORMAL)
            
            # Brightness 拉吧 (0~200 -> -100~100，預設為 100)
            init_b = self.brightness + 100
            cv2.createTrackbar('Brightness', 'Camera Calibration', init_b, 200, self.on_brightness_change)
            
            # Contrast 拉吧 (0~100，預設為 50)
            init_c = self.contrast
            cv2.createTrackbar('Contrast', 'Camera Calibration', init_c, 100, self.on_contrast_change)
            
            self.gui_initialized = True
            return True
        except Exception as e:
            self.get_logger().error(f"❌ 無法建立本地視窗: {e}")
            return False

    def run_gui(self):
        """GUI 輪詢迴圈。"""
        self.get_logger().info("⏳ 等待原生相機影像流...")
        while rclpy.ok() and self.current_frame is None:
            time.sleep(0.1)
            
        if not rclpy.ok():
            return

        if not self.init_gui():
            self.run_cli()
            return
            
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎮 本地 GUI 視窗操作說明：")
        self.get_logger().info("  - 拖動 Brightness 拉吧：微調亮度偏移 (預設 100 代表不偏移)")
        self.get_logger().info("  - 拖動 Contrast 拉吧：調整對比度增益 (預設 50 代表原始對比)")
        self.get_logger().info("  - 在視窗焦點下按 [S] 鍵：儲存當前校正設定")
        self.get_logger().info("  - 在視窗焦點下按 [R] 鍵：重設為預設模式")
        self.get_logger().info("  - 在視窗焦點下按 [Q] 或 [ESC]：安全退出工具")
        self.get_logger().info("=" * 60)

        try:
            while rclpy.ok():
                # 這裡不需要 spin_once，背景 spin 執行緒會處理影像更新
                # 這裡 GUI 畫面顯示「調光後影像」
                if self.calibrated_frame is not None:
                    display_img = self.calibrated_frame.copy()
                    h, w, _ = display_img.shape
                    
                    cv2.putText(display_img, f"Brightness Offset: {self.brightness:+d}", (20, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(display_img, f"Contrast Scale: {self.contrast} (x{1.0 + (self.contrast - 50) * 0.02:.2f})", (20, 75), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(display_img, f"Avg Brightness: {self.calibrated_avg_val:.1f} (Raw: {self.avg_gray_val:.1f})", (20, 110), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1, cv2.LINE_AA)
                    
                    cv2.putText(display_img, "[S] Save | [R] Reset | [Q] Quit", (20, h - 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
                    
                    cv2.imshow('Camera Calibration', display_img)
                
                key = cv2.waitKey(30) & 0xFF
                if key == ord('q') or key == 27:
                    self.get_logger().info("👋 收到退出訊號，正在關閉視窗...")
                    break
                elif key == ord('s') or key == ord('S'):
                    self.save_config(self.brightness, self.contrast)
                elif key == ord('r') or key == ord('R'):
                    self.reset_to_auto()
        finally:
            cv2.destroyAllWindows()

    def run_cli(self):
        """當無顯示器支援時，自動降級啟動終端機控制台互動模式。"""
        self.get_logger().warn("⚠️ 未偵測到可用的 X11 顯示器，已自動啟用 [終端機互動校正模式]")
        
        # 啟動背景狀態印出線程，提供實時影像平均亮度反饋
        stop_event = threading.Event()
        def print_status_loop():
            while not stop_event.is_set() and rclpy.ok():
                # \r 用於在同一行重複刷新顯示
                sys.stdout.write(
                    f"\r📊 [實時狀態] 原生亮度: {self.avg_gray_val:.1f} -> 調光後亮度: {self.calibrated_avg_val:.1f} | 亮度偏移: {self.brightness:+d}, 對比增益: {self.contrast} "
                )
                sys.stdout.flush()
                time.sleep(0.4)
                
        threading.Thread(target=print_status_loop, daemon=True).start()

        # 等待第一幀影像
        while rclpy.ok() and self.current_frame is None:
            time.sleep(0.1)

        time.sleep(0.5)  # 讓日誌順序好看
        print("\n" + "=" * 60)
        print("🎮 終端機控制指令說明：")
        print("  - 輸入 整數 (-100 到 100)：設定亮度偏移值 (0 為不變)")
        print("  - 輸入 c後接整數 (如 c50)：設定對比度增益 (0 到 100，50 為原始)")
        print("  - 輸入 s ：儲存當前設定到 JSON")
        print("  - 輸入 r ：重設亮度和對比度為預設值 (0, 50)")
        print("  - 輸入 q ：安全退出工具")
        print("=" * 60)

        try:
            while rclpy.ok():
                try:
                    # 為了能讓背景 thread 流暢印出，我們在這裡做非阻塞 input
                    user_input = input("\n📝 請輸入指令: ").strip().lower()
                except EOFError:
                    # 當在 Docker compose 背景運行無終端輸入時，捕獲 EOF 並進入純轉發掛起狀態
                    self.get_logger().info("📡 偵測到無終端機標準輸入 (EOF)，已自動切換為純背景影像調光轉發模式。")
                    while rclpy.ok():
                        time.sleep(1.0)
                    break
                
                if user_input == 'q':
                    self.get_logger().info("👋 正在關閉相機校正工具...")
                    break
                elif user_input == 's':
                    self.save_config(self.brightness, self.contrast)
                    print("✅ 設定已成功存檔！")
                elif user_input == 'r':
                    self.reset_to_auto()
                    print("🔄 已重設為預設模式！")
                elif user_input.startswith('c'):
                    try:
                        val = int(user_input[1:])
                        if 0 <= val <= 100:
                            self.contrast = val
                            print(f"👉 對比度已設為: {val}")
                        else:
                            print("❌ 對比度範圍應為 0 ~ 100")
                    except ValueError:
                        print("❌ 輸入無效，例如: c50")
                else:
                    try:
                        val = int(user_input)
                        if -100 <= val <= 100:
                            self.brightness = val
                            print(f"👉 亮度偏移已設為: {val:+d}")
                        else:
                            print("❌ 亮度偏移範圍應為 -100 ~ 100")
                    except ValueError:
                        if user_input != "":
                            print("❌ 未知指令。請輸入數字調整亮度，或輸入 c數字 調整對比度。")
        finally:
            stop_event.set()

# ==============================================================================
# 🚀 進入點
# ==============================================================================
def main():
    rclpy.init()
    node = CameraCalibrationTool()
    
    # 進行 X11 圖形沙盒測試
    x11_ok = check_x11_support()
    
    # 用獨立的 ROS 背景 spin 執行緒，確保影像處理與發布不會受到 CLI input 阻塞
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()
    
    try:
        if x11_ok:
            node.run_gui()
        else:
            node.run_cli()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
