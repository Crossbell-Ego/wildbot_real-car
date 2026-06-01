import rclpy
from rclpy.node import Node
import json
import os
import sys
import time
import math
import threading
from arm_interface import ArmInterface
from yolo_msgs.msg import DetectionArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class GrabExecutor(Node):
    def __init__(self):
        super().__init__('grab_executor')
        self.arm = ArmInterface(self)
        
        # 取得腳本所在目錄的絕對路徑，確保能正確讀取同目錄下的 arm_poses.json
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.pose_file = os.path.join(script_dir, "arm_poses.json")
        self.saved_poses = self.load_poses()
        
        self.last_print_time = 0.0  # 用於限制終端機列印頻率
        self.state = 'IDLE'  # 可為: IDLE, WAIT_YOLO, ROTATING, ALIGNING, DONE
        self.factor = 0.9615    # 打滑補償係數 (例如在草地上設為 1.2)
        self.distance_offset = 0.0      # 距離補償值 (公尺)，負值代表少走 (例如 -0.02 代表少走 2 公分)，可用於修正移動過頭的問題
        
        # YOLO 目標物對齊與旋轉相關變數
        self.target_class_name = None
        self.target_visible = True       # 追蹤目標物是否仍在相機視野中
        self.latest_target_y = 0.0
        self.latest_target_x = 0.0
        self.target_y_threshold = 0.02  # 左右對齊閾值 (公尺)，當 |y| 小於此值時視為對齊中心
        self.rotation_speed = 0.15       # 原地旋轉對齊的速度 (rad/s)
        self.align_kp = 2.5              # 走直線時的左右校正比例係數 (P controller)
        
        # 底盤控制狀態變數
        self.target_chassis_dist = 0.0
        self.chassis_speed = 0.12
        self.chassis_start_x = None
        self.chassis_start_y = None
        self.current_x = None
        self.current_y = None
        self.distance_traveled = 0.0

        # 訂閱 YOLO 3D 的 topic '/yolo/detections_3d'
        self.detections_sub = self.create_subscription(
            DetectionArray,
            '/yolo/detections_3d',
            self.detections_callback,
            10
        )

        # 訂閱里程計 odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/base_controller/odom',
            self.odom_callback,
            10
        )

        # 發布底盤速度 cmd_vel
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/base_controller/cmd_vel',
            10
        )

        # 建立定時底盤控制迴圈 (20Hz)
        self.chassis_timer = self.create_timer(0.05, self.chassis_control_loop)
        
        self.get_logger().info("✅ 已訂閱 '/yolo/detections_3d' 與 '/base_controller/odom'，已建立 cmd_vel 發布器。")

    def load_poses(self):
        """載入 JSON 點位檔案。"""
        if os.path.exists(self.pose_file):
            try:
                with open(self.pose_file, 'r') as f:
                    return json.load(f)
            except Exception as e:
                self.get_logger().error(f"❌ 讀取 {self.pose_file} 失敗: {e}")
        return {}

    def start_alignment_mode(self, factor):
        """啟動對齊模式，設定補償係數"""
        self.factor = factor
        self.state = 'WAIT_YOLO'
        self.get_logger().info(f"📡 已啟動自動對齊與補償模式 (補償係數: {self.factor})，等待偵測目標...")

    def detections_callback(self, msg):
        """接收 YOLO 3D 的偵測結果並判斷是否執行對齊。"""
        # 若移至第二點位或已完成，直接返回
        if self.state in ['MOVING_TO_SLOT_2', 'DONE']:
            return

        # 1. 旋轉對齊模式 (ROTATING) 與前後移動對齊模式 (ALIGNING)：持續更新鎖定目標物的實時座標
        if self.state in ['ROTATING', 'ALIGNING']:
            target_det = None
            min_dist = float('inf')
            if msg.detections:
                for det in msg.detections:
                    if det.class_name == self.target_class_name and det.score >= 0.5:
                        # 如果有兩隻相同的熊，追蹤距離最近的那隻
                        if det.bbox3d.center.position.x < min_dist:
                            min_dist = det.bbox3d.center.position.x
                            target_det = det
            if target_det is not None:
                self.latest_target_y = target_det.bbox3d.center.position.y
                self.latest_target_x = target_det.bbox3d.center.position.x
                if not self.target_visible:
                    self.target_visible = True
                    self.get_logger().info("👁️ 目標物重新出現在視野中，恢復實時更新座標")
            else:
                # 目標消失（進入盲區），保留記憶中的最後座標，不清零
                if self.target_visible:
                    self.target_visible = False
                    self.get_logger().info(
                        f"🔇 目標物離開視野（盲區），使用記憶位置繼續：X={self.latest_target_x*100:.1f} cm, Y={self.latest_target_y*100:.1f} cm"
                    )
            return

        # 2. 其餘狀態若偵測為空直接返回
        if not msg.detections:
            return

        # 2. 若非處於等待偵測狀態，僅維持實時顯示資訊
        if self.state != 'WAIT_YOLO':
            self.print_detection_status(msg)
            return

        # 3. 等待目標物模式 (WAIT_YOLO)：尋找信心度 >= 0.7 且距離最近的目標物
        target_det = None
        min_dist = float('inf')
        for det in msg.detections:
            if det.score >= 0.7:
                if det.bbox3d.center.position.x < min_dist:
                    min_dist = det.bbox3d.center.position.x
                    target_det = det

        if target_det is None:
            return

        # 鎖定目標物資訊
        self.target_class_name = target_det.class_name
        self.latest_target_y = target_det.bbox3d.center.position.y
        self.latest_target_x = target_det.bbox3d.center.position.x

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"🎯 鎖定 YOLO 目標物: {self.target_class_name} (信心度: {target_det.score:.2f})")
        
        # 判斷是否偏離相機中心 (Y 軸偏移大於閾值)
        if abs(self.latest_target_y) > self.target_y_threshold:
            self.state = 'ROTATING'
            self.get_logger().info(f"🔄 目標偏離中心 Y: {self.latest_target_y*100:.2f} cm，啟動原地旋轉對齊模式...")
            self.get_logger().info("=" * 60)
        else:
            # 偏差在閾值內，直接進入前後移動對齊狀態
            self.state = 'ALIGNING'
            bumper_x = self.arm.BUMPER_X if hasattr(self.arm, 'BUMPER_X') else 0.13063
            x_to_bumper = self.latest_target_x - bumper_x + 0.07
            q = self.arm.current_positions
            x_g, z_g = self.arm.get_coordinates(q[0], q[1])
            diff = x_to_bumper - x_g

            self.get_logger().info(f"📊 物體距前擋板: {x_to_bumper*100:.2f} cm | 夾爪距前擋板: {x_g*100:.2f} cm")
            self.get_logger().info(f"↔️ 計算 X 軸差距 (前進距離): {diff*100:.2f} cm")
            self.get_logger().info("=" * 60)

            # 初始化底盤移動設定
            self.start_chassis_move(diff)

    def start_chassis_move(self, distance):
        """根據計算出的差距，啟動底盤移動"""
        # 套用打滑補償係數與距離補償值
        self.target_chassis_dist = abs(distance) * self.factor + self.distance_offset
        if self.target_chassis_dist < 0.0:
            self.target_chassis_dist = 0.0
            
        # 根據正負判斷前進與後退，使用 self.chassis_speed 設定的值
        speed_val = abs(self.chassis_speed)
        self.chassis_speed = speed_val if distance >= 0 else -speed_val
        
        self.chassis_start_x = None
        self.chassis_start_y = None
        self.distance_traveled = 0.0
        self.get_logger().info(f"🚗 啟動移動：目標差距 {distance*100:.2f} cm (打滑與補償後目標: {self.target_chassis_dist*100:.2f} cm，補償值: {self.distance_offset*100:.2f} cm)")

    def odom_callback(self, msg):
        """記錄里程計實時座標。"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        if self.state == 'ALIGNING':
            if self.chassis_start_x is None:
                self.chassis_start_x = self.current_x
                self.chassis_start_y = self.current_y
                self.get_logger().info(f"📍 里程計起始位點已載入: ({self.chassis_start_x:.3f}, {self.chassis_start_y:.3f})")
            
            dx = self.current_x - self.chassis_start_x
            dy = self.current_y - self.chassis_start_y
            self.distance_traveled = math.hypot(dx, dy)

    def chassis_control_loop(self):
        """底盤運動控制迴圈 (20Hz)"""
        # 1. 旋轉對齊邏輯
        if self.state == 'ROTATING':
            if abs(self.latest_target_y) <= self.target_y_threshold:
                # 旋轉對齊完成，停下小車並進入前後對齊階段
                self.stop_chassis()
                self.get_logger().info(f"🔄 左右旋轉對齊完成！當前偏差 Y: {self.latest_target_y*100:.2f} cm，切換為前後移動對齊...")
                
                # 計算前後對齊差距
                self.state = 'ALIGNING'
                bumper_x = self.arm.BUMPER_X if hasattr(self.arm, 'BUMPER_X') else 0.13063
                x_to_bumper = self.latest_target_x - bumper_x + 0.07
                q = self.arm.current_positions
                x_g, z_g = self.arm.get_coordinates(q[0], q[1])
                diff = x_to_bumper - x_g
                
                self.start_chassis_move(diff)
            else:
                # 繼續原地旋轉
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.twist.linear.x = 0.0
                # 修正轉向：目標在左邊 (Y > 0) 時，angular.z 應為正值 (向左轉)
                msg.twist.angular.z = self.rotation_speed if self.latest_target_y > 0 else -self.rotation_speed
                self.cmd_pub.publish(msg)
                self.get_logger().info(
                    f"🔄 旋轉對齊中... 目前偏差 Y: {self.latest_target_y*100:.2f} cm",
                    throttle_duration_sec=0.5
                )
            return

        if self.state != 'ALIGNING' or self.chassis_start_x is None:
            return

        self.get_logger().info(
            f"🚗 移動進度: {self.distance_traveled*100:.2f} cm / {self.target_chassis_dist*100:.2f} cm",
            throttle_duration_sec=0.2
        )

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if self.distance_traveled >= self.target_chassis_dist:
            # 抵達目標，停下小車並結束底盤對齊
            self.stop_chassis()
            self.get_logger().info(f"🎉 自動對齊完成！實際物理移動約: {self.distance_traveled / self.factor * 100:.2f} cm")
            self.state = 'MOVING_TO_SLOT_2'
            # 啟動背景執行序列移至第二點位與夾取動作
            self.execute_slot_2_and_grab()
        else:
            # 繼續移動
            msg.twist.linear.x = self.chassis_speed
            
            # 邊走邊校正功能已取消，保持直線前進
            msg.twist.angular.z = 0.0
                
            self.cmd_pub.publish(msg)
            self.get_logger().info(
                f"📡 前進對齊中: vx={self.chassis_speed:.3f} m/s, wz={msg.twist.angular.z:.3f} rad/s | 目前 Y 軸偏差: {self.latest_target_y*100:.2f} cm",
                throttle_duration_sec=0.5
            )

    def stop_chassis(self):
        """停止底盤移動"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        for _ in range(5):
            self.cmd_pub.publish(msg)
            time.sleep(0.02)

    def execute_slot_2_and_grab(self):
        """在背景執行移至第二點位與夾取動作，避免阻塞 ROS 2 主執行緒"""
        threading.Thread(target=self._sequence_thread, daemon=True).start()

    def _sequence_thread(self):
        try:
            self.get_logger().info("🦾 正在移動手臂至第二點位 (slot '2' 夾取位置)...")
            target_pose = self.saved_poses["2"]
            
            # 使用 teleop_mode=False 發送，因為這是精確控制
            self.arm.target_positions = list(target_pose)
            self.arm.send_goal(target_pose, duration=1.5, teleop_mode=False)
            
            # 等待手臂與夾爪抵達 (1.5秒移動時間 + 0.5秒緩衝)
            time.sleep(2.0)
            self.get_logger().info("✅ 手臂已抵達第二點位 (夾爪已閉合)。")
            
            # 💡 依據安全防護規範：夾取動作執行 0.5 秒後，自動退回 2度 (0.035 rad) 釋放堵轉壓力，防止馬達燒毀
            time.sleep(0.5)
            self.get_logger().info("🔒 啟動防燒毀安全釋放機制：夾爪自動退回 2° (0.035 rad)...")
            
            release_pose = list(self.arm.current_positions)
            # 確保角度不會低於安全值 (2.93 rad)，這裡加上 0.035 rad 稍微張開
            release_pose[2] = min(4.19, release_pose[2] + 0.035)
            
            self.arm.target_positions = release_pose
            self.arm.send_goal(release_pose, duration=0.5, teleop_mode=False)
            
            time.sleep(1.0)
            self.get_logger().info("🎉 夾取與防燒毀動作執行完畢！")
            
            # 3. 移動至第三點位 (slot '3'，例如抬起或運送位置)
            if "3" in self.saved_poses:
                self.get_logger().info("🦾 正在移動手臂至第三點位 (slot '3' 抬起/運送位置)...")
                target_pose_3 = self.saved_poses["3"]
                self.arm.target_positions = list(target_pose_3)
                self.arm.send_goal(target_pose_3, duration=2.0, teleop_mode=False)
                # 等待手臂與夾爪抵達 (2.0秒移動時間 + 0.5秒緩衝)
                time.sleep(2.5)
                self.get_logger().info("✅ 手臂已抵達第三點位。")
                
                # 3.5. 夾取與舉升完成後，小車安全後退離區
                self.get_logger().info("🚗 夾取與舉升完成，小車啟動安全後退離區...")
                backup_dist = 0.25  # 後退距離 (25 公分)
                start_x = self.current_x
                start_y = self.current_y
                if start_x is not None and start_y is not None:
                    twist_msg = TwistStamped()
                    twist_msg.header.frame_id = 'base_link'
                    
                    while rclpy.ok():
                        twist_msg.header.stamp = self.get_clock().now().to_msg()
                        twist_msg.twist.linear.x = -0.08  # 後退速度: 8 cm/s
                        twist_msg.twist.angular.z = 0.0
                        
                        dx = self.current_x - start_x
                        dy = self.current_y - start_y
                        dist = math.hypot(dx, dy)
                        
                        if dist >= backup_dist:
                            break
                            
                        self.cmd_pub.publish(twist_msg)
                        time.sleep(0.05)
                        
                    self.stop_chassis()
                    self.get_logger().info("✅ 安全後退完成。")
            else:
                self.get_logger().warning("⚠️ 找不到第三點位 (slot '3')，跳過此步驟。")
                
            self.get_logger().info("🎉 所有動作執行完畢！程式即將結束。")
            self.state = 'DONE'
            
            # 延遲關閉節點，確保所有命令都已完整執行並送出
            time.sleep(0.5)
            rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"❌ 夾取序列執行出錯: {e}")

    def print_detection_status(self, msg):
        """實時在終端機印出 YOLO 3D 的三維座標與手臂當前座標"""
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_print_time < 0.5:
            return
            
        self.last_print_time = now
        
        arm_coords_str = "\033[1;30m[手臂未初始化或無數據]\033[0m"
        if hasattr(self, 'arm') and self.arm.initialized:
            q = self.arm.current_positions
            x_g, z_g = self.arm.get_coordinates(q[0], q[1])
            x_j2, z_j2 = self.arm.get_joint2_coordinates(q[0])
            arm_coords_str = (
                f"\033[1;35m夾爪 X={x_g*100:.1f} cm, Z={z_g*100:.1f} cm\033[0m | "
                f"\033[1;36m第二軸 X={x_j2*100:.1f} cm, Z={z_j2*100:.1f} cm\033[0m"
            )
            
        print("\n" + "="*65)
        print(f"🎯 \033[1;36m[YOLO 3D 實時偵測 - 即時更新中]\033[0m 收到 {len(msg.detections)} 個偵測目標")
        print(f"🦾 \033[1;35m[手臂實時物理座標]\033[0m: {arm_coords_str}")
        print("-"*65)
        for i, det in enumerate(msg.detections):
            x_raw = det.bbox3d.center.position.x
            bumper_x = self.arm.BUMPER_X if hasattr(self.arm, 'BUMPER_X') else 0.13063
            x_to_bumper = x_raw - bumper_x + 0.07
            
            y = det.bbox3d.center.position.y
            z = det.bbox3d.center.position.z
            w = det.bbox3d.size.x
            h = det.bbox3d.size.y
            d = det.bbox3d.size.z
            
            print(f"  \033[1;32m[{i+1}] 物體名稱: {det.class_name}\033[0m (信心度: {det.score:.2f})")
            print(f"      📍 X (距前擋板): \033[1;33m{x_to_bumper*100:.1f} cm\033[0m  (base_link: {x_raw:.3f}m)")
            print(f"      📍 Y (左右偏差): \033[1;33m{y*100:.1f} cm\033[0m  (base_link: {y:.3f}m)")
            print(f"      📍 Z (離地高度): \033[1;33m{z*100:.1f} cm\033[0m  (base_link: {z:.3f}m)")
            print(f"      📏 外觀尺寸 (W, H, D): ({w:.3f}, {h:.3f}, {d:.3f}) 公尺")
            print(f"      🔩 座標參考系 (Frame): {det.bbox3d.frame_id}")
        print("="*65)

    def move_to_slot(self, slot_name, duration=2.0):
        """移動到指定名稱的點位。"""
        if slot_name not in self.saved_poses:
            self.get_logger().error(f"❌ 找不到點位: {slot_name}")
            return False
        
        target = self.saved_poses[slot_name]
        self.get_logger().info(f"🚀 正在移動至點位 [{slot_name}]...")
        
        # 同步內部目標值並發送
        self.arm.target_positions = list(target)
        self.arm.send_goal(target, duration=duration, teleop_mode=False)
        
        # 使用非阻塞的 spin 循環等待移動完成，確保 JointState 能實時更新
        start_wait = time.time()
        while time.time() - start_wait < (duration + 0.5):
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # 計算抵達後的實時座標
        q = self.arm.current_positions
        x_m, z_m = self.arm.get_joint2_coordinates(q[0])
        x_g, z_g = self.arm.get_coordinates(q[0], q[1])
        
        self.get_logger().info(
            f"✅ 抵達點位 [{slot_name}] (第二軸 X(距前擋板): {x_m*100:.1f} cm, Z(離地): {z_m*100:.1f} cm | "
            f"夾爪 X(距前擋板): {x_g*100:.1f} cm, Z(離地): {z_g*100:.1f} cm)"
        )
        return True

def main():
    rclpy.init()
    node = GrabExecutor()
    
    # 稍微等待 JointState 同步 (增加等待時間至 15 秒，避免 Docker 重啟後 DDS 探索延遲導致失敗)
    print("⏳ 正在初始化手臂接口 (最多等待 15 秒)...")
    for _ in range(150):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.arm.initialized:
            break
            
    if not node.arm.initialized:
        print("❌ 無法取得手臂當前狀態，請檢查機器人連線。")
        node.destroy_node()
        rclpy.shutdown()
        return

    # 判斷輸入參數是否皆為 slot 名稱（例如：python3 "camera and grab.py" 1 2 1）
    is_sequence = len(sys.argv) > 1 and all(arg in node.saved_poses for arg in sys.argv[1:])
    
    if is_sequence:
        # 若參數為點位名稱，執行序列移動
        for slot in sys.argv[1:]:
            if not node.move_to_slot(slot):
                break
    else:
        # 預設模式：自動對齊與打滑補償模式
        # 可帶入單一浮點數作為打滑補償係數，例如：python3 "camera and grab.py" 1.2
        # 若在草地執行，預設值為 1.29 ；一般路面請手動傳入 1.0
        factor = 0.9091
        if len(sys.argv) > 1:
            try:
                factor = float(sys.argv[1])
            except ValueError:
                pass
                
        print("\n" + "="*60)
        print(f"🤖 啟動【YOLO 自動對齊與距離補償模式】(補償係數: {factor})")
        print("1. 正在將手臂初始化移動至第一點位 (slot '1')...")
        print("="*60 + "\n")
        
        # 第一步：先移動到點位 '1'
        if not node.move_to_slot("1"):
            print("❌ 移動至點位 '1' 失敗，終止程式。")
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            return
            
        print("\n✅ 手臂已就位。")
        node.start_alignment_mode(factor)
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("\n👋 使用者中斷，停止執行。")
            node.stop_chassis()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
