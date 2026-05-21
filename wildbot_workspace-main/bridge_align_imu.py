#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from yolo_msgs.msg import DetectionArray
from arm_interface import ArmInterface  # 引入手臂控制介面
import sys
import math
import time

class BridgeAlignIMU(Node):
    def __init__(self, target_class="bridge", mode="cross", speed=0.15, factor=1.0):
        super().__init__('bridge_align_imu_node')
        
        # 橋樑幾何參數
        self.slope_len = 0.66    # 斜坡長度 66 cm
        self.flat_len = 0.70     # 橋頂平地 70 cm
        
        # 模式與設定
        self.target_class = target_class
        self.mode = mode.lower()  # 'cross', 'midpoint', 'grab_return'
        self.linear_speed = speed
        self.factor = factor      # 打滑補償係數
        
        # 計算各模式目標距離
        self.dist_cross = (self.slope_len + self.flat_len + self.slope_len + 0.20) * self.factor
        self.dist_midpoint = (self.slope_len + (self.flat_len / 2.0)) * self.factor
        
        # 控制閾值與參數
        self.target_y_threshold = 15.0   # 左右對齊閾值 (像素 px)
        self.rotation_speed = 0.12        # 原地旋轉對齊速度 (rad/s)
        self.kp_yaw = 3.5                 # 💡 直行控制比例係數 (從 2.0 提高到 3.5 以更積極修正)
        self.max_correct_w = 0.40         # 💡 直線修正最大角速度上限 (從 0.25 提高到 0.40 rad/s)
        
        # 手臂安全抬升位置 (夾爪半開，且第二軸抬高)
        self.safe_lift_pose = [1.90, 1.35, 3.0]
        
        # 狀態機變數
        # 狀態包含: WAIT_DETECTION, ROTATING, LOCK_IMU, RAISING_ARM, CROSSING_FORWARD, GRABBING, REVERSING, DONE
        self.state = 'WAIT_DETECTION'
        self.last_print_time = 0.0
        
        # 傳感器數據
        self.current_x = None
        self.current_y = None
        self.start_x = None
        self.start_y = None
        self.distance_traveled = 0.0
        
        self.current_yaw = None
        self.current_pitch = None
        self.target_yaw = None
        
        self.latest_target_y = 0.0
        self.latest_target_x = 0.0
        self.last_detection_time = 0.0
        
        # 計時器變數
        self.raise_start_time = None
        self.grabbing_start_time = None
        
        # 初始化通用手臂接口
        self.arm = ArmInterface(self)
        
        # ROS 通訊
        self.cmd_pub = self.create_publisher(TwistStamped, '/base_controller/cmd_vel', 10)
        
        self.yolo_sub = self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.yolo_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/base_controller/odom',
            self.odom_callback,
            10
        )
        
        # 定時控制迴圈 (20Hz) - 先暫停，待手臂初始化成功後再啟用
        self.timer = self.create_timer(0.05, self.control_loop)
        self.timer.cancel()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"🤖 啟動【YOLO 中心對齊 + IMU 鎖定直行】過橋輔助系統 V3 (含手臂安全抬升)")
        self.get_logger().info(f"🎯 目標物件: '{self.target_class}' | 運行模式: '{self.mode}'")
        if self.mode == 'cross':
            self.get_logger().info(f"📏 目標：完整過橋 (目標距離: {self.dist_cross:.2f} m，打滑補償: {self.factor})")
        elif self.mode == 'midpoint':
            self.get_logger().info(f"📏 目標：停於橋頂中點 (目標距離: {self.dist_midpoint:.2f} m，打滑補償: {self.factor})")
        elif self.mode == 'grab_return':
            self.get_logger().info(f"📏 目標：爬至中點 ({self.dist_midpoint:.2f} m) -> 停留夾取 -> 倒車退回起點")
        self.get_logger().info(f"⚡ 設定速度: 前進 {self.linear_speed:.2f} m/s | 修正係數: Kp={self.kp_yaw}")
        self.get_logger().info("⏳ 等待 YOLO / IMU / Odom 數據初始化...")
        self.get_logger().info("=" * 60)

    def quaternion_to_euler(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        return yaw, pitch

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def imu_callback(self, msg):
        yaw, pitch = self.quaternion_to_euler(msg.orientation)
        self.current_yaw = yaw
        self.current_pitch = pitch

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        if self.state in ['CROSSING_FORWARD', 'REVERSING']:
            if self.start_x is None:
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.get_logger().info(f"📍 里程計起點已鎖定: ({self.start_x:.3f}, {self.start_y:.3f})")
            
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            self.distance_traveled = math.hypot(dx, dy)

    def yolo_callback(self, msg):
        # 💡 除錯輸出：每 2 秒列印一次接收到的所有 2D 偵測物體，方便排查是否未發布或類別名稱有差
        classes_in_msg = [f"{d.class_name}({d.score:.2f})" for d in msg.detections]
        self.get_logger().info(f"🔍 接收到 2D 偵測資料: {classes_in_msg}", throttle_duration_sec=2.0)

        if not msg.detections:
            return
            
        target_det = None
        for det in msg.detections:
            # 💡 大小寫不敏感匹配，提高相容性
            if det.class_name.lower() == self.target_class.lower() and det.score >= 0.4:
                target_det = det
                break
                
        if target_det is not None:
            # 影像解析度為 1280x720，中心為 640
            # 偏差計算：左偏為正，右偏為負 (與 3D 的 base_link 坐標系 Y 軸方向一致)
            self.latest_target_y = 640.0 - target_det.bbox.center.position.x
            self.latest_target_x = target_det.bbox.center.position.y
            self.last_detection_time = time.time()

    def control_loop(self):
        now = time.time()
        print_status = False
        if now - self.last_print_time >= 0.5:
            print_status = True
            self.last_print_time = now

        # 基礎通訊檢查
        if self.current_yaw is None or self.current_x is None:
            if print_status:
                self.get_logger().info("⏳ 等待 IMU / Odom 數據中...")
            return

        # 狀態機邏輯
        if self.state == 'WAIT_DETECTION':
            if print_status:
                self.get_logger().info(f"📡 搜尋目標中... (尚未偵測到物件: '{self.target_class}')")
                
            if now - self.last_detection_time < 0.5 and self.last_detection_time > 0:
                self.get_logger().info(f"🎯 已偵測到目標 '{self.target_class}'！偏差 Y: {self.latest_target_y:.1f} 像素 (px)")
                self.state = 'ROTATING'

        elif self.state == 'ROTATING':
            if now - self.last_detection_time > 1.5:
                self.get_logger().warn("⚠️ YOLO 目標遺失，退回搜尋狀態。")
                self.stop_chassis()
                self.state = 'WAIT_DETECTION'
                return

            if abs(self.latest_target_y) <= self.target_y_threshold:
                self.stop_chassis()
                self.get_logger().info(f"✅ 中心對齊完成！目前偏差 Y: {self.latest_target_y:.1f} 像素 (px). 進入鎖定朝向階段...")
                self.state = 'LOCK_IMU'
            else:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = self.rotation_speed if self.latest_target_y > 0 else -self.rotation_speed
                self.cmd_pub.publish(msg)
                if print_status:
                    self.get_logger().info(f"🔄 對齊旋轉中... 偏差 Y: {self.latest_target_y:.1f} 像素 (px)")

        elif self.state == 'LOCK_IMU':
            self.target_yaw = self.current_yaw
            self.get_logger().info(f"🔒 鎖定 IMU 朝向目標角 (Yaw): {self.target_yaw:.3f} 弧度 ({math.degrees(self.target_yaw):.1f}°)")
            
            # 💡 開始安全抬升手臂
            self.get_logger().info("🦾 開始安全抬升機械手臂至運送高度...")
            self.arm.target_positions = list(self.safe_lift_pose)
            # 使用 action 模式 (teleop_mode=False) 以精確控制移動時間
            self.arm.send_goal(self.safe_lift_pose, duration=2.0, teleop_mode=False)
            
            self.raise_start_time = time.time()
            self.state = 'RAISING_ARM'

        elif self.state == 'RAISING_ARM':
            self.stop_chassis()  # 手臂抬升時小車保持原地不動
            elapsed = time.time() - self.raise_start_time
            if print_status:
                self.get_logger().info(f"⏳ 正在抬升機械手臂中... 已等待 {elapsed:.1f} 秒 / 2.5 秒")
                
            if elapsed >= 2.5:
                self.get_logger().info("✅ 機械手臂安全抬升完成！小車開始前進上橋...")
                self.start_x = None
                self.start_y = None
                self.distance_traveled = 0.0
                self.state = 'CROSSING_FORWARD'

        elif self.state == 'CROSSING_FORWARD':
            if self.start_x is None:
                return

            target_dist = self.dist_cross if self.mode == 'cross' else self.dist_midpoint
            pitch_deg = math.degrees(self.current_pitch) if self.current_pitch is not None else 0.0

            if print_status:
                self.get_logger().info(f"🚗 前進中: {self.distance_traveled*100:.2f} cm / {target_dist*100:.2f} cm | 偏差: {math.degrees(self.normalize_angle(self.target_yaw - self.current_yaw)):.2f}° | 坡度 (Pitch): {pitch_deg:.1f}°")

            if self.distance_traveled >= target_dist:
                self.stop_chassis()
                self.get_logger().info(f"🎉 已抵達前進目標位置！實際前進: {self.distance_traveled*100:.2f} cm")
                
                if self.mode == 'grab_return':
                    self.get_logger().info("🦾 切換至【夾取階段】，停止並等待中...")
                    self.grabbing_start_time = time.time()
                    self.state = 'GRABBING'
                else:
                    self.get_logger().info("🏁 任務結束！")
                    self.state = 'DONE'
                    self.create_timer(0.2, lambda: rclpy.shutdown())
            else:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.twist.linear.x = self.linear_speed
                
                # IMU 直線航向修正
                yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
                yaw_cmd = self.kp_yaw * yaw_error
                msg.twist.angular.z = max(-self.max_correct_w, min(self.max_correct_w, yaw_cmd))
                self.cmd_pub.publish(msg)

        elif self.state == 'GRABBING':
            elapsed = time.time() - self.grabbing_start_time
            if print_status:
                self.get_logger().info(f"⏳ 正在進行夾取/停留動作中... 已等待 {elapsed:.1f} 秒 / 3.0 秒")
                
            if elapsed >= 3.0:
                self.get_logger().info("🔒 夾取完成！開始執行【倒車退回起點階段】...")
                self.start_x = None
                self.start_y = None
                self.distance_traveled = 0.0
                self.state = 'REVERSING'
            else:
                self.stop_chassis()

        elif self.state == 'REVERSING':
            if self.start_x is None:
                return

            pitch_deg = math.degrees(self.current_pitch) if self.current_pitch is not None else 0.0
            if print_status:
                self.get_logger().info(f"⏪ 倒車中: {self.distance_traveled*100:.2f} cm / {self.dist_midpoint*100:.2f} cm | 偏差: {math.degrees(self.normalize_angle(self.target_yaw - self.current_yaw)):.2f}° | 坡度 (Pitch): {pitch_deg:.1f}°")

            if self.distance_traveled >= self.dist_midpoint:
                self.stop_chassis()
                self.get_logger().info(f"🎉 倒車完成！小車已安全退回起點。實際倒退: {self.distance_traveled*100:.2f} cm")
                self.state = 'DONE'
                self.create_timer(0.2, lambda: rclpy.shutdown())
            else:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.twist.linear.x = -self.linear_speed
                
                # 💡 修正後的方向修正：倒車與前進在朝向校正上方向相同，以確保 Yaw 回歸 target_yaw
                yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
                yaw_cmd = self.kp_yaw * yaw_error
                msg.twist.angular.z = max(-self.max_correct_w, min(self.max_correct_w, yaw_cmd))
                self.cmd_pub.publish(msg)

        elif self.state == 'DONE':
            pass

    def stop_chassis(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        for _ in range(5):
            self.cmd_pub.publish(msg)
            time.sleep(0.02)

def main():
    target_class = "bridge"
    mode = "cross"
    speed = 0.15
    factor = 1.0

    if len(sys.argv) > 1:
        target_class = sys.argv[1]
    if len(sys.argv) > 2:
        mode = sys.argv[2]
    if len(sys.argv) > 3:
        try:
            speed = float(sys.argv[3])
        except ValueError:
            print("⚠️ 速度參數錯誤，使用預設值 0.15m/s")
    if len(sys.argv) > 4:
        try:
            factor = float(sys.argv[4])
        except ValueError:
            print("⚠️ 補償係數錯誤，使用預設值 1.0")

    rclpy.init()
    node = BridgeAlignIMU(target_class, mode, speed, factor)
    
    # 等待手臂介面初始化
    node.get_logger().info("⏳ 正在初始化手臂接口 (最多等待 25 秒)...")
    start_time = time.time()
    while time.time() - start_time < 25.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.arm.initialized:
            node.get_logger().info("✅ 手臂接口初始化成功！")
            node.timer.reset()  # 💡 啟用定時控制迴圈
            break
            
    if not node.arm.initialized:
        node.get_logger().error("❌ 無法獲取手臂 JointState 狀態，請確認硬體與 ros2_control 已正確啟動！")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("🛑 偵測到使用者中斷，緊急停車！")
        node.stop_chassis()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
