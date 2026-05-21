#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from yolo_msgs.msg import DetectionArray
from arm_interface import ArmInterface
import sys
import math
import time

class DoorOpener(Node):
    def __init__(self, target_class="knob", speed=0.08):
        super().__init__('door_opener_node')
        
        self.target_class = target_class
        self.linear_speed = speed
        
        # 對齊參數
        self.target_y_threshold = 15.0   # 左右對齊像素閾值
        self.rotation_speed = 0.10        # 旋轉速度 (rad/s)
        self.kp_yaw = 3.5
        
        # 狀態機
        # WAIT_DETECTION -> ALIGNING -> APPROACHING -> REACHED_GATE -> ACTION_PREPARE -> ACTION_PRESS -> ACTION_PUSH -> ACTION_RETRACT -> DONE
        self.state = 'WAIT_DETECTION'
        self.last_print_time = 0.0
        
        # 傳感器數據
        self.current_yaw = None
        self.current_x = None
        self.current_y = None
        self.start_x = None
        self.start_y = None
        self.distance_traveled = 0.0
        
        self.latest_target_y = 0.0
        self.last_detection_time = 0.0
        self.approach_start_time = None
        self.action_start_time = None
        self.prep_q2 = 0.78
        
        # 初始化手臂介面
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
        self.get_logger().info("🤖 啟動【YOLO 門把偵測與自動下壓開門系統】V1.0")
        self.get_logger().info(f"🎯 目標物件: '{self.target_class}' | 靠近速度: {self.linear_speed:.2f} m/s")
        self.get_logger().info("⏳ 等待 YOLO / IMU 數據初始化...")
        self.get_logger().info("=" * 60)

    def solve_ik(self, x, z):
        """
        手臂逆向運動學 (IK) 求解器。
        輸入：
          x: 夾爪相對於小車前擋板的水平距離 (m)
          z: 夾爪相對於地面的垂直高度 (m)
        輸出：
          (q1, q2): 大臂與小臂的關節弧度值 (若無解則回傳 None)
        """
        L1 = 0.080      # 第一臂長
        L2 = 0.110      # 第二臂長
        BASE_X = 0.165  # 手臂底座 X 軸偏移
        BUMPER_X = 0.13063  # 前擋板 X 軸偏移
        base_z_ground = 0.1266  # 第一軸旋轉中心離地高度

        # 轉換成相對於第一軸旋轉中心的座標 (Xc, Zc)
        Xc = x + BUMPER_X - BASE_X
        Zc = z - base_z_ground

        # 檢查是否在可達工作範圍內
        D2 = Xc**2 + Zc**2
        cos_dtheta = (D2 - L1**2 - L2**2) / (2.0 * L1 * L2)
        if abs(cos_dtheta) > 1.0:
            return None

        # 取得肘下折解 (Elbow-down)
        dtheta = math.acos(cos_dtheta)
        
        # 方程求解 A cos(theta1) - B sin(theta1) = Xc
        A = L1 + L2 * math.cos(dtheta)
        B = L2 * math.sin(dtheta)
        
        theta1 = math.atan2(Zc * A - Xc * B, Xc * A + Zc * B)
        theta2 = theta1 + dtheta

        # 轉回實體馬達的關節角度 q1, q2 (弧度)
        q1 = 2.0 - theta1
        q2 = 4.12177 - (q1 + theta2)
        
        return q1, q2

    def quaternion_to_euler(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def imu_callback(self, msg):
        self.current_yaw = self.quaternion_to_euler(msg.orientation)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def yolo_callback(self, msg):
        # 💡 除錯輸出：每 2 秒列印一次接收到的所有 2D 偵測物體，方便排查是否未發布或類別名稱有差
        classes_in_msg = [f"{d.class_name}({d.score:.2f})" for d in msg.detections]
        self.get_logger().info(f"🔍 接收到 2D 偵測資料: {classes_in_msg}", throttle_duration_sec=2.0)

        if not msg.detections:
            return
            
        target_det = None
        for det in msg.detections:
            if det.class_name.lower() == self.target_class.lower() and det.score >= 0.4:
                target_det = det
                break
                
        if target_det is not None:
            # 640 為影像水平中心
            self.latest_target_y = 640.0 - target_det.bbox.center.position.x
            self.last_detection_time = time.time()

    def control_loop(self):
        now = time.time()
        print_status = (now - self.last_print_time >= 0.5)
        if print_status:
            self.last_print_time = now

        if self.current_yaw is None or self.current_x is None:
            if print_status:
                self.get_logger().info("⏳ 等待 IMU / Odom 數據中...")
            return

        # 狀態機邏輯
        if self.state == 'WAIT_DETECTION':
            if print_status:
                self.get_logger().info(f"📡 正在搜尋門把... (尚未偵測到物件: '{self.target_class}')")
            if now - self.last_detection_time < 0.5 and self.last_detection_time > 0:
                self.get_logger().info(f"🎯 已偵測到門把 '{self.target_class}'！開始對齊中心...")
                self.state = 'ALIGNING'

        elif self.state == 'ALIGNING':
            if now - self.last_detection_time > 1.5:
                self.get_logger().warn("⚠️ 門把遺失，退回搜尋狀態。")
                self.stop_chassis()
                self.state = 'WAIT_DETECTION'
                return

            if abs(self.latest_target_y) <= self.target_y_threshold:
                self.stop_chassis()
                self.get_logger().info("✅ 門把對齊完成！開始緩慢前進逼近門把...")
                self.approach_start_time = time.time()
                self.state = 'APPROACHING'
            else:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = self.rotation_speed if self.latest_target_y > 0 else -self.rotation_speed
                self.cmd_pub.publish(msg)
                if print_status:
                    self.get_logger().info(f"🔄 對齊旋轉中... 偏差 Y: {self.latest_target_y:.1f} 像素")

        elif self.state == 'APPROACHING':
            # 💡 實機安全策略：對齊門把後，直接向前行駛 1.8 秒以靠近門把 (不依賴易受噪訊影響的遠近距離)
            elapsed = time.time() - self.approach_start_time
            if print_status:
                self.get_logger().info(f"🚗 前進逼近中... 已行駛 {elapsed:.1f} 秒 / 1.8 秒")
                
            if elapsed >= 1.8:
                self.stop_chassis()
                self.get_logger().info("✅ 已到達門把下壓就位距離！開始執行開門程序...")
                self.state = 'ACTION_PREPARE'
            else:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.twist.linear.x = self.linear_speed
                msg.twist.angular.z = 0.0
                self.cmd_pub.publish(msg)

        elif self.state == 'ACTION_PREPARE':
            self.stop_chassis()
            # 計算門把上方 3 公分位置的關節角度 (X=12cm, Z=25cm)
            angles = self.solve_ik(0.12, 0.25)
            if angles is None:
                self.get_logger().error("❌ 預備位置 IK 求解失敗，無法執行動作！")
                self.state = 'DONE'
                return
                
            q1, q2 = angles
            self.prep_q2 = q2  # 💡 記錄準備時 the 第二軸角度
            # 夾爪維持閉合狀態 3.0 rad
            self.get_logger().info(f"🦾 手臂伸出至門把上方 (夾爪閉合)... (q1: {q1:.3f}, q2: {q2:.3f})")
            self.arm.send_goal([q1, q2, 3.0], duration=1.5, teleop_mode=False)
            self.action_start_time = time.time()
            self.state = 'ACTION_PRESS'

        elif self.state == 'ACTION_PRESS':
            self.stop_chassis()
            elapsed = time.time() - self.action_start_time
            if elapsed >= 1.8:
                # 計算壓下門把位置的關節角度 (X=12cm, Z=16cm)
                angles = self.solve_ik(0.12, 0.16)
                if angles is None:
                    self.get_logger().error("❌ 下壓位置 IK 求解失敗！")
                    self.state = 'DONE'
                    return
                    
                q1, _ = angles
                # 💡 第二軸保持不變，僅以第一軸 (q1) 下壓，夾爪維持閉合 3.0 rad
                self.get_logger().info(f"🦾 僅旋轉第一軸往下壓動門把 (夾爪閉合)！... (q1: {q1:.3f}, q2 (固定): {self.prep_q2:.3f})")
                self.arm.send_goal([q1, self.prep_q2, 3.0], duration=1.0, teleop_mode=False)
                self.action_start_time = time.time()
                self.state = 'ACTION_PUSH'

        elif self.state == 'ACTION_PUSH':
            # 保持手臂下壓狀態，同時小車緩慢前進將門推開
            elapsed = time.time() - self.action_start_time
            if elapsed >= 1.2:
                push_elapsed = elapsed - 1.2
                if print_status:
                    self.get_logger().info(f"🚪 正在推門中... 已前進 {push_elapsed:.1f} 秒 / 1.5 秒")
                    
                if push_elapsed >= 1.5:
                    self.stop_chassis()
                    self.get_logger().info("✅ 門已推開！開始收回手臂...")
                    self.state = 'ACTION_RETRACT'
                else:
                    msg = TwistStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'
                    msg.twist.linear.x = 0.12  # 推門速度
                    msg.twist.angular.z = 0.0
                    self.cmd_pub.publish(msg)
            else:
                self.stop_chassis()

        elif self.state == 'ACTION_RETRACT':
            self.stop_chassis()
            # 收回安全姿勢 (大臂與小臂安全折回)
            safe_pose = [1.90, 1.35, 3.0]
            self.get_logger().info("🦾 機械手臂收回至安全抬升位置...")
            self.arm.send_goal(safe_pose, duration=1.5, teleop_mode=False)
            self.action_start_time = time.time()
            self.state = 'DONE'

        elif self.state == 'DONE':
            self.stop_chassis()
            elapsed = time.time() - self.action_start_time
            if elapsed >= 1.5:
                self.get_logger().info("🏁 開門任務圓滿完成！")
                rclpy.shutdown()

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
    target_class = "knob"
    speed = 0.08
    
    if len(sys.argv) > 1:
        target_class = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            speed = float(sys.argv[2])
        except ValueError:
            print("⚠️ 速度參數錯誤，使用預設值 0.08 m/s")

    rclpy.init()
    node = DoorOpener(target_class, speed)
    
    # 等待手臂介面就緒 (防止排程搶佔)
    node.get_logger().info("⏳ 正在初始化手臂接口 (最多等待 25 秒)...")
    start_time = time.time()
    while time.time() - start_time < 25.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.arm.initialized:
            node.get_logger().info("✅ 手臂接口初始化成功！")
            node.timer.reset()  # 啟動控制定時器
            break
            
    if not node.arm.initialized:
        node.get_logger().error("❌ 無法獲取手臂 JointState 狀態，請確認硬體與 ros2_control 已正確啟動！")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("🛑 偵測到使用者中斷，緊急停車並收回手臂！")
        node.stop_chassis()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
