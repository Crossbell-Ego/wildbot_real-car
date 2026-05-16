#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
import math
from yolo_msgs.msg import DetectionArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def clamp(value, lower, upper):
    return min(upper, max(lower, value))

class ArmGrabTaskNode(Node):
    def __init__(self):
        super().__init__('arm_grab_task_node')
        
        # 1. 參數設定
        self.declare_parameter('target_class', 'bear')
        self.target_class = self.get_parameter('target_class').value
        
        # 手機手臂長度 (單位: 公尺)
        self.l1 = 0.08  # 第一段長度 (大臂)
        self.l2 = 0.12  # 第二段長度 (小臂 + 夾爪)
        
        # 2. 狀態控制
        self.is_busy = False
        self.last_grab_time = 0
        
        # 3. ROS 訂閱與發布
        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/detections_3d',
            self.yolo_callback,
            10)
            
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10)
            
        self.get_logger().info(f"=== 抓取任務節點已啟動 (幾何 IK 模式)，鎖定目標：{self.target_class} ===")
        self.get_logger().info("ℹ️ 提示：程式會持續追蹤目標，請在終端機按下 'g' 鍵開始抓取。")
        
        # 儲存最新的目標位置
        self.latest_target_pos = None
        
        # 啟動鍵盤監聽執行緒
        import threading
        self.kb_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.kb_thread.start()

    def keyboard_listener(self):
        import sys
        import termios
        import tty
        
        def getch():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

        while rclpy.ok():
            char = getch()
            if char == 'g':
                if self.latest_target_pos:
                    self.get_logger().info("🚀 收到指令，開始執行抓取流程！")
                    self.start_grab_process(self.latest_target_pos.x, self.latest_target_pos.y, self.latest_target_pos.z)
                else:
                    self.get_logger().warn("⚠️ 尚未偵測到目標，無法抓取。")
            elif char == '\x03': # Ctrl+C
                break

    def yolo_callback(self, msg):
        if self.is_busy:
            return
            
        for detection in msg.detections:
            if detection.class_name == self.target_class:
                pos = detection.bbox3d.center.position
                score = detection.score
                # 過濾低信心度目標 (門檻值 0.7)
                if score < 0.7:
                    continue
                # 簡單的有效性檢查 (Z 軸需大於 0)
                if pos.z > 0:
                    self.latest_target_pos = pos
                    # 計算距離與提示 (不自動抓取)
                    dx = pos.x - 0.165
                    dz = pos.z - 0.12
                    dist_cm = math.hypot(dx, dz) * 100
                    
                    self.get_logger().info(
                        f"🎯 發現 {self.target_class}！信心度: {score:.2f}, "
                        f"位置: X={pos.x*100:.1f}cm, Z={pos.z*100:.1f}cm, "
                        f"距離手臂: {dist_cm:.1f}cm [按 g 抓取]", 
                        throttle_duration_sec=1.0
                    )
                    break

    def start_grab_process(self, x, y, z):
        # 預先檢查是否可達，避免做白工
        if self.solve_ik(x, z) is None:
            self.get_logger().warn("❌ 目標不可達，取消抓取任務。")
            self.is_busy = False
            return

        self.is_busy = True
        try:
            # 步驟 1: 張開夾爪 (240度)
            self.get_logger().info("1. 張開夾爪...")
            self.move_gripper(240.0)
            time.sleep(1.0)
            
            # 步驟 2: 移動到預備位 (目標上方 5cm)
            self.get_logger().info("2. 移動到預備位...")
            if not self.move_to_xyz(x, y, z + 0.05, duration=2.0):
                raise Exception("無法到達預備位")
            time.sleep(2.5)
            
            # 步驟 3: 下降到抓取位
            self.get_logger().info("3. 下降抓取...")
            if not self.move_to_xyz(x, y, z, duration=1.5):
                raise Exception("無法到達抓取位")
            time.sleep(2.0)
            
            # 步驟 4: 閉合夾爪 (168度)
            self.get_logger().info("4. 閉合夾爪...")
            self.move_gripper(168.0)
            time.sleep(0.5)
            
            # 步驟 5: 執行防燒毀邏輯 (退回 2 度)
            self.get_logger().info("5. 執行防燒毀保護 (退回 2 度)...")
            self.move_gripper(170.0)
            time.sleep(0.2)
            
            # 步驟 6: 抬起物品
            self.get_logger().info("6. 抬起目標...")
            self.move_to_xyz(x, y, z + 0.15, duration=2.0)
            time.sleep(2.5)
            
            self.get_logger().info("✅ 抓取任務完成！")
            
        except Exception as e:
            self.get_logger().error(f"❌ 抓取過程發生錯誤: {e}")
        
        self.last_grab_time = time.time()
        self.is_busy = False

    def solve_ik(self, x, z):
        """幾何逆向運動學解算 (2-Link 平面)"""
        # 手臂物理極限
        max_reach = self.l1 + self.l2
        
        # 設定手臂相對於 base_link 的位置 (手臂現在在車頭方向 X+)
        dx = x - 0.165 # 手臂基座 X 偏移 (相對於 base_link)
        dz = z - 0.12  # 手臂基座 Z 偏移 (高度)
        
        dist = math.hypot(dx, dz)
        
        # 顯示詳細位置資訊
        self.get_logger().info("-" * 40)
        self.get_logger().info(f"📍 目標世界位置 (base_link): X={x:.2f}, Z={z:.2f}")
        self.get_logger().info(f"📏 相對於手臂基座距離: {dist:.3f} m (手臂極限: {max_reach:.3f} m)")
        
        if dist > max_reach:
            need_closer = dist - max_reach
            self.get_logger().warn(f"⚠️ 目標太遠！需要再靠近約 {need_closer*100:.1f} cm 才能抓取")
            self.get_logger().info("-" * 40)
            return None
        elif dist < abs(self.l1 - self.l2):
            self.get_logger().warn(f"⚠️ 目標太近！(低於最小手臂半徑)")
            self.get_logger().info("-" * 40)
            return None
        
        self.get_logger().info("✅ 目標在可抓取範圍內，開始計算角度...")
        self.get_logger().info("-" * 40)
            
        # 計算 q2 (Elbow)
        cos_q2 = (dx**2 + dz**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_q2 = clamp(cos_q2, -1.0, 1.0)
        q2 = math.acos(cos_q2) # Elbow down
        
        # 計算 q1 (Shoulder)
        k1 = self.l1 + self.l2 * math.cos(q2)
        k2 = self.l2 * math.sin(q2)
        q1 = math.atan2(dz, dx) - math.atan2(k2, k1)
        
        return [q1, q2]

    def move_to_xyz(self, x, y, z, duration=2.0):
        # 由於手臂在 X-Z 平面運動，我們主要處理 x 和 z
        ik_results = self.solve_ik(x, z)
        
        if ik_results is None:
            return False
            
        # 轉換為角度進行安全檢查 (radians to degrees)
        arm_1_deg = math.degrees(ik_results[0])
        arm_2_deg = math.degrees(ik_results[1])
        
        self.get_logger().info(f"計算結果: Arm1={arm_1_deg:.2f}°, Arm2={arm_2_deg:.2f}°")
        
        # 安全檢查 (依照 rule.md)
        if not (30 <= arm_1_deg <= 210):
            self.get_logger().error(f"Arm 1 角度超出安全範圍: {arm_1_deg:.2f}°")
            return False
        if not (0 <= arm_2_deg <= 240):
            self.get_logger().error(f"Arm 2 角度超出安全範圍: {arm_2_deg:.2f}°")
            return False

        # 發布軌跡指令
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg() # 重要：加入時間戳
        msg.header.frame_id = "base_link"
        msg.joint_names = ['arm_1_joint', 'arm_2_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [float(ik_results[0]), float(ik_results[1])]
        point.time_from_start.sec = int(duration)
        msg.points.append(point)
        self.arm_pub.publish(msg)
        return True

    def move_gripper(self, degree):
        # 夾爪安全限制 (168 ~ 240)
        safe_degree = clamp(degree, 168.0, 240.0)
        rad = math.radians(safe_degree)
        
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg() # 重要：加入時間戳
        msg.joint_names = ['gripper_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [float(rad)]
        point.time_from_start.nanosec = 500000000 # 0.5s
        msg.points.append(point)
        self.arm_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmGrabTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
