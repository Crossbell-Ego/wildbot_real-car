#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import os
import sys
import time
import math
import threading
from arm_interface import ArmInterface
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class PlaceExecutor(Node):
    def __init__(self):
        super().__init__('place_executor')
        self.arm = ArmInterface(self)
        
        # 取得腳本所在目錄的絕對路徑，確保能正確讀取同目錄下的 arm_poses.json
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.pose_file = os.path.join(script_dir, "arm_poses.json")
        self.saved_poses = self.load_poses()
        
        self.current_x = None
        self.current_y = None

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
        
        self.get_logger().info("✅ odom 訂閱器與 cmd_vel 發布器初始化完成。")

    def load_poses(self):
        """載入 JSON 點位檔案。"""
        if os.path.exists(self.pose_file):
            try:
                with open(self.pose_file, 'r') as f:
                    return json.load(f)
            except Exception as e:
                self.get_logger().error(f"❌ 讀取 {self.pose_file} 失敗: {e}")
        return {}

    def odom_callback(self, msg):
        """記錄里程計實時座標。"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

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

    def execute_place_sequence(self):
        """執行放物與後退收尾動作"""
        try:
            # 1. 將手臂降低至放物點位 (slot '2' 放置位置)
            self.get_logger().info("🦾 正在降低手臂至放置點位 (slot '2')...")
            if not self.move_to_slot("2", duration=2.5):
                self.get_logger().error("❌ 移動至點位 '2' 失敗")
                return False
                
            time.sleep(0.5)
            
            # 2. 打開夾爪釋放物件 (slot '1' 開爪位置)
            self.get_logger().info("🔓 正在打開夾爪釋放物件 (slot '1')...")
            if not self.move_to_slot("1", duration=1.5):
                self.get_logger().error("❌ 移動至點位 '1' 失敗")
                return False
                
            time.sleep(1.0)
            
            # 3. 小車安全後退離區，避免碰撞或壓到物件
            self.get_logger().info("🚗 啟動安全後退離區...")
            backup_dist = 0.20  # 後退 20 公分
            start_x = self.current_x
            start_y = self.current_y
            if start_x is not None and start_y is not None:
                twist_msg = TwistStamped()
                twist_msg.header.frame_id = 'base_link'
                
                while rclpy.ok():
                    twist_msg.header.stamp = self.get_clock().now().to_msg()
                    twist_msg.twist.linear.x = -0.08  # 後退速度: 8 cm/s
                    twist_msg.twist.angular.z = 0.0
                    
                    # 必須 spin 讓里程計數據更新
                    rclpy.spin_once(self, timeout_sec=0.01)
                    
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
                self.get_logger().warning("⚠️ 無法獲取里程計座標，跳過後退動作。")

            # 4. 手臂收回安全姿勢 (slot '0')
            self.get_logger().info("🦾 正在收回手臂至安全起始點位 (slot '0')...")
            self.move_to_slot("0", duration=2.0)
            
            self.get_logger().info("🎉 放物與收尾動作全部完成！")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ 放物序列執行出錯: {e}")
            return False

def main():
    rclpy.init()
    node = PlaceExecutor()
    
    # 等待 JointState 同步
    print("⏳ 正在初始化手臂接口...")
    for _ in range(150):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.arm.initialized:
            break
            
    if not node.arm.initialized:
        print("❌ 無法取得手臂當前狀態，請檢查機器人連線。")
        node.destroy_node()
        rclpy.shutdown()
        return

    # 執行放置序列
    node.execute_place_sequence()
    
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
