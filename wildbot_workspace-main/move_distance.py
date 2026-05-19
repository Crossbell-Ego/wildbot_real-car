#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import sys
import math
import time

class MoveDistance(Node):
    def __init__(self, target_distance, speed=0.05, factor=1.0):
        super().__init__('move_distance_node')
        
        self.target_distance = abs(target_distance) * factor
        # 速度方向與目標距離正負一致 (正為前進，負為後退)
        self.speed = speed if target_distance >= 0 else -speed
        
        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None
        self.distance_traveled = 0.0
        self.is_done = False
        
        # 發布 cmd_vel
        self.cmd_pub = self.create_publisher(TwistStamped, '/base_controller/cmd_vel', 10)
        
        # 訂閱 odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/base_controller/odom',
            self.odom_callback,
            10
        )
        
        # 定時控制迴圈 (20Hz / 0.05秒)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f"🎯 目標移動距離: {target_distance * 100:.1f} cm (打滑補償後目標: {self.target_distance * 100:.1f} cm, 速度: {self.speed:.3f} m/s, 補償係數: {factor})")
        self.get_logger().info("⏳ 等待里程計 (odom) 數據初始化...")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(f"✅ 里程計已成功初始化！起始坐標: ({self.start_x:.3f}, {self.start_y:.3f})")

        # 計算目前累積移動距離 (尤拉距離)
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        self.distance_traveled = math.hypot(dx, dy)

    def control_loop(self):
        if self.start_x is None:
            return  # 等待 odom 數據載入
            
        if self.is_done:
            return

        # 限制日誌列印頻率以避免洗板 (每 0.2 秒印一次進度)
        self.get_logger().info(
            f"🚗 已移動: {self.distance_traveled * 100:.2f} cm / {self.target_distance * 100:.2f} cm",
            throttle_duration_sec=0.2
        )

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if self.distance_traveled >= self.target_distance:
            # 抵達目標，發送停止指令
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"🎉 抵達目標！實際行駛: {self.distance_traveled * 100:.2f} cm")
            self.is_done = True
            # 延遲一點點關閉節點，確保停下指令發送完畢
            self.create_timer(0.2, lambda: rclpy.shutdown())
        else:
            # 繼續朝目標速度移動
            msg.twist.linear.x = self.speed
            msg.twist.angular.z = 0.0
            self.cmd_pub.publish(msg)

def main():
    if len(sys.argv) < 2:
        print("❌ 缺少參數！")
        print("使用方式: python3 move_distance.py <距離(公尺)> [速度(m/s)] [打滑補償係數]")
        print("範例（前進 5cm）: python3 move_distance.py 0.05")
        print("範例（後退 10cm）: python3 move_distance.py -0.1")
        print("範例（草地打滑補償 1.2 倍，實際前進 30cm）: python3 move_distance.py 0.3 0.05 1.2")
        return
        
    try:
        target_dist = float(sys.argv[1])
    except ValueError:
        print("❌ 距離參數必須為數字！")
        return

    speed = 0.05  # 預設安全低速 (5 cm/s)，確保能精準停靠並減少慣性偏差
    if len(sys.argv) > 2:
        try:
            speed = abs(float(sys.argv[2]))
        except ValueError:
            print("❌ 速度參數必須為數字！")
            return

    factor = 1.0
    if len(sys.argv) > 3:
        try:
            factor = float(sys.argv[3])
        except ValueError:
            print("❌ 補償係數必須為數字！")
            return

    rclpy.init()
    node = MoveDistance(target_dist, speed, factor)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("🛑 偵測到 Ctrl+C 中斷，立即停下小車！")
        # 建立臨時節點緊急煞車
        stop_node = Node('stop_node')
        pub = stop_node.create_publisher(TwistStamped, '/base_controller/cmd_vel', 10)
        msg = TwistStamped()
        msg.header.stamp = stop_node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        # 連續發送 5 次以確保底盤收到煞車訊號
        for _ in range(5):
            pub.publish(msg)
            time.sleep(0.02)
        stop_node.destroy_node()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
