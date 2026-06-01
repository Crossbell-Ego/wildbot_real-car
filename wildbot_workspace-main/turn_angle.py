#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import sys
import math
import time

def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class TurnAngle(Node):
    def __init__(self, target_angle_deg, speed=0.5):
        super().__init__('turn_angle_node')
        
        self.target_angle = math.radians(abs(target_angle_deg))
        # 速度方向與目標角度正負一致 (正為左轉，負為右轉)
        self.speed = speed if target_angle_deg >= 0 else -speed
        
        self.start_yaw = None
        self.current_yaw = None
        self.last_yaw = None
        self.angle_traveled = 0.0
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
        
        # 定時控制迴圈
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f"🎯 目標旋轉角度: {target_angle_deg} 度 (速度: {self.speed:.3f} rad/s)")
        self.get_logger().info("⏳ 等待里程計 (odom) 數據初始化...")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = get_yaw_from_quaternion(q)
        
        if self.start_yaw is None:
            self.start_yaw = yaw
            self.last_yaw = yaw
            self.get_logger().info(f"✅ 里程計已成功初始化！起始 Yaw: {math.degrees(self.start_yaw):.1f} 度")

        self.current_yaw = yaw
        
        if self.start_yaw is not None:
            delta_yaw = self.current_yaw - self.last_yaw
            # 處理角度包角 (wrap around) 的問題 (-pi 到 pi)
            while delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            while delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi
                
            self.angle_traveled += abs(delta_yaw)
            self.last_yaw = self.current_yaw

    def control_loop(self):
        if self.start_yaw is None:
            return  # 等待 odom 數據載入
            
        if self.is_done:
            return

        self.get_logger().info(
            f"🚗 已旋轉: {math.degrees(self.angle_traveled):.1f} 度 / {math.degrees(self.target_angle):.1f} 度",
            throttle_duration_sec=0.2
        )

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if self.angle_traveled >= self.target_angle:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"🎉 抵達目標角度！實際旋轉: {math.degrees(self.angle_traveled):.1f} 度")
            self.is_done = True
            self.create_timer(0.2, lambda: rclpy.shutdown())
        else:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = self.speed
            self.cmd_pub.publish(msg)

def main():
    if len(sys.argv) < 2:
        print("❌ 缺少參數！使用方式: python3 turn_angle.py <角度(度)> [速度(rad/s)]")
        return
        
    try:
        target_angle_deg = float(sys.argv[1])
    except ValueError:
        print("❌ 角度參數必須為數字！")
        return

    speed = 0.5
    if len(sys.argv) > 2:
        try:
            speed = abs(float(sys.argv[2]))
        except ValueError:
            print("❌ 速度參數必須為數字！")
            return

    rclpy.init()
    node = TurnAngle(target_angle_deg, speed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("🛑 偵測到 Ctrl+C 中斷，立即停下小車！")
        stop_node = Node('stop_node')
        pub = stop_node.create_publisher(TwistStamped, '/base_controller/cmd_vel', 10)
        msg = TwistStamped()
        msg.header.stamp = stop_node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
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
