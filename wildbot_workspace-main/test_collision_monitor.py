#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TestCollisionMonitor(Node):
    def __init__(self):
        super().__init__('test_collision_monitor')
        self.sub_smoothed = self.create_subscription(
            TwistStamped,
            '/cmd_vel_smoothed',
            self.smoothed_callback,
            10
        )
        self.sub_nav = self.create_subscription(
            TwistStamped,
            '/nav2/cmd_vel',
            self.nav_callback,
            10
        )
        self.sub_base = self.create_subscription(
            TwistStamped,
            '/base_controller/cmd_vel',
            self.base_callback,
            10
        )
        self.get_logger().info("TestCollisionMonitor node started. Monitoring topics...")

    def smoothed_callback(self, msg):
        self.get_logger().info(f"[/cmd_vel_smoothed] x={msg.twist.linear.x:.4f}, z={msg.twist.angular.z:.4f}")

    def nav_callback(self, msg):
        self.get_logger().info(f"[/nav2/cmd_vel] x={msg.twist.linear.x:.4f}, z={msg.twist.angular.z:.4f}")

    def base_callback(self, msg):
        self.get_logger().info(f"[/base_controller/cmd_vel] x={msg.twist.linear.x:.4f}, z={msg.twist.angular.z:.4f}")

def main():
    rclpy.init()
    node = TestCollisionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
