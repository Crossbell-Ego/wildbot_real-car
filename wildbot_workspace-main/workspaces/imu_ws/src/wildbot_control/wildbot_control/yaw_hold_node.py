#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TwistStamped


def normalize_angle_deg(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


class YawHoldNode(Node):
    def __init__(self):
        super().__init__('yaw_hold_node')

        self.declare_parameter('yaw_topic', '/imu/rpy_deg')
        self.declare_parameter('cmd_vel_topic', '/base_controller/cmd_vel')

        # 前進速度，原本你手動給 x: 0.2
        self.declare_parameter('linear_x', 0.2)

        # Yaw 修正參數
        self.declare_parameter('kp', 0.01)
        self.declare_parameter('deadband_deg', 2.0)
        self.declare_parameter('max_angular_z', 0.2)

        self.yaw_topic = self.get_parameter('yaw_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_x = self.get_parameter('linear_x').value
        self.kp = self.get_parameter('kp').value
        self.deadband_deg = self.get_parameter('deadband_deg').value
        self.max_angular_z = self.get_parameter('max_angular_z').value

        self.target_yaw = None
        self.current_yaw = None

        self.sub = self.create_subscription(
            Vector3,
            self.yaw_topic,
            self.yaw_callback,
            10
        )

        self.pub = self.create_publisher(
            TwistStamped,
            self.cmd_vel_topic,
            10
        )

        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

        self.get_logger().info(f'Subscribe yaw from: {self.yaw_topic}')
        self.get_logger().info(f'Publish cmd_vel to: {self.cmd_vel_topic}')
        self.get_logger().info(f'linear_x = {self.linear_x:.3f} m/s')
        self.get_logger().info('Waiting for yaw...')

    def yaw_callback(self, msg):
        # /imu/rpy_deg: x=roll, y=pitch, z=yaw
        self.current_yaw = msg.z

        if self.target_yaw is None:
            self.target_yaw = self.current_yaw
            self.get_logger().info(f'Target yaw set to {self.target_yaw:.2f} deg')

    def control_loop(self):
        if self.current_yaw is None or self.target_yaw is None:
            return

        error_deg = normalize_angle_deg(self.current_yaw - self.target_yaw)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # 保持往前走
        cmd.twist.linear.x = self.linear_x
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0

        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0

        # Yaw 誤差太小就不修正
        if abs(error_deg) < self.deadband_deg:
            cmd.twist.angular.z = 0.0
        else:
            angular_z = self.kp * error_deg
            angular_z = max(-self.max_angular_z, min(self.max_angular_z, angular_z))
            cmd.twist.angular.z = angular_z

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = YawHoldNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    stop_cmd = TwistStamped()
    stop_cmd.header.stamp = node.get_clock().now().to_msg()
    stop_cmd.header.frame_id = 'base_link'
    node.pub.publish(stop_cmd)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
