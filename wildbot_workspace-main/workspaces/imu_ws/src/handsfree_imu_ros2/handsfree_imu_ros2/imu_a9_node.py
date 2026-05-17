#!/usr/bin/env python3

import io
import contextlib

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import serial

from handsfree_imu_ros2 import python_driver_a9 as a9


class ImuA9RpyNode(Node):
    def __init__(self):
        super().__init__('imu_a9_rpy_node')

        self.declare_parameter('port', '/dev/imu_a9')
        self.declare_parameter('baud', 921600)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value

        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=0.05
        )

        self.rpy_pub = self.create_publisher(Vector3, '/imu/rpy_deg', 10)

        self.last_rpy = None

        self.timer = self.create_timer(0.002, self.timer_callback)

        self.get_logger().info(f'IMU opened: {self.port}, baud={self.baud}')
        self.get_logger().info('Publishing Roll/Pitch/Yaw degree on /imu/rpy_deg')

    def timer_callback(self):
        try:
            buff_count = self.ser.in_waiting
        except AttributeError:
            buff_count = self.ser.inWaiting()

        if buff_count <= 0:
            return

        buff_data = self.ser.read(buff_count)

        for i in range(buff_count):
            with contextlib.redirect_stdout(io.StringIO()):
                a9.handleSerialData(buff_data[i])

        roll = float(a9.angle_degree[0])
        pitch = float(a9.angle_degree[1]) # 還原標準順序
        yaw = float(a9.angle_degree[2])

        current_rpy = (roll, pitch, yaw)

        if current_rpy == self.last_rpy:
            return

        self.last_rpy = current_rpy

        msg = Vector3()
        msg.x = roll
        msg.y = pitch
        msg.z = yaw

        self.rpy_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuA9RpyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
