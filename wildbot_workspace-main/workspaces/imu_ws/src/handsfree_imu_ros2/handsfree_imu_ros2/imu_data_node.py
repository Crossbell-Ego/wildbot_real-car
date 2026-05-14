#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import serial
import math
from handsfree_imu_ros2 import python_driver_a9 as a9

def euler_to_quaternion(roll, pitch, yaw):
    """將尤拉角 (deg) 轉換為四元數"""
    roll, pitch, yaw = map(math.radians, [roll, pitch, yaw])
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class ImuA9DataNode(Node):
    def __init__(self):
        super().__init__('imu_a9_data_node')
        self.declare_parameter('port', '/dev/imu_a9')
        self.declare_parameter('baud', 921600)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.01, self.timer_callback) # 100Hz
        self.get_logger().info(f'IMU Data Node (Standard Imu Msg) started on {port}')

    def timer_callback(self):
        try:
            if self.ser.in_waiting <= 0: return
            buff_data = self.ser.read(self.ser.in_waiting)
            for b in buff_data:
                a9.handleSerialData(b)

            # 封裝標準 Imu 訊息
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # 1. 姿態 (Orientation)
            msg.orientation = euler_to_quaternion(a9.angle_degree[0], a9.angle_degree[1], a9.angle_degree[2])
            
            # 2. 角速度 (Angular Velocity) - Rad/s
            msg.angular_velocity.x = float(a9.angularVelocity[0])
            msg.angular_velocity.y = float(a9.angularVelocity[1])
            msg.angular_velocity.z = float(a9.angularVelocity[2])

            # 3. 線加速度 (Linear Acceleration) - m/s^2
            acc_k = math.sqrt(sum(x**2 for x in a9.acceleration))
            if acc_k > 0:
                msg.linear_acceleration.x = a9.acceleration[0] * 9.8 / acc_k
                msg.linear_acceleration.y = a9.acceleration[1] * 9.8 / acc_k
                msg.linear_acceleration.z = a9.acceleration[2] * 9.8 / acc_k

            self.imu_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"IMU reading error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuA9DataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
