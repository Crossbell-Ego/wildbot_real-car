#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarNoiseCheck(Node):
    def __init__(self):
        super().__init__('lidar_noise_check')
        self.sub_raw = self.create_subscription(LaserScan, '/scan_tmp', self.raw_callback, 10)
        self.sub_filtered = self.create_subscription(LaserScan, '/scan', self.filtered_callback, 10)
        self.get_logger().info("LidarNoiseCheck node started. Listening to /scan_tmp and /scan...")

    def print_close_points(self, msg, label):
        close_points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if 0.05 < r < 0.65:
                deg = math.degrees(angle)
                close_points.append((deg, r))
            angle += msg.angle_increment
        
        if close_points:
            self.get_logger().info(f"[{label}] Found {len(close_points)} close-range points:")
            # Print a summary of the angles and ranges
            for deg, r in close_points[:15]:
                self.get_logger().info(f"  Angle: {deg:6.1f} deg, Range: {r:.3f} m")
            if len(close_points) > 15:
                self.get_logger().info(f"  ... and {len(close_points) - 15} more points")
        else:
            self.get_logger().info(f"[{label}] No close-range points (<0.65m) detected.")

    def raw_callback(self, msg):
        self.print_close_points(msg, "RAW (/scan_tmp)")

    def filtered_callback(self, msg):
        self.print_close_points(msg, "FILTERED (/scan)")

def main():
    rclpy.init()
    node = LidarNoiseCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
