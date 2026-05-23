#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self, x, y, qz, qw):
        super().__init__('set_initial_pose_node')
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        self.x = x
        self.y = y
        self.qz = qz
        self.qw = qw
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.publish_count = 0

    def timer_callback(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        
        # Normalize quaternion to prevent "malformed message" errors from AMCL validation
        qz = self.qz
        qw = self.qw
        norm = (qz**2 + qw**2)**0.5
        if norm > 0.0:
            qz /= norm
            qw /= norm
            
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        
        # Populate 36 covariance values
        cov = [0.0] * 36
        cov[0] = 0.25   # x covariance
        cov[7] = 0.25   # y covariance
        cov[35] = 0.0685 # yaw covariance
        msg.pose.covariance = cov
        
        self.publisher_.publish(msg)
        self.get_logger().info(f"📤 Published initial pose to /initialpose: X={self.x}, Y={self.y}")
        self.publish_count += 1
        
        if self.publish_count >= 50:
            self.get_logger().info("✅ Completed publishing initial pose 50 times. Exiting.")
            sys.exit(0)

def main():
    rclpy.init()
    # Default to the home position coordinates
    x = 4.66
    y = -3.135
    qz = 0.958
    qw = 0.287
    
    if len(sys.argv) >= 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    if len(sys.argv) >= 5:
        qz = float(sys.argv[3])
        qw = float(sys.argv[4])
        
    node = InitialPosePublisher(x, y, qz, qw)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
