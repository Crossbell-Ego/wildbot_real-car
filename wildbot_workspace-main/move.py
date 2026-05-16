import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
from rclpy.qos import QoSProfile, ReliabilityPolicy

class MinimalMove(Node):
    def __init__(self, x, z):
        super().__init__('minimal_move')
        # 使用與手把腳本一致的設定，確保底盤能正確接收
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher_ = self.create_publisher(TwistStamped, '/base_controller/cmd_vel', qos)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        self.x = x
        self.z = z
        self.get_logger().info(f'🚀 正在執行指令移動: x={x}, z={z} (按下 Ctrl+C 停止)')

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.x
        msg.twist.angular.z = self.z
        self.publisher_.publish(msg)

def main():
    # 預設前進 0.2
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2
    z = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    
    rclpy.init()
    node = MinimalMove(x, z)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
