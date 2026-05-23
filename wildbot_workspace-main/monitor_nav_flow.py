import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
import time
import threading

class NavFlowMonitor(Node):
    def __init__(self):
        super().__init__('nav_flow_monitor')
        
        self.counts = {
            'cmd_vel_nav': 0,
            'cmd_vel_smoothed': 0,
            'nav2_cmd_vel': 0,
            'base_controller_cmd_vel': 0
        }
        
        self.last_msg = {}
        
        # Subscribe to all stages
        self.sub_nav = self.create_subscription(
            TwistStamped, '/cmd_vel_nav', self.cb_nav, 10)
        self.sub_smoothed = self.create_subscription(
            TwistStamped, '/cmd_vel_smoothed', self.cb_smoothed, 10)
        self.sub_nav2 = self.create_subscription(
            TwistStamped, '/nav2/cmd_vel', self.cb_nav2, 10)
        self.sub_base = self.create_subscription(
            TwistStamped, '/base_controller/cmd_vel', self.cb_base, 10)
            
        # Goal publisher (changed to /goal_pose)
        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10)
            
    def cb_nav(self, msg):
        self.counts['cmd_vel_nav'] += 1
        self.last_msg['cmd_vel_nav'] = msg
        
    def cb_smoothed(self, msg):
        self.counts['cmd_vel_smoothed'] += 1
        self.last_msg['cmd_vel_smoothed'] = msg
        
    def cb_nav2(self, msg):
        self.counts['nav2_cmd_vel'] += 1
        self.last_msg['nav2_cmd_vel'] = msg
        
    def cb_base(self, msg):
        self.counts['base_controller_cmd_vel'] += 1
        self.last_msg['base_controller_cmd_vel'] = msg

def main():
    rclpy.init()
    monitor = NavFlowMonitor()
    
    # Spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    spin_thread.start()
    
    print("=== Nav2 Velocity Flow Diagnostics ===")
    print("1. Publishing goal to (x: 1.801, y: -2.340) via /goal_pose...")
    
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = monitor.get_clock().now().to_msg()
    goal.pose.position.x = 1.801
    goal.pose.position.y = -2.340
    goal.pose.orientation.w = 1.0
    
    # Publish goal multiple times to ensure Nav2 receives it
    for _ in range(5):
        monitor.goal_pub.publish(goal)
        time.sleep(0.2)
        
    print("2. Monitoring topics for 8 seconds...")
    for i in range(16):
        time.sleep(0.5)
        print(f"Time {0.5 * (i+1):.1f}s | "
              f"cmd_vel_nav: {monitor.counts['cmd_vel_nav']} | "
              f"smoothed: {monitor.counts['cmd_vel_smoothed']} | "
              f"nav2_cmd_vel: {monitor.counts['nav2_cmd_vel']} | "
              f"base_cmd_vel: {monitor.counts['base_controller_cmd_vel']}")
              
    print("\n3. Last received messages sample:")
    for topic, msg in monitor.last_msg.items():
        print(f"\n[{topic}] frame_id={msg.header.frame_id}, stamp_sec={msg.header.stamp.sec}, stamp_nanosec={msg.header.stamp.nanosec}")
        print(f"  linear: x={msg.twist.linear.x:.3f}, y={msg.twist.linear.y:.3f}, z={msg.twist.linear.z:.3f}")
        print(f"  angular: x={msg.twist.angular.x:.3f}, y={msg.twist.angular.y:.3f}, z={msg.twist.angular.z:.3f}")
        
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
