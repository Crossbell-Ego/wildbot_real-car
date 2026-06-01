#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import math
import sys
import time

class LidarControlTest(Node):
    def __init__(self, mode, target_val=0.5, lidar_offset_deg=8.6):
        super().__init__('lidar_control_test')
        self.mode = mode  # 'go', 'turn_right', 'turn_left', 'back'
        self.target_val = target_val
        self.lidar_offset_deg = lidar_offset_deg  # 光達安裝偏角修正 (度)
        
        self.cmd_pub = self.create_publisher(TwistStamped, '/base_controller/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.initial_rear_dist = None
        self.initial_left_dist = None
        
        self.current_rear_dist = None
        self.current_rear_angle = None
        self.current_left_dist = None
        self.current_left_angle = None
        
        self.is_done = False
        
        # 控制參數
        self.speed_go = 0.10      # 前進/後退速度 (m/s)
        self.speed_turn = 0.35    # 旋轉速度 (rad/s)
        self.kp_align = 1.0       # 直行修正比例係數 (P controller)
        
        self.get_logger().info(f"🚀 LidarControlTest 啟動！模式: {self.mode.upper()}, 目標值: {self.target_val}")
        self.get_logger().info(f"🔧 已套用光達安裝校正偏角: {self.lidar_offset_deg}°")
        self.get_logger().info("⏳ 正在等待雷達 (/scan) 數據進行初始校準...")

    def get_closest_point(self, msg, min_deg, max_deg, window_size=5):
        """
        在指定角度範圍內尋找最近的反射點，並做平滑處理以過濾雜訊。
        """
        min_rad = math.radians(min_deg)
        max_rad = math.radians(max_deg)
        
        valid_points = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min or r >= msg.range_max:
                continue
            
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_rad = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
            
            # 判斷角度是否在範圍內
            in_sector = False
            if min_rad <= max_rad:
                in_sector = min_rad <= angle_rad <= max_rad
            else: # 跨越 pi / -pi 的情況
                in_sector = angle_rad >= min_rad or angle_rad <= max_rad
                
            if in_sector:
                valid_points.append((angle_rad, r, i))
                
        if not valid_points:
            return None, None
            
        smoothed_points = []
        n_ranges = len(msg.ranges)
        for angle_rad, r, idx in valid_points:
            neighbors = []
            for offset in range(-window_size // 2 + 1, window_size // 2 + 1):
                n_idx = (idx + offset) % n_ranges
                val = msg.ranges[n_idx]
                if math.isfinite(val) and val > msg.range_min and val < msg.range_max:
                    neighbors.append(val)
            if neighbors:
                smoothed_r = sum(neighbors) / len(neighbors)
                smoothed_points.append((angle_rad, smoothed_r))
            else:
                smoothed_points.append((angle_rad, r))
                
        if not smoothed_points:
            return None, None
            
        closest = min(smoothed_points, key=lambda x: x[1])
        return closest[0], closest[1]

    def scan_callback(self, msg):
        # 1. 讀取後方 (車尾) 的牆面 (搜尋區間同步位移)
        search_rear_min = -30.0 + self.lidar_offset_deg
        search_rear_max = 30.0 + self.lidar_offset_deg
        angle_r, dist_r = self.get_closest_point(msg, search_rear_min, search_rear_max)
        if dist_r is not None:
            self.current_rear_dist = dist_r
            # 將量測到的雷達角度扣除安裝偏角，還原成相對於小車真正的車尾角度
            self.current_rear_angle = math.degrees(angle_r) - self.lidar_offset_deg
            
            if self.initial_rear_dist is None:
                self.initial_rear_dist = dist_r
                self.get_logger().info(f"✅ 成功校準車尾後牆初始距離: {self.initial_rear_dist * 100:.1f} cm")
                
        # 2. 讀取左側的牆面 (💡 已修正：在雷達中，小車物理左側在負角度區間，搜尋區間變為 [-120, -60] 度)
        search_left_min = -120.0 + self.lidar_offset_deg
        search_left_max = -60.0 + self.lidar_offset_deg
        angle_l, dist_l = self.get_closest_point(msg, search_left_min, search_left_max)
        if dist_l is not None:
            self.current_left_dist = dist_l
            # 將量測到的雷達角度扣除安裝偏角，還原成相對於小車真正的左側角度
            self.current_left_angle = math.degrees(angle_l) - self.lidar_offset_deg
            
            if self.initial_left_dist is None:
                self.initial_left_dist = dist_l
                self.get_logger().info(f"✅ 成功校準左側牆壁初始距離: {self.initial_left_dist * 100:.1f} cm")

    def stop_chassis(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        for _ in range(5):
            self.cmd_pub.publish(msg)
            time.sleep(0.02)

    def control_loop(self):
        if self.initial_rear_dist is None or self.current_rear_dist is None:
            return # 等待校準完成
            
        if self.is_done:
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # ----------------- 模式 1: 直行去程 (GO) -----------------
        if self.mode == 'go':
            dist_traveled = self.current_rear_dist - self.initial_rear_dist
            self.get_logger().info(
                f"🚗 [前進] 已行駛: {dist_traveled*100:.1f} / {self.target_val*100:.1f} cm | 後牆修正角: {self.current_rear_angle:.1f}° | 左牆距: {self.current_left_dist*100:.1f} cm",
                throttle_duration_sec=0.2
            )
            
            if dist_traveled >= self.target_val:
                self.stop_chassis()
                self.get_logger().info(f"🎉 抵達直行目標點！實際行駛: {dist_traveled*100:.1f} cm")
                self.is_done = True
                self.create_timer(0.2, lambda: rclpy.shutdown())
            else:
                msg.twist.linear.x = self.speed_go
                error_angle_rad = math.radians(self.current_rear_angle)
                msg.twist.angular.z = self.kp_align * error_angle_rad
                self.cmd_pub.publish(msg)

        # ----------------- 模式 2: 原地右轉 90 度 (TURN_RIGHT) -----------------
        elif self.mode == 'turn_right':
            self.get_logger().info(
                f"🔄 [右轉] 目前最近牆面修正角: {self.current_left_angle:.1f}° | 距離: {self.current_left_dist*100:.1f} cm",
                throttle_duration_sec=0.2
            )
            
            dist_diff = abs(self.current_left_dist - self.initial_rear_dist) if self.current_left_dist else 999.0
            
            # 💡 已修正：右轉 90 度後，後牆轉到小車左側，還原後角度應為 -90.0 度
            if self.current_left_angle is not None and abs(self.current_left_angle + 90.0) <= 2.0 and dist_diff < 0.25:
                self.stop_chassis()
                self.get_logger().info(f"🎉 右轉 90 度完成！目前後牆在左側: {self.current_left_angle:.1f}°")
                self.is_done = True
                self.create_timer(0.2, lambda: rclpy.shutdown())
            else:
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = - self.speed_turn
                self.cmd_pub.publish(msg)

        # ----------------- 模式 3: 原地左轉 90 度 (TURN_LEFT) -----------------
        elif self.mode == 'turn_left':
            self.get_logger().info(
                f"🔄 [左轉] 目前最近牆面修正角: {self.current_rear_angle:.1f}° | 距離: {self.current_rear_dist*100:.1f} cm",
                throttle_duration_sec=0.2
            )
            
            dist_diff = abs(self.current_rear_dist - self.initial_rear_dist) if self.current_rear_dist else 999.0
            
            if self.current_rear_angle is not None and abs(self.current_rear_angle) <= 2.0 and dist_diff < 0.25:
                self.stop_chassis()
                self.get_logger().info(f"🎉 左轉 90 度回正完成！目前後牆在車尾: {self.current_rear_angle:.1f}°")
                self.is_done = True
                self.create_timer(0.2, lambda: rclpy.shutdown())
            else:
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = self.speed_turn
                self.cmd_pub.publish(msg)

        # ----------------- 模式 4: 倒退回航 (BACK) -----------------
        elif self.mode == 'back':
            dist_error = self.current_rear_dist - self.initial_rear_dist
            self.get_logger().info(
                f"🚗 [倒車] 距起點剩餘: {dist_error*100:.1f} cm | 後牆修正角: {self.current_rear_angle:.1f}°",
                throttle_duration_sec=0.2
            )
            
            if dist_error <= 0.01:
                self.stop_chassis()
                self.get_logger().info(f"🎉 已成功退回起點！實際與後牆距離: {self.current_rear_dist*100:.1f} cm (偏差: {dist_error*100:.1f} cm)")
                self.is_done = True
                self.create_timer(0.2, lambda: rclpy.shutdown())
            else:
                msg.twist.linear.x = - self.speed_go
                error_angle_rad = math.radians(self.current_rear_angle)
                msg.twist.angular.z = self.kp_align * error_angle_rad
                self.cmd_pub.publish(msg)

def main():
    if len(sys.argv) < 2:
        print("❌ 缺少模式參數！")
        print("使用方式: python3 test_lidar_control.py <go/turn_right/turn_left/back> [目標值] [偏角(度)]")
        print("範例: python3 test_lidar_control.py go 0.5 20.0")
        return
        
    mode = sys.argv[1].lower()
    if mode not in ['go', 'turn_right', 'turn_left', 'back']:
        print("❌ 模式必須為 go, turn_right, turn_left 或 back")
        return
        
    target_val = 0.50
    if len(sys.argv) > 2:
        try:
            target_val = float(sys.argv[2])
        except ValueError:
            print("❌ 目標值必須為數字！")
            return
            
    lidar_offset_deg = 8.6
    if len(sys.argv) > 3:
        try:
            lidar_offset_deg = float(sys.argv[3])
        except ValueError:
            print("❌ 偏角值必須為數字！")
            return
            
    rclpy.init()
    node = LidarControlTest(mode, target_val, lidar_offset_deg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("🛑 偵測到 Ctrl+C 中斷，立即停下小車！")
        node.stop_chassis()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
