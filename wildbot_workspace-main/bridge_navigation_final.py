#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Float32
from rclpy.qos import QoSProfile, qos_profile_sensor_data

import math
import sys
import time
import os
import re
import threading
import select
import termios
import tty

# ANSI 控制碼定義，用於美化終端機輸出
CLEAR_SCREEN = "\033[2J\033[H"
COLOR_RESET = "\033[0m"
COLOR_BOLD = "\033[1m"
COLOR_RED = "\033[1;31m"
COLOR_GREEN = "\033[1;32m"
COLOR_YELLOW = "\033[1;33m"
COLOR_BLUE = "\033[1;34m"
COLOR_PURPLE = "\033[1;35m"
COLOR_CYAN = "\033[1;36m"

# ==============================================================================
# ⚙️ 實體小車與任務調參配置區 (全域常數定義)
# ==============================================================================
# 💡 您可以在這裡快速修改所有與運動、距離、角度相關的參數

# 1. 偏角修正與雷達幾何尺寸補償 (公尺)
LIDAR_OFFSET_DEG = 9.4          # 雷達安裝偏角修正 (度)
CHASSIS_TO_LIDAR = 0.14           # 光達中心至車體中心之補償距離 (x方向)，預設 14cm (0.14)

# 2. 直行與定位校正 (GO) 任務參數
WALL_TO_BRIDGE_CENTER = 1.25       # 起點後牆到橋中心總距離，預設 200cm (2.0)
STAGE1_GO_DISTANCE = 0.10         # GO 階段 1 里程計前進距離 (避開雷達死區)，預設 10cm (0.10)
GO_SPEED = 0.20                   # 直行速度 (m/s)，預設 0.10 m/s
BRIDGE_EDGE_TRIGGER_DIST = 0.16   # 檢測到橋邊緣的側邊雷達觸發閥值，預設 16cm (0.16)
BRIDGE_EDGE_TARGET_DIST = 0.30    # 檢測到橋邊緣後，車身中心預期從觸發點繼續前進的距離，預設 30cm (0.30)

# 3. 轉彎與雷達對齊 (TURN) 任務參數
TURN_SPEED = 0.35                 # 轉彎時的自轉角速度 (rad/s)，預設 0.35 rad/s
ALIGN_KP = 0.35                   # 雷達角度精細校正對齊的 P 控制器比例增益，預設 0.35
ALIGN_MAX_SPEED = 0.12            # 雷達角度精細校正對齊的最大角速度限制，預設 0.12 rad/s
ROTATION_FACTOR = 1.0             # 原地旋轉里程計打滑/偏差補償係數 (實際旋轉角度 / 里程計回報角度)
                                  # 若小車自轉打滑導致實際轉不夠 90°，請將此係數調大 (例如 1.05)
                                  # 若小車實際轉太多 (超過 90°)，請將此係數調小 (例如 0.95)

# 4. 過橋與下坡 (MOVE_ODOM) 任務參數
CROSS_BRIDGE_DISTANCE = 2.7       # 里程計過橋最大前進距離，預設 270cm (2.7)
MOVE_ODOM_SPEED = 0.20            # 過橋時的前進速度，預設 0.12 m/s
DOWNSLOPE_PITCH_THRESHOLD = 3.0   # 判定開始下坡的 Pitch 仰角閥值 (度)，預設 3.0°
LEVEL_PITCH_THRESHOLD = 1.0       # 判定下坡結束回平的 Pitch 仰角閥值 (度)，預設 1.0°
# ==============================================================================

class BridgeNavigation(Node):
    def __init__(self):
        super().__init__('bridge_navigation')
        
        # 1. 內部狀態變數
        self.lidar_offset_deg = self.load_lidar_offset()
        self.overheated = False
        self.temperatures = [0.0, 0.0, 0.0]
        
        # 實時四向雷達距離
        self.front_dist = None
        self.rear_dist = None
        self.left_dist = None
        self.right_dist = None
        
        # 實時雷達偏角
        self.current_rear_angle = 0.0
        self.current_left_angle = 0.0
        self.current_right_angle = 0.0
        self.raw_rear_angle = None
        self.raw_left_angle = None
        self.raw_right_angle = None
        
        # 里程計位姿、IMU 仰角與當前運動模式
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.current_pitch = None
        self.current_mode = 'idle'
        
        # 2. 訂閱者與發布者
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/base_controller/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.temp_sub = self.create_subscription(
            Float64MultiArray,
            '/arm_joint_temperatures',
            self.temperature_callback,
            qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(TwistStamped, '/base_controller/cmd_vel', 10)
        
        # 3. 用於 Foxglove 實時監控的四向距離發布者
        self.front_dist_pub = self.create_publisher(Float32, '/lidar_dist/front', 10)
        self.rear_dist_pub = self.create_publisher(Float32, '/lidar_dist/rear', 10)
        self.left_dist_pub = self.create_publisher(Float32, '/lidar_dist/left', 10)
        self.right_dist_pub = self.create_publisher(Float32, '/lidar_dist/right', 10)
        
        # 4. 用於 Foxglove 視覺化過濾後的雷達點雲話題
        self.front_scan_pub = self.create_publisher(LaserScan, '/scan/front', 10)
        self.rear_scan_pub = self.create_publisher(LaserScan, '/scan/rear', 10)
        self.left_scan_pub = self.create_publisher(LaserScan, '/scan/left', 10)
        self.right_scan_pub = self.create_publisher(LaserScan, '/scan/right', 10)
        
        self.get_logger().info("BridgeNavigation 節點初始化完成。")

    def load_lidar_offset(self):
        """動態自 test_lidar_orientation.py 讀取最新的偏角修正值"""
        dir_path = os.path.dirname(os.path.abspath(__file__))
        orientation_path = os.path.join(dir_path, "test_lidar_orientation.py")
        if os.path.exists(orientation_path):
            try:
                with open(orientation_path, "r", encoding="utf-8") as f:
                    content = f.read()
                # 搜尋 self.lidar_offset_deg = <數字>
                match = re.search(r"self\.lidar_offset_deg\s*=\s*([\d\.\-]+)", content)
                if match:
                    val = float(match.group(1))
                    self.get_logger().info(f"⚙️ 自動從 test_lidar_orientation.py 載入最新偏角: {val}°")
                    return val
            except Exception as e:
                self.get_logger().warn(f"⚠️ 無法讀取 test_lidar_orientation.py，使用預設值: {e}")
        return LIDAR_OFFSET_DEG

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def quaternion_to_euler(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        return yaw, pitch

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)

    def imu_callback(self, msg):
        _, pitch = self.quaternion_to_euler(msg.orientation)
        self.current_pitch = pitch

    def temperature_callback(self, msg):
        """監控馬達溫度，提供警告與過熱保護"""
        if len(msg.data) >= 2:
            self.temperatures = list(msg.data)
            max_temp = max(self.temperatures)
            
            if max_temp >= 70.0:
                if not self.overheated:
                    self.overheated = True
                    self.get_logger().error(f"🚨🚨🚨 馬達嚴重過熱: {max_temp:.1f}°C！觸發緊急即停保護，已鎖定底盤！")
                    self.stop_chassis()
            elif max_temp <= 60.0:
                if self.overheated:
                    self.overheated = False
                    self.get_logger().info(f"❄️ 馬達溫度已回落: {max_temp:.1f}°C，解除過熱保護")

    def get_closest_point(self, msg, min_deg, max_deg):
        min_rad = math.radians(min_deg)
        max_rad = math.radians(max_deg)
        
        valid_points = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min or r >= msg.range_max:
                continue
            
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_rad = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
            
            in_sector = False
            if min_rad <= max_rad:
                in_sector = min_rad <= angle_rad <= max_rad
            else:
                in_sector = angle_rad >= min_rad or angle_rad <= max_rad
                
            if in_sector:
                valid_points.append((angle_rad, r))
                
        if not valid_points:
            return None, None
            
        closest = min(valid_points, key=lambda x: x[1])
        return closest[0], closest[1]

    def scan_callback(self, msg):
        if self.current_mode == 'go':
            angle_win = 0.5
        elif self.current_mode in ['turn_left', 'turn_right', 'align_wall']:
            angle_win = 30.0  # 旋轉與對齊校正時放寬窗口，防止打滑或偏角過大丟失牆面
        else:
            angle_win = 10.0
        
        # 1. 前 (Front)：180度 附近
        search_front_min = (180.0 - angle_win) + self.lidar_offset_deg
        search_front_max = (-180.0 + angle_win) + self.lidar_offset_deg
        _, dist_f = self.get_closest_point(msg, search_front_min, search_front_max)
        self.front_dist = dist_f
        
        # 2. 後 (Rear)：0度 附近
        search_rear_min = -angle_win + self.lidar_offset_deg
        search_rear_max = angle_win + self.lidar_offset_deg
        angle_r, dist_rear = self.get_closest_point(msg, search_rear_min, search_rear_max)
        self.rear_dist = dist_rear
        if angle_r is not None:
            self.current_rear_angle = math.degrees(angle_r) - self.lidar_offset_deg
            self.raw_rear_angle = math.degrees(angle_r)
        else:
            self.current_rear_angle = None
            self.raw_rear_angle = None
            
        # 3. 左 (Left)：-90度 附近
        search_left_min = (-90.0 - angle_win) + self.lidar_offset_deg
        search_left_max = (-90.0 + angle_win) + self.lidar_offset_deg
        angle_l, dist_l = self.get_closest_point(msg, search_left_min, search_left_max)
        self.left_dist = dist_l
        if angle_l is not None:
            self.current_left_angle = math.degrees(angle_l) - self.lidar_offset_deg
            self.raw_left_angle = math.degrees(angle_l)
        else:
            self.current_left_angle = None
            self.raw_left_angle = None
            
        # 4. 右 (Right)：90度 附近
        search_right_min = (90.0 - angle_win) + self.lidar_offset_deg
        search_right_max = (90.0 + angle_win) + self.lidar_offset_deg
        angle_r_val, dist_r = self.get_closest_point(msg, search_right_min, search_right_max)
        self.right_dist = dist_r
        if angle_r_val is not None:
            self.current_right_angle = math.degrees(angle_r_val) - self.lidar_offset_deg
            self.raw_right_angle = math.degrees(angle_r_val)
        else:
            self.current_right_angle = None
            self.raw_right_angle = None
            
        # 5. 發布四向距離到獨立話題，供 Foxglove 連線顯示與即時繪圖
        front_msg = Float32()
        front_msg.data = float(self.front_dist) if self.front_dist is not None else -1.0
        self.front_dist_pub.publish(front_msg)
        
        rear_msg = Float32()
        rear_msg.data = float(self.rear_dist) if self.rear_dist is not None else -1.0
        self.rear_dist_pub.publish(rear_msg)
        
        left_msg = Float32()
        left_msg.data = float(self.left_dist) if self.left_dist is not None else -1.0
        self.left_dist_pub.publish(left_msg)
        
        right_msg = Float32()
        right_msg.data = float(self.right_dist) if self.right_dist is not None else -1.0
        self.right_dist_pub.publish(right_msg)
        
        # 6. 發布過濾後的雷達資料以供 Foxglove 視覺化
        self.filter_and_publish_scan(msg, search_front_min, search_front_max, self.front_scan_pub)
        self.filter_and_publish_scan(msg, search_rear_min, search_rear_max, self.rear_scan_pub)
        self.filter_and_publish_scan(msg, search_left_min, search_left_max, self.left_scan_pub)
        self.filter_and_publish_scan(msg, search_right_min, search_right_max, self.right_scan_pub)

    def filter_and_publish_scan(self, original_msg, min_deg, max_deg, publisher):
        """將 LaserScan 過濾，只保留 min_deg 到 max_deg 區間，發布至對應 topic"""
        filtered_msg = LaserScan()
        filtered_msg.header = original_msg.header
        filtered_msg.angle_min = original_msg.angle_min
        filtered_msg.angle_max = original_msg.angle_max
        filtered_msg.angle_increment = original_msg.angle_increment
        filtered_msg.time_increment = original_msg.time_increment
        filtered_msg.scan_time = original_msg.scan_time
        filtered_msg.range_min = original_msg.range_min
        filtered_msg.range_max = original_msg.range_max
        
        min_rad = math.radians(min_deg)
        max_rad = math.radians(max_deg)
        
        new_ranges = []
        for i, r in enumerate(original_msg.ranges):
            angle_rad = original_msg.angle_min + i * original_msg.angle_increment
            angle_rad = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
            
            in_sector = False
            if min_rad <= max_rad:
                in_sector = min_rad <= angle_rad <= max_rad
            else: # 跨越 pi / -pi 的情況
                in_sector = angle_rad >= min_rad or angle_rad <= max_rad
                
            if in_sector:
                new_ranges.append(r)
            else:
                new_ranges.append(float('inf'))
                
        filtered_msg.ranges = new_ranges
        if len(original_msg.intensities) > 0:
            new_intensities = []
            for i, intensity in enumerate(original_msg.intensities):
                angle_rad = original_msg.angle_min + i * original_msg.angle_increment
                angle_rad = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
                
                in_sector = False
                if min_rad <= max_rad:
                    in_sector = min_rad <= angle_rad <= max_rad
                else:
                    in_sector = angle_rad >= min_rad or angle_rad <= max_rad
                    
                if in_sector:
                    new_intensities.append(intensity)
                else:
                    new_intensities.append(0.0)
            filtered_msg.intensities = new_intensities
            
        publisher.publish(filtered_msg)

    def stop_chassis(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        for _ in range(5):
            self.cmd_pub.publish(msg)
            time.sleep(0.02)

    def check_space_pressed(self):
        """非阻塞檢測空白鍵是否被按下"""
        if select.select([sys.stdin], [], [], 0.0)[0]:
            ch = sys.stdin.read(1)
            if ch == ' ':
                return True
        return False

    def run_go_to_center(self, target_val=WALL_TO_BRIDGE_CENTER, side='right'):
        self.current_mode = 'idle'  # 先保持 idle 狀態使用寬雷達視窗
        rate = self.create_rate(20)
        
        # 里程計起點
        start_x = self.odom_x
        start_y = self.odom_y
        
        stage1_target = STAGE1_GO_DISTANCE  # 第一階段里程計前進距離
        print(f"\n{COLOR_CYAN}🚀 [步驟 1/3：GO] 階段 1 - 先使用里程計前進 {stage1_target*100:.0f} cm 以避開雷達盲區...{COLOR_RESET}")
        
        # 階段 1 前進
        while rclpy.ok():
            if self.overheated:
                print(f"\n{COLOR_RED}🚨 馬達過熱保護！中止任務。{COLOR_RESET}")
                return False
            if self.check_space_pressed():
                print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                return False
                
            dx = self.odom_x - start_x
            dy = self.odom_y - start_y
            dist_traveled = math.hypot(dx, dy)
            print(f"   🚗 [階段 1] 已行駛: {dist_traveled*100:.1f} / {stage1_target*100:.1f} cm", end="\r")
            
            if dist_traveled >= stage1_target:
                break
                
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = GO_SPEED
            msg.twist.angular.z = 0.0
            self.cmd_pub.publish(msg)
            rate.sleep()
            
        self.stop_chassis()
        time.sleep(0.4)
        
        # 暫時將模式設為 'idle' 讀取寬雷達值
        self.current_mode = 'idle'
        time.sleep(0.15)
        
        samples = []
        print(f"\n{COLOR_YELLOW}📊 正在讀取並過濾當前雷達後牆距離...{COLOR_RESET}")
        for _ in range(5):
            if self.rear_dist is not None:
                samples.append(self.rear_dist)
            time.sleep(0.1)
            
        if not samples:
            print(f"\n{COLOR_RED}❌ 無法讀取車尾雷達距離，停止測試！{COLOR_RESET}")
            return False
            
        current_rear = sum(samples) / len(samples)
        
        # 第二階段計算剩餘前進距離，幾何補償對齊為全域設定 CHASSIS_TO_LIDAR
        go_distance_stage2 = target_val - current_rear - CHASSIS_TO_LIDAR
        print(f"   📊 [換算完成] 後牆至橋中心總目標: {target_val*100:.1f} cm")
        print(f"   📊 [當前狀態] 目前車尾雷達後牆距: {current_rear*100:.1f} cm")
        print(f"   📊 [剩餘前進] 第二階段預估需前進: {go_distance_stage2*100:.1f} cm (已扣除 {CHASSIS_TO_LIDAR*100:.1f}cm 補償)")
        
        if go_distance_stage2 <= 0:
            print(f"\n{COLOR_GREEN}🎉 換算後剩餘距離小於等於 0，小車已抵達或超越橋中心！{COLOR_RESET}")
            return True
            
        # 重新設回 'go' 模式以收窄雷達窗口
        self.current_mode = 'go'
        time.sleep(0.5)
        
        print(f"\n{COLOR_CYAN}🚀 [步驟 1/3：GO] 階段 2 - 開始前進剩餘距離 {go_distance_stage2*100:.1f} cm...{COLOR_RESET}")
        
        start_x_s2 = self.odom_x
        start_y_s2 = self.odom_y
        
        align_triggered = False
        align_start_x = None
        align_start_y = None
        align_target_dist = BRIDGE_EDGE_TARGET_DIST - CHASSIS_TO_LIDAR
        
        while rclpy.ok():
            if self.overheated:
                print(f"\n{COLOR_RED}🚨 馬達過熱保護！中止任務。{COLOR_RESET}")
                return False
            if self.check_space_pressed():
                print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                return False
                
            # 根據 side 決定檢測哪一側雷達
            if side == 'left':
                if not align_triggered and self.left_dist is not None:
                    if self.left_dist < BRIDGE_EDGE_TRIGGER_DIST:
                        align_triggered = True
                        align_start_x = self.odom_x
                        align_start_y = self.odom_y
                        print(f"\n{COLOR_YELLOW}🎯 [左側校正觸發] 左牆距小於 {BRIDGE_EDGE_TRIGGER_DIST*100:.1f}cm ({self.left_dist*100:.1f} cm，已檢測到橋)！改以當前點前進 {align_target_dist*100:.1f} cm 停靠。{COLOR_RESET}")
            else:
                if not align_triggered and self.right_dist is not None:
                    if self.right_dist < BRIDGE_EDGE_TRIGGER_DIST:
                        align_triggered = True
                        align_start_x = self.odom_x
                        align_start_y = self.odom_y
                        print(f"\n{COLOR_YELLOW}🎯 [右側校正觸發] 右牆距小於 {BRIDGE_EDGE_TRIGGER_DIST*100:.1f}cm ({self.right_dist*100:.1f} cm，已檢測到橋)！改以當前點前進 {align_target_dist*100:.1f} cm 停靠。{COLOR_RESET}")
            
            # 根據是否觸發校正來計算前進距離與目標
            if align_triggered:
                dx = self.odom_x - align_start_x
                dy = self.odom_y - align_start_y
                dist_traveled_s2 = math.hypot(dx, dy)
                target_dist_current = align_target_dist
            else:
                dx = self.odom_x - start_x_s2
                dy = self.odom_y - start_y_s2
                dist_traveled_s2 = math.hypot(dx, dy)
                target_dist_current = go_distance_stage2
            
            rear_display = self.rear_dist * 100.0 if self.rear_dist is not None else 999.0
            side_display = (self.left_dist * 100.0 if side == 'left' else self.right_dist * 100.0) if (self.left_dist if side == 'left' else self.right_dist) is not None else 999.0
            side_name = "左" if side == 'left' else "右"
            status_prefix = "🚗 [校正前進中]" if align_triggered else "🚗 [階段 2 前進中]"
            print(f"   {status_prefix} 已行駛: {dist_traveled_s2*100:.1f} / {target_dist_current*100:.1f} cm | 雷達後牆距: {rear_display:.1f} cm | {side_name}牆距: {side_display:.1f} cm", end="\r")
            
            if dist_traveled_s2 >= target_dist_current:
                if align_triggered:
                    print(f"\n{COLOR_GREEN}🎉 觸發{side_name}側校正抵達橋中心！實際行駛: {dist_traveled_s2*100:.1f} cm (自檢測點){COLOR_RESET}")
                else:
                    print(f"\n{COLOR_GREEN}🎉 抵達預設橋中心！實際總行駛距離: {(stage1_target + dist_traveled_s2)*100:.1f} cm{COLOR_RESET}")
                break
                
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = GO_SPEED
            msg.twist.angular.z = 0.0
            self.cmd_pub.publish(msg)
            rate.sleep()
            
        self.stop_chassis()
        return True

    def run_turn_right_90(self):
        self.current_mode = 'idle'
        
        # 同步並獲取右轉前的 initial_rear 距離，用於距離差 (dist_diff) 校驗
        print(f"\n{COLOR_YELLOW}⏳ 右轉初始化，正在記錄初始車尾距離...{COLOR_RESET}")
        for _ in range(20):
            if self.rear_dist is not None:
                break
            time.sleep(0.05)
            
        initial_rear = self.rear_dist if self.rear_dist is not None else 999.0
        
        rate = self.create_rate(20)
        start_yaw = self.odom_yaw
        
        print(f"\n{COLOR_CYAN}🔄 [步驟 2/3：TURN_RIGHT] 階段 1 - 里程計快速右轉 90 度...{COLOR_RESET}")
        
        # 階段 1 右轉
        while rclpy.ok():
            if self.overheated:
                print(f"\n{COLOR_RED}🚨 馬達過熱保護！中止任務。{COLOR_RESET}")
                return False
            if self.check_space_pressed():
                print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                return False

            yaw_diff = abs(self.odom_yaw - start_yaw)
            if yaw_diff > math.pi:
                yaw_diff = 2.0 * math.pi - yaw_diff
            deg_diff = math.degrees(yaw_diff)
            
            dist_diff = abs(self.right_dist - initial_rear) if self.right_dist else 999.0
            
            rear_display = self.rear_dist * 100.0 if self.rear_dist is not None else 999.0
            right_display = self.right_dist * 100.0 if self.right_dist is not None else 999.0
            print(f"   🔄 [右轉中] Odom角: {deg_diff:.1f}°/{90.0*ROTATION_FACTOR:.1f}° | 後牆距: {rear_display:.1f} cm | 右牆距: {right_display:.1f} cm | 距離差: {dist_diff*100:.1f} cm", end="\r")
            
            if deg_diff >= 90.0 * ROTATION_FACTOR:
                print(f"\n{COLOR_GREEN}🎉 里程計右轉完成。接下來開始第二次雷達精細校正...{COLOR_RESET}")
                break
                
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = -TURN_SPEED
            self.cmd_pub.publish(msg)
            rate.sleep()

        # 暫時煞停小車，等待 0.5 秒讓雷達穩定
        self.stop_chassis()
        time.sleep(0.5)
        
        # 階段 2：第二次精細校正 (修正對齊目標為：車尾對齊 0.0°，左側對齊 -90.0°)
        kp_align = ALIGN_KP
        max_speed_align = ALIGN_MAX_SPEED
        stable_count = 0
        align_start_time = time.time()
        
        print(f"\n{COLOR_CYAN}🔄 [步驟 2/3：TURN_RIGHT] 階段 2 - 開始使用車尾雷達(目標 0°)與左側雷達(目標 -90°)進行精細對齊 (設有 5 秒超時保護)...{COLOR_RESET}")
        
        while rclpy.ok():
            if self.overheated:
                print(f"\n{COLOR_RED}🚨 馬達過熱保護！中止任務。{COLOR_RESET}")
                return False
            if self.check_space_pressed():
                print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                return False
                
            # 超時保護
            if time.time() - align_start_time > 5.0:
                print(f"\n{COLOR_YELLOW}⚠️ [精細校正] 右轉對齊已超時 5 秒，使用當前姿勢強制結束對齊！{COLOR_RESET}")
                break
                
            # 優先使用車尾雷達 (0.0°)，備用使用左側雷達 (-90.0°)
            error_deg = None
            ref_sensor = "無"
            
            if self.current_rear_angle is not None:
                error_deg = self.current_rear_angle
                ref_sensor = "車尾雷達 (0°)"
            elif self.current_left_angle is not None:
                error_deg = self.current_left_angle + 90.0
                ref_sensor = "左側雷達 (-90°)"
                
            if error_deg is None:
                f_disp = f"{self.front_dist*100:.1f}" if self.front_dist is not None else "--"
                r_disp = f"{self.rear_dist*100:.1f}" if self.rear_dist is not None else "--"
                l_disp = f"{self.left_dist*100:.1f}" if self.left_dist is not None else "--"
                rg_disp = f"{self.right_dist*100:.1f}" if self.right_dist is not None else "--"
                print(f"   ⚠️ 等待雷達對齊角度更新... | 即時距離(cm) - 前:{f_disp} 後:{r_disp} 左:{l_disp} 右:{rg_disp}      ", end="\r")
                time.sleep(0.1)
                continue
            
            rear_display = self.rear_dist * 100.0 if self.rear_dist is not None else 999.0
            left_display = self.left_dist * 100.0 if self.left_dist is not None else 999.0
            print(f"   🔄 [精細校正中] 參考來源: {ref_sensor} | 誤差: {error_deg:+.1f}° | 後牆距: {rear_display:.1f} cm | 左牆距: {left_display:.1f} cm", end="\r")
            
            # 誤差小於 1.5 度且連續 3 次穩定
            if abs(error_deg) <= 1.5:
                stable_count += 1
                if stable_count >= 3:
                    print(f"\n{COLOR_GREEN}🎉 第二次精細對齊完成！參考來源: {ref_sensor}，誤差: {error_deg:+.1f}°{COLOR_RESET}")
                    break
            else:
                stable_count = 0
                
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = 0.0
            
            # 比例控制自轉角速度
            error_rad = math.radians(error_deg)
            wz = kp_align * error_rad
            wz = max(-max_speed_align, min(max_speed_align, wz))
            
            if abs(error_deg) > 2.0:
                if abs(wz) < 0.035:
                    wz = 0.035 if wz > 0 else -0.035
                    
            msg.twist.angular.z = wz
            self.cmd_pub.publish(msg)
            rate.sleep()

        self.stop_chassis()
        return True

    def run_turn_left_90(self):
        self.current_mode = 'idle'
        
        # 同步並獲取左轉前的 initial_rear 距離，用於距離差 (dist_diff) 校驗
        print(f"\n{COLOR_YELLOW}⏳ 左轉初始化，正在記錄初始車尾距離...{COLOR_RESET}")
        for _ in range(20):
            if self.rear_dist is not None:
                break
            time.sleep(0.05)
            
        initial_rear = self.rear_dist if self.rear_dist is not None else 999.0
        
        rate = self.create_rate(20)
        start_yaw = self.odom_yaw
        
        print(f"\n{COLOR_CYAN}🔄 [步驟 2/3：TURN_LEFT] 階段 1 - 里程計快速左轉 90 度...{COLOR_RESET}")
        
        # 階段 1 左轉
        while rclpy.ok():
            if self.overheated:
                print(f"\n{COLOR_RED}🚨 馬達過熱保護！中止任務。{COLOR_RESET}")
                return False
            if self.check_space_pressed():
                print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                return False

            yaw_diff = abs(self.odom_yaw - start_yaw)
            if yaw_diff > math.pi:
                yaw_diff = 2.0 * math.pi - yaw_diff
            deg_diff = math.degrees(yaw_diff)
            
            # 左轉 90 度後，原本後方的牆會在小車的左側
            dist_diff = abs(self.left_dist - initial_rear) if self.left_dist else 999.0
            
            rear_display = self.rear_dist * 100.0 if self.rear_dist is not None else 999.0
            left_display = self.left_dist * 100.0 if self.left_dist is not None else 999.0
            print(f"   🔄 [左轉中] Odom角: {deg_diff:.1f}°/{90.0*ROTATION_FACTOR:.1f}° | 後牆距: {rear_display:.1f} cm | 左牆距: {left_display:.1f} cm | 距離差: {dist_diff*100:.1f} cm", end="\r")
            
            if deg_diff >= 90.0 * ROTATION_FACTOR:
                print(f"\n{COLOR_GREEN}🎉 里程計左轉完成。接下來開始第二次雷達精細校正...{COLOR_RESET}")
                break
                
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = TURN_SPEED  # 正值為左轉
            self.cmd_pub.publish(msg)
            rate.sleep()

        # 暫時煞停小車，等待 0.5 秒讓雷達穩定
        self.stop_chassis()
        time.sleep(0.5)
        
        # 階段 2：第二次精細校正 (修正對齊目標為：車尾對齊 0.0°，右側對齊 90.0°)
        kp_align = ALIGN_KP
        max_speed_align = ALIGN_MAX_SPEED
        stable_count = 0
        align_start_time = time.time()
        
        print(f"\n{COLOR_CYAN}🔄 [步驟 2/3：TURN_LEFT] 階段 2 - 開始使用車尾雷達(目標 0°)與右側雷達(目標 90°)進行精細對齊 (設有 5 秒超時保護)...{COLOR_RESET}")
        
        while rclpy.ok():
            if self.overheated:
                print(f"\n{COLOR_RED}🚨 馬達過熱保護！中止任務。{COLOR_RESET}")
                return False
            if self.check_space_pressed():
                print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                return False
                
            # 超時保護
            if time.time() - align_start_time > 5.0:
                print(f"\n{COLOR_YELLOW}⚠️ [精細校正] 左轉對齊已超時 5 秒，使用當前姿勢強制結束對齊！{COLOR_RESET}")
                break
                
            # 優先使用車尾雷達 (0.0°)，備用使用右側雷達 (90.0°)
            error_deg = None
            ref_sensor = "無"
            
            if self.current_rear_angle is not None:
                error_deg = self.current_rear_angle
                ref_sensor = "車尾雷達 (0°)"
            elif self.current_right_angle is not None:
                error_deg = self.current_right_angle - 90.0
                ref_sensor = "右側雷達 (90°)"
                
            if error_deg is None:
                f_disp = f"{self.front_dist*100:.1f}" if self.front_dist is not None else "--"
                r_disp = f"{self.rear_dist*100:.1f}" if self.rear_dist is not None else "--"
                l_disp = f"{self.left_dist*100:.1f}" if self.left_dist is not None else "--"
                rg_disp = f"{self.right_dist*100:.1f}" if self.right_dist is not None else "--"
                print(f"   ⚠️ 等待雷達對齊角度更新... | 即時距離(cm) - 前:{f_disp} 後:{r_disp} 左:{l_disp} 右:{rg_disp}      ", end="\r")
                time.sleep(0.1)
                continue
            
            rear_display = self.rear_dist * 100.0 if self.rear_dist is not None else 999.0
            right_display = self.right_dist * 100.0 if self.right_dist is not None else 999.0
            print(f"   🔄 [精細校正中] 參考來源: {ref_sensor} | 誤差: {error_deg:+.1f}° | 後牆距: {rear_display:.1f} cm | 右牆距: {right_display:.1f} cm", end="\r")
            
            # 誤差小於 1.5 度且連續 3 次穩定
            if abs(error_deg) <= 1.5:
                stable_count += 1
                if stable_count >= 3:
                    print(f"\n{COLOR_GREEN}🎉 第二次精細對齊完成！參考來源: {ref_sensor}，誤差: {error_deg:+.1f}°{COLOR_RESET}")
                    break
            else:
                stable_count = 0
                
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = 0.0
            
            # 比例控制自轉角速度
            error_rad = math.radians(error_deg)
            wz = kp_align * error_rad
            wz = max(-max_speed_align, min(max_speed_align, wz))
            
            if abs(error_deg) > 2.0:
                if abs(wz) < 0.035:
                    wz = 0.035 if wz > 0 else -0.035
                    
            msg.twist.angular.z = wz
            self.cmd_pub.publish(msg)
            rate.sleep()

        self.stop_chassis()
        return True

    def run_cross_bridge(self, target_val=CROSS_BRIDGE_DISTANCE, speed=MOVE_ODOM_SPEED):
        self.current_mode = 'idle'
        rate = self.create_rate(20)
        start_x = self.odom_x
        start_y = self.odom_y
        
        print(f"\n{COLOR_CYAN}🚀 [步驟 3/3：MOVE_ODOM] 里程前進過橋：目標最大距離 {target_val*100:.1f} cm，速度 {speed:.3f} m/s...{COLOR_RESET}")
        
        downslope_triggered = False
        
        while rclpy.ok():
            if self.overheated:
                print(f"\n{COLOR_RED}🚨 馬達過熱保護！中止任務。{COLOR_RESET}")
                return False
            if self.check_space_pressed():
                print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                return False
            
            # 計算已行駛距離
            dx = self.odom_x - start_x
            dy = self.odom_y - start_y
            dist_traveled = math.hypot(dx, dy)
            
            pitch_deg = math.degrees(self.current_pitch) if self.current_pitch is not None else 999.0
            
            # 下坡仰角回正監測
            is_aligned_level = False
            if self.current_pitch is not None:
                # 實測：上橋仰角為負，下橋仰角為正。因此大於等於 DOWNSLOPE_PITCH_THRESHOLD 度代表正處於下坡段
                if not downslope_triggered and pitch_deg >= DOWNSLOPE_PITCH_THRESHOLD:
                    downslope_triggered = True
                    print(f"\n{COLOR_YELLOW}🚨 [仰角監測] 偵測到小車開始下坡 (Pitch: {pitch_deg:+.1f}°)，等待回正...{COLOR_RESET}")
                
                # 正在下坡中且仰角降低到小於等於 LEVEL_PITCH_THRESHOLD 度，代表回到水平地面
                if downslope_triggered and pitch_deg <= LEVEL_PITCH_THRESHOLD:
                    is_aligned_level = True
            
            status_str = f" | 仰角: {pitch_deg:+.1f}° (下坡中)" if downslope_triggered else f" | 仰角: {pitch_deg:+.1f}°"
            print(f"   🚗 已行駛: {dist_traveled*100:.1f} / {target_val*100:.1f} cm{status_str}", end="\r")
            
            if is_aligned_level:
                print(f"\n{COLOR_GREEN}🎉 偵測到仰角降回水平 (Pitch: {pitch_deg:+.1f}°)，已成功下橋！提前煞停！{COLOR_RESET}")
                break
            
            if dist_traveled >= target_val:
                print(f"\n{COLOR_GREEN}🎉 抵達預設里程上限！實際行駛: {dist_traveled*100:.1f} cm{COLOR_RESET}")
                break
            
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = speed
            msg.twist.angular.z = 0.0
            self.cmd_pub.publish(msg)
            rate.sleep()

        self.stop_chassis()
        return True


    def run_turn_right_odom(self, target_deg=90.0):
        self.current_mode = 'idle'
        time.sleep(0.5)
        rate = self.create_rate(20)
        start_yaw = self.odom_yaw
        print(f"\n\033[1;36m🔄 [純里程計右轉] 階段 - 快速右轉 {target_deg} 度...\033[0m")
        
        while rclpy.ok():
            if self.overheated:
                print(f"\n\033[1;31m🚨 馬達過熱保護！中止任務。\033[0m")
                return False
            if self.check_space_pressed():
                print(f"\n\033[1;31m🛑 偵測到空白鍵按下！緊急即停！\033[0m")
                return False

            yaw_diff = abs(self.odom_yaw - start_yaw)
            if yaw_diff > math.pi:
                yaw_diff = 2.0 * math.pi - yaw_diff
            deg_diff = math.degrees(yaw_diff)
            
            print(f"   🔄 [右轉中] Odom角: {deg_diff:.1f}°/{target_deg*ROTATION_FACTOR:.1f}°", end="\r")
            
            if deg_diff >= target_deg * ROTATION_FACTOR:
                print(f"\n\033[1;32m🎉 純里程計右轉 {target_deg:.1f} 度完成。\033[0m")
                break
                
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = -TURN_SPEED
            self.cmd_pub.publish(msg)
            rate.sleep()
            
        self.stop_chassis()
        return True

    def run_turn_left_odom(self, target_deg=90.0):
        self.current_mode = 'idle'
        time.sleep(0.5)
        rate = self.create_rate(20)
        start_yaw = self.odom_yaw
        print(f"\n\033[1;36m🔄 [純里程計左轉] 階段 - 快速左轉 {target_deg} 度...\033[0m")
        
        while rclpy.ok():
            if self.overheated:
                print(f"\n\033[1;31m🚨 馬達過熱保護！中止任務。\033[0m")
                return False
            if self.check_space_pressed():
                print(f"\n\033[1;31m🛑 偵測到空白鍵按下！緊急即停！\033[0m")
                return False

            yaw_diff = abs(self.odom_yaw - start_yaw)
            if yaw_diff > math.pi:
                yaw_diff = 2.0 * math.pi - yaw_diff
            deg_diff = math.degrees(yaw_diff)
            
            print(f"   🔄 [左轉中] Odom角: {deg_diff:.1f}°/{target_deg*ROTATION_FACTOR:.1f}°", end="\r")
            
            if deg_diff >= target_deg * ROTATION_FACTOR:
                print(f"\n\033[1;32m🎉 純里程計左轉 {target_deg:.1f} 度完成。\033[0m")
                break
                
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = TURN_SPEED
            self.cmd_pub.publish(msg)
            rate.sleep()
            
        self.stop_chassis()
        return True

    def run_move_distance(self, target_val=0.5, speed=0.2, direction=1):
        self.current_mode = 'idle'
        actual_speed = abs(speed) * direction
        print(f"\n\033[1;36m🚀 [純里程計移動] 目標距離 {target_val*100:.1f} cm，速度 {actual_speed:.3f} m/s\033[0m")
        
        start_x = self.odom_x
        start_y = self.odom_y
        rate = self.create_rate(20)
        
        while rclpy.ok():
            if self.overheated:
                print(f"\n\033[1;31m🚨 馬達過熱保護！中止任務。\033[0m")
                return False
            if self.check_space_pressed():
                print(f"\n\033[1;31m🛑 偵測到空白鍵按下！緊急即停！\033[0m")
                return False
            
            dx = self.odom_x - start_x
            dy = self.odom_y - start_y
            dist_traveled = math.hypot(dx, dy)
            
            print(f"   🚗 已行駛: {dist_traveled*100:.1f} / {target_val*100:.1f} cm", end="\r")
            
            if dist_traveled >= target_val:
                print(f"\n\033[1;32m🎉 抵達預設里程上限！實際行駛: {dist_traveled*100:.1f} cm\033[0m")
                break
            
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = actual_speed
            msg.twist.angular.z = 0.0
            self.cmd_pub.publish(msg)
            rate.sleep()
            
        self.stop_chassis()
        return True


    def main_task_flow(self, side='right'):
        print(f"\n\033[1;33m⏳ 正在初始化測試環境，等待感測器同步...\033[0m")
        
        # 起點時車尾雷達可能處於盲區，因此只強制要求前向雷達與里程計同步
        for _ in range(40):
            if self.front_dist is not None and self.odom_x is not None:
                break
            time.sleep(0.05)
            
        if self.odom_x is None:
            print(f"\n\033[1;31m❌ 里程計感測器同步逾時，請確認底盤驅動是否已啟動！\033[0m")
            return
            
        print(f"\n\033[1;32m✅ 感測器同步完成，開始載入起點配置...\033[0m")
        print(f"\033[1m⚠️  提示：在運動過程中，隨時按下【空白鍵】即可立即停止小車並終止任務！\033[0m\n")
        time.sleep(1.0)
        
        # 1. 運動測試：直行前進至橋中心 (GO) ，距離125cm
        if not self.run_go_to_center(target_val=1.25, side=side): return
        time.sleep(0.5)

        # 2. 運動測試：原地左轉 (純里程計，可自訂角度) ，角度93
        if not self.run_turn_left_odom(target_deg=93.0): return
        time.sleep(0.5)

        # 3. 純里程計移動過橋 (自訂距離/下坡仰角回正監測) (MOVE_ODOM) ，距離270cm
        if not self.run_cross_bridge(target_val=2.70, speed=0.2): return
        time.sleep(0.5)

        # 4. 純里程計前進/後退 (自訂距離/速度/打滑補償) (MOVE_DISTANCE)，距離30cm
        if not self.run_move_distance(target_val=0.30, speed=0.2, direction=1): return
        time.sleep(0.5)

        # 5. 運動測試：原地右轉 (純里程計，可自訂角度)，88度
        if not self.run_turn_right_odom(target_deg=88.0): return
        time.sleep(0.5)

        # 6. 純里程計前進/後退 (自訂距離/速度/打滑補償) (MOVE_DISTANCE) ，距離80cm
        if not self.run_move_distance(target_val=0.80, speed=0.2, direction=1): return
        time.sleep(0.5)

        # 7. 運動測試：原地右轉 (純里程計，可自訂角度)，89度
        if not self.run_turn_right_odom(target_deg=89.0): return
        time.sleep(0.5)

        # 8. 純里程計前進/後退 (自訂距離/速度/打滑補償) (MOVE_DISTANCE) ，距離305cm
        if not self.run_move_distance(target_val=3.05, speed=0.2, direction=1): return
        time.sleep(0.5)

        # 9. 運動測試：原地右轉 (純里程計，可自訂角度)，88度
        if not self.run_turn_right_odom(target_deg=88.0): return
        time.sleep(0.5)

        # 10. 純里程計前進/後退 (自訂距離/速度/打滑補償) (MOVE_DISTANCE) ，距離175cm
        if not self.run_move_distance(target_val=1.75, speed=0.2, direction=1): return
        time.sleep(0.5)
            
        print(f"\n\033[1;32m🎉🎉🎉 恭喜！過橋全自動化導航任務圓滿完成！\033[0m")

def main():
    rclpy.init()
    node = BridgeNavigation()
    
    side = 'right'  # 預設為右側起點
    
    # 支援動態參數輸入：L/R 代表起始位置，數字代表偏角修正
    # 例如：./bnav L -20.0
    for arg in sys.argv[1:]:
        if arg.upper() in ['L', 'R']:
            side = 'left' if arg.upper() == 'L' else 'right'
        else:
            try:
                node.lidar_offset_deg = float(arg)
                node.get_logger().info(f"⚙️ 從命令行參數載入 lidar_offset_deg: {node.lidar_offset_deg}°")
            except ValueError:
                node.get_logger().error(f"⚠️ 無法解析的參數: {arg}，方向請輸入 L 或 R，偏角請輸入數字。")
                
    # 建立背景執行緒來處理 ROS2 的 spin (接收話題與發布)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # 備份終端機設定，以便切換到非阻塞讀取模式
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        node.main_task_flow(side=side)
    except KeyboardInterrupt:
        print(f"\n\n{COLOR_RED}🛑🛑🛑 偵測到 Ctrl+C 中斷，強制煞停小車！{COLOR_RESET}")
    finally:
        # 確保還原終端機設定，防止終端機輸入混亂
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.stop_chassis()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
