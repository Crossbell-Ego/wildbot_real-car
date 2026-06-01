#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
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

class TestLidarOrientation(Node):
    def __init__(self):
        super().__init__('test_lidar_orientation')
        
        # 1. 內部狀態變數
        self.lidar_offset_deg = -2.4
        self.rotation_factor = 1.0  # 原地旋轉里程計打滑/偏差補償係數 (實際旋轉角度 / 里程計回報角度)
        self.overheated = False
        self.temperatures = [0.0, 0.0, 0.0]
        self.is_calibrating = False
        self.calibration_samples = []
        
        # 實時四向雷達距離
        self.front_dist = None
        self.rear_dist = None
        self.left_dist = None
        self.right_dist = None
        
        # 實時雷達偏角 (還原後)
        self.current_rear_angle = 0.0
        self.current_left_angle = 0.0
        self.current_right_angle = 0.0
        self.raw_rear_angle = None
        self.raw_left_angle = None
        self.raw_right_angle = None
        
        # 里程計位姿與當前運動模式
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.current_mode = 'idle'
        self.current_pitch = None
        
        # 2. 訂閱者與發布者
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/base_controller/odom', self.odom_callback, 10)
        self.temp_sub = self.create_subscription(
            Float64MultiArray,
            '/arm_joint_temperatures',
            self.temperature_callback,
            qos_profile_sensor_data
        )
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/base_controller/cmd_vel', 10)
        
        # 3. 過濾後的雷達話題發布器，用於 Foxglove 視覺化
        self.front_scan_pub = self.create_publisher(LaserScan, '/scan/front', 10)
        self.rear_scan_pub = self.create_publisher(LaserScan, '/scan/rear', 10)
        self.left_scan_pub = self.create_publisher(LaserScan, '/scan/left', 10)
        self.right_scan_pub = self.create_publisher(LaserScan, '/scan/right', 10)
        
        self.get_logger().info("TestLidarOrientation CLI 節點初始化完成。")

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

    def imu_callback(self, msg):
        _, pitch = self.quaternion_to_euler(msg.orientation)
        self.current_pitch = pitch

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)

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
        """尋找指定角度範圍內最近的反射點與其真實角度"""
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
        # 根據目前運動模式動態調整雷達感測的扇形角度寬度，以保證直行時防雜訊，旋轉/監看時防漏幀
        if self.current_mode == 'go':
            angle_win = 0.5
        elif self.current_mode in ['turn_left', 'turn_right', 'align_wall', 'turn_left_odom', 'turn_right_odom']:
            angle_win = 15.0  # 旋轉與對齊校正時放寬窗口，防止打滑或偏角過大丟失牆面
        else:
            angle_win = 30.0
        
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
            if self.is_calibrating:
                self.calibration_samples.append(math.degrees(angle_r))
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

        # 5. 發布過濾後的雷達資料以供 Foxglove 視覺化
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
        """發布零速度指令安全停止小車"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        for _ in range(5):
            self.cmd_pub.publish(msg)
            time.sleep(0.02)

    # ==============================================================================
    # 💾 偏角校正檔案回寫
    # ==============================================================================
    def update_lidar_offset_files(self, new_val):
        dir_path = os.path.dirname(os.path.abspath(__file__))
        control_path = os.path.join(dir_path, "test_lidar_control.py")
        coordinator_path = os.path.join(dir_path, "workspaces/src/arm_ik/arm_ik/lidar_competition_coordinator.py")
        self_path = os.path.abspath(__file__)
        
        success = True
        
        # 1. 修改 test_lidar_control.py 中的 lidar_offset_deg
        if os.path.exists(control_path):
            try:
                with open(control_path, "r", encoding="utf-8") as f:
                    content = f.read()
                content = re.sub(r"(lidar_offset_deg\s*=\s*)([\d\.\-]+)", rf"\g<1>{new_val:.1f}", content)
                with open(control_path, "w", encoding="utf-8") as f:
                    f.write(content)
            except Exception:
                success = False

        # 2. 修改 lidar_competition_coordinator.py 中的 lidar_offset_deg
        if os.path.exists(coordinator_path):
            try:
                with open(coordinator_path, "r", encoding="utf-8") as f:
                    content = f.read()
                content = re.sub(r"(lidar_offset_deg\s*=\s*)([\d\.\-]+)", rf"\g<1>{new_val:.1f}", content)
                with open(coordinator_path, "w", encoding="utf-8") as f:
                    f.write(content)
            except Exception:
                success = False
                
        # 3. 修改自己 (test_lidar_orientation.py) 中的 lidar_offset_deg
        if os.path.exists(self_path):
            try:
                with open(self_path, "r", encoding="utf-8") as f:
                    content = f.read()
                content = re.sub(r"(self\.lidar_offset_deg\s*=\s*)([\d\.\-]+)", rf"\g<1>{new_val:.1f}", content)
                with open(self_path, "w", encoding="utf-8") as f:
                    f.write(content)
            except Exception:
                success = False
                
        return success

    # ==============================================================================
    # 🏎️ 單步測試與自動對齊運動狀態機 (支援 空白鍵 / Ctrl+C 緊急即停)
    # ==============================================================================
    def run_movement_test(self, mode, target_val=0.5, speed=0.10, direction=1):
        self.current_mode = 'idle'  # 先保持 idle 狀態以使用較寬的雷達搜索角度 (10.0度)，便於同步與取得 initial_rear
        if self.overheated:
            print(f"\n{COLOR_RED}❌ 機械手臂馬達過熱中！無法啟動運動測試。{COLOR_RESET}")
            time.sleep(1.5)
            return

        print(f"\n{COLOR_YELLOW}⏳ 正在初始化測試環境，等待感測器同步...{COLOR_RESET}")
        for _ in range(40):
            # 如果是 go 模式，車尾雷達在起點可能在盲區，因此不強制要求其同步
            if mode == 'go':
                if self.front_dist is not None and self.odom_x is not None:
                    break
            else:
                if self.front_dist is not None and self.rear_dist is not None and self.odom_x is not None:
                    break
            time.sleep(0.05)

        # 針對非 go 模式，rear_dist 必須存在；對於所有模式，odom_x 必須存在
        if self.odom_x is None or (mode != 'go' and self.rear_dist is None):
            print(f"{COLOR_RED}❌ 感測器同步逾時，雷達或底盤驅動是否未啟動？{COLOR_RESET}")
            time.sleep(2.0)
            return

        # 在寬窗口下取得穩定的初始距離（go 模式下允許為 None）
        initial_rear = self.rear_dist
        initial_left = self.left_dist
        start_x = self.odom_x
        start_y = self.odom_y
        start_yaw = self.odom_yaw
        
        # 同步並取得初始距離後，正式切換到測試模式
        self.current_mode = mode
        
        rate = self.create_rate(20)
        
        print(f"\n{COLOR_GREEN}🚀 運動控制啟動！測試模式: {mode.upper()}{COLOR_RESET}")
        print(f"{COLOR_BOLD}⚠️  提示：在運動過程中，隨時按下【空白鍵】即可立即煞車並返回主選單！{COLOR_RESET}\n")
        time.sleep(1.0)

        # 備份終端機設定，以便切換到非阻塞讀取模式
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            # 設定終端機為 cbreak 模式（按鍵不需 Enter 即可即時讀取）
            tty.setcbreak(sys.stdin.fileno())
            
            def check_space_pressed():
                """非阻塞檢測空白鍵是否被按下"""
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    ch = sys.stdin.read(1)
                    if ch == ' ':
                        return True
                return False

            # ----------------- 模式 1: 前進 (GO) -----------------
            if mode == 'go':
                stage1_target = 0.10  # 第一階段先走 10 cm 以避開雷達近距離盲區
                print(f"\n{COLOR_CYAN}🚀 [階段 1] 先使用里程計前進 {stage1_target*100:.0f} cm 以避開雷達盲區...{COLOR_RESET}")
                
                # 第一階段前進
                while rclpy.ok():
                    if self.overheated:
                        print(f"\n{COLOR_RED}🚨 過熱保護觸發！中斷測試{COLOR_RESET}")
                        break
                    
                    if check_space_pressed():
                        print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                        break
                        
                    dx = self.odom_x - start_x
                    dy = self.odom_y - start_y
                    dist_traveled = math.hypot(dx, dy)
                    
                    print(f"   🚗 [階段 1 前進中] 已行駛: {dist_traveled*100:.1f} / {stage1_target*100:.1f} cm", end="\r")
                    
                    if dist_traveled >= stage1_target:
                        break
                        
                    msg = TwistStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'
                    msg.twist.linear.x = 0.2
                    msg.twist.angular.z = 0.0
                    self.cmd_pub.publish(msg)
                    rate.sleep()
                
                # 第一階段結束，先短暫煞停小車以利讀取穩定的雷達數值
                self.stop_chassis()
                time.sleep(1.0)
                
                # 暫時將模式設為 'idle'，以使用寬窗口 (10.0度) 讀取雷達值
                self.current_mode = 'idle'
                time.sleep(0.5)  # 等待雷達在寬窗口下更新數據
                
                # 讀取當前的雷達後牆距，採樣 15 次以過濾噪聲
                samples = []
                print(f"\n{COLOR_YELLOW}📊 正在讀取並過濾當前雷達後牆距離...{COLOR_RESET}")
                for _ in range(15):
                    if self.rear_dist is not None:
                        samples.append(self.rear_dist)
                    time.sleep(0.1)
                
                if not samples:
                    print(f"\n{COLOR_RED}❌ 無法讀取車尾雷達距離，停止測試！{COLOR_RESET}")
                    return
                
                current_rear = sum(samples) / len(samples)
                
                # 第二階段計算剩餘前進距離
                # target_val 代表後面牆壁到橋中心的總距離 (公尺)
                # 減去 14cm (0.14m) 的幾何補償，使小車中心精確落在橋中心
                go_distance_stage2 = target_val - current_rear - 0.14
                
                print(f"   📊 [換算完成] 後牆至橋中心總目標: {target_val*100:.1f} cm")
                print(f"   📊 [當前狀態] 目前車尾雷達後牆距: {current_rear*100:.1f} cm")
                print(f"   📊 [剩餘前進] 第二階段預估需前進: {go_distance_stage2*100:.1f} cm")
                
                if go_distance_stage2 <= 0:
                    print(f"\n{COLOR_GREEN}🎉 換算後剩餘距離小於等於 0，小車已抵達或超越橋中心！{COLOR_RESET}")
                    return
                
                # 重新設回 'go' 模式以收窄雷達窗口，並開始前進
                self.current_mode = 'go'
                time.sleep(0.5)
                
                print(f"\n{COLOR_CYAN}🚀 [階段 2] 開始前進剩餘距離 {go_distance_stage2*100:.1f} cm...{COLOR_RESET}")
                
                # 重新紀錄第二階段的 Odom 起點
                start_x_s2 = self.odom_x
                start_y_s2 = self.odom_y
                
                # 初始化左側雷達校正相關變數
                left_align_triggered = False
                align_start_x = None
                align_start_y = None
                align_target_dist = 0.30 - 0.135  # 走 30 cm - 車身距離 (13.5 cm) = 16.5 cm
                
                # 第二階段前進
                while rclpy.ok():
                    if self.overheated:
                        print(f"\n{COLOR_RED}🚨 過熱保護觸發！中斷測試{COLOR_RESET}")
                        break
                    
                    if check_space_pressed():
                        print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                        break
                        
                    # 左側雷達校正判定：小於 16cm (0.16m) 時觸發
                    if not left_align_triggered and self.left_dist is not None:
                        if self.left_dist < 0.16:
                            left_align_triggered = True
                            align_start_x = self.odom_x
                            align_start_y = self.odom_y
                            print(f"\n{COLOR_YELLOW}🎯 [左側校正觸發] 左牆距小於 16cm ({self.left_dist*100:.1f} cm，已檢測到橋)！改以當前點前進 {align_target_dist*100:.1f} cm 停靠。{COLOR_RESET}")
                    
                    # 根據是否觸發校正來計算前進距離與目標
                    if left_align_triggered:
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
                    left_display = self.left_dist * 100.0 if self.left_dist is not None else 999.0
                    status_prefix = "🚗 [校正前進中]" if left_align_triggered else "🚗 [階段 2 前進中]"
                    print(f"   {status_prefix} 已行駛: {dist_traveled_s2*100:.1f} / {target_dist_current*100:.1f} cm | 雷達後牆距: {rear_display:.1f} cm | 左牆距: {left_display:.1f} cm", end="\r")
                    
                    if dist_traveled_s2 >= target_dist_current:
                        if left_align_triggered:
                            print(f"\n{COLOR_GREEN}🎉 觸發左側校正抵達橋中心！實際行駛: {dist_traveled_s2*100:.1f} cm (自檢測點){COLOR_RESET}")
                        else:
                            print(f"\n{COLOR_GREEN}🎉 抵達預設橋中心！實際總行駛距離: {(stage1_target + dist_traveled_s2)*100:.1f} cm{COLOR_RESET}")
                        break
                        
                    msg = TwistStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'
                    msg.twist.linear.x = 0.2
                    msg.twist.angular.z = 0.0
                    self.cmd_pub.publish(msg)
                    rate.sleep()

            # ----------------- 模式 2: 原地右轉 (TURN_RIGHT / TURN_RIGHT_ODOM) -----------------
            elif mode in ['turn_right', 'turn_right_odom']:
                target_deg = target_val if mode == 'turn_right_odom' else 90.0
                # 階段 1：快速右轉 (里程計達標為止)
                while rclpy.ok():
                    if self.overheated:
                        break
                        
                    if check_space_pressed():
                        print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                        break

                    yaw_diff = abs(self.odom_yaw - start_yaw)
                    if yaw_diff > math.pi:
                        yaw_diff = 2.0 * math.pi - yaw_diff
                    deg_diff = math.degrees(yaw_diff)
                    
                    dist_diff = abs(self.left_dist - initial_rear) if self.left_dist else 999.0
                    
                    is_odom_aligned = (deg_diff >= target_deg * self.rotation_factor)
                    
                    rear_display = self.rear_dist * 100.0 if self.rear_dist is not None else 999.0
                    right_display = self.right_dist * 100.0 if self.right_dist is not None else 999.0
                    print(f"   🔄 [右轉中] Odom角: {deg_diff:.1f}°/{target_deg*self.rotation_factor:.1f}° | 後牆距: {rear_display:.1f} cm | 右牆距: {right_display:.1f} cm | 距離差: {dist_diff*100:.1f} cm", end="\r")
                    
                    if is_odom_aligned:
                        if mode == 'turn_right_odom':
                            print(f"\n{COLOR_GREEN}🎉 純里程計右轉 {target_deg:.1f} 度完成。{COLOR_RESET}")
                            self.stop_chassis()
                            return
                        print(f"\n{COLOR_GREEN}🎉 里程計右轉完成。接下來開始第二次雷達精細校正...{COLOR_RESET}")
                        break
                        
                    msg = TwistStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'
                    msg.twist.linear.x = 0.0
                    msg.twist.angular.z = -0.35
                    self.cmd_pub.publish(msg)
                    rate.sleep()

                # 暫時煞停小車，等待 0.5 秒讓雷達穩定
                self.stop_chassis()
                time.sleep(0.5)
                
                # 階段 2：第二次精細校正 (校正右轉後右側雷達為 90 度)
                kp_align = 0.35
                max_speed_align = 0.12
                stable_count = 0
                align_start_time = time.time()
                
                print(f"\n{COLOR_CYAN}🔄 [第二次校正] 開始使用右側雷達對齊 90.0° (設有 5 秒超時保護)...{COLOR_RESET}")
                
                while rclpy.ok():
                    if self.overheated:
                        break
                    if check_space_pressed():
                        print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                        break
                        
                    if time.time() - align_start_time > 5.0:
                        print(f"\n{COLOR_YELLOW}⚠️ [精細校正] 右轉對齊超時 5 秒，強制結束對齊以保護底盤！{COLOR_RESET}")
                        break
                        
                    if self.current_right_angle is None:
                        f_disp = f"{self.front_dist*100:.1f}" if self.front_dist is not None else "--"
                        r_disp = f"{self.rear_dist*100:.1f}" if self.rear_dist is not None else "--"
                        l_disp = f"{self.left_dist*100:.1f}" if self.left_dist is not None else "--"
                        rg_disp = f"{self.right_dist*100:.1f}" if self.right_dist is not None else "--"
                        print(f"   ⚠️ 等待右側雷達角度更新... | 即時距離(cm) - 前:{f_disp} 後:{r_disp} 左:{l_disp} 右:{rg_disp}      ", end="\r")
                        time.sleep(0.1)
                        continue
                    
                    # 誤差值 = 當前右側角度 - 90.0
                    error_deg = self.current_right_angle - 90.0
                    
                    rear_display = self.rear_dist * 100.0 if self.rear_dist is not None else 999.0
                    right_display = self.right_dist * 100.0 if self.right_dist is not None else 999.0
                    print(f"   🔄 [精細校正中] 右側雷達角: {self.current_right_angle:.1f}° (誤差: {error_deg:+.1f}°) | 後牆距: {rear_display:.1f} cm | 右牆距: {right_display:.1f} cm", end="\r")
                    
                    # 誤差小於 1.5 度且連續 3 次穩定
                    if abs(error_deg) <= 1.5:
                        stable_count += 1
                        if stable_count >= 3:
                            print(f"\n{COLOR_GREEN}🎉 第二次精細對齊完成！最終右側雷達角: {self.current_right_angle:.1f}°{COLOR_RESET}")
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

            # ----------------- 模式 3: 原地左轉 (TURN_LEFT / TURN_LEFT_ODOM) -----------------
            elif mode in ['turn_left', 'turn_left_odom']:
                target_deg = target_val if mode == 'turn_left_odom' else 90.0
                # 階段 1：快速左轉 (里程計達標為止)
                while rclpy.ok():
                    if self.overheated:
                        break
                        
                    if check_space_pressed():
                        print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                        break

                    yaw_diff = abs(self.odom_yaw - start_yaw)
                    if yaw_diff > math.pi:
                        yaw_diff = 2.0 * math.pi - yaw_diff
                    deg_diff = math.degrees(yaw_diff)
                    
                    dist_diff = abs(self.rear_dist - initial_rear) if self.rear_dist else 999.0
                    
                    is_odom_aligned = (deg_diff >= target_deg * self.rotation_factor)
                    
                    rear_display = self.rear_dist * 100.0 if self.rear_dist is not None else 999.0
                    left_display = self.left_dist * 100.0 if self.left_dist is not None else 999.0
                    print(f"   🔄 [左轉中] Odom角: {deg_diff:.1f}°/{target_deg*self.rotation_factor:.1f}° | 後牆距: {rear_display:.1f} cm | 左牆距: {left_display:.1f} cm", end="\r")
                    
                    if is_odom_aligned:
                        if mode == 'turn_left_odom':
                            print(f"\n{COLOR_GREEN}🎉 純里程計左轉 {target_deg:.1f} 度完成。{COLOR_RESET}")
                            self.stop_chassis()
                            return
                        print(f"\n{COLOR_GREEN}🎉 里程計左轉完成。接下來開始第二次雷達精細校正...{COLOR_RESET}")
                        break
                        
                    msg = TwistStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'
                    msg.twist.linear.x = 0.0
                    msg.twist.angular.z = 0.35
                    self.cmd_pub.publish(msg)
                    rate.sleep()

                # 暫時煞停小車，等待 0.5 秒讓雷達穩定
                self.stop_chassis()
                time.sleep(0.5)
                
                # 階段 2：第二次精細校正 (校正左轉後左側雷達為 -90 度)
                kp_align = 0.35
                max_speed_align = 0.12
                stable_count = 0
                align_start_time = time.time()
                
                print(f"\n{COLOR_CYAN}🔄 [第二次校正] 開始使用左側雷達對齊 -90.0° (設有 5 秒超時保護)...{COLOR_RESET}")
                
                while rclpy.ok():
                    if self.overheated:
                        break
                    if check_space_pressed():
                        print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                        break
                        
                    if time.time() - align_start_time > 5.0:
                        print(f"\n{COLOR_YELLOW}⚠️ [精細校正] 左轉對齊超時 5 秒，強制結束對齊以保護底盤！{COLOR_RESET}")
                        break
                        
                    if self.current_left_angle is None:
                        f_disp = f"{self.front_dist*100:.1f}" if self.front_dist is not None else "--"
                        r_disp = f"{self.rear_dist*100:.1f}" if self.rear_dist is not None else "--"
                        l_disp = f"{self.left_dist*100:.1f}" if self.left_dist is not None else "--"
                        rg_disp = f"{self.right_dist*100:.1f}" if self.right_dist is not None else "--"
                        print(f"   ⚠️ 等待左側雷達角度更新... | 即時距離(cm) - 前:{f_disp} 後:{r_disp} 左:{l_disp} 右:{rg_disp}      ", end="\r")
                        time.sleep(0.1)
                        continue
                    
                    # 誤差值 = 當前左側角度 - (-90.0)
                    error_deg = self.current_left_angle + 90.0
                    
                    rear_display = self.rear_dist * 100.0 if self.rear_dist is not None else 999.0
                    left_display = self.left_dist * 100.0 if self.left_dist is not None else 999.0
                    print(f"   🔄 [精細校正中] 左側雷達角: {self.current_left_angle:.1f}° (誤差: {error_deg:+.1f}°) | 後牆距: {rear_display:.1f} cm | 左牆距: {left_display:.1f} cm", end="\r")
                    
                    # 誤差小於 1.5 度且連續 3 次穩定
                    if abs(error_deg) <= 1.5:
                        stable_count += 1
                        if stable_count >= 3:
                            print(f"\n{COLOR_GREEN}🎉 第二次精細對齊完成！最終左側雷達角: {self.current_left_angle:.1f}°{COLOR_RESET}")
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

            # ----------------- 模式 4: 雙閉環倒退 (BACK) -----------------
            elif mode == 'back':
                while rclpy.ok():
                    if self.overheated:
                        break
                        
                    if check_space_pressed():
                        print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                        break

                    dist_error = self.rear_dist - 0.25 # 預設起點基準是 25cm
                    if initial_rear < 0.85:
                        dist_error = self.rear_dist - initial_rear
                        
                    _rear_a = f"{self.current_rear_angle:.1f}°" if self.current_rear_angle is not None else "--°"
                    print(f"   🚗 [倒車中] 離起點基準剩餘: {dist_error*100:.1f} cm | 車尾修正角: {_rear_a}", end="\r")
                    
                    if dist_error <= 0.01:
                        print(f"\n{COLOR_GREEN}🎉 已成功退回起點！實際與後牆距離: {self.rear_dist*100:.1f} cm{COLOR_RESET}")
                        break
                        
                    msg = TwistStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'
                    msg.twist.linear.x = -0.2
                    # 雙閉環偏角修正
                    error_angle = math.radians(self.current_rear_angle)
                    msg.twist.angular.z = 1.0 * error_angle
                    self.cmd_pub.publish(msg)
                    rate.sleep()

            # ----------------- 模式 5: 自動對齊後牆 (ALIGN_WALL) -----------------
            elif mode == 'align_wall':
                kp_yaw = 0.35      # 降小比例增益，防修正過猛震盪
                max_speed = 0.12   # 降低最大自轉速度，使動作更平緩
                stable_count = 0   # 穩定計數器
                align_start_time = time.time()
                
                while rclpy.ok():
                    if self.overheated:
                        break
                        
                    if check_space_pressed():
                        print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                        break

                    if time.time() - align_start_time > 5.0:
                        print(f"\n{COLOR_YELLOW}⚠️ [自動對齊] 後牆對齊超時 5 秒，強制結束對齊以保護底盤！{COLOR_RESET}")
                        break

                    error_deg = self.current_rear_angle
                    print(f"   🔄 [自動對齊中] 當前車尾偏角: {error_deg:+.1f}° | 目標值: 0.0°", end="\r")
                    
                    # 偏差小於 1.5 度時累計，連續 2 次穩定在區間內才視為完成，防跳變或衝過頭
                    if abs(error_deg) <= 1.5:
                        stable_count += 1
                        if stable_count >= 2:
                            print(f"\n{COLOR_GREEN}🎉 自動對齊完成並已穩定！最終車尾偏角: {error_deg:+.1f}°{COLOR_RESET}")
                            break
                    else:
                        stable_count = 0
                        
                    msg = TwistStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'
                    msg.twist.linear.x = 0.0
                    
                    # 比例控制自轉角速度
                    error_rad = math.radians(error_deg)
                    wz = kp_yaw * error_rad
                    wz = max(-max_speed, min(max_speed, wz))
                    
                    # 當偏差大於 2.0 度時，才設置防死區的最小轉速，防馬達抖動轉不動
                    # 當偏差小於 2.0 度時，不設最小速度限制，讓速度隨距離自然衰減至零，保證順滑煞停
                    if abs(error_deg) > 2.0:
                        if abs(wz) < 0.035:
                            wz = 0.035 if wz > 0 else -0.035
                        
                    msg.twist.angular.z = wz
                    self.cmd_pub.publish(msg)
                    rate.sleep()

            # ----------------- 模式 6: 純里程計前進/後退 (MOVE_ODOM) -----------------
            elif mode == 'move_odom':
                # target_val: 預期移動距離 (m) (即橋的總長度保底，已含打滑補償)
                # speed: 車速 (m/s), direction: +1 前進, -1 後退
                actual_speed = abs(speed) * direction
                
                print(f"\n{COLOR_CYAN}🚀 里程計移動中：目標最大距離 {target_val*100:.1f} cm，速度 {actual_speed:.3f} m/s{COLOR_RESET}")
                
                downslope_triggered = False
                
                while rclpy.ok():
                    if self.overheated:
                        print(f"\n{COLOR_RED}🚨 過熱保護觸發！中斷測試{COLOR_RESET}")
                        break
                    
                    if check_space_pressed():
                        print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                        break
                    
                    # 計算已行駛距離
                    dx = self.odom_x - start_x
                    dy = self.odom_y - start_y
                    dist_traveled = math.hypot(dx, dy)
                    
                    pitch_deg = math.degrees(self.current_pitch) if self.current_pitch is not None else 999.0
                    
                    # 下坡仰角回正監測 (僅在車頭前進且 IMU 數據有效時啟用)
                    is_aligned_level = False
                    if direction >= 0 and self.current_pitch is not None:
                        # 根據實實測：上橋仰角為負，下橋仰角為正。因此大於等於 3.0 度代表正處於下坡段
                        if not downslope_triggered and pitch_deg >= 3.0:
                            downslope_triggered = True
                            print(f"\n{COLOR_YELLOW}🚨 [仰角監測] 偵測到小車開始下坡 (Pitch: {pitch_deg:+.1f}°)，等待回正...{COLOR_RESET}")
                        
                        # 正在下坡中且仰角降低到小於等於 1.0 度，代表回到水平地面
                        if downslope_triggered and pitch_deg <= 1.0:
                            is_aligned_level = True
                    
                    status_str = ""
                    if direction >= 0 and self.current_pitch is not None:
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
                    msg.twist.linear.x = actual_speed
                    msg.twist.angular.z = 0.0
                    self.cmd_pub.publish(msg)
                    rate.sleep()

            # ----------------- 模式 7: 純里程計前進/後退 (MOVE_DISTANCE) -----------------
            elif mode == 'move_distance':
                actual_speed = abs(speed) * direction
                print(f"\n{COLOR_CYAN}🚀 純里程計移動中：目標距離 {target_val*100:.1f} cm，速度 {actual_speed:.3f} m/s{COLOR_RESET}")
                
                while rclpy.ok():
                    if self.overheated:
                        print(f"\n{COLOR_RED}🚨 過熱保護觸發！中斷測試{COLOR_RESET}")
                        break
                    
                    if check_space_pressed():
                        print(f"\n{COLOR_RED}🛑 偵測到空白鍵按下！緊急即停！{COLOR_RESET}")
                        break
                    
                    dx = self.odom_x - start_x
                    dy = self.odom_y - start_y
                    dist_traveled = math.hypot(dx, dy)
                    
                    print(f"   🚗 已行駛: {dist_traveled*100:.1f} / {target_val*100:.1f} cm", end="\r")
                    
                    if dist_traveled >= target_val:
                        print(f"\n{COLOR_GREEN}🎉 抵達預設里程上限！實際行駛: {dist_traveled*100:.1f} cm{COLOR_RESET}")
                        break
                    
                    msg = TwistStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'
                    msg.twist.linear.x = actual_speed
                    msg.twist.angular.z = 0.0
                    self.cmd_pub.publish(msg)
                    rate.sleep()


        except KeyboardInterrupt:
            print(f"\n\n{COLOR_RED}🛑🛑🛑 緊急即停 (E-Stop) 被觸發！強制停止小車！{COLOR_RESET}")
            
        finally:
            # 確保還原終端機設定，防止終端機輸入混亂
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.stop_chassis()
            self.current_mode = 'idle'
            print(f"\n{COLOR_YELLOW}⏳ 小車已煞停。將在 3 秒後自動返回主選單...{COLOR_RESET}")
            time.sleep(3.0)

    # ==============================================================================
    # 🖥️ 終端機選單與監控介面 (純 CLI 互動)
    # ==============================================================================
    def run_realtime_monitor(self):
        """實時在終端機刷新列印四向距離，按 'q' 鍵可返回主選單"""
        # 先進行一次完整的清空螢幕，並將游標移到 Home
        print(CLEAR_SCREEN)
        sys.stdout.write("\033[H")
        sys.stdout.flush()
        
        # 備份終端機設定
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            # 切換到 cbreak 模式以進行即時按鍵讀取
            tty.setcbreak(sys.stdin.fileno())
            
            while rclpy.ok():
                # 檢測是否按下 'q' 或 'Q'
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    ch = sys.stdin.read(1)
                    if ch.lower() == 'q':
                        break
                
                # 每次輸出前，將游標重置回頂部，避免折行導致畫面錯亂滾動
                sys.stdout.write("\033[H")
                
                # 每一行末尾都加上 \033[K (清除該行原本殘留的文字)
                sys.stdout.write(f"📡 {COLOR_BOLD}開始實時監看雷達距離 (按 'q' 鍵可隨時返回主選單){COLOR_RESET}\033[K\n")
                sys.stdout.write("="*60 + "\033[K\n")
                
                front_str = self.format_dist(self.front_dist)
                rear_str = self.format_dist(self.rear_dist)
                left_str = self.format_dist(self.left_dist)
                right_str = self.format_dist(self.right_dist)
                
                _rear_a = f"{self.current_rear_angle:+.1f}°" if self.current_rear_angle is not None else "--°"
                _left_a = f"{self.current_left_angle:+.1f}°" if self.current_left_angle is not None else "--°"
                sys.stdout.write(f"  前 (Front): {front_str} | 後 (Rear): {rear_str} (偏角: {_rear_a})\033[K\n")
                sys.stdout.write(f"  左 (Left) : {left_str} (偏角: {_left_a}) | 右 (Right): {right_str}\033[K\n")
                
                _pitch_a = f"{math.degrees(self.current_pitch):+.1f}°" if self.current_pitch is not None else "--°"
                if self.odom_x is not None:
                    sys.stdout.write(f"  Odom 位姿 : X = {self.odom_x*100:+.1f} cm, Y = {self.odom_y*100:+.1f} cm, Yaw = {math.degrees(self.odom_yaw):+.1f}° | Pitch = {_pitch_a}\033[K\n")
                else:
                    sys.stdout.write(f"  Odom 位姿 : ⏳ 等待底盤數據...\033[K\n")
                
                temp_str = ", ".join([f"M{i+1}={t:.1f}°C" for i, t in enumerate(self.temperatures)])
                overheat_status = f"{COLOR_RED}[過熱鎖定]{COLOR_RESET}" if self.overheated else f"{COLOR_GREEN}[正常]{COLOR_RESET}"
                sys.stdout.write(f"  馬達溫度  : {temp_str}  狀態: {overheat_status}\033[K\n")
                sys.stdout.write("="*60 + "\033[K\n")
                
                sys.stdout.flush()
                time.sleep(0.2)
                
        except KeyboardInterrupt:
            pass
            
        finally:
            # 確保還原終端機設定，防止終端機輸入混亂
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            # 清理螢幕游標位置到這幾行下方
            print("\n\n\n\n\n")
            print(f"{COLOR_YELLOW}↩️ 已返回主選單。{COLOR_RESET}")
            time.sleep(0.5)

    def format_dist(self, val):
        if val is None or val > 10.0:
            return f"{COLOR_RED}-- cm{COLOR_RESET}"
        cm = val * 100
        if cm < 30.0:
            return f"{COLOR_RED}{cm:.1f} cm (防撞警告!){COLOR_RESET}"
        return f"{COLOR_GREEN}{cm:.1f} cm{COLOR_RESET}"

    def menu_loop(self):
        """互動選單主循環"""
        # 等待初次雷達同步
        print("⏳ 正在等待雷達數據同步...")
        for _ in range(30):
            if self.front_dist is not None:
                break
            time.sleep(0.1)

        while rclpy.ok():
            print(CLEAR_SCREEN)
            print("="*60)
            print(f"🤖  {COLOR_BOLD}Wildbot 雷達朝向檢測與運動測試工具 (純 CLI 版){COLOR_RESET}  🤖")
            print("="*60)
            print(f" 🔧 當前偏角修正 (lidar_offset_deg) : {COLOR_CYAN}{self.lidar_offset_deg:.1f}°{COLOR_RESET}")
            
            # 列印即時狀態摘要
            print(f" 📡 【四向距離快速摘要】:")
            _rear_a = f"{self.current_rear_angle:+.1f}°" if self.current_rear_angle is not None else "--°"
            _left_a = f"{self.current_left_angle:+.1f}°" if self.current_left_angle is not None else "--°"
            _pitch_a = f"{math.degrees(self.current_pitch):+.1f}°" if self.current_pitch is not None else "--°"
            print(f"   - 前 (Front): {self.format_dist(self.front_dist)}")
            print(f"   - 後 (Rear) : {self.format_dist(self.rear_dist)} (偏角: {_rear_a})")
            print(f"   - 左 (Left) : {self.format_dist(self.left_dist)} (偏角: {_left_a})")
            print(f"   - 右 (Right): {self.format_dist(self.right_dist)} | Pitch: {_pitch_a}")
            
            # 溫度與過熱狀態
            if self.overheated:
                print(f" 🌡️  馬達溫度 : {COLOR_RED}🚨 馬達嚴重過熱保護中！已鎖定所有移動指令。{COLOR_RESET}")
            else:
                print(f" 🌡️  馬達溫度 : Q1={self.temperatures[0]:.1f}°C, Q2={self.temperatures[1]:.1f}°C, Grip={self.temperatures[2]:.1f}°C")
            print("-"*60)
            
            print("  1. 一鍵自動校正並保存雷達偏角 (採樣並回寫硬碟)")
            print("  2. 實時監看雷達距離 (不斷刷新顯示)")
            print("  3. 運動測試：直行前進至橋中心 (GO)")
            print("  4. 運動測試：原地右轉 90度 + 光達校正 (TURN_RIGHT)")
            print("  5. 運動測試：原地左轉 90度 + 光達校正 (TURN_LEFT)")
            print("  6. 運動測試：原地右轉 (純里程計，可自訂角度)")
            print("  7. 運動測試：原地左轉 (純里程計，可自訂角度)")
            print("  8. 運動測試：雙閉環倒退回起點 (BACK)")
            print("  9. 原地自動對齊後牆 (Auto-Align to 0°)")
            print("  10. 純里程計移動過橋 (自訂距離/下坡仰角回正監測) (MOVE_ODOM)")
            print("  11. 純里程計前進/後退 (自訂距離/速度/打滑補償) (MOVE_DISTANCE)")
            print("  12. 離開程式")
            print("="*60)
            
            choice = input(f"{COLOR_BOLD}請輸入選項 (1-12): {COLOR_RESET}").strip()
            
            if choice == '1':
                print(f"\n{COLOR_YELLOW}📌 前提：請確保小車已手動擺正（車尾與後方牆壁完全平行貼齊）{COLOR_RESET}")
                confirm = input("📊 確定已擺正並開始自動校正？(y/n, 預設 y): ").strip().lower()
                if confirm != 'n':
                    self.calibration_samples = []
                    self.is_calibrating = True
                    print(f"⏳ 正在接收雷達數據並進行採樣（收集 10 幀以過濾噪聲）...")
                    
                    # 等待採樣完成或超時 (3秒)
                    start_time = time.time()
                    while len(self.calibration_samples) < 10 and (time.time() - start_time < 3.0) and rclpy.ok():
                        time.sleep(0.05)
                        
                    self.is_calibrating = False
                    
                    if len(self.calibration_samples) >= 10:
                        avg_offset = sum(self.calibration_samples) / len(self.calibration_samples)
                        print(f"\n{COLOR_GREEN}🎉 數據採樣完成！{COLOR_RESET}")
                        print(f"👉 偵測到真實偏角修正值 (lidar_offset_deg): {avg_offset:.2f}°")
                        
                        success = self.update_lidar_offset_files(avg_offset)
                        if success:
                            self.lidar_offset_deg = avg_offset  # 即時套用
                            print(f"{COLOR_GREEN}✅ 偏角已成功寫入硬碟儲存，並已即時套用！{COLOR_RESET}")
                        else:
                            print(f"{COLOR_RED}❌ 部分檔案寫入失敗，請確認檔案是否存在。{COLOR_RESET}")
                    else:
                        print(f"{COLOR_RED}❌ 採樣逾時或失敗，未收到足夠的雷達數據，請檢查雷達節點。{COLOR_RESET}")
                time.sleep(2.0)
                
            elif choice == '2':
                self.run_realtime_monitor()
                
            elif choice == '3':
                try:
                    dist_input = input(f"\n📏 請輸入後面牆壁到橋中心的距離 (公分，預設 200): ").strip()
                    wall_to_bridge_cm = float(dist_input) if dist_input else 200.0
                    target_wall_to_bridge_m = wall_to_bridge_cm / 100.0
                    self.run_movement_test('go', target_wall_to_bridge_m)
                except ValueError:
                    print(f"\n{COLOR_RED}❌ 輸入錯誤，距離必須為數字！{COLOR_RESET}")
                    time.sleep(1.5)
                
            elif choice == '4':
                self.run_movement_test('turn_right')
                
            elif choice == '5':
                self.run_movement_test('turn_left')
                
            elif choice == '6':
                try:
                    angle_input = input(f"\n🔄 請輸入要右轉的角度 (度，預設 90.0): ").strip()
                    target_angle = float(angle_input) if angle_input else 90.0
                    self.run_movement_test('turn_right_odom', target_angle)
                except ValueError:
                    print(f"\n{COLOR_RED}❌ 輸入錯誤，角度必須為數字！{COLOR_RESET}")
                    time.sleep(1.5)
                
            elif choice == '7':
                try:
                    angle_input = input(f"\n🔄 請輸入要左轉的角度 (度，預設 90.0): ").strip()
                    target_angle = float(angle_input) if angle_input else 90.0
                    self.run_movement_test('turn_left_odom', target_angle)
                except ValueError:
                    print(f"\n{COLOR_RED}❌ 輸入錯誤，角度必須為數字！{COLOR_RESET}")
                    time.sleep(1.5)
                
            elif choice == '8':
                self.run_movement_test('back')
                
            elif choice == '9':
                self.run_movement_test('align_wall')
                
            elif choice == '10':
                try:
                    dist_input = input(f"\n📲 請輸入上橋與橋頂里程前進距離 (公分，預設 130): ").strip()
                    dist_cm = float(dist_input) if dist_input else 130.0
                    
                    speed_input = input(f"   速度 (m/s，預設 0.12): ").strip()
                    speed_val = float(speed_input) if speed_input else 0.12
                    speed_val = abs(speed_val)  # 方向由距離正負決定
                    
                    factor_input = input(f"   打滑補償係數 (預設 1.0): ").strip()
                    factor_val = float(factor_input) if factor_input else 1.0
                    
                    target_m = (dist_cm / 100.0) * factor_val
                    self.run_movement_test('move_odom', target_m, speed_val, 1 if dist_cm >= 0 else -1)
                except ValueError:
                    print(f"\n{COLOR_RED}❌ 輸入錯誤，所有參數必須為數字！{COLOR_RESET}")
                    time.sleep(1.5)
                
            elif choice == '11':
                try:
                    dist_input = input(f"\n📲 請輸入要移動的距離 (公分，正為前進，負為後退，預設 50.0): ").strip()
                    dist_cm = float(dist_input) if dist_input else 50.0
                    
                    speed_input = input(f"   速度 (m/s，預設 0.05): ").strip()
                    speed_val = float(speed_input) if speed_input else 0.05
                    speed_val = abs(speed_val)  # 方向由距離正負決定
                    
                    factor_input = input(f"   打滑補償係數 (預設 1.0): ").strip()
                    factor_val = float(factor_input) if factor_input else 1.0
                    
                    target_m = (abs(dist_cm) / 100.0) * factor_val
                    self.run_movement_test('move_distance', target_m, speed_val, 1 if dist_cm >= 0 else -1)
                except ValueError:
                    print(f"\n{COLOR_RED}❌ 輸入錯誤，所有參數必須為數字！{COLOR_RESET}")
                    time.sleep(1.5)
                
            elif choice == '12':
                print(f"\n👋 {COLOR_BOLD}正在退出，感謝使用！{COLOR_RESET}")
                break
                
            else:
                print(f"{COLOR_RED}❌ 無效選項，請重新選擇。{COLOR_RESET}")
                time.sleep(0.8)

def main():
    rclpy.init()
    node = TestLidarOrientation()
    
    # 支援透過命令列參數直接指定偏角，如: python3 test_lidar_orientation.py 20.0
    # 或透過 --auto-go 執行自動前進測試
    if len(sys.argv) > 1:
        if sys.argv[1] == '--auto-go':
            try:
                dist_cm = float(sys.argv[2])
                spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
                spin_thread.start()
                
                print("⏳ 正在等待雷達數據同步...")
                for _ in range(30):
                    if node.front_dist is not None:
                        break
                    time.sleep(0.1)
                    
                node.run_movement_test('go', dist_cm / 100.0)
                node.stop_chassis()
                # 為了避免 ROS 2 C++ 底層在多執行緒下 shutdown() 時引發 terminate called without an active exception (錯誤碼 -6)
                # 這裡強制關閉進程，因為我們已經送出煞車指令了
                import os
                os._exit(0)
            except Exception as e:
                print(f"❌ 執行失敗: {e}")
                import os
                os._exit(1)
        else:
            try:
                node.lidar_offset_deg = float(sys.argv[1])
                node.get_logger().info(f"⚙️ 從命令列參數載入 lidar_offset_deg: {node.lidar_offset_deg}°")
            except ValueError:
                node.get_logger().error("⚠️ 偏角參數必須為數字，將採用預設值。")
    
    # 建立背景執行緒來處理 ROS2 的 spin (接收話題與發布)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        node.menu_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_chassis()

if __name__ == '__main__':
    main()
