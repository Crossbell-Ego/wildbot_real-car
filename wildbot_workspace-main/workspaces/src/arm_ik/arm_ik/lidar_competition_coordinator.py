#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
import math
import os
import sys
import subprocess
import threading

from yolo_msgs.msg import DetectionArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class LidarCompetitionCoordinator(Node):
    def __init__(self, target_class='bear', go_dist=0.50, approach_dist=0.38, slip_factor=0.9615, lidar_offset_deg=8.6):
        super().__init__('lidar_competition_coordinator')
        
        # 1. 任務設定與狀態初始化
        self.target_class = target_class
        self.go_dist = go_dist
        self.approach_dist = approach_dist
        self.slip_factor = slip_factor
        self.lidar_offset_deg = lidar_offset_deg  # 光達安裝偏角修正 (預設 20.0度)
        
        # 狀態機
        self.state = 'IDLE'
        self.detect_consecutive_count = 0
        
        # 2. 起點光達定位目標值 (若在起點出發則動態鎖定，若在起點外則使用預設常數)
        self.target_rear_dist = 0.25  # 預設起點後牆距離 (公尺)
        self.target_left_dist = 0.35  # 預設起點左牆距離 (公尺)
        self.is_started_at_home = False # 是否在起點角落出發
        
        # 實時雷達數據 (已扣除 lidar_offset_deg 修正)
        self.current_rear_dist = None
        self.current_rear_angle = None
        self.current_left_dist = None
        self.current_left_angle = None
        
        # 實時里程計數據
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        
        # Odom 控制起點記錄
        self.odom_start_x = None
        self.odom_start_y = None
        self.odom_start_yaw = None
        
        # 3. 速度控制發布器 (底盤)
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/base_controller/cmd_vel',
            10
        )
        
        # 4. 訂閱 YOLO 3D
        self.detections_sub = self.create_subscription(
            DetectionArray,
            '/yolo/detections_3d',
            self.detections_callback,
            10
        )
        
        # 5. 訂閱雷達 /scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # 6. 訂閱里程計
        self.odom_sub = self.create_subscription(
            Odometry,
            '/base_controller/odom',
            self.odom_callback,
            10
        )
        
        # 控制迴圈 (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # 7. 啟動初始化與直行旋轉序列
        threading.Thread(target=self.startup_sequence, daemon=True).start()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 Wildbot 光達+里程計任務協調器 (左右方向修正版) 已啟動！")
        self.get_logger().info(f"🎯 鎖定目標物：{self.target_class}")
        self.get_logger().info(f"📏 去程前進距離：{self.go_dist} 公尺")
        self.get_logger().info(f"🔧 光達安裝校正偏角：{self.lidar_offset_deg}°")
        self.get_logger().info("⏳ 正在執行啟動序列：手臂初始化...")
        self.get_logger().info("=" * 60)

    def yaw_from_quaternion(self, q):
        """將四元數轉換為偏航角 (Yaw)"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)

    def get_closest_point(self, msg, min_deg, max_deg, window_size=5):
        """在指定角度範圍內尋找最近的反射點，並做平滑處理以過濾雜訊。"""
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
        # 1. 讀取車尾後牆 (搜尋範圍考慮安裝偏角，並同步平移)
        search_rear_min = -30.0 + self.lidar_offset_deg
        search_rear_max = 30.0 + self.lidar_offset_deg
        angle_r, dist_r = self.get_closest_point(msg, search_rear_min, search_rear_max)
        if dist_r is not None:
            self.current_rear_dist = dist_r
            self.current_rear_angle = math.degrees(angle_r) - self.lidar_offset_deg
            
        # 2. 讀取左側牆壁 (💡 已修正：物理左側在雷達中是負角度區間，搜尋區間變為 [-120, -60] 度)
        search_left_min = -120.0 + self.lidar_offset_deg
        search_left_max = -60.0 + self.lidar_offset_deg
        angle_l, dist_l = self.get_closest_point(msg, search_left_min, search_left_max)
        if dist_l is not None:
            self.current_left_dist = dist_l
            self.current_left_angle = math.degrees(angle_l) - self.lidar_offset_deg

    def startup_sequence(self):
        """啟動序列：先初始化手臂，再進行起點校準與出發判定"""
        self.state = 'ARM_INIT'
        script_dir = os.path.dirname(os.path.abspath(__file__))
        arm_script = os.path.join(script_dir, "grab_execute.py")
        
        self.get_logger().info("🦾 [啟動序列 1/3] 正在初始化手臂至點位 1...")
        try:
            result = subprocess.run(
                ["python3", arm_script, "1"],
                stdout=sys.stdout,
                stderr=sys.stderr,
                text=True,
                check=False
            )
            if result.returncode == 0:
                self.get_logger().info("✅ 手臂已成功初始化至點位 1")
            else:
                self.get_logger().warning(f"⚠️ 手臂初始化腳本異常 (代碼: {result.returncode})，仍繼續任務")
        except Exception as e:
            self.get_logger().error(f"❌ 呼叫 grab_execute.py 失敗: {e}，仍繼續任務")

        # 偵測是否在起點 (等待最多 2.5 秒)
        self.get_logger().info("⏳ 正在檢測小車起點狀態 (最多等待 2.5 秒)...")
        start_wait = time.time()
        rear_samples = []
        left_samples = []
        
        while time.time() - start_wait < 2.5:
            if self.current_rear_dist is not None:
                rear_samples.append(self.current_rear_dist)
            if self.current_left_dist is not None:
                left_samples.append(self.current_left_dist)
            time.sleep(0.1)
            
        avg_rear = sum(rear_samples) / len(rear_samples) if rear_samples else 999.0
        avg_left = sum(left_samples) / len(left_samples) if left_samples else 999.0
        
        if avg_rear < 0.85 and avg_left < 0.85:
            self.target_rear_dist = avg_rear
            self.target_left_dist = avg_left
            self.is_started_at_home = True
            self.get_logger().info(f"✅ [檢測結果] 小車處於起點 L 角落！")
            self.get_logger().info(f"📍 鎖定起點校準值 -> 後牆距: {self.target_rear_dist*100:.1f} cm | 左牆距: {self.target_left_dist*100:.1f} cm")
        else:
            self.is_started_at_home = False
            self.get_logger().warn(f"⚠️ [檢測結果] 小車【未在起點】！(雷達值 -> 後: {avg_rear*100:.1f} cm, 左: {avg_left*100:.1f} cm)")
            self.get_logger().warn(f"⚠️ 將自動切換為里程計 (Odom) 輔助控制，並採用預設起點基準值 (後牆: {self.target_rear_dist*100:.1f} cm, 左牆: {self.target_left_dist*100:.1f} cm)")
            
        self.state = 'GO_STRAIGHT'
        self.get_logger().info(f"🚗 [啟動序列 2/3] 啟動直行，目標距離: {self.go_dist * 100:.1f} cm")

    def send_stop_command(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        for _ in range(5):
            self.cmd_pub.publish(msg)
            time.sleep(0.02)

    def detections_callback(self, msg):
        if self.state != 'WAIT_YOLO':
            return
            
        valid_targets = []
        for det in msg.detections:
            if det.class_name == self.target_class and det.score >= 0.65:
                pos = det.bbox3d.center.position
                if pos.x > 0:
                    valid_targets.append(det)
                    
        if not valid_targets:
            if self.detect_consecutive_count > 0:
                self.detect_consecutive_count -= 1
            return
            
        self.detect_consecutive_count += 1
        if self.detect_consecutive_count > 10:
            self.detect_consecutive_count = 10
            
        if self.detect_consecutive_count < 3:
            self.get_logger().info(
                f"⏳ 偵測到目標 '{self.target_class}'，累積確認中 ({self.detect_consecutive_count}/3)...",
                throttle_duration_sec=0.5
            )
            return
            
        closest_det = min(
            valid_targets,
            key=lambda d: d.bbox3d.center.position.x**2 + d.bbox3d.center.position.y**2
        )
        
        pos = closest_det.bbox3d.center.position
        self.get_logger().info(f"🎯 鎖定最近的目標 '{self.target_class}' (累積確認成功！信心度: {closest_det.score:.2f})")
        self.get_logger().info(f"📍 目標位置: X={pos.x:.3f}m, Y={pos.y:.3f}m, Z={pos.z:.3f}m")
        
        self.detect_consecutive_count = 10
        self.trigger_grab_sequence()

    def trigger_grab_sequence(self):
        self.state = 'GRABBING'
        threading.Thread(target=self._grab_thread, daemon=True).start()

    def _grab_thread(self):
        self.get_logger().info("🦾 啟動精準對齊與夾取序列...")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        script_path = os.path.join(script_dir, "camera and grab.py")
        
        try:
            result = subprocess.run(
                ["python3", script_path, str(self.slip_factor)],
                stdout=sys.stdout,
                stderr=sys.stderr,
                text=True,
                check=False
            )
            
            if result.returncode == 0:
                self.get_logger().info("✅ 夾取成功，準備啟動回航。")
                self.return_home()
            else:
                self.get_logger().error(f"❌ 夾取失敗 (代碼: {result.returncode})，重新執行去程序列...")
                self.detect_consecutive_count = 0
                threading.Thread(target=self.startup_sequence, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"❌ 執行夾取腳本出錯: {e}")
            self.detect_consecutive_count = 0
            threading.Thread(target=self.startup_sequence, daemon=True).start()

    def return_home(self):
        self.state = 'TURN_LEFT_90'
        self.odom_start_yaw = self.current_yaw
        self.get_logger().info("🏠 啟動回航！第一步：原地左轉 90 度回正...")

    def trigger_place_sequence(self):
        self.state = 'PLACING'
        threading.Thread(target=self._place_thread, daemon=True).start()

    def _place_thread(self):
        self.get_logger().info("🦾 啟動放物序列：手臂移至點位 1...")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        arm_script = os.path.join(script_dir, "grab_execute.py")

        try:
            result = subprocess.run(
                ["python3", arm_script, "1"],
                stdout=sys.stdout,
                stderr=sys.stderr,
                text=True,
                check=False
            )
            if result.returncode == 0:
                self.get_logger().info("🎉 放物成功！任務全數完成。")
            else:
                self.get_logger().error(f"❌ 放物腳本異常終止，結束代碼：{result.returncode}")
        except Exception as e:
            self.get_logger().error(f"❌ 執行放物腳本失敗: {e}")

        self.state = 'DONE'
        self.send_stop_command()
        time.sleep(1.0)
        rclpy.shutdown()

    def control_loop(self):
        if self.state in ['IDLE', 'ARM_INIT', 'WAIT_YOLO', 'GRABBING', 'PLACING', 'DONE']:
            return
            
        if self.current_x is None or self.current_yaw is None:
            return
            
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 控制常數
        speed_go = 0.12
        speed_turn = 0.35
        kp_yaw = 1.0
        kp_y_align = 45.0
        max_yaw_offset = 12.0

        # ----------------- 狀態: GO_STRAIGHT (直行前進 50cm) -----------------
        if self.state == 'GO_STRAIGHT':
            if self.odom_start_x is None:
                self.odom_start_x = self.current_x
                self.odom_start_y = self.current_y
                self.odom_start_yaw = self.current_yaw
                
            dx = self.current_x - self.odom_start_x
            dy = self.current_y - self.odom_start_y
            dist_traveled = math.hypot(dx, dy)
            
            sensor_info = ""
            if self.is_started_at_home and self.current_rear_dist is not None:
                dist_traveled_sensor = self.current_rear_dist - self.target_rear_dist
                sensor_info = f" | 光達量測: {dist_traveled_sensor*100:.1f} cm"
                
            self.get_logger().info(
                f"🚗 [前進] Odom已行駛: {dist_traveled*100:.1f} / {self.go_dist*100:.1f} cm{sensor_info} | 車尾角度: {self.current_rear_angle:.1f}°",
                throttle_duration_sec=0.2
            )
            
            finished = False
            if self.is_started_at_home and self.current_rear_dist is not None:
                finished = (self.current_rear_dist - self.target_rear_dist) >= self.go_dist
            else:
                finished = dist_traveled >= self.go_dist
                
            if finished:
                self.send_stop_command()
                self.get_logger().info(f"🎉 抵達直行目標！準備右轉 90 度...")
                self.odom_start_yaw = self.current_yaw
                self.state = 'TURN_RIGHT_90'
            else:
                msg.twist.linear.x = speed_go
                if self.is_started_at_home and self.current_left_dist is not None and self.current_rear_angle is not None:
                    target_yaw_deg = kp_y_align * (self.current_left_dist - self.target_left_dist)
                    target_yaw_deg = np.clip(target_yaw_deg, -max_yaw_offset, max_yaw_offset)
                    error_angle_deg = self.current_rear_angle - target_yaw_deg
                    msg.twist.angular.z = kp_yaw * math.radians(error_angle_deg)
                else:
                    msg.twist.angular.z = 0.0
                self.cmd_pub.publish(msg)

        # ----------------- 狀態: TURN_RIGHT_90 (右轉 90 度) -----------------
        elif self.state == 'TURN_RIGHT_90':
            yaw_diff = abs(self.current_yaw - self.odom_start_yaw)
            if yaw_diff > math.pi:
                yaw_diff = 2.0 * math.pi - yaw_diff
                
            self.get_logger().info(
                f"🔄 [右轉] Odom轉向: {math.degrees(yaw_diff):.1f}° / 90.0°",
                throttle_duration_sec=0.2
            )
            
            finished = False
            if self.is_started_at_home and self.current_left_dist is not None and self.current_left_angle is not None:
                dist_diff = abs(self.current_left_dist - self.target_rear_dist)
                # 💡 已修正：物理左側在雷達中是負角度區間，對齊目標值為 -90.0
                if abs(self.current_left_angle + 90.0) <= 2.0 and dist_diff < 0.25:
                    finished = True
            
            if not finished and yaw_diff >= 1.50:
                finished = True
                
            if finished:
                self.send_stop_command()
                self.get_logger().info(f"🎉 右轉 90 度完成！切換為 WAIT_YOLO 等待目標偵測。")
                self.state = 'WAIT_YOLO'
            else:
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = - speed_turn
                self.cmd_pub.publish(msg)

        # ----------------- 狀態: TURN_LEFT_90 (左轉 90 度回正) -----------------
        elif self.state == 'TURN_LEFT_90':
            yaw_diff = abs(self.current_yaw - self.odom_start_yaw)
            if yaw_diff > math.pi:
                yaw_diff = 2.0 * math.pi - yaw_diff
                
            self.get_logger().info(
                f"🔄 [左轉] Odom轉向: {math.degrees(yaw_diff):.1f}° / 90.0°",
                throttle_duration_sec=0.2
            )
            
            finished = False
            if self.current_rear_dist is not None and self.current_rear_angle is not None:
                dist_diff = abs(self.current_rear_dist - self.target_rear_dist)
                if abs(self.current_rear_angle) <= 2.0 and dist_diff < 0.25:
                    finished = True
                    
            if not finished and yaw_diff >= 1.50:
                finished = True
                
            if finished:
                self.send_stop_command()
                self.get_logger().info(f"🎉 左轉回正完成！準備啟動倒車回航...")
                self.odom_start_x = self.current_x
                self.odom_start_y = self.current_y
                self.state = 'GO_BACK'
            else:
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = speed_turn
                self.cmd_pub.publish(msg)

        # ----------------- 狀態: GO_BACK (雙閉環倒車回航) -----------------
        elif self.state == 'GO_BACK':
            dx = self.current_x - self.odom_start_x
            dy = self.current_y - self.odom_start_y
            odom_dist_back = math.hypot(dx, dy)
            
            is_lidar_active = (self.current_rear_dist is not None and self.current_rear_dist < 0.85 and 
                               self.current_left_dist is not None and self.current_left_dist < 0.85)
            
            if is_lidar_active:
                dist_error = self.current_rear_dist - self.target_rear_dist
                self.get_logger().info(
                    f"🚗 [倒車-光達鎖定] 距起點剩餘: {dist_error*100:.1f} cm | 左牆距: {self.current_left_dist*100:.1f} cm | 車尾修正角: {self.current_rear_angle:.1f}°",
                    throttle_duration_sec=0.2
                )
                
                if dist_error <= 0.01:
                    self.send_stop_command()
                    self.get_logger().info(f"🎉 [光達] 精準抵達起點！後牆距: {self.current_rear_dist*100:.1f} cm | 左牆距: {self.current_left_dist*100:.1f} cm")
                    self.trigger_place_sequence()
                else:
                    target_yaw_deg = - kp_y_align * (self.current_left_dist - self.target_left_dist)
                    target_yaw_deg = np.clip(target_yaw_deg, -max_yaw_offset, max_yaw_offset)
                    error_angle_deg = self.current_rear_angle - target_yaw_deg
                    
                    msg.twist.linear.x = - speed_go
                    msg.twist.angular.z = kp_yaw * math.radians(error_angle_deg)
                    self.cmd_pub.publish(msg)
            else:
                self.get_logger().info(
                    f"🚗 [倒車-Odom盲退] 已倒退: {odom_dist_back*100:.1f} cm (搜尋起點牆壁中...)",
                    throttle_duration_sec=0.2
                )
                
                if odom_dist_back >= (self.go_dist * 1.2):
                    self.send_stop_command()
                    self.get_logger().error(f"❌ 倒退安全超限但未尋獲後牆！強制停下，準備放物。")
                    self.trigger_place_sequence()
                else:
                    msg.twist.linear.x = - speed_go
                    msg.twist.angular.z = 0.0
                    self.cmd_pub.publish(msg)

def main():
    go_dist = 0.50
    approach_dist = 0.38
    slip_factor = 0.9615
    lidar_offset_deg = 8.6
    
    if len(sys.argv) > 1:
        try:
            go_dist = float(sys.argv[1])
        except ValueError:
            pass
    if len(sys.argv) > 2:
        try:
            approach_dist = float(sys.argv[2])
        except ValueError:
            pass
    if len(sys.argv) > 3:
        try:
            slip_factor = float(sys.argv[3])
        except ValueError:
            pass
    if len(sys.argv) > 4:
        try:
            lidar_offset_deg = float(sys.argv[4])
        except ValueError:
            pass
            
    rclpy.init()
    node = LidarCompetitionCoordinator('bear', go_dist, approach_dist, slip_factor, lidar_offset_deg)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("🛑 偵測到 Ctrl+C 中斷，立即停下小車！")
        node.send_stop_command()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
