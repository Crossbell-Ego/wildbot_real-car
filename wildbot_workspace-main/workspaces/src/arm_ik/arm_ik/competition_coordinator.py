#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import time
import math
import os
import sys
import subprocess
import threading

from yolo_msgs.msg import DetectionArray
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_srvs.srv import Empty

class CompetitionCoordinator(Node):
    def __init__(self, target_class='black_bear', approach_dist=0.35, slip_factor=1.0):
        super().__init__('competition_coordinator')
        
        # 1. 任務設定與狀態初始化
        self.target_class = target_class
        self.approach_dist = approach_dist
        self.slip_factor = slip_factor
        self.state = 'IDLE' # 狀態: IDLE, NAVIGATING_TO_WAYPOINT, NAVIGATING_TO_TARGET, GRABBING, RETURNING, DONE
        self.current_goal_handle = None
        self.detect_consecutive_count = 0
        
        # 2. TF 監聽器初始化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 3. ROS Action 與 Topic 初始化
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/base_controller/cmd_vel',
            10
        )
        
        self.detections_sub = self.create_subscription(
            DetectionArray,
            '/yolo/detections_3d',
            self.detections_callback,
            10
        )
        
        # 4. 建立服務客戶端來清除 costmap
        self.clear_local_costmap_client = self.create_client(Empty, '/local_costmap/clear_entirely_local_costmap')
        self.clear_global_costmap_client = self.create_client(Empty, '/global_costmap/clear_entirely_global_costmap')
        
        # 4. 固定起始位置作為回航點 (Home Pose) - 以地圖原點為預設值
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_qx = 0.0
        self.home_qy = 0.0
        self.home_qz = 0.0
        self.home_qw = 1.0

        # 5. 啟動初始化序列：手臂歸位 → 導航至巡邏點
        threading.Thread(target=self.startup_sequence, daemon=True).start()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 Wildbot 競賽任務協調器已啟動！")
        self.get_logger().info(f"🎯 鎖定目標物：{self.target_class}")
        self.get_logger().info(f"📏 導航停留距離：{self.approach_dist} 公尺")
        self.get_logger().info(f"🚗 夾取打滑補償：{self.slip_factor}")
        self.get_logger().info("⏳ 正在執行啟動序列：手臂初始化 → 導航至巡邏點...")
        self.get_logger().info("=" * 60)

    def startup_sequence(self):
        """啟動序列：先將手臂初始化至點位 1，再導航至 Nav2 點位 1"""
        script_dir = os.path.dirname(os.path.abspath(__file__))

        # --- Step 1: 手臂初始化到點位 1 ---
        self.get_logger().info("🦾 [啟動序列 1/2] 正在初始化手臂至點位 1...")
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
                self.get_logger().info("✅ 手臂已成功初始化至點位 1")
            else:
                self.get_logger().warning(f"⚠️ 手臂初始化腳本異常 (代碼: {result.returncode})，仍繼續執行導航")
        except Exception as e:
            self.get_logger().error(f"❌ 呼叫 grab_execute.py 失敗: {e}，仍繼續執行導航")

        # --- Step 2: 導航至 Nav2 點位 1 ---
        self.get_logger().info("🚀 [啟動序列 2/2] 正在導航至 Nav2 點位 1...")
        self.state = 'NAVIGATING_TO_WAYPOINT'
        nav_script = os.path.join(script_dir, "nav_execute.py")
        try:
            result = subprocess.run(
                ["python3", nav_script, "1"],
                stdout=sys.stdout,
                stderr=sys.stderr,
                text=True,
                check=False
            )
            if result.returncode == 0:
                self.get_logger().info("✅ 已抵達 Nav2 點位 1，切換為 IDLE 等待目標偵測")
            else:
                self.get_logger().warning(f"⚠️ 導航至點位 1 異常 (代碼: {result.returncode})")
        except Exception as e:
            self.get_logger().error(f"❌ 呼叫 nav_execute.py 失敗: {e}")
        finally:
            self.state = 'IDLE'

    def send_stop_command(self):
        """發送零速度指令強制底盤剎車"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def detections_callback(self, msg):
        """YOLO 3D 偵測回呼：當處於 IDLE 或巡邏中且偵測到目標物時觸發導航流程"""
        if self.state not in ['IDLE', 'NAVIGATING_TO_WAYPOINT']:
            return
            
        # 篩選符合條件的目標物 (標籤: 'bear')
        valid_targets = []
        for det in msg.detections:
            if det.class_name == self.target_class and det.score >= 0.65:
                pos = det.bbox3d.center.position
                if pos.x > 0:  # 確保在機器人前方
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
            
        # 選擇距離最近的目標 (以 X^2 + Y^2 距離平方作為依據)
        closest_det = min(
            valid_targets,
            key=lambda d: d.bbox3d.center.position.x**2 + d.bbox3d.center.position.y**2
        )
        
        pos = closest_det.bbox3d.center.position
        self.get_logger().info(f"🎯 鎖定最近的目標 '{self.target_class}' (累積確認成功！信心度: {closest_det.score:.2f})")
        self.get_logger().info(f"📍 目標相對於底盤位置: X={pos.x:.3f}m, Y={pos.y:.3f}m, Z={pos.z:.3f}m")
        
        # 若在前往巡邏點途中看到目標，主動搶占/取消當前導航
        if self.state == 'NAVIGATING_TO_WAYPOINT':
            self.get_logger().info("🛑 巡邏途中偵測到目標！正在主動取消巡邏點導航以進行搶占...")
            self.send_stop_command()
            if self.current_goal_handle is not None:
                try:
                    self.current_goal_handle.cancel_goal_async()
                except Exception as e:
                    self.get_logger().warning(f"取消導航目標時發生異常: {e}")
                self.current_goal_handle = None
                
        # 重置計數器至上限，防止 startup_sequence 結束後立即再次觸發
        self.detect_consecutive_count = 10
        # 直接觸發對齊與夾取序列，不再透過 Nav2 導航靠近目標
        self.trigger_grab_sequence()

    def compute_and_send_goal(self, target_pos):
        """根據目標物在 base_link 下的座標，計算 map 下的對齊停留點並發送給 Nav2"""
        x_obj = target_pos.x
        y_obj = target_pos.y
        
        # 1. 計算小車朝向目標物所需的偏航角 (Yaw)
        theta = math.atan2(y_obj, x_obj)
        
        # 2. 計算在 base_link 座標系下，小車欲停留位置 (物體前方 approach_dist 公尺)
        x_goal_local = x_obj - self.approach_dist * math.cos(theta)
        y_goal_local = y_obj - self.approach_dist * math.sin(theta)
        
        self.get_logger().info("⏳ 正在查詢 TF 樹進行座標變換...")
        
        # 最多等待 5 秒取得 TF 變換
        start_time = time.time()
        transform = None
        while time.time() - start_time < 5.0 and rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                break
            except Exception:
                time.sleep(0.1)
                
        if transform is None:
            self.get_logger().error("❌ 無法獲取 map -> base_link 的 TF 轉換，重置狀態為 IDLE")
            self.state = 'IDLE'
            return
            
        # 3. 提取目前小車在 map 中的位置與姿態
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        
        current_yaw = self.yaw_from_quaternion(qx, qy, qz, qw)
        
        # 4. 將 base_link 下的對齊停留點位置轉換至 map 座標系
        q = [qw, qx, qy, qz]  # [w, x, y, z]
        v = [x_goal_local, y_goal_local, 0.0]
        v_transformed = self.qv_mult(q, v)
        
        x_goal_map = v_transformed[0] + tx
        y_goal_map = v_transformed[1] + ty
        
        # 5. 計算小車在 map 座標系下應面向目標的 yaw 角度
        yaw_goal_map = current_yaw + theta
        qx_g, qy_g, qz_g, qw_g = self.quaternion_from_yaw(yaw_goal_map)
        
        # 6. 建構 PoseStamped 目標並發送給 Nav2
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x_goal_map
        pose.pose.position.y = y_goal_map
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx_g
        pose.pose.orientation.y = qy_g
        pose.pose.orientation.z = qz_g
        pose.pose.orientation.w = qw_g
        
        self.get_logger().info(f"📍 座標轉換完成！計算所得 Map 停留點為: X={x_goal_map:.3f}m, Y={y_goal_map:.3f}m, Yaw={yaw_goal_map:.2f} rad")
        self.send_nav_goal(pose)

    def send_nav_goal(self, pose):
        """向 Nav2 行動伺服器發送導航目標"""
        self.get_logger().info("📡 正在等待 Nav2 導航伺服器...")
        self.nav_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info("🚀 發送導航目標點！")
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.nav_goal_response_cb)

    def nav_goal_response_cb(self, future):
        """處理導航請求的響應"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"❌ 導航請求被拒絕，當前狀態為 {self.state}")
            if self.state in ['NAVIGATING_TO_WAYPOINT', 'NAVIGATING_TO_TARGET']:
                self.state = 'IDLE'
            return
            
        self.get_logger().info(f"✅ 導航請求已接受 (狀態: {self.state})，小車正在移動中...")
        self.current_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_result_cb)

    def nav_result_cb(self, future):
        """處理導航完成後的結果"""
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.state == 'NAVIGATING_TO_WAYPOINT':
                self.get_logger().info("📍 已抵達巡邏目標點，但尚未看見目標。切換至 IDLE 狀態等待偵測...")
                self.state = 'IDLE'
            elif self.state == 'NAVIGATING_TO_TARGET':
                self.get_logger().info("🎉 小車已順利抵達目標物前方！準備執行對齊夾取...")
                self.trigger_grab_sequence()
            elif self.state == 'RETURNING':
                self.get_logger().info("🏠 小車已返抵起點！準備執行放物動作...")
                self.trigger_place_sequence()
        else:
            # 如果是因為搶占而被主動取消，此時狀態已轉為 NAVIGATING_TO_TARGET，故不應重置為 IDLE
            if self.state in ['NAVIGATING_TO_WAYPOINT', 'RETURNING']:
                self.get_logger().error(f"❌ 導航未成功完成，狀態碼：{status}，重置狀態為 IDLE")
                self.state = 'IDLE'

    def trigger_place_sequence(self):
        """切換狀態並在獨立執行緒中執行放物動作，避免阻礙 ROS2 主迴圈"""
        self.state = 'PLACING'
        threading.Thread(target=self._place_thread, daemon=True).start()

    def _place_thread(self):
        self.get_logger().info("🦾 啟動放物序列：手臂移至點位 1（夾爪張開）...")
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
                self.get_logger().info("🎉 放物成功！夾爪已張開，任務全數完成。")
            else:
                self.get_logger().error(f"❌ 放物腳本異常終止，結束代碼：{result.returncode}")
        except Exception as e:
            self.get_logger().error(f"❌ 執行放物腳本失敗: {e}")

        self.state = 'DONE'
        
        self.get_logger().info("🛑 任務完成，正在清除所有導航與小車動作...")
        # 1. 強制發送零速度指令剎車
        self.send_stop_command()
        
        # 2. 取消當前可能仍在執行的 Nav2 導航目標
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warning(f"取消導航目標時發生異常: {e}")
            self.current_goal_handle = None
            
        time.sleep(1.0)
        rclpy.shutdown()

    def trigger_grab_sequence(self):
        """切換狀態並在獨立執行緒中調用 camera and grab.py，避免阻塞 ROS2 主迴圈"""
        self.state = 'GRABBING'
        threading.Thread(target=self._grab_thread, daemon=True).start()

    def _grab_thread(self):
        self.get_logger().info("⏳ 等待底盤完全靜止與 Nav2 取消目標...")
        time.sleep(1.0)
        self.get_logger().info("🦾 啟動精準底盤對齊與機械手臂夾取序列...")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        script_path = os.path.join(script_dir, "camera and grab.py")
        
        try:
            # 調用 camera and grab.py 並傳入補償參數
            result = subprocess.run(
                ["python3", script_path, str(self.slip_factor)],
                stdout=sys.stdout,
                stderr=sys.stderr,
                text=True,
                check=False
            )
            
            if result.returncode == 0:
                self.get_logger().info("✅ 夾取序列執行成功，等待手臂穩定後準備回航...")
                time.sleep(2.0)  # 確保手臂馬達已物理到位，再啟動底盤導航
                self.get_logger().info("🏠 手臂已穩定，開始回航。")
                self.return_home()
            else:
                self.get_logger().error(f"❌ 夾取腳本異常終止，結束代碼：{result.returncode}，重新執行啟動序列...")
                self.detect_consecutive_count = 0
                threading.Thread(target=self.startup_sequence, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"❌ 執行夾取腳本失敗: {e}")
            self.detect_consecutive_count = 0
            threading.Thread(target=self.startup_sequence, daemon=True).start()

    def clear_costmaps(self):
        """呼叫 Nav2 服務清除 local 與 global costmap 中的暫時性障礙物"""
        self.get_logger().info("🧹 正在呼叫服務清除 Nav2 Costmaps 中的暫時性障礙物...")
        
        # 呼叫 local costmap 清除服務
        if self.clear_local_costmap_client.service_is_ready():
            req = Empty.Request()
            self.clear_local_costmap_client.call_async(req)
            
        # 呼叫 global costmap 清除服務
        if self.clear_global_costmap_client.service_is_ready():
            req = Empty.Request()
            self.clear_global_costmap_client.call_async(req)
            
        time.sleep(0.5) # 稍微等待清除生效

    def return_home(self):
        """使用 nav_execute.py 導航回到 Nav2 點位 2（起點/放物區）"""
        self.state = 'RETURNING'
        self.get_logger().info("🏠 夾取成功！準備回航至 Nav2 點位 2...")
        threading.Thread(target=self._return_home_thread, daemon=True).start()

    def _return_home_thread(self):
        # 先清除 costmap，避免殘留障礙物阻礙回航路徑
        self.clear_costmaps()

        script_dir = os.path.dirname(os.path.abspath(__file__))
        nav_script = os.path.join(script_dir, "nav_execute.py")
        try:
            result = subprocess.run(
                ["python3", nav_script, "2"],
                stdout=sys.stdout,
                stderr=sys.stderr,
                text=True,
                check=False
            )
            if result.returncode == 0:
                self.get_logger().info("✅ 已抵達 Nav2 點位 2，準備執行放物動作...")
                self.trigger_place_sequence()
            else:
                self.get_logger().error(f"❌ 回航至點位 2 失敗 (代碼: {result.returncode})，重置為 IDLE")
                self.state = 'IDLE'
        except Exception as e:
            self.get_logger().error(f"❌ 呼叫 nav_execute.py 失敗: {e}")
            self.state = 'IDLE'

    # --- 空間幾何數學輔助函數 ---
    def yaw_from_quaternion(self, x, y, z, w):
        """將四元數轉換為偏航角 (Yaw)"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def quaternion_from_yaw(self, yaw):
        """將偏航角 (Yaw) 轉換為四元數"""
        return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def qv_mult(self, q, v):
        """使用四元數旋轉三維向量"""
        q = np.array(q, dtype=np.float64)
        v = np.array(v, dtype=np.float64)
        qvec = q[1:]
        uv = np.cross(qvec, v)
        uuv = np.cross(qvec, uv)
        return v + 2.0 * (uv * q[0] + uuv)

def main():
    rclpy.init()
    
    # 固定參數設定（目標標籤固定為 'bear'）
    approach_dist = 0.38  # 導航停留距離 (公尺)
    slip_factor = 1.0     # 夾取打滑補償
    
    # 支援命令列參數調整 approach_dist 與 slip_factor
    if len(sys.argv) > 1:
        try:
            approach_dist = float(sys.argv[1])
        except ValueError:
            pass
    if len(sys.argv) > 2:
        try:
            slip_factor = float(sys.argv[2])
        except ValueError:
            pass
            
    node = CompetitionCoordinator('bear', approach_dist, slip_factor)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 協調器被使用者中斷。")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
