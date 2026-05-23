import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import sys
import termios
import tty
import select
import math
import json
import os
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_srvs.srv import Empty

class NavTeach(Node):
    def __init__(self):
        super().__init__('nav_teach')
        
        # 設定儲存點位的 JSON 檔案路徑 (與此腳本同目錄下)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.pose_file = os.path.join(script_dir, "nav_poses.json")
        self.saved_poses = self.load_poses()
        
        self.recall_mode = False # 是否處於回放/導航模式
        self.current_goal_handle = None
        
        # 初始化 TF 監聽器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 初始化 Nav2 導航 Action 客戶端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 初始化底盤控制速度 Publisher (用於緊急煞車/停止)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/base_controller/cmd_vel',
            10
        )
        
        # 初始化 Costmap 清除服務客戶端
        self.clear_local_costmap_client = self.create_client(Empty, '/local_costmap/clear_entirely_local_costmap')
        self.clear_global_costmap_client = self.create_client(Empty, '/global_costmap/clear_entirely_global_costmap')
        
        # 顯示啟動資訊
        self.get_logger().info("==========================================")
        self.get_logger().info("🤖 Nav2 座標教導/回放工具 - 已啟動")
        self.get_logger().info(f"儲存檔案: {self.pose_file}")
        self.get_logger().info("鍵盤操作：")
        self.get_logger().info("  0 - 9 : [紀錄/移動] 依當前模式儲存或導航至點位")
        self.get_logger().info("  R     : [切換] 進入/退出回放模式 (自動導航)")
        self.get_logger().info("  I     : 印出當前實時位置 (TF map -> base_link)")
        self.get_logger().info("  C     : 清除 Nav2 Costmaps")
        self.get_logger().info("  S     : 停止導航與緊急煞車")
        self.get_logger().info("  X     : 退出程式")
        self.get_logger().info("==========================================")

    def load_poses(self):
        """從 JSON 檔案載入點位。"""
        if os.path.exists(self.pose_file):
            try:
                with open(self.pose_file, 'r') as f:
                    poses = json.load(f)
                self.get_logger().info(f"📂 已從檔案載入 {len(poses)} 個導航點位")
                return poses
            except Exception as e:
                self.get_logger().error(f"❌ 載入導航點位失敗: {e}")
        return {}

    def yaw_from_quaternion(self, x, y, z, w):
        """將四元數轉換為偏航角 (Yaw)"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def save_pose(self, slot):
        """獲取目前小車位置並儲存至指定插槽。"""
        try:
            # 查詢目前的 map -> base_link 變換
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            yaw = self.yaw_from_quaternion(qx, qy, qz, qw)
            yaw_deg = math.degrees(yaw)
            
            # 儲存座標與四元數
            self.saved_poses[str(slot)] = {
                'x': tx,
                'y': ty,
                'qx': qx,
                'qy': qy,
                'qz': qz,
                'qw': qw,
                'yaw': yaw
            }
            
            with open(self.pose_file, 'w') as f:
                json.dump(self.saved_poses, f, indent=4)
                
            print(f"\n✅ [已紀錄導航點位 {slot}]")
            print(f"   座標: X: {tx:.3f} m, Y: {ty:.3f} m")
            print(f"   朝向: Yaw: {yaw:.3f} rad ({yaw_deg:.1f}°)")
            print(f"   四元數: [x: {qx:.4f}, y: {qy:.4f}, z: {qz:.4f}, w: {qw:.4f}]")
            print(f"💾 檔案已更新: {self.pose_file}")
        except Exception as e:
            self.get_logger().error(f"❌ 獲取目前座標或儲存點位失敗: {e}")

    def move_to_pose(self, slot):
        """導航小車到指定的點位。"""
        slot_str = str(slot)
        if slot_str not in self.saved_poses:
            print(f"⚠️  導航點位 {slot} 尚未紀錄，無法移動")
            return
            
        pose_data = self.saved_poses[slot_str]
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pose_data['x']
        pose.pose.position.y = pose_data['y']
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = pose_data['qx']
        pose.pose.orientation.y = pose_data['qy']
        pose.pose.orientation.z = pose_data['qz']
        pose.pose.orientation.w = pose_data['qw']
        
        print(f"\n🚀 正在導航至點位 {slot}: X={pose_data['x']:.3f}, Y={pose_data['y']:.3f}...")
        self.send_nav_goal(pose)

    def send_nav_goal(self, pose):
        """向 Nav2 行動伺服器發送導航目標"""
        if self.current_goal_handle is not None:
            print("🛑 偵測到正在執行的導航任務，正在取消舊任務...")
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warning(f"取消舊導航目標失敗: {e}")
            self.current_goal_handle = None

        print("📡 正在等待 Nav2 導航伺服器...")
        self.nav_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.nav_goal_response_cb)

    def nav_goal_response_cb(self, future):
        """處理導航請求的響應"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("❌ 導航目標被 Nav2 拒絕")
            return
        print("✅ 導航目標已接受，小車開始移動...")
        self.current_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_result_cb)

    def nav_result_cb(self, future):
        """處理導航完成後的結果"""
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print("\n🎉 小車已順利抵達導航點！")
        elif status == GoalStatus.STATUS_CANCELED:
            print("\n🛑 導航已被取消")
        else:
            print(f"\n❌ 導航結束，狀態碼：{status}")
        self.current_goal_handle = None

    def stop_navigation(self):
        """停止導航並發送零速度指令強制底盤剎車"""
        print("\n🛑 正在停止導航並煞車...")
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warning(f"取消導航目標時發生異常: {e}")
            self.current_goal_handle = None
            
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        print("✅ 已發送零速度指令")

    def clear_costmaps(self):
        """呼叫 Nav2 服務清除 local 與 global costmap"""
        print("\n🧹 正在清除 Nav2 Costmaps 中的障礙物...")
        
        if self.clear_local_costmap_client.service_is_ready():
            req = Empty.Request()
            self.clear_local_costmap_client.call_async(req)
            print("   -> 已呼叫清除 local costmap 服務")
        else:
            print("   -> ⚠️ 無法連線至 local costmap 清除服務")
            
        if self.clear_global_costmap_client.service_is_ready():
            req = Empty.Request()
            self.clear_global_costmap_client.call_async(req)
            print("   -> 已呼叫清除 global costmap 服務")
        else:
            print("   -> ⚠️ 無法連線至 global costmap 清除服務")

    def print_status(self):
        """印出當前實時位置。"""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            yaw = self.yaw_from_quaternion(qx, qy, qz, qw)
            yaw_deg = math.degrees(yaw)
            
            print("\n📍 [小車目前實時座標]")
            print(f"   X: {tx:.3f} m, Y: {ty:.3f} m")
            print(f"   朝向: Yaw: {yaw:.3f} rad ({yaw_deg:.1f}°)")
            print(f"   四元數: [x: {qx:.4f}, y: {qy:.4f}, z: {qz:.4f}, w: {qw:.4f}]")
        except Exception as e:
            print(f"\n⏳ 正在等待 TF 訊號 (map -> base_link)... ({e})")

def main():
    rclpy.init()
    node = NavTeach()
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            # 處理 ROS 2 回呼 (如 TF 與 Action)
            rclpy.spin_once(node, timeout_sec=0.05)
            
            # 檢查鍵盤輸入
            if select.select([sys.stdin], [], [], 0)[0]:
                char = sys.stdin.read(1).lower()
                
                if char.isdigit():
                    if node.recall_mode:
                        node.move_to_pose(char)
                    else:
                        node.save_pose(char)
                elif char == 'r':
                    node.recall_mode = not node.recall_mode
                    status = "【回放模式 (按數字自動導航)】" if node.recall_mode else "【紀錄模式 (按數字儲存當前點)】"
                    print(f"\n🔄 模式切換: {status}")
                elif char == 'i':
                    node.print_status()
                elif char == 'c':
                    node.clear_costmaps()
                elif char == 's':
                    node.stop_navigation()
                elif char == 'x':
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
