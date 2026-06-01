import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import json
import os
import sys
import time
import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_srvs.srv import Empty

class NavExecutor(Node):
    def __init__(self):
        super().__init__('nav_executor')
        
        # 設定讀取點位的 JSON 檔案路徑 (與此腳本同目錄下)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.pose_file = os.path.join(script_dir, "nav_poses.json")
        self.saved_poses = self.load_poses()
        
        # 初始化 Nav2 導航 Action 客戶端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 初始化 Costmap 清除服務客戶端
        self.clear_local_costmap_client = self.create_client(Empty, '/local_costmap/clear_entirely_local_costmap')
        self.clear_global_costmap_client = self.create_client(Empty, '/global_costmap/clear_entirely_global_costmap')

    def load_poses(self):
        """載入 JSON 點位檔案。"""
        if os.path.exists(self.pose_file):
            try:
                with open(self.pose_file, 'r') as f:
                    return json.load(f)
            except Exception as e:
                self.get_logger().error(f"❌ 讀取 {self.pose_file} 失敗: {e}")
        return {}

    def clear_costmaps(self):
        """呼叫 Nav2 服務清除 local 與 global costmap"""
        self.get_logger().info("🧹 正在清除 Nav2 Costmaps 中的暫時性障礙物...")
        
        futures = []
        if self.clear_local_costmap_client.service_is_ready():
            req = Empty.Request()
            futures.append(self.clear_local_costmap_client.call_async(req))
            
        if self.clear_global_costmap_client.service_is_ready():
            req = Empty.Request()
            futures.append(self.clear_global_costmap_client.call_async(req))
            
        if futures:
            # 稍微等待 0.5 秒讓清除服務執行完畢
            start_wait = time.time()
            while time.time() - start_wait < 0.5:
                rclpy.spin_once(self, timeout_sec=0.05)
            self.get_logger().info("✅ Costmaps 已清除")

    def navigate_to_slot(self, slot_name):
        """導航到指定名稱的點位並等待完成。"""
        slot_str = str(slot_name)
        if slot_str not in self.saved_poses:
            self.get_logger().error(f"❌ 找不到導航點位: {slot_str}")
            return False
            
        pose_data = self.saved_poses[slot_str]
        
        # 建構 PoseStamped 目標
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
        
        self.get_logger().info(f"📡 正在等待 Nav2 導航伺服器...")
        self.nav_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info(f"🚀 正在導航至點位 [{slot_str}] (X={pose_data['x']:.3f}, Y={pose_data['y']:.3f})...")
        
        # 異步發送目標
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        
        # 等待目標響應 (非阻塞)
        while not send_goal_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"❌ 導航點位 [{slot_str}] 被 Nav2 拒絕")
            return False
            
        self.get_logger().info(f"✅ 導航請求已接受，小車開始移動...")
        
        # 取得結果 Future
        result_future = goal_handle.get_result_async()
        
        # 循環等待導航結果 (非阻塞)
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            
        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"🎉 順利抵達導航點位 [{slot_str}]！")
            return True
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error(f"🛑 前往 [{slot_str}] 的導航任務已被取消")
            return False
        else:
            self.get_logger().error(f"❌ 前往 [{slot_str}] 的導航任務失敗，狀態碼：{status}")
            return False

def main():
    rclpy.init()
    node = NavExecutor()
    success = True  # 追蹤所有導航是否成功
    
    # 檢查是否有命令列參數
    if len(sys.argv) > 1:
        # 支援多個參數按順序執行，例如: python3 nav_execute.py 1 2 1
        for slot in sys.argv[1:]:
            # 導航前先清理 costmaps 避開殘留動態障礙物
            node.clear_costmaps()
            if not node.navigate_to_slot(slot):
                print(f"❌ 導航序列在中途被中斷")
                success = False
                break
    else:
        # 沒有參數時列出所有可用導航點位
        print("\n" + "="*35)
        print("📋 可用導航點位清單：")
        if not node.saved_poses:
            print("  (尚未紀錄任何點位)")
        else:
            for key in sorted(node.saved_poses.keys()):
                pose = node.saved_poses[key]
                yaw = pose.get('yaw', 0.0)
                print(f"  - 點位 [{key}] : (X={pose['x']:.3f} m, Y={pose['y']:.3f} m, Yaw={math.degrees(yaw):.1f}°)")
        print("="*35)
        print("使用範例：")
        print("  python3 nav_execute.py 1      (自動導航至點位 1)")
        print("  python3 nav_execute.py 1 2 1  (依序執行導航 1 -> 2 -> 1)")
        print("="*35)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)  # 成功 exit 0，失敗 exit 1

if __name__ == '__main__':
    main()
