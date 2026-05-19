import rclpy
from rclpy.node import Node
import json
import os
import sys
import time
from arm_interface import ArmInterface

class GrabExecutor(Node):
    def __init__(self):
        super().__init__('grab_executor')
        self.arm = ArmInterface(self)
        self.pose_file = "arm_poses.json"
        self.saved_poses = self.load_poses()

    def load_poses(self):
        """載入 JSON 點位檔案。"""
        if os.path.exists(self.pose_file):
            try:
                with open(self.pose_file, 'r') as f:
                    return json.load(f)
            except Exception as e:
                self.get_logger().error(f"❌ 讀取 {self.pose_file} 失敗: {e}")
        return {}

    def move_to_slot(self, slot_name, duration=2.0):
        """移動到指定名稱的點位。"""
        if slot_name not in self.saved_poses:
            self.get_logger().error(f"❌ 找不到點位: {slot_name}")
            return False
        
        target = self.saved_poses[slot_name]
        self.get_logger().info(f"🚀 正在移動至點位 [{slot_name}]...")
        
        # 同步內部目標值並發送
        self.arm.target_positions = list(target)
        self.arm.send_goal(target, duration=duration, teleop_mode=False)
        
        # 改用非阻塞的 spin 循環等待移動完成，確保 JointState 在這期間能實時更新
        start_wait = time.time()
        while time.time() - start_wait < (duration + 0.5):
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # 計算抵達後的實時座標
        q = self.arm.current_positions
        x_m, z_m = self.arm.get_joint2_coordinates(q[0])
        x_g, z_g = self.arm.get_coordinates(q[0], q[1])
        
        self.get_logger().info(
            f"✅ 抵達點位 [{slot_name}] (第二軸 X(距前擋板): {x_m*100:.1f} cm, Z(離地): {z_m*100:.1f} cm | "
            f"夾爪 X(距前擋板): {x_g*100:.1f} cm, Z(離地): {z_g*100:.1f} cm)"
        )
        return True

def main():
    rclpy.init()
    node = GrabExecutor()
    
    # 稍微等待 JointState 同步
    print("⏳ 正在初始化手臂接口...")
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.arm.initialized:
            break
            
    if not node.arm.initialized:
        print("❌ 無法取得手臂當前狀態，請檢查機器人連線。")
        node.destroy_node()
        rclpy.shutdown()
        return

    # 檢查是否有命令列參數
    if len(sys.argv) > 1:
        # 支援多個參數，例如: python3 grab_execute.py 1 2 1
        for slot in sys.argv[1:]:
            if not node.move_to_slot(slot):
                break
    else:
        # 沒有參數時列出所有點位
        print("\n" + "="*30)
        print("📋 可用點位清單：")
        for key in sorted(node.saved_poses.keys()):
            print(f"  - {key}")
        print("="*30)
        print("使用範例：")
        print("  python3 grab_execute.py 1      (移動到點位 1)")
        print("  python3 grab_execute.py 1 2 1  (執行 1->2->1 序列)")
        print("="*30)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
