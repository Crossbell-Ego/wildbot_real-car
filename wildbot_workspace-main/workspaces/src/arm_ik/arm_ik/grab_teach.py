import rclpy
from rclpy.node import Node
import sys
import termios
import tty
import select
import math
import json
import os
from arm_interface import ArmInterface

class GrabTeach(Node):
    def __init__(self):
        super().__init__('grab_teach')
        
        # 初始化通用手臂接口
        self.arm = ArmInterface(self)
        self.pose_file = "arm_poses.json"
        self.saved_poses = self.load_poses()
        
        self.recall_mode = False # 是否處於回放模式
        
        # 顯示資訊
        self.get_logger().info("==========================================")
        self.get_logger().info("🤖 高級教導/回放模式 - 已啟動")
        self.get_logger().info(f"儲存檔案: {os.path.abspath(self.pose_file)}")
        self.get_logger().info("鍵盤操作：")
        self.get_logger().info("  0 - 9 : [紀錄模式] 紀錄當前位置")
        self.get_logger().info("  R     : [切換] 進入/退出回放模式 (自動移動)")
        self.get_logger().info("  I     : 印出當前實時位置")
        self.get_logger().info("  X     : 退出程式")
        self.get_logger().info("==========================================")

    def load_poses(self):
        """從 JSON 檔案載入點位。"""
        if os.path.exists(self.pose_file):
            try:
                with open(self.pose_file, 'r') as f:
                    poses = json.load(f)
                self.get_logger().info(f"📂 已從檔案載入 {len(poses)} 個點位")
                return poses
            except Exception as e:
                self.get_logger().error(f"❌ 載入點位失敗: {e}")
        return {}

    def save_pose(self, slot):
        if not self.arm.initialized:
            self.get_logger().error("❌ 尚未接收到手臂位置，無法紀錄")
            return
        
        pos = list(self.arm.current_positions)
        # JSON 儲存為 list，key 為字串
        self.saved_poses[str(slot)] = pos
        
        try:
            with open(self.pose_file, 'w') as f:
                json.dump(self.saved_poses, f, indent=4)
            
            q_deg = [math.degrees(x) for x in pos]
            x_m, z_m = self.arm.get_joint2_coordinates(pos[0])
            x_g, z_g = self.arm.get_coordinates(pos[0], pos[1])
            
            print(f"\n✅ [已紀錄點位 {slot}]")
            print(f"   第二軸中心座標: X: {x_m*100:.1f} cm, Z(離地): {z_m*100:.1f} cm")
            print(f"   夾爪抓取點座標: X: {x_g*100:.1f} cm, Z(離地): {z_g*100:.1f} cm")
            print(f"   角度 (Deg): Q1={q_deg[0]:.1f}, Q2={q_deg[1]:.1f}, Grip={q_deg[2]:.1f}")
            print(f"   弧度 (Rad): {pos}")
            print(f"💾 檔案已更新: {self.pose_file}")
        except Exception as e:
            self.get_logger().error(f"❌ 儲存點位失敗: {e}")

    def move_to_pose(self, slot):
        """移動手臂到指定點位。"""
        slot_str = str(slot)
        if slot_str not in self.saved_poses:
            print(f"⚠️  點位 {slot} 尚未紀錄，無法移動")
            return
        
        print(f"🚀 正在移動至點位 {slot}...")
        target = self.saved_poses[slot_str]
        
        # 強制同步一次目標值，避免計算 delta 時出錯
        self.arm.target_positions = list(target)
        self.arm.send_goal(target, duration=1.5, teleop_mode=False)

    def print_status(self):
        if not self.arm.initialized:
            print("⏳ 正在等待 JointState 訊號...")
            return
        q = self.arm.current_positions
        t = self.arm.temperatures
        x_m, z_m = self.arm.get_joint2_coordinates(q[0])
        x_g, z_g = self.arm.get_coordinates(q[0], q[1])
        
        status_line = "🔥 [過熱鎖定]" if self.arm.overheated else "✅ [狀態正常]"
        print(f"\n{status_line}")
        print(f"📍 [第二軸中心實時座標] X: {x_m*100:.1f} cm, Z(離地): {z_m*100:.1f} cm")
        print(f"📍 [夾爪抓取點實時座標] X: {x_g*100:.1f} cm, Z(離地): {z_g*100:.1f} cm")
        print(f"🌡️  [馬達溫度] Q1: {t[0]:.1f}°C, Q2: {t[1]:.1f}°C, Grip: {t[2]:.1f}°C")
        print(f"⚙️  [實時角度] Q1: {math.degrees(q[0]):.1f}°, Q2: {math.degrees(q[1]):.1f}°, Grip: {math.degrees(q[2]):.1f}°")

def main():
    rclpy.init()
    node = GrabTeach()
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                char = sys.stdin.read(1).lower()
                
                if char.isdigit():
                    if node.recall_mode:
                        node.move_to_pose(char)
                    else:
                        node.save_pose(char)
                elif char == 'r':
                    node.recall_mode = not node.recall_mode
                    status = "【回放模式 (按數字移動)】" if node.recall_mode else "【紀錄模式 (按數字儲存)】"
                    print(f"\n🔄 模式切換: {status}")
                elif char == 'i':
                    node.print_status()
                elif char == 'x':
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
