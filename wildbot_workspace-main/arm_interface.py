import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import time

class ArmInterface:
    """
    Wildbot 機械手臂通用控制類別。
    封裝了實時位置同步、水平連動、安全限位與夾爪保護邏輯。
    """
    def __init__(self, node):
        self.node = node
        self.joint_names = ['arm_1_joint', 'arm_2_joint', 'gripper_joint']
        self.current_positions = [0.0, 0.0, 0.0]
        self.target_positions = [0.0, 0.0, 0.0]
        self.initialized = False
        self.horizontal_sum = None
        self.last_cmd_time = 0.0  # 上次發送指令的時間
        
        # 水平校正偏移量 (≈ 10°)
        self.HORIZONTAL_CORRECTION_RAD = 0.1745
        
        # 限位設定 (rad)
        self.LIMITS = {
            'arm_1': (0.523, 3.665),  # 30° ~ 210°
            'arm_2': (0.0, 4.188),    # 0° ~ 240°
            'gripper': (2.93, 4.19)   # 168° ~ 240°
        }

        # 物理幾何參數 (公尺) - 用於安全檢查與座標計算
        self.L1 = 0.080      # 第一臂長
        self.L2 = 0.11       # 第二臂長 (已更新)
        self.BASE_X = 0.165  # 手臂底座 X 偏移
        # 弧度總合限位 (Q1 + Q2 的最大值)
        # 根據您的實驗數據：174.5° + 60.2° = 234.7° (4.096 rad) 是撞擊臨界點
        # 我們將安全邊界設在 234.0° (4.084 rad)，直接用弧度判斷最準確
        self.JOINT_SUM_MAX = 4.084 

        # 使用感測器專用 QoS，提高相容性 (Best Effort)
        self.subscription = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data)
            
        self.arm_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

    def get_coordinates(self, q1, q2):
        """
        計算夾爪在 base_link 座標系下的實時位置 (X, Z)。
        (註：此功能僅供顯示參考，不參與安全攔截)
        """
        x = self.BASE_X + self.L1 * math.cos(q1) + self.L2 * math.cos(q1 + q2)
        z = self.BASE_Z + self.L1 * math.sin(q1) + self.L2 * math.sin(q1 + q2)
        return x, z

    def check_safety(self, q1, q2):
        """
        利用「兩軸弧度總和」檢查是否會撞地。
        這能避免因實體尺寸測量誤差導致的高度計算錯誤。
        """
        # 1. 基礎單軸角度限位
        sq1 = max(self.LIMITS['arm_1'][0], min(self.LIMITS['arm_1'][1], q1))
        sq2 = max(self.LIMITS['arm_2'][0], min(self.LIMITS['arm_2'][1], q2))

        # 2. 弧度總合限制 (Q1 + Q2)
        current_sum = sq1 + sq2
        if current_sum > self.JOINT_SUM_MAX:
            self.node.get_logger().warn(
                f"🛡️  觸發弧度保護：Q1+Q2={math.degrees(current_sum):.1f}° (上限 {math.degrees(self.JOINT_SUM_MAX):.1f}°)"
            )
            # 如果超過上限，維持舊位置不變
            return self.target_positions[0], self.target_positions[1]

        return sq1, sq2

    def joint_state_callback(self, msg):
        """
        同步機械手臂的物理狀態。
        """
        positions = {}
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names:
                positions[name] = pos
        
        if len(positions) >= 3:
            self.current_positions = [
                positions['arm_1_joint'],
                positions['arm_2_joint'],
                positions['gripper_joint']
            ]
            if not self.initialized:
                self.target_positions = list(self.current_positions)
                self.initialized = True

    def sync_targets(self, force=False):
        """
        智慧同步目標位置。
        如果距離上次操作超過 0.5 秒，或者 force=True，則從物理狀態同步。
        避免連續操作時因回傳延遲導致移動被拉回。
        """
        now = self.node.get_clock().now().nanoseconds / 1e9
        if force or (now - self.last_cmd_time > 0.5):
            self.target_positions = list(self.current_positions)

    def send_goal(self, positions, duration=0.2):
        """
        發送軌跡控制指令。
        """
        if not self.arm_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error("❌ 手臂控制器 Action Server 不在線上")
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        
        goal_msg.trajectory.points = [point]
        self.arm_client.send_goal_async(goal_msg)
        self.last_cmd_time = self.node.get_clock().now().nanoseconds / 1e9

    def move_arm_1(self, delta):
        """獨立控制大臂。"""
        if not self.initialized: return
        self.sync_targets() # 自動偵測是否需要同步
        
        # 安全檢查與限位
        new_q1, new_q2 = self.check_safety(self.target_positions[0] + delta, self.target_positions[1])
        
        self.target_positions[0] = new_q1
        self.target_positions[1] = new_q2
        # 獨立控制時失效水平常數
        self.horizontal_sum = None
        self.send_goal(self.target_positions)

    def move_arm_2(self, delta):
        """獨立控制小臂。"""
        if not self.initialized: return
        self.sync_targets()
        
        # 安全檢查與限位
        new_q1, new_q2 = self.check_safety(self.target_positions[0], self.target_positions[1] + delta)
        
        self.target_positions[0] = new_q1
        self.target_positions[1] = new_q2
        # 獨立控制時失效水平常數
        self.horizontal_sum = None
        self.send_goal(self.target_positions)

    def move_horizontal(self, delta):
        """水平連動控制：保持小臂與地面水平。"""
        if not self.initialized: return
        self.sync_targets()
        
        # 如果常數失效，則根據目前位置重新計算
        if self.horizontal_sum is None:
            self.horizontal_sum = (
                self.target_positions[0] + self.target_positions[1] + self.HORIZONTAL_CORRECTION_RAD
            )
        
        # 計算初步目標
        raw_q1 = self.target_positions[0] + delta
        raw_q2 = self.horizontal_sum - raw_q1
        
        # 通過安全過濾器
        new_q1, new_q2 = self.check_safety(raw_q1, raw_q2)
        
        self.target_positions[0] = new_q1
        self.target_positions[1] = new_q2
        self.send_goal(self.target_positions)

    def move_gripper(self, delta):
        """控制夾爪開合。"""
        if not self.initialized: return
        self.sync_targets()
        
        new_grip = max(self.LIMITS['gripper'][0], min(self.LIMITS['gripper'][1], self.target_positions[2] + delta))
        self.target_positions[2] = new_grip
        self.send_goal(self.target_positions)
        
        # 防燒毀機制：閉合到底時自動退回
        if delta < 0 and new_grip <= (self.LIMITS['gripper'][0] + 0.02):
            time.sleep(0.5)
            self.sync_targets(force=True) # 強制同步以執行保護動作
            self.target_positions[2] = self.current_positions[2] + 0.035  # 退回約 2 度
            self.send_goal(self.target_positions)
            self.node.get_logger().warn("⚠️ 觸發防燒毀保護：夾爪已自動退回 2 度")
