import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time

class ArmInterface:
    """
    Wildbot 機械手臂通用控制類別。
    封裝了實時位置同步、水平連動、安全限位與溫度保護邏輯。
    """
    def __init__(self, node):
        self.node = node
        self.joint_names = ['arm_1_joint', 'arm_2_joint', 'gripper_joint']
        self.current_positions = [0.0, 0.0, 0.0]
        self.target_positions = [0.0, 0.0, 0.0]
        self.temperatures = [0.0, 0.0, 0.0]  # [Q1, Q2, Grip]
        self.overheated = False
        self.initialized = False
        self.horizontal_sum = None
        self.last_cmd_time = 0.0  # 上次發送指令的時間
        
        # 限位設定 (rad)
        self.LIMITS = {
            'arm_1': (0.523, 3.665),  # 30° ~ 210°
            'arm_2': (0.0, 4.188),    # 0° ~ 240°
        }

        # 物理幾何參數 (公尺) - 用於安全檢查與座標計算
        self.L1 = 0.080      # 第一臂長
        self.L2 = 0.11       # 第二臂長 (已更新)
        self.BASE_X = 0.165  # 手臂底座 X 偏移
        self.BASE_Z = 0.120  # 手臂底座高度 (補回原先漏掉的定義)

        # 使用感測器專用 QoS，提高相容性 (Best Effort)
        self.subscription = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data)

        # 訂閱馬達溫度，進行過熱停機保護
        self.temp_sub = self.node.create_subscription(
            Float32MultiArray,
            '/arm_joint_temperatures',
            self.temperature_callback,
            qos_profile_sensor_data)
            
        # 建立 Topic 發布者，實現高速無延遲遙控 (極致流暢、防抖)
        self.arm_pub = self.node.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10)
            
        self.arm_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

    def get_coordinates(self, q1, q2):
        """
        計算夾爪（抓取點）在 base_link 座標系下的實時位置 (X, Z)（包含實體皮尺對位校準）。
        當小臂水平時，其 Z 軸高度將與第二軸中心完全一致。
        """
        theta1 = 2.0 - q1
        
        # 實測完美水平時 q1 + q2 = 4.12177 rad，此時物理傾角 theta2 為 0
        theta2 = 4.12177 - (q1 + q2)
        
        base_z_ground = 0.1266
        
        x = self.BASE_X + self.L1 * math.cos(theta1) + self.L2 * math.cos(theta2)
        z = base_z_ground + self.L1 * math.sin(theta1) + self.L2 * math.sin(theta2)
        return x, z

    def get_joint2_coordinates(self, q1):
        """
        計算第二軸 (arm_2_joint) 在 base_link 座標系下的位置 (X, Z)（包含實體皮尺對位校準）。
        """
        # 實體皮尺對位校準映射關係
        theta1 = 2.0 - q1
        
        # 第一軸旋轉中心實際離地高度為 12.66 cm (0.1266 m)
        base_z_ground = 0.1266
        
        x = self.BASE_X + self.L1 * math.cos(theta1)
        z = base_z_ground + self.L1 * math.sin(theta1)
        return x, z


    def check_safety(self, q1, q2):
        """
        基礎單軸角度限位安全檢查。
        """
        # 基礎單軸角度限位
        sq1 = max(self.LIMITS['arm_1'][0], min(self.LIMITS['arm_1'][1], q1))
        sq2 = max(self.LIMITS['arm_2'][0], min(self.LIMITS['arm_2'][1], q2))
        
        if q1 < self.LIMITS['arm_1'][0] or q1 > self.LIMITS['arm_1'][1]:
            self.node.get_logger().warn(
                f"⚠️ 大臂 (arm_1_joint) 嘗試超出安全限制 {math.degrees(self.LIMITS['arm_1'][0]):.1f}° ~ {math.degrees(self.LIMITS['arm_1'][1]):.1f}°，目前已攔截並限制在 {math.degrees(sq1):.1f}°！",
                throttle_duration_sec=1.0
            )
        if q2 < self.LIMITS['arm_2'][0] or q2 > self.LIMITS['arm_2'][1]:
            self.node.get_logger().warn(
                f"⚠️ 小臂 (arm_2_joint) 嘗試超出安全限制 {math.degrees(self.LIMITS['arm_2'][0]):.1f}° ~ {math.degrees(self.LIMITS['arm_2'][1]):.1f}°，目前已攔截並限制在 {math.degrees(sq2):.1f}°！",
                throttle_duration_sec=1.0
            )
            
        return sq1, sq2

    def joint_state_callback(self, msg):
        """
        同步機械手臂的物理狀態，具備高容錯的 2-joint 與 3-joint 相容能力。
        """
        positions = {}
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names:
                positions[name] = pos
        
        # 只要能獲得兩大核心軸且數值有效即可初始化
        if 'arm_1_joint' in positions and 'arm_2_joint' in positions:
            # 💡 安全防護：若讀取到 arm_1_joint 為 0.0，代表 ROS 2 control 驅動尚未完成串口硬體通訊 (物理極限最小為 0.523)
            # 此時必須拒絕初始化，防止因為讀取到零點而導致手臂在啟動時突然暴衝！
            if abs(positions['arm_1_joint']) < 0.01:
                return

            self.current_positions[0] = positions['arm_1_joint']
            self.current_positions[1] = positions['arm_2_joint']
            if 'gripper_joint' in positions:
                self.current_positions[2] = positions['gripper_joint']
            
            if not self.initialized:
                self.target_positions = list(self.current_positions)
                self.initialized = True

    def temperature_callback(self, msg):
        """
        監控大臂、小臂、夾爪馬達的溫度，提供警告與過熱保護。
        """
        if len(msg.data) >= 2:
            self.temperatures = list(msg.data)
            max_temp = max(self.temperatures)
            
            # 65度警告，70度強制停機
            if max_temp >= 70.0:
                if not self.overheated:
                    self.overheated = True
                    self.node.get_logger().error(
                        f"🚨🚨🚨 馬達嚴重過熱！當前最高溫度: {max_temp:.1f}°C (閥值 70.0°C) 進入強制停機保護"
                    )
            elif max_temp <= 60.0:
                if self.overheated:
                    self.overheated = False
                    self.node.get_logger().info(
                        f"❄️ 馬達溫度已回落: {max_temp:.1f}°C (低於 60.0°C) 解除停機保護"
                    )
            
            # 65度至70度之間發出警告
            if max_temp >= 65.0 and max_temp < 70.0:
                temp_str = ", ".join([f"M{i+1}={t:.1f}°C" for i, t in enumerate(self.temperatures)])
                self.node.get_logger().warn(
                    f"⚠️ 馬達溫度偏高: {temp_str} (警告臨界 65.0°C)"
                )

    def sync_targets(self, force=False):
        """
        智慧同步目標位置。
        如果距離上次操作超過 0.5 秒，或者 force=True，則從物理狀態同步。
        避免連續操作時因回傳延遲導致移動被拉回。
        """
        now = self.node.get_clock().now().nanoseconds / 1e9
        if force or (now - self.last_cmd_time > 0.5):
            self.target_positions = list(self.current_positions)

    def send_goal(self, positions, duration=0.08, teleop_mode=True):
        """
        發送軌跡控制指令（過熱時攔截）。
        支援雙模式：
        - teleop_mode=True (預設)：透過 Topic 直接發布點位，無 Action 狀態機延遲與搶佔，操控極致順滑、零抖動。
        - teleop_mode=False：透過 Action Client 發送，提供精確狀態與回放等待。
        """
        if self.overheated:
            self.node.get_logger().error("❌ 手臂處於過熱停機保護狀態，拒絕執行移動指令！")
            return

        if teleop_mode:
            # 高速 Topic 發布模式 (無搶佔延遲，實現極致順滑運動)
            msg = JointTrajectory()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.joint_names = self.joint_names
            
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions]
            point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
            
            msg.points = [point]
            self.arm_pub.publish(msg)
            self.last_cmd_time = self.node.get_clock().now().nanoseconds / 1e9
        else:
            # 精確 Action 模式 (供教導回放使用)
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
        """水平連動控制：保持小臂與地面絕對水平（夾爪與第二軸中心同高）。"""
        if not self.initialized: return
        self.sync_targets()
        
        # 強制小臂與地面絕對水平，根據實體校準點位，完美水平時 q1 + q2 = 4.12177
        self.horizontal_sum = 4.12177
        
        # 計算初步目標 (大臂前進 delta，小臂自動反向跟隨以保持絕對水平)
        raw_q1 = self.target_positions[0] + delta
        raw_q2 = self.horizontal_sum - raw_q1
        
        # 通過安全過濾器
        new_q1, new_q2 = self.check_safety(raw_q1, raw_q2)
        
        self.target_positions[0] = new_q1
        self.target_positions[1] = new_q2
        self.send_goal(self.target_positions)

    def move_gripper(self, delta):
        """控制夾爪開合 (漸進/增量式)。"""
        if not self.initialized: return
        self.sync_targets()
        
        raw_gripper = self.target_positions[2] + delta
        # 安全限制：全開 4.19 rad (240度)，閉合極限 2.93 rad (168度)
        new_gripper = max(2.93, min(4.19, raw_gripper))
        
        if raw_gripper < 2.93 or raw_gripper > 4.19:
            self.node.get_logger().warn(
                f"⚠️ 夾爪 (gripper_joint) 嘗試超出安全限制 168.0° (2.93 rad) ~ 240.0° (4.19 rad)，目前已攔截並限制在 {math.degrees(new_gripper):.1f}°！",
                throttle_duration_sec=1.0
            )
        
        self.target_positions[2] = new_gripper
        self.send_goal(self.target_positions)

