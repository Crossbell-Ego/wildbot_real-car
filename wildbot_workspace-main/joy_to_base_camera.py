#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from arm_interface import ArmInterface


class JoyBaseCameraGripper(Node):
    def __init__(self):
        super().__init__('joy_base_camera_gripper')

        # =========================
        # Topic settings
        # =========================
        self.cmd_vel_topic = '/base_controller/cmd_vel'
        self.joy_topic = '/joy'

        # =========================
        # Speed settings
        # =========================
        self.max_linear_x = 0.25      # 前後速度 m/s
        self.max_angular_z = 0.8      # 原地旋轉速度 rad/s
        self.deadzone = 0.08
        self.emergency_stop_button = 6  # L1 / LB

        # =========================
        # Arm fixed pose
        # =========================
        self.arm_1_hold = 1.0
        self.arm_2_hold = 1.0

        # =========================
        # Gripper safe values
        # =========================
        self.gripper_open_rad = 4.19       # 240度，全開
        self.gripper_half_rad = 3       # 180度，保守夾取
        self.gripper_close_rad = 2.93      # 168度，接近閉合極限，不建議更小



        # =========================
        # ROS publishers/subscribers
        # =========================
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.cmd_vel_topic,
            10
        )

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        joy_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.joy_sub = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            joy_qos
        )



        # 初始化通用手臂接口
        self.arm = ArmInterface(self)
        
        self.last_buttons = None
        self.axes_neutralized = False  # 💡 安全防護：啟動時必須等待所有搖桿回到中位，才開始控制手臂，避免暴衝！

        # 目前底盤速度命令，base_controller 吃 TwistStamped
        self.current_twist = TwistStamped()
        self.current_twist.header.frame_id = 'base_link'
        self.emergency_stop_active = False

        # 定時發布底盤速度，50 Hz (減少延遲)
        self.timer = self.create_timer(0.02, self.publish_cmd_vel)

        self.get_logger().info('Checking arm_controller action server...')
        if not self.arm.arm_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('Arm controller action server NOT found! Arm/Gripper functions will be disabled.')
        else:
            self.get_logger().info('Arm controller action server connected.')

        self.get_logger().info('Joy + Base + Camera + Gripper node started.')
        self.get_logger().info('Base cmd topic: /base_controller/cmd_vel')
        self.get_logger().info('Base cmd type: geometry_msgs/msg/TwistStamped')
        self.get_logger().info('Control mode: axes[1]->linear.x, axes[3]->angular.z')


    # =========================
    # Utility
    # =========================
    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def button_pressed(self, msg, index):
        """
        偵測按鍵從 0 -> 1 的瞬間，避免按住時一直觸發。
        """
        if index >= len(msg.buttons):
            return False

        if not self.last_buttons:
            return msg.buttons[index] == 1

        if index >= len(self.last_buttons):
            return False

        return self.last_buttons[index] == 0 and msg.buttons[index] == 1

    def make_stop_twist(self):
        twist = TwistStamped()
        twist.header.frame_id = 'base_link'
        twist.header.stamp = self.get_clock().now().to_msg()
        return twist

    def stop_base_now(self):
        self.current_twist = self.make_stop_twist()
        self.cmd_pub.publish(self.current_twist)



    def gripper_open(self):
        self.get_logger().info('Gripper open.')
        self.arm.send_goal([self.arm.current_positions[0], self.arm.current_positions[1], 4.19], duration=1.0)

    def gripper_half_close(self):
        self.get_logger().info('Gripper half close.')
        self.arm.send_goal([self.arm.current_positions[0], self.arm.current_positions[1], 3.14], duration=1.0)

    def gripper_safe_close(self):
        self.get_logger().warn('Gripper close to safe limit.')
        self.arm.send_goal([self.arm.current_positions[0], self.arm.current_positions[1], 2.93], duration=1.0)

    # =========================
    # Joystick
    # =========================
    def joy_callback(self, msg):
        """
        Xbox / ROG Raikiri 常見 mapping：

          axes[1] = 左搖桿上下
          axes[3] = 右搖桿左右

          buttons[0] = A
          buttons[1] = B
          buttons[2] = X
          buttons[3] = Y

        本程式不使用平移：
          twist.twist.linear.y 固定為 0.0
        """

        # 改為 Toggle 切換邏輯：按一下鎖定，再按一下解鎖
        if self.button_pressed(msg, self.emergency_stop_button):
            self.emergency_stop_active = not self.emergency_stop_active
            if self.emergency_stop_active:
                self.get_logger().error('!!! EMERGENCY STOP LOCKED !!! 小車與手臂已強制停死')
                self.stop_base_now()
                # 立即停止手臂：將目標位置同步為當前觀測到的位置並發送。
                self.arm.sync_targets()
                self.arm.send_goal(self.arm.current_positions, duration=0.01)
                self.arm.horizontal_sum = None
                self.get_logger().warn("Arm/Gripper movement HALTED.")
            else:
                self.get_logger().error('!!! EMERGENCY STOP UNLOCKED !!! 已解除鎖定，恢復移動與手臂控制')
            
            self.last_buttons = list(msg.buttons)
            return

        # 如果處於即停鎖定狀態，直接無視後續所有手把輸入 (底盤、手臂、夾爪)
        if self.emergency_stop_active:
            self.last_buttons = list(msg.buttons)
            return

        left_y = self.apply_deadzone(msg.axes[1]) if len(msg.axes) > 1 else 0.0
        right_x = self.apply_deadzone(msg.axes[0]) if len(msg.axes) > 0 else 0.0

        twist = TwistStamped()
        twist.header.frame_id = 'base_link'
        twist.header.stamp = self.get_clock().now().to_msg()

        # 前進 / 後退 (修正方向：移除負號以符合實際搖桿操作)
        twist.twist.linear.x = left_y * self.max_linear_x

        # 不使用平移
        twist.twist.linear.y = 0.0

        # 原地旋轉 (使用之前測出的 axes[0])
        twist.twist.angular.z = right_x * self.max_angular_z

        self.current_twist = twist



        # 限制手臂控制指令發送頻率 (避免高頻手把事件瘋狂搶佔 Action 導致抖動與卡頓)
        now = self.get_clock().now().nanoseconds / 1e9
        if not hasattr(self, 'last_arm_cmd_time'):
            self.last_arm_cmd_time = 0.0

        # 依據 verified 對應表：右搖桿上下 (Vertical) 固定為 msg.axes[3]
        right_y_stick = self.apply_deadzone(msg.axes[3]) if len(msg.axes) > 3 else 0.0

        # 💡 安全中位判定：如果右搖桿上下尚未歸零，必須在手把首次回到中位後才允許控制，避免暴衝！
        if not self.axes_neutralized:
            if abs(right_y_stick) < 0.01:
                self.axes_neutralized = True
                self.get_logger().info("✅ 手把右搖桿已成功歸零/中位校準，解除手臂安全鎖。")
            else:
                # 尚未中位，強制設為 0.0 避免暴衝
                right_y_stick = 0.0

        l2_pressed = False
        r2_pressed = False
        if len(msg.axes) > 4:
            # 類比扳機預設為 1.0，按壓時值會減少至 0.1 以下
            l2_pressed = (msg.axes[4] < 0.1)
        if len(msg.axes) > 5:
            r2_pressed = (msg.axes[5] < 0.1)

        has_arm_input = False
        if len(msg.buttons) > 4 and msg.buttons[4] == 1: has_arm_input = True
        if len(msg.buttons) > 0 and msg.buttons[0] == 1: has_arm_input = True
        if (len(msg.buttons) > 2 and msg.buttons[2] == 1) or (len(msg.buttons) > 3 and msg.buttons[3] == 1): has_arm_input = True
        if len(msg.buttons) > 1 and msg.buttons[1] == 1: has_arm_input = True
        if abs(right_y_stick) > 0.05: has_arm_input = True
        if l2_pressed or r2_pressed: has_arm_input = True

        if has_arm_input:
            if now - self.last_arm_cmd_time < 0.040:  # 提升至 25Hz (40ms) 發送頻率，達到真實即時控制
                self.last_buttons = list(msg.buttons)
                return
            self.last_arm_cmd_time = now

        # Y (按鈕 4): 控制 joint 1 往上
        if len(msg.buttons) > 4 and msg.buttons[4] == 1:
            self.arm.move_arm_1(-0.040)

        # A (按鈕 0): 控制 joint 1 往下
        if len(msg.buttons) > 0 and msg.buttons[0] == 1:
            self.arm.move_arm_1(0.040)

        # X (按鈕 2 或 3): 控制 joint 2 往上
        if (len(msg.buttons) > 2 and msg.buttons[2] == 1) or (len(msg.buttons) > 3 and msg.buttons[3] == 1):
            self.arm.move_arm_2(-0.040)

        # B (按鈕 1): 控制 joint 2 往下
        if len(msg.buttons) > 1 and msg.buttons[1] == 1:
            self.arm.move_arm_2(0.040)

        # 💡 L2 鍵 (LT 類比，msg.axes[4]) 按住：夾爪持續張開 (增量控制)
        if l2_pressed:
            self.arm.move_gripper(0.040)

        # 💡 R2 鍵 (RT 類比，msg.axes[5]) 按住：夾爪持續閉合 (增量控制)
        if r2_pressed:
            self.arm.move_gripper(-0.040)


        # 右搖桿左右：無功能
        # 右搖桿上下：水平連動控制 (保持第二軸/小臂與地面水平)
        if abs(right_y_stick) > 0.05:
            # 向上推 (1.0) 時為負，大臂往上 (arm_1 減少)，小臂自動反向補償 -> 保持水平伸長 (Extend)
            self.arm.move_horizontal(-right_y_stick * 0.040)

        self.last_buttons = list(msg.buttons)

    # =========================
    # Base command
    # =========================
    def publish_cmd_vel(self):
        # 如果正在急停，必須強制發布 0.0 以覆蓋其他所有指令
        if self.emergency_stop_active:
            self.stop_base_now()
            return

        # 增加容錯空間 (Deadzone)，確保細微雜訊不會觸發發布
        is_joy_moving = (
            abs(self.current_twist.twist.linear.x) > 0.01 or
            abs(self.current_twist.twist.angular.z) > 0.01
        )

        # 只有在手把「有動作」時才發布，否則保持沉默，讓位給其他指令 (如 ros2 topic pub)
        if is_joy_moving:
            self.current_twist.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(self.current_twist)


def main(args=None):
    rclpy.init(args=args)

    node = JoyBaseCameraGripper()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception:
        pass
    finally:
        # 關閉程式時停止底盤 (防護：避免 context 已失效時發布報錯)
        try:
            stop_msg = TwistStamped()
            stop_msg.header.frame_id = 'base_link'
            stop_msg.header.stamp = node.get_clock().now().to_msg()
            node.cmd_pub.publish(stop_msg)
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()