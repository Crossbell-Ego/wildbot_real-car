#!/usr/bin/env python3

import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import Joy, Image
from geometry_msgs.msg import TwistStamped

from cv_bridge import CvBridge
import cv2

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
        self.image_topic = '/camera/color/image_raw'

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
        # Photo save folder
        # =========================
        self.save_dir = '/workspaces/photos'
        os.makedirs(self.save_dir, exist_ok=True)

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

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        # 新增：監聽關節狀態，用來解鎖安全鎖
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 初始化通用手臂接口
        self.arm = ArmInterface(self)
        
        self.latest_image = None
        self.last_photo_time = 0.0
        self.photo_cooldown = 0.8

        # 目前底盤速度命令，base_controller 吃 TwistStamped
        self.current_twist = TwistStamped()
        self.current_twist.header.frame_id = 'base_link'
        self.emergency_stop_active = False

        # 定時發布底盤速度，50 Hz (減少延遲)
        self.timer = self.create_timer(0.02, self.publish_cmd_vel)

        self.get_logger().info('Checking arm_controller action server...')
        if not self.arm_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('Arm controller action server NOT found! Arm/Gripper functions will be disabled.')
        else:
            self.get_logger().info('Arm controller action server connected.')

        self.get_logger().info('Joy + Base + Camera + Gripper node started.')
        self.get_logger().info('Base cmd topic: /base_controller/cmd_vel')
        self.get_logger().info('Base cmd type: geometry_msgs/msg/TwistStamped')
        self.get_logger().info('Control mode: axes[1]->linear.x, axes[3]->angular.z')
        self.get_logger().info(f'Photos will be saved to: {self.save_dir}')

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

    # =========================
    # Camera
    # =========================
    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    def take_picture(self):
        now = time.time()

        if now - self.last_photo_time < self.photo_cooldown:
            return

        self.last_photo_time = now

        if self.latest_image is None:
            self.get_logger().warn('No image received yet.')
            return

        os.makedirs(self.save_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'photo_{timestamp}.jpg'
        filepath = os.path.join(self.save_dir, filename)

        try:
            ok = cv2.imwrite(filepath, self.latest_image)

            if ok and os.path.exists(filepath):
                self.get_logger().info(f'Photo saved: {filepath}')
            else:
                self.get_logger().error(f'cv2.imwrite failed, file not created: {filepath}')

        except Exception as e:
            self.get_logger().error(f'Failed to save photo: {e}')

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

        # 偵錯用：印出按下的按鍵編號，方便確認 Y 鍵是幾號
        for i, b in enumerate(msg.buttons):
            if b == 1:
                self.get_logger().warn(f"BUTTON PRESSED: {i}")

        # Y (按鈕 4): 控制 joint 1 往上
        if len(msg.buttons) > 4 and msg.buttons[4] == 1:
            self.arm.move_arm_1(-0.01)

        # A (按鈕 0): 控制 joint 1 往下
        if len(msg.buttons) > 0 and msg.buttons[0] == 1:
            self.arm.move_arm_1(0.01)

        # X (按鈕 2 或 3): 控制 joint 2 往上
        if (len(msg.buttons) > 2 and msg.buttons[2] == 1) or (len(msg.buttons) > 3 and msg.buttons[3] == 1):
            self.arm.move_arm_2(-0.01)
            if self.button_pressed(msg, 2) or self.button_pressed(msg, 3):
                self.take_picture()

        # B (按鈕 1): 控制 joint 2 往下
        if len(msg.buttons) > 1 and msg.buttons[1] == 1:
            self.arm.move_arm_2(0.01)

        # L2 (按鈕 8)：按住持續打開
        if len(msg.buttons) > 8 and msg.buttons[8] == 1:
            self.arm.move_gripper(0.02)  # 調降速度

        # R2 (按鈕 9)：按住持續閉合
        if len(msg.buttons) > 9 and msg.buttons[9] == 1:
            self.arm.move_gripper(-0.02) # 調降速度

        # 右搖桿左右 (Index 2): 水平連動控制 (保持第二軸與地面水平)
        right_x_stick = self.apply_deadzone(msg.axes[2]) if len(msg.axes) > 2 else 0.0
        if abs(right_x_stick) > 0.05:
            # 向右推 (1.0) 時 delta 為負，大臂往上 (arm_1 減少)，小臂自動反向補償 -> 伸長
            self.arm.move_horizontal(-right_x_stick * 0.01)

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

    except KeyboardInterrupt:
        pass

    finally:
        # 關閉程式時停止底盤
        stop_msg = TwistStamped()
        stop_msg.header.frame_id = 'base_link'
        stop_msg.header.stamp = node.get_clock().now().to_msg()

        node.cmd_pub.publish(stop_msg)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
