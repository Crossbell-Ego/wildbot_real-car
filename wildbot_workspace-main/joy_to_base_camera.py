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

        self.joy_sub = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            10
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

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.bridge = CvBridge()
        self.latest_image = None

        # 防止按鍵長按連續觸發
        self.last_buttons = []

        # 拍照冷卻時間
        self.target_joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.arm_initialized = False  # 安全鎖：未讀到目前位置前不准動
        self.last_photo_time = 0.0
        self.photo_cooldown = 0.8

        # 目前底盤速度命令，base_controller 吃 TwistStamped
        self.current_twist = TwistStamped()
        self.current_twist.header.frame_id = 'base_link'

        # 定時發布底盤速度，20 Hz
        self.timer = self.create_timer(0.05, self.publish_cmd_vel)

        self.get_logger().info('Checking arm_controller action server...')
        if not self.arm_client.wait_for_server(timeout_sec=2.0):
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

    def joint_state_callback(self, msg):
        # 從 /joint_states 找出我們需要的關節
        try:
            arm_1_idx = msg.name.index('arm_1_joint')
            arm_2_idx = msg.name.index('arm_2_joint')
            gripper_idx = msg.name.index('gripper_joint')
            
            # 更新目前位置
            self.current_joint_positions = [
                msg.position[arm_1_idx],
                msg.position[arm_2_idx],
                msg.position[gripper_idx]
            ]
            
            # 如果還沒初始化，進行同步並解鎖
            if not self.arm_initialized:
                self.target_joint_positions = list(self.current_joint_positions)
                self.arm_initialized = True
                self.get_logger().warn("!!! ARM POSITION SYNCED & UNLOCKED !!!")
        except (ValueError, IndexError):
            # 如果還沒在 /joint_states 看到這些關節，先跳過
            pass

    def arm_feedback_callback(self, feedback):
        pass

    def send_arm_goal(self, positions, move_time_sec=0.1):
        # 安全檢查
        if not self.arm_initialized:
            return

        if not self.arm_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'arm_1_joint', 
            'arm_2_joint', 
            'gripper_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = positions
        
        sec = int(move_time_sec)
        nanosec = int((move_time_sec - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec

        goal_msg.trajectory.points = [point]
        
        # 連續發送時不印出 feedback 避免洗頻
        self.arm_client.send_goal_async(goal_msg)

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

    # =========================
    # Gripper / Arm
    # =========================
    def increment_gripper(self, delta_rad):
        if not self.arm_initialized:
            return
            
        # 夾爪在 index 2
        current = self.target_joint_positions[2]
        new_val = max(2.93, min(4.19, current + delta_rad))
        
        if abs(new_val - current) > 0.001:
            self.target_joint_positions[2] = new_val
            # arm_1, arm_2 保持 target_joint_positions 裡面的當前值，只有夾爪改變
            self.send_arm_goal(self.target_joint_positions, move_time_sec=0.1)

    def gripper_open(self):
        self.get_logger().info('Gripper open.')
        if self.arm_initialized:
            self.target_joint_positions[2] = 4.19
            self.send_arm_goal(self.target_joint_positions, move_time_sec=1.0)

    def gripper_half_close(self):
        self.get_logger().info('Gripper half close.')
        if self.arm_initialized:
            self.target_joint_positions[2] = 3.14
            self.send_arm_goal(self.target_joint_positions, move_time_sec=1.0)

    def gripper_safe_close(self):
        self.get_logger().warn('Gripper close to safe limit.')
        if self.arm_initialized:
            self.target_joint_positions[2] = 2.93
            self.send_arm_goal(self.target_joint_positions, move_time_sec=1.0)

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

        left_y = self.apply_deadzone(msg.axes[1]) if len(msg.axes) > 1 else 0.0
        right_x = self.apply_deadzone(msg.axes[0]) if len(msg.axes) > 0 else 0.0

        twist = TwistStamped()
        twist.header.frame_id = 'base_link'
        twist.header.stamp = self.get_clock().now().to_msg()

        # 前進 / 後退 (Jazzy 底盤通常 - 為前進，視馬達接線而定)
        twist.twist.linear.x = -left_y * self.max_linear_x

        # 不使用平移
        twist.twist.linear.y = 0.0

        # 原地旋轉 (使用之前測出的 axes[0])
        twist.twist.angular.z = right_x * self.max_angular_z

        self.current_twist = twist

        # A：夾爪全開
        if self.button_pressed(msg, 0):
            self.gripper_open()

        # B：保守夾取
        if self.button_pressed(msg, 1):
            self.gripper_half_close()

        # X：拍照
        if self.button_pressed(msg, 2):
            self.take_picture()

        # Y：接近閉合極限
        if self.button_pressed(msg, 3):
            self.gripper_safe_close()

        # L2 (按鈕 8)：按住持續打開
        if len(msg.buttons) > 8 and msg.buttons[8] == 1:
            self.increment_gripper(0.08)  # 數字越大開越快

        # R2 (按鈕 9)：按住持續閉合
        if len(msg.buttons) > 9 and msg.buttons[9] == 1:
            self.increment_gripper(-0.08) # 數字越小合越快

        self.last_buttons = list(msg.buttons)

    # =========================
    # Base command
    # =========================
    def publish_cmd_vel(self):
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