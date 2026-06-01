import re

with open("bridge_navigation_final_back_2.py", "r") as f:
    content = f.read()

# 1. Remove imports
content = content.replace("from arm_interface import ArmInterface\n", "")
content = content.replace("from std_msgs.msg import Float64MultiArray, Float32", "from std_msgs.msg import Float32")
content = content.replace("from rclpy.qos import QoSProfile, qos_profile_sensor_data", "from rclpy.qos import QoSProfile")

# 2. Remove init variables
content = content.replace("        self.overheated = False\n", "")
content = content.replace("        self.temperatures = [0.0, 0.0, 0.0]\n", "")

# 3. Remove temp_sub
temp_sub_str = """        self.temp_sub = self.create_subscription(
            Float64MultiArray,
            '/arm_joint_temperatures',
            self.temperature_callback,
            qos_profile_sensor_data
        )
"""
content = content.replace(temp_sub_str, "")

# 4. Remove arm interface init
arm_init_str = """        # 5. 初始化機械手臂控制介面
        self.arm = ArmInterface(self)
"""
content = content.replace(arm_init_str, "")

# 5. Remove temperature_callback
temp_callback_str = """    def temperature_callback(self, msg):
        \"\"\"監控馬達溫度，提供警告與過熱保護\"\"\"
        if len(msg.data) >= 2:
            self.temperatures = list(msg.data)
            max_temp = max(self.temperatures)
            
            if max_temp >= 70.0:
                if not self.overheated:
                    self.overheated = True
                    self.get_logger().error(f"🚨🚨🚨 馬達嚴重過熱: {max_temp:.1f}°C！觸發緊急即停保護，已鎖定底盤！")
                    self.stop_chassis()
            elif max_temp <= 60.0:
                if self.overheated:
                    self.overheated = False
                    self.get_logger().info(f"❄️ 馬達溫度已回落: {max_temp:.1f}°C，解除過熱保護")

"""
content = content.replace(temp_callback_str, "")

# 6. Remove overheated checks (type 1)
overheat_1 = """            if self.overheated:
                print(f"\\n{COLOR_RED}🚨 馬達過熱保護！中止任務。{COLOR_RESET}")
                return False
"""
content = content.replace(overheat_1, "")

# 7. Remove overheated checks (type 2)
overheat_2 = """            if self.overheated:
                print(f"\\n\\033[1;31m🚨 馬達過熱保護！中止任務。\\033[0m")
                return False
"""
content = content.replace(overheat_2, "")

# 8. Modify synchronization check
sync_check_old = """        for _ in range(40):
            if self.front_dist is not None and self.odom_x is not None and self.arm.initialized:
                break
            time.sleep(0.05)
            
        if self.odom_x is None or not self.arm.initialized:
            print(f"\\n\\033[1;31m❌ 感測器或手臂同步逾時，請確認底盤驅動與手臂驅動是否已啟動！\\033[0m")
            return"""
sync_check_new = """        for _ in range(40):
            if self.front_dist is not None and self.odom_x is not None:
                break
            time.sleep(0.05)
            
        if self.odom_x is None:
            print(f"\\n\\033[1;31m❌ 感測器同步逾時，請確認底盤驅動是否已啟動！\\033[0m")
            return"""
content = content.replace(sync_check_old, sync_check_new)

# 9. Remove arm movement to point 5
arm_move = """        print(f"\\n\\033[1;36m🦾 正在移動手臂到教點 5...\\033[0m")
        point_5 = [1.3362240753268588, 1.2398819006167716, 4.1887902047863905]
        self.arm.target_positions = list(point_5)
        self.arm.send_goal(point_5, duration=1.5, teleop_mode=False)
        time.sleep(2.0)
"""
content = content.replace(arm_move, "")

with open("bridge_navigation_final_back_2.py", "w") as f:
    f.write(content)

print("Done")
