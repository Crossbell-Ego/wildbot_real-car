    source /opt/ros/jazzy/setup.bash
    source /workspaces/imu_ws/install/setup.bash
    ros2 run handsfree_imu_ros2 imu_a9_node --ros-args -p port:=/dev/imu_a9 -p baud:=921600
    ```
*   **視窗 B (讀取參數)：**
    `ros2 topic echo /imu/rpy_deg`
*   **開發參考：** 訂閱 Topic `/imu/rpy_deg`，類型為 `geometry_msgs/msg/Vector3`。

### 底盤移動測試
*   **手動發送指令：** (最高速：10.844)
    
```bash
    ros2 topic pub -r 10 /base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
    ```
*   **讀取實時參數：** `ros2 topic echo /joint_states`

### 車頭校正 (Yaw Hold)
```bash
ros2 run wildbot_control yaw_hold_node --ros-args -p linear_x:=0.0 -p kp:=0.02 -p deadband_deg:=1.5 -p max_angular_z:=0.3