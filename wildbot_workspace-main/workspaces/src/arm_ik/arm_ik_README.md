# arm_ik

ROS2 Python 節點，依照 `kros_car_persistent.xacro` 內的實體小車手臂幾何，實作 2-DOF 平面手臂解析逆運動學。

使用方式：

1. 在工作區根目錄執行：

```bash
colcon build --packages-select arm_ik
source install/setup.bash
ros2 launch arm_ik ik.launch.py
```

2. 發送目標到主題 `ik_target` (type: `geometry_msgs/Point`)。

預設使用 `Point.x` 作水平距離、`Point.z` 作高度，單位是公尺。為了相容舊範例，也可以把參數 `use_msg_z` 設為 `false` 後改用 `Point.y` 當高度。

節點預設會：

- 在 `joint_states` 發出 `sensor_msgs/JointState`，關節名稱為 `arm_1_joint`、`arm_2_joint`、`gripper_joint`。
- 在 `/arm_controller/commands` 發出 `std_msgs/Float64MultiArray`，資料順序同上，可接 forward position controller。
- 在執行 `ik_node` 的終端機按下 `i`，會顯示目前 `arm_1_joint`、`arm_2_joint`、`gripper_joint` 角度，以及夾爪中心的 `x` / `z` 座標。

重要參數：

- `l1`: 第一段手臂長度，預設 `0.080`。
- `l2`: 第二段到夾爪中心長度，預設 `0.0755`。
- `shoulder_offset` / `elbow_offset`: 實體伺服零點校正。
- `shoulder_direction` / `elbow_direction`: 實體伺服方向校正，若動作相反可設為 `-1.0`。
- `elbow_up`: 切換 elbow-up / elbow-down 解。
