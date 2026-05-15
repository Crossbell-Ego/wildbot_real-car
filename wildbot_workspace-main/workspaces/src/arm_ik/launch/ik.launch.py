from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_ik',
            executable='ik_node',
            name='arm_ik_node',
            output='screen',
            parameters=[{
                'l1': 0.080,
                'l2': 0.0755,
                'joint_names': ['arm_1_joint', 'arm_2_joint', 'gripper_joint'],
                'command_topic': '/arm_controller/commands',
                'joint_state_topic': 'joint_states',
                'target_topic': 'ik_target',
                'use_msg_z': True,
            }],
        ),
    ])
