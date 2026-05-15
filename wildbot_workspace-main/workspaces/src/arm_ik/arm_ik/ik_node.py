import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def clamp(value, lower, upper):
    return min(upper, max(lower, value))


def deg_to_rad(degrees):
    return math.radians(degrees)


def rad_to_deg(radians):
    return math.degrees(radians)


class IKNode(Node):
    def __init__(self):
        super().__init__('arm_ik_node')
        # Defaults come from kros_car_persistent.xacro:
        # shoulder -> elbow ~= 0.080 m, elbow -> gripper center ~= 0.0755 m.
        self.declare_parameter('l1', 0.080)
        self.declare_parameter('l2', 0.0755)
        self.declare_parameter('joint_names', ['arm_1_joint', 'arm_2_joint', 'gripper_joint'])
        self.declare_parameter('command_topic', '/arm_controller/commands')
        self.declare_parameter('joint_state_topic', 'joint_states')
        self.declare_parameter('target_topic', 'ik_target')
        self.declare_parameter('use_msg_z', True)
        self.declare_parameter('elbow_up', False)
        self.declare_parameter('base_x', 0.0)
        self.declare_parameter('base_z', 0.0)
        self.declare_parameter('shoulder_offset', 0.0)
        self.declare_parameter('elbow_offset', 0.0)
        self.declare_parameter('shoulder_direction', 1.0)
        self.declare_parameter('elbow_direction', 1.0)
        self.declare_parameter('shoulder_min', deg_to_rad(30.0))
        self.declare_parameter('shoulder_max', deg_to_rad(210.0))
        self.declare_parameter('elbow_min', deg_to_rad(0.0))
        self.declare_parameter('elbow_max', deg_to_rad(240.0))
        self.declare_parameter('gripper_min', deg_to_rad(168.0))
        self.declare_parameter('gripper_max', deg_to_rad(240.0))
        self.declare_parameter('gripper_position', deg_to_rad(240.0))
        self.declare_parameter('auto_release_delay_sec', 0.5)
        self.declare_parameter('auto_release_delta', deg_to_rad(2.0))
        self.declare_parameter('min_target_z', 0.0)
        self.declare_parameter('publish_joint_states', True)
        self.declare_parameter('publish_position_command', True)

        self.l1 = float(self.get_parameter('l1').value)
        self.l2 = float(self.get_parameter('l2').value)
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.use_msg_z = bool(self.get_parameter('use_msg_z').value)
        self.elbow_up = bool(self.get_parameter('elbow_up').value)
        self.base_x = float(self.get_parameter('base_x').value)
        self.base_z = float(self.get_parameter('base_z').value)
        self.shoulder_offset = float(self.get_parameter('shoulder_offset').value)
        self.elbow_offset = float(self.get_parameter('elbow_offset').value)
        self.shoulder_direction = float(self.get_parameter('shoulder_direction').value)
        self.elbow_direction = float(self.get_parameter('elbow_direction').value)
        self.shoulder_min = float(self.get_parameter('shoulder_min').value)
        self.shoulder_max = float(self.get_parameter('shoulder_max').value)
        self.elbow_min = float(self.get_parameter('elbow_min').value)
        self.elbow_max = float(self.get_parameter('elbow_max').value)
        self.gripper_min = float(self.get_parameter('gripper_min').value)
        self.gripper_max = float(self.get_parameter('gripper_max').value)
        self.gripper_position = float(self.get_parameter('gripper_position').value)
        self.auto_release_delay_sec = float(self.get_parameter('auto_release_delay_sec').value)
        self.auto_release_delta = float(self.get_parameter('auto_release_delta').value)
        self.min_target_z = float(self.get_parameter('min_target_z').value)
        self.publish_joint_states = bool(self.get_parameter('publish_joint_states').value)
        self.publish_position_command = bool(self.get_parameter('publish_position_command').value)
        self.release_timer = None
        self.last_positions = None

        if len(self.joint_names) < 2:
            raise ValueError('joint_names must contain at least shoulder and elbow joints')
        if self.gripper_position < self.gripper_min:
            self.get_logger().warn(
                'gripper_position %.1f deg is unsafe; clamped to %.1f deg'
                % (rad_to_deg(self.gripper_position), rad_to_deg(self.gripper_min))
            )
            self.gripper_position = self.gripper_min
        if self.gripper_position > self.gripper_max:
            self.get_logger().warn(
                'gripper_position %.1f deg exceeds full-open limit; clamped to %.1f deg'
                % (rad_to_deg(self.gripper_position), rad_to_deg(self.gripper_max))
            )
            self.gripper_position = self.gripper_max
        if self.auto_release_delta < 0.0:
            self.get_logger().warn('auto_release_delta was negative; using absolute value')
            self.auto_release_delta = abs(self.auto_release_delta)

        self.joint_state_pub = None
        self.command_pub = None
        if self.publish_joint_states:
            self.joint_state_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        if self.publish_position_command:
            self.command_pub = self.create_publisher(Float64MultiArray, self.command_topic, 10)

        self.sub = self.create_subscription(Point, self.target_topic, self.ik_callback, 10)
        self.get_logger().info(
            'arm_ik_node started: joints=%s l1=%.4f l2=%.4f target=%s command=%s'
            % (self.joint_names, self.l1, self.l2, self.target_topic, self.command_topic)
        )
        self.get_logger().info(
            'safety limits: arm_1=[30, 210] deg arm_2=[0, 240] deg gripper=[168, 240] deg'
        )

    def ik_callback(self, msg: Point):
        # The physical arm is a 2-DOF pitch-pitch chain, so solve in the X-Z
        # plane. For the old demo topic, z=0 and y can still be used as height.
        target_x = msg.x
        target_z = msg.z if self.use_msg_z or abs(msg.z) > 1e-9 else msg.y
        if target_z < self.min_target_z:
            self.get_logger().warn(
                'Target z %.3f is below floor safety limit %.3f; command rejected'
                % (target_z, self.min_target_z)
            )
            return

        dx = target_x - self.base_x
        dz = target_z - self.base_z
        dist = math.hypot(dx, dz)
        if dist > (self.l1 + self.l2) or dist < abs(self.l1 - self.l2):
            self.get_logger().warn(
                'Target unreachable: target=(%.3f, %.3f) arm_frame=(%.3f, %.3f) reach=[%.3f, %.3f]'
                % (target_x, target_z, dx, dz, abs(self.l1 - self.l2), self.l1 + self.l2)
            )
            return

        cos_q2 = (dx*dx + dz*dz - self.l1*self.l1 - self.l2*self.l2) / (2 * self.l1 * self.l2)
        cos_q2 = clamp(cos_q2, -1.0, 1.0)
        q2 = math.acos(cos_q2)
        if self.elbow_up:
            q2 = -q2

        k1 = self.l1 + self.l2 * math.cos(q2)
        k2 = self.l2 * math.sin(q2)
        q1 = math.atan2(dz, dx) - math.atan2(k2, k1)
        elbow_z = self.base_z + self.l1 * math.sin(q1)
        if elbow_z < self.min_target_z:
            self.get_logger().warn(
                'Elbow z %.3f is below floor safety limit %.3f; command rejected'
                % (elbow_z, self.min_target_z)
            )
            return

        shoulder = self.shoulder_direction * q1 + self.shoulder_offset
        elbow = self.elbow_direction * q2 + self.elbow_offset

        positions = [shoulder, elbow]
        if len(self.joint_names) > 2:
            positions.append(self.gripper_position)

        if not self.validate_safety_limits(positions):
            return

        self.publish_positions(positions)
        self.schedule_gripper_release(positions)

        self.get_logger().info(
            'IK target=(%.3f, %.3f) -> %s'
            % (target_x, target_z, ', '.join('%.3f' % value for value in positions))
        )

    def validate_safety_limits(self, positions):
        shoulder = positions[0]
        elbow = positions[1]
        if not (self.shoulder_min <= shoulder <= self.shoulder_max):
            self.get_logger().warn(
                'arm_1_joint %.1f deg outside safety limits [30.0, 210.0]; command rejected'
                % rad_to_deg(shoulder)
            )
            return False
        if not (self.elbow_min <= elbow <= self.elbow_max):
            self.get_logger().warn(
                'arm_2_joint %.1f deg outside safety limits [0.0, 240.0]; command rejected'
                % rad_to_deg(elbow)
            )
            return False
        if len(positions) > 2:
            gripper = positions[2]
            if gripper < self.gripper_min:
                self.get_logger().error(
                    'gripper_joint %.1f deg is below 168.0 deg. Over-clamping risk; command rejected'
                    % rad_to_deg(gripper)
                )
                return False
            if gripper > self.gripper_max:
                self.get_logger().warn(
                    'gripper_joint %.1f deg exceeds full-open limit 240.0 deg; command rejected'
                    % rad_to_deg(gripper)
                )
                return False
        return True

    def publish_positions(self, positions):
        self.last_positions = list(positions)
        if self.joint_state_pub is not None:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.joint_names[:len(positions)]
            js.position = positions
            self.joint_state_pub.publish(js)

        if self.command_pub is not None:
            command = Float64MultiArray()
            command.data = positions
            self.command_pub.publish(command)

    def schedule_gripper_release(self, positions):
        if len(positions) <= 2:
            return
        if positions[2] >= self.gripper_max:
            return
        if self.release_timer is not None:
            self.release_timer.cancel()
            self.destroy_timer(self.release_timer)
            self.release_timer = None
        self.release_timer = self.create_timer(self.auto_release_delay_sec, self.auto_release_gripper)

    def auto_release_gripper(self):
        if self.release_timer is not None:
            self.release_timer.cancel()
            self.destroy_timer(self.release_timer)
            self.release_timer = None
        if self.last_positions is None or len(self.last_positions) <= 2:
            return
        released_positions = list(self.last_positions)
        released_positions[2] = min(self.gripper_max, released_positions[2] + self.auto_release_delta)
        if not self.validate_safety_limits(released_positions):
            return
        self.publish_positions(released_positions)
        self.get_logger().info(
            'Auto-release gripper by %.1f deg -> %.1f deg'
            % (rad_to_deg(self.auto_release_delta), rad_to_deg(released_positions[2]))
        )


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
