import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarNanValueFilterNode(Node):
    def __init__(self):
        super().__init__("lidar_nan_value_filter_node")

        self.declare_parameter("input_topic", "/scan_tmp")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("use_inf_for_invalid", True)
        self.declare_parameter("replace_nan", True)
        self.declare_parameter("replace_zero", True)
        self.declare_parameter("self_mask_enabled", True)
        self.declare_parameter(
            "self_mask_sectors",
            [
                "-45:45:0.55",
                "135:180:0.45",
                "-180:-135:0.45",
            ],
        )

        self.use_inf_for_invalid = self.get_parameter("use_inf_for_invalid").value
        self.replace_nan = self.get_parameter("replace_nan").value
        self.replace_zero = self.get_parameter("replace_zero").value
        self.self_mask_enabled = self.get_parameter("self_mask_enabled").value
        self.self_mask_sectors = self._parse_sectors(
            self.get_parameter("self_mask_sectors").value
        )

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self._scan_callback,
            10,
        )

        self.get_logger().info(
            f"Filtering LaserScan {input_topic} -> {output_topic}; "
            f"self mask sectors: {self.self_mask_sectors}"
        )

    def _parse_sectors(self, sector_specs):
        sectors = []
        for spec in sector_specs:
            try:
                start_deg, end_deg, max_range_m = [float(part) for part in spec.split(":")]
            except (AttributeError, ValueError):
                self.get_logger().warning(
                    f"Ignoring invalid self_mask_sectors entry: {spec!r}. "
                    "Expected 'start_deg:end_deg:max_range_m'."
                )
                continue

            sectors.append(
                (
                    self._normalize_deg(start_deg),
                    self._normalize_deg(end_deg),
                    max_range_m,
                )
            )
        return sectors

    def _scan_callback(self, scan_msg):
        filtered_msg = LaserScan()
        filtered_msg.header = scan_msg.header
        filtered_msg.angle_min = scan_msg.angle_min
        filtered_msg.angle_max = scan_msg.angle_max
        filtered_msg.angle_increment = scan_msg.angle_increment
        filtered_msg.time_increment = scan_msg.time_increment
        filtered_msg.scan_time = scan_msg.scan_time
        filtered_msg.range_min = scan_msg.range_min
        filtered_msg.range_max = scan_msg.range_max
        filtered_msg.intensities = list(scan_msg.intensities)

        invalid_value = math.inf if self.use_inf_for_invalid else 0.0
        ranges = []

        for index, value in enumerate(scan_msg.ranges):
            angle_rad = scan_msg.angle_min + index * scan_msg.angle_increment
            angle_deg = self._normalize_deg(math.degrees(angle_rad))

            if self._should_replace_invalid(value, scan_msg):
                ranges.append(invalid_value)
            elif self.self_mask_enabled and self._inside_self_mask(angle_deg, value):
                ranges.append(invalid_value)
            else:
                ranges.append(value)

        filtered_msg.ranges = ranges
        self.publisher.publish(filtered_msg)

    def _should_replace_invalid(self, value, scan_msg):
        if self.replace_nan and not math.isfinite(value):
            return True
        if self.replace_zero and value <= 0.0:
            return True
        return value < scan_msg.range_min or value > scan_msg.range_max

    def _inside_self_mask(self, angle_deg, range_m):
        for start_deg, end_deg, max_range_m in self.self_mask_sectors:
            if range_m <= max_range_m and self._angle_in_sector(
                angle_deg,
                start_deg,
                end_deg,
            ):
                return True
        return False

    @staticmethod
    def _normalize_deg(angle_deg):
        return ((angle_deg + 180.0) % 360.0) - 180.0

    @staticmethod
    def _angle_in_sector(angle_deg, start_deg, end_deg):
        if start_deg <= end_deg:
            return start_deg <= angle_deg <= end_deg
        return angle_deg >= start_deg or angle_deg <= end_deg


def main(args=None):
    rclpy.init(args=args)
    node = LidarNanValueFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
