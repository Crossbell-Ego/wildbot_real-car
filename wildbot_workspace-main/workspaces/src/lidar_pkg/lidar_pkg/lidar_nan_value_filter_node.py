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
        self.declare_parameter("outlier_filter_enabled", True)
        self.declare_parameter("outlier_max_neighbor_diff", 0.2)
        self.declare_parameter("outlier_window_size", 2)
        self.declare_parameter("outlier_min_neighbors", 1)

        self.use_inf_for_invalid = self.get_parameter("use_inf_for_invalid").value
        self.replace_nan = self.get_parameter("replace_nan").value
        self.replace_zero = self.get_parameter("replace_zero").value
        self.self_mask_enabled = self.get_parameter("self_mask_enabled").value
        self.self_mask_sectors = self._parse_sectors(
            self.get_parameter("self_mask_sectors").value
        )
        self.outlier_filter_enabled = self.get_parameter("outlier_filter_enabled").value
        self.outlier_max_neighbor_diff = self.get_parameter("outlier_max_neighbor_diff").value
        self.outlier_window_size = self.get_parameter("outlier_window_size").value
        self.outlier_min_neighbors = self.get_parameter("outlier_min_neighbors").value

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
            f"self mask sectors: {self.self_mask_sectors}; "
            f"outlier filter: {self.outlier_filter_enabled} (diff={self.outlier_max_neighbor_diff}, window={self.outlier_window_size}, min={self.outlier_min_neighbors})"
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

        if self.outlier_filter_enabled:
            ranges = self._filter_outliers(ranges)

        filtered_msg.ranges = ranges
        self.publisher.publish(filtered_msg)

    def _filter_outliers(self, ranges):
        n = len(ranges)
        filtered_ranges = list(ranges)
        invalid_value = math.inf if self.use_inf_for_invalid else 0.0

        K = self.outlier_window_size
        max_diff = self.outlier_max_neighbor_diff
        min_support = self.outlier_min_neighbors

        for i in range(n):
            val = ranges[i]
            if not math.isfinite(val) or val <= 0.0 or val == invalid_value:
                continue

            support = 0
            for offset in range(-K, K + 1):
                if offset == 0:
                    continue
                neighbor_idx = (i + offset) % n
                neighbor_val = ranges[neighbor_idx]
                if math.isfinite(neighbor_val) and neighbor_val > 0.0 and neighbor_val != invalid_value:
                    if abs(val - neighbor_val) <= max_diff:
                        support += 1
                        if support >= min_support:
                            break

            if support < min_support:
                filtered_ranges[i] = invalid_value

        return filtered_ranges

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
