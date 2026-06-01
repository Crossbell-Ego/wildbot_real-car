#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math
import sys

class FoxgloveLidarVisualizer(Node):
    def __init__(self, lidar_offset_deg=20.0):
        super().__init__('foxglove_lidar_visualizer')
        
        self.lidar_offset_deg = lidar_offset_deg  # 光達相對於車身中心的安裝偏角
        
        # 訂閱雷達 Topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # 發布 MarkerArray 到 Foxglove
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/diagnostics/lidar_orientation_markers',
            10
        )
        
        self.frame_id = 'laser'
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("📡 Foxglove 雷達方向視覺化節點已啟動！")
        self.get_logger().info(f"🔧 已套用光達安裝偏角修正: {self.lidar_offset_deg}°")
        self.get_logger().info("👉 已修正小車物理左右與雷達坐標系相反之問題。")
        self.get_logger().info("👉 請在 Foxglove 中開啟 [3D Panel] 並訂閱 `/diagnostics/lidar_orientation_markers`")
        self.get_logger().info("=" * 60)

    def scan_callback(self, msg):
        self.frame_id = msg.header.frame_id

    def create_arrow_marker(self, marker_id, angle_deg, text, color_rgb):
        """建立一個 3D 箭頭 Marker"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lidar_arrows"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # 箭頭起點與終點 (指向雷達外側)
        angle_rad = math.radians(angle_deg)
        start_pt = Point(x=0.0, y=0.0, z=0.0)
        end_pt = Point(x=0.6 * math.cos(angle_rad), y=0.6 * math.sin(angle_rad), z=0.0)
        
        marker.points = [start_pt, end_pt]
        
        # 尺寸
        marker.scale.x = 0.04
        marker.scale.y = 0.08
        marker.scale.z = 0.10
        
        # 顏色
        marker.color.r = float(color_rgb[0])
        marker.color.g = float(color_rgb[1])
        marker.color.b = float(color_rgb[2])
        marker.color.a = 0.9
        
        marker.lifetime = rclpy.duration.Duration(seconds=1.5).to_msg()
        return marker

    def create_text_marker(self, marker_id, angle_deg, text, color_rgb):
        """建立一個 3D 文字標籤 Marker"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lidar_texts"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # 文字位置 (放在箭頭外側，並稍微浮高 0.1m)
        angle_rad = math.radians(angle_deg)
        marker.pose.position.x = 0.75 * math.cos(angle_rad)
        marker.pose.position.y = 0.75 * math.sin(angle_rad)
        marker.pose.position.z = 0.1
        
        marker.scale.z = 0.12
        marker.text = text
        
        # 顏色
        marker.color.r = float(color_rgb[0])
        marker.color.g = float(color_rgb[1])
        marker.color.b = float(color_rgb[2])
        marker.color.a = 1.0
        
        marker.lifetime = rclpy.duration.Duration(seconds=1.5).to_msg()
        return marker

    def publish_markers(self):
        marker_array = MarkerArray()
        
        # 💡 已修正：在雷達座標系中，-90° 指向小車物理左側，90° 指向小車物理右側
        directions = [
            (0.0 - self.lidar_offset_deg, "REAR [車尾 0°]", (1.0, 0.2, 0.2)),      # 紅色
            (-90.0 - self.lidar_offset_deg, "LEFT [車左 90°]", (0.2, 1.0, 0.2)),    # 綠色 (指向雷達 -90)
            (90.0 - self.lidar_offset_deg, "RIGHT [車右 -90°]", (0.2, 0.5, 1.0)),   # 藍色 (指向雷達 90)
            (180.0 - self.lidar_offset_deg, "FRONT [車頭 180°]", (1.0, 1.0, 0.2))   # 黃色
        ]
        
        for i, (angle, label, color) in enumerate(directions):
            arrow = self.create_arrow_marker(i, angle, label, color)
            text = self.create_text_marker(i + 10, angle, label, color)
            
            marker_array.markers.append(arrow)
            marker_array.markers.append(text)
            
        self.marker_pub.publish(marker_array)

def main():
    lidar_offset_deg = 20.0
    if len(sys.argv) > 1:
        try:
            lidar_offset_deg = float(sys.argv[1])
        except ValueError:
            pass
            
    rclpy.init()
    node = FoxgloveLidarVisualizer(lidar_offset_deg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 視覺化節點已手動關閉。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
