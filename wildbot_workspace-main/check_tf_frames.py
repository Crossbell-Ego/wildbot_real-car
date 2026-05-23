#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import time

class TFInspector(Node):
    def __init__(self):
        super().__init__('tf_inspector')
        self.frames = {}
        self.sub_tf = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.sub_tf_static = self.create_subscription(TFMessage, '/tf_static', self.tf_static_callback, 10)

    def tf_callback(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            self.frames[child] = (parent, "dynamic")

    def tf_static_callback(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            self.frames[child] = (parent, "static")

def main():
    rclpy.init()
    node = TFInspector()
    print("Collecting TF frames for 3 seconds...")
    start_time = time.time()
    while time.time() - start_time < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    print("\n=== Detected TF Frames ===")
    for child, (parent, tf_type) in sorted(node.frames.items()):
        print(f"Child: {child:20} -> Parent: {parent:20} ({tf_type})")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
