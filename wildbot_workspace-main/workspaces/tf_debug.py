import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

def main():
    rclpy.init()
    node = Node('tf_debugger')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    
    print("⏳ 正在收集 TF 坐標轉換，請稍候 2 秒...")
    start = time.time()
    while time.time() - start < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        
    print("\n========= TF Buffer 內的所有 Frames =========")
    print(buffer.all_frames_as_yaml())
    print("=============================================")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
