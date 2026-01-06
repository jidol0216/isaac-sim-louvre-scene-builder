#!/usr/bin/env python3
"""
Depth Image Republisher
32FC1 (float meters) -> 16UC1 (uint16 millimeters) 변환
RViz2, rqt_image_view에서 시각화 가능
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np


class DepthRepublisher(Node):
    def __init__(self):
        super().__init__('depth_republisher')
        
        # 파라미터
        self.declare_parameter('input_topic', '/depth')
        self.declare_parameter('output_topic', '/depth_image')
        self.declare_parameter('max_depth', 10.0)  # meters
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.max_depth = self.get_parameter('max_depth').value
        
        # Subscriber & Publisher
        self.sub = self.create_subscription(
            Image,
            input_topic,
            self.depth_callback,
            10
        )
        
        self.pub = self.create_publisher(Image, output_topic, 10)
        
        self.get_logger().info(f'Depth Republisher started: {input_topic} -> {output_topic}')
        self.get_logger().info(f'Max depth: {self.max_depth}m')

    def depth_callback(self, msg: Image):
        # 32FC1 -> numpy array
        if msg.encoding != '32FC1':
            self.get_logger().warn(f'Expected 32FC1, got {msg.encoding}')
            return
        
        # Convert to numpy
        depth_data = np.frombuffer(msg.data, dtype=np.float32)
        depth_data = depth_data.reshape((msg.height, msg.width))
        
        # Handle inf/nan values
        depth_data = np.nan_to_num(depth_data, nan=0.0, posinf=self.max_depth, neginf=0.0)
        
        # Clamp to max depth
        depth_data = np.clip(depth_data, 0.0, self.max_depth)
        
        # Convert to millimeters (16UC1)
        depth_mm = (depth_data * 1000.0).astype(np.uint16)
        
        # Create output message
        out_msg = Image()
        out_msg.header = msg.header
        out_msg.height = msg.height
        out_msg.width = msg.width
        out_msg.encoding = '16UC1'
        out_msg.is_bigendian = False
        out_msg.step = msg.width * 2  # 2 bytes per pixel
        out_msg.data = depth_mm.tobytes()
        
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
