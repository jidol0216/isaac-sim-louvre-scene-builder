#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserScanMerger(Node):
    def __init__(self):
        super().__init__('laser_scan_merger')
        
        self.front_scan = None
        self.rear_scan = None
        
        self.front_sub = self.create_subscription(
            LaserScan, '/front_scan', self.front_callback, 10)
        self.rear_sub = self.create_subscription(
            LaserScan, '/rear_scan', self.rear_callback, 10)
        
        self.merged_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        self.timer = self.create_timer(0.05, self.merge_and_publish)
        
        self.get_logger().info('Laser Scan Merger started')

    def front_callback(self, msg):
        self.front_scan = msg

    def rear_callback(self, msg):
        self.rear_scan = msg

    def merge_and_publish(self):
        if self.front_scan is None:
            return
            
        merged = LaserScan()
        merged.header.stamp = self.front_scan.header.stamp
        merged.header.frame_id = 'base_link'
        
        merged.angle_min = -math.pi
        merged.angle_max = math.pi
        merged.angle_increment = self.front_scan.angle_increment
        merged.time_increment = 0.0
        merged.scan_time = 0.05
        merged.range_min = self.front_scan.range_min
        merged.range_max = self.front_scan.range_max
        
        num_readings = int((merged.angle_max - merged.angle_min) / merged.angle_increment)
        ranges = [float('inf')] * num_readings
        
        if self.front_scan is not None:
            for i, r in enumerate(self.front_scan.ranges):
                angle = self.front_scan.angle_min + i * self.front_scan.angle_increment
                idx = int((angle - merged.angle_min) / merged.angle_increment)
                if 0 <= idx < num_readings and not math.isinf(r) and not math.isnan(r):
                    if r < ranges[idx]:
                        ranges[idx] = r
        
        if self.rear_scan is not None:
            for i, r in enumerate(self.rear_scan.ranges):
                angle = self.rear_scan.angle_min + i * self.rear_scan.angle_increment + math.pi
                if angle > math.pi:
                    angle -= 2 * math.pi
                idx = int((angle - merged.angle_min) / merged.angle_increment)
                if 0 <= idx < num_readings and not math.isinf(r) and not math.isnan(r):
                    if r < ranges[idx]:
                        ranges[idx] = r
        
        merged.ranges = ranges
        merged.intensities = []
        
        self.merged_pub.publish(merged)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
