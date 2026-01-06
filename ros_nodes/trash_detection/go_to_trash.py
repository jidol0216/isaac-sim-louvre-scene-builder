#!/usr/bin/env python3
"""쓰레기 감지 시 가장 가까운 쓰레기로 직접 돌진 (cmd_vel 사용)"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
import random
from ultralytics import YOLO
import os


def imgmsg_to_cv2(msg):
    dtype = np.uint8
    if msg.encoding == 'rgb8':
        channels = 3
    elif msg.encoding == 'bgr8':
        channels = 3
    elif msg.encoding == '32FC1':
        dtype = np.float32
        channels = 1
    else:
        channels = 1 if msg.encoding == 'mono8' else 3
    
    img = np.frombuffer(msg.data, dtype=dtype)
    if channels == 1 and dtype == np.float32:
        img = img.reshape(msg.height, msg.width)
    else:
        img = img.reshape(msg.height, msg.width, channels)
    return img


class GoToTrash(Node):
    def __init__(self):
        super().__init__('go_to_trash')
        self.get_logger().info('=== Go To Trash Node Starting ===')
        
        self.declare_parameter('detection_confidence', 0.7)
        self.declare_parameter('camera_fov_h', 69.4)
        self.declare_parameter('min_y_ratio', 0.3)
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        
        self.confidence_threshold = self.get_parameter('detection_confidence').value
        self.camera_fov_h = self.get_parameter('camera_fov_h').value
        self.min_y_ratio = self.get_parameter('min_y_ratio').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        self.current_pose = None
        self.current_yaw = 0.0
        self.is_charging = False
        self.is_exploring = False
        self.no_detection_count = 0
        self.explore_threshold = 20
        
        # Target tracking
        self.target_distance = None
        self.target_angle_offset = None
        self.charge_start_time = None
        
        self.rgb_sub = self.create_subscription(Image, '/rgb', self.cb_rgb, 10)
        self.depth_sub = self.create_subscription(Image, '/depth', self.cb_depth, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.latest_depth = None
        self.image_width = 640
        self.image_height = 480
        
        model_path = os.path.join(os.path.dirname(__file__), 'trash1.pt')
        if not os.path.exists(model_path):
            model_path = os.path.join(os.path.dirname(__file__), 'trash.pt')
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('=== Ready to detect and CHARGE at trash (cmd_vel) ===')
        
        # Control loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        """Main control loop - handles charging and exploration"""
        now = self.get_clock().now().nanoseconds / 1e9
        
        if self.is_charging:
            # Check if we've been charging too long (timeout)
            if self.charge_start_time and (now - self.charge_start_time) > 10.0:
                self.stop_charging()
                self.get_logger().info('⏱️ Charge timeout, ready for new target')
            return
        
        if self.is_exploring:
            return
        
        # Start exploration if no detection for a while
        if self.no_detection_count >= self.explore_threshold:
            self.start_exploration()

    def cb_odom(self, msg):
        self.current_pose = msg.pose.pose
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

    def cb_depth(self, msg):
        self.latest_depth = imgmsg_to_cv2(msg)
        self.image_width = msg.width
        self.image_height = msg.height

    def get_depth_at_bbox(self, bbox):
        if self.latest_depth is None:
            return None
        xmin, ymin, xmax, ymax = bbox
        cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
        roi = 15
        x1, x2 = max(0, int(cx - roi)), min(self.image_width, int(cx + roi))
        y1, y2 = max(0, int(cy - roi)), min(self.image_height, int(cy + roi))
        depth_roi = self.latest_depth[y1:y2, x1:x2]
        valid = depth_roi[depth_roi > 0.1]
        return float(np.median(valid)) if len(valid) > 0 else None

    def start_exploration(self):
        if self.current_pose is None:
            return
        self.is_exploring = True
        self.get_logger().info('🔍 Exploring...')
        twist = Twist()
        twist.angular.z = random.choice([-0.5, 0.5])
        twist.linear.x = 0.3
        self.cmd_vel_pub.publish(twist)
        
        # Explore for 2 seconds then stop
        self.create_timer(2.0, self.stop_exploration_once)

    def stop_exploration_once(self):
        if self.is_exploring:
            self.stop_movement()
            self.is_exploring = False
            self.no_detection_count = 0
            self.get_logger().info('🔍 Exploration done')
        self.destroy_timer

    def stop_movement(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def stop_charging(self):
        self.stop_movement()
        self.is_charging = False
        self.target_distance = None
        self.target_angle_offset = None
        self.charge_start_time = None

    def cb_rgb(self, msg):
        if self.is_exploring:
            return
        
        if self.latest_depth is None:
            return
        
        cv_img = imgmsg_to_cv2(msg)
        results = self.model(cv_img, verbose=False)
        
        min_y_pixel = self.image_height * self.min_y_ratio
        
        detections = []
        for result in results:
            if result.boxes:
                for box in result.boxes:
                    conf = float(box.conf[0].cpu().numpy())
                    if conf > self.confidence_threshold:
                        xyxy = box.xyxy[0].cpu().numpy()
                        ymin = xyxy[1]
                        
                        if ymin < min_y_pixel:
                            continue
                        
                        cls_id = int(box.cls[0].cpu().numpy())
                        dist = self.get_depth_at_bbox(xyxy)
                        if dist and dist < 10.0:
                            cx = (xyxy[0] + xyxy[2]) / 2
                            detections.append({
                                'bbox': xyxy, 'conf': conf,
                                'class': self.model.names[cls_id], 
                                'distance': dist,
                                'cx': cx
                            })
        
        if not detections:
            self.no_detection_count += 1
            if self.is_charging:
                # Lost target while charging - keep going briefly
                pass
            return
        
        self.no_detection_count = 0
        
        # Find closest trash
        closest = min(detections, key=lambda d: d['distance'])
        
        # Calculate angle offset from image center
        angle_offset = (closest['cx'] - self.image_width / 2) * math.radians(self.camera_fov_h) / self.image_width
        
        self.target_distance = closest['distance']
        self.target_angle_offset = angle_offset
        
        self.get_logger().info(f'📊 {len(detections)} trash, closest: {closest["class"]} @ {closest["distance"]:.2f}m (angle: {math.degrees(angle_offset):.1f}°)')
        
        # Charge at trash!
        self.charge_at_target(closest)

    def charge_at_target(self, det):
        """Directly drive towards trash using cmd_vel"""
        if self.target_distance is None:
            return
        
        twist = Twist()
        
        # Proportional control for angular velocity
        # Turn towards trash
        angle_error = self.target_angle_offset
        twist.angular.z = float(-self.angular_speed * angle_error * 2.0)  # Negative because positive angle = turn left
        
        # Clamp angular velocity
        twist.angular.z = float(max(-1.5, min(1.5, twist.angular.z)))
        
        # Linear velocity based on distance
        if abs(angle_error) < 0.3:  # Within ~17 degrees, go forward
            # Speed proportional to distance, but max out
            twist.linear.x = float(min(self.linear_speed, self.target_distance * 0.3))
            twist.linear.x = float(max(0.2, twist.linear.x))  # Minimum speed
        else:
            # Turn in place first
            twist.linear.x = 0.1
        
        # Stop when very close
        if self.target_distance < 0.3:
            self.get_logger().info(f'🎯 REACHED {det["class"]}!')
            self.stop_charging()
            return
        
        self.cmd_vel_pub.publish(twist)
        
        if not self.is_charging:
            self.is_charging = True
            self.charge_start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f'🚀 CHARGING at {det["class"]} @ {self.target_distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = GoToTrash()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
