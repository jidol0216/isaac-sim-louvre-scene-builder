#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from ultralytics import YOLO
import os

def imgmsg_to_cv2(msg):
    """sensor_msgs/Image를 OpenCV 이미지로 변환"""
    dtype = np.uint8
    if msg.encoding == 'rgb8':
        channels = 3
    elif msg.encoding == 'bgr8':
        channels = 3
    elif msg.encoding == 'rgba8' or msg.encoding == 'bgra8':
        channels = 4
    elif msg.encoding == 'mono8':
        channels = 1
    else:
        channels = 3  # default
    
    img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
    return img

def cv2_to_imgmsg(cv_img, encoding='rgb8'):
    """OpenCV 이미지를 sensor_msgs/Image로 변환"""
    msg = Image()
    msg.height = cv_img.shape[0]
    msg.width = cv_img.shape[1]
    msg.encoding = encoding
    msg.step = cv_img.shape[1] * cv_img.shape[2] if len(cv_img.shape) == 3 else cv_img.shape[1]
    msg.data = cv_img.tobytes()
    return msg

class TrashDetector(Node):
    def __init__(self):
        super().__init__('trash_detection')
        print('[DEBUG] Node initialized')
        self.get_logger().info('=== Trash Detection Node Starting ===')
        
        self.sub = self.create_subscription(Image, '/rgb', self.cb_image, 10)
        self.pub = self.create_publisher(Image, '/trash_detections', 10)
        self.get_logger().info('Subscription and Publisher created')
        
        # YOLO 모델 로드
        model_path = os.path.join(os.path.dirname(__file__), 'trash.pt')
        if not os.path.exists(model_path):
            model_path = os.path.join(os.path.dirname(__file__), 'trash1.pt')
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        print(f'[DEBUG] Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('YOLO model loaded successfully!')
        print('[DEBUG] YOLO model loaded successfully!')
        self.get_logger().info('=== Ready to detect trash ===')
        
        self.img_height = 480  # 기본값

    def infer(self, img):
        """YOLO 추론 수행"""
        results = self.model(img, verbose=False)
        detections = []
        min_y_pixel = self.img_height * 0.3  # 상위 30% 무시
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    conf = float(box.conf[0].cpu().numpy())
                    if conf < 0.7:
                        continue
                    xyxy = box.xyxy[0].cpu().numpy()
                    # 상위 30% 영역 무시
                    if xyxy[1] < min_y_pixel:
                        continue
                    cls_id = int(box.cls[0].cpu().numpy())
                    cls_name = self.model.names[cls_id]
                    detections.append((xyxy[0], xyxy[1], xyxy[2], xyxy[3], conf, cls_name))
        
        return detections

    def cb_image(self, msg):
        self.get_logger().info('Image received!')
        self.img_height = msg.height
        cv_img = imgmsg_to_cv2(msg)
        dets = self.infer(cv_img)
        self.get_logger().info(f'Detected {len(dets)} objects')
        vis = cv_img.copy()
        for (xmin,ymin,xmax,ymax,score,cls) in dets:
            cv2.rectangle(vis, (int(xmin),int(ymin)), (int(xmax),int(ymax)), (0,255,0), 2)
            cv2.putText(vis, f'{cls}:{score:.2f}', (int(xmin),int(ymin)-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1)
        # 화면에 표시 (RGB -> BGR 변환)
        vis_bgr = cv2.cvtColor(vis, cv2.COLOR_RGB2BGR)
        cv2.imshow('Trash Detection', vis_bgr)
        cv2.waitKey(1)
        
        out_msg = cv2_to_imgmsg(vis, 'rgb8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)

def main(args=None):
    print('[DEBUG] Starting main function')
    rclpy.init(args=args)
    print('[DEBUG] rclpy initialized')
    node = TrashDetector()
    print('[DEBUG] TrashDetector node created')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
