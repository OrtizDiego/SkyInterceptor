#!/usr/bin/env python3

"""
@file target_detector.py
@brief YOLOv8 target detection node for Interceptor System
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from interceptor_interfaces.msg import TargetDetection
import time

class TargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')
        
        self.get_logger().info("============================================")
        self.get_logger().info("YOLO Target Detector Starting...")
        self.get_logger().info("============================================")
        
        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('device', 'cpu') 
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        device = self.get_parameter('device').get_parameter_value().string_value
        
        # Initialize YOLO
        self.model = YOLO(model_path)
        self.model.to(device)
        
        self.bridge = CvBridge()
        
        # Class mapping for Interceptor System
        # YOLO COCO: 0: person, 2: car, 7: truck
        self.target_classes = {0: 0, 2: 1, 7: 2} 
        self.class_names = {0: 'person', 1: 'car', 2: 'truck'}
        
        # Subscribers
        self.subscription = self.create_subscription(
            Image,
            '/stereo/left/image_raw',
            self.image_callback,
            10)
            
        # Publishers
        self.publisher = self.create_publisher(TargetDetection, '/target/detection_2d', 10)
        self.viz_publisher = self.create_publisher(Image, '/target/detection_viz', 10)
        
        self.get_logger().info(f"YOLOv8 initialized on {device}")
        self.get_logger().info("Monitoring for: person, car, truck")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run Inference
        results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                
                # Filter classes
                if cls_id in self.target_classes:
                    interceptor_cls = self.target_classes[cls_id]
                    conf = float(box.conf[0])
                    
                    # Get coordinates
                    b = box.xywh[0] 
                    
                    # Create Detection Message
                    det_msg = TargetDetection()
                    det_msg.header = msg.header
                    det_msg.class_id = interceptor_cls
                    det_msg.class_name = self.class_names[interceptor_cls]
                    det_msg.confidence = conf
                    
                    # Bounding Box (top-left x, top-left y, w, h)
                    det_msg.bbox_x = int(b[0] - b[2]/2)
                    det_msg.bbox_y = int(b[1] - b[3]/2)
                    det_msg.bbox_width = int(b[2])
                    det_msg.bbox_height = int(b[3])
                    
                    det_msg.world_position_valid = False
                    
                    self.publisher.publish(det_msg)
                    
            # Visualization (published only if there are subscribers)
            if self.viz_publisher.get_subscription_count() > 0:
                annotated_frame = result.plot()
                viz_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                viz_msg.header = msg.header
                self.viz_publisher.publish(viz_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TargetDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
