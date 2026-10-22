#!/usr/bin/env python3

"""
YOLOv8 target detection node for the Interceptor System.

The node runs YOLOv8 on the stereo left image and publishes one
TargetDetection per detected object on /target/detection_2d. The 3D
localizer pairs those detections with the depth map and publishes
/target/detection_3d.

The image is decoded with NumPy instead of cv_bridge. The ROS 2 Humble
cv_bridge Python binding is compiled against the NumPy 1.x C API, so
importing it with NumPy 2.x installed raises "AttributeError: _ARRAY_API
not found" and then segfaults, which killed this node on the way in. A
sensor_msgs/Image is a plain byte buffer, so decoding it here works with
NumPy 1 and NumPy 2 alike and removes the dependency on the binding. The
C++ cv_bridge used by the depth processor and the localizer is unaffected
by this and is left alone.
"""

import time

import numpy as np
import rclpy
import torch
from interceptor_interfaces.msg import TargetDetection
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

# sensor_msgs/Image encoding -> (number of channels, the bytes are in RGB order)
# Only the encodings the stereo camera can produce are listed.
IMAGE_ENCODINGS = {
    'bgr8': (3, False),
    'rgb8': (3, True),
    'bgra8': (4, False),
    'rgba8': (4, True),
    'mono8': (1, False),
}

# COCO class id -> class name, for the classes the detector can report.
COCO_CLASS_NAMES = {
    0: 'person',
    1: 'bicycle',
    2: 'car',
    7: 'truck',
}

# Class name -> TargetDetection constant. The names are the ones the tracker
# uses in follow.eligible_classes and intercept.eligible_classes, so the
# detector and the tracker agree on the ids.
CLASS_IDS = {
    'person': TargetDetection.PERSON,
    'bicycle': TargetDetection.BICYCLE,
    'car': TargetDetection.CAR,
    'truck': TargetDetection.TRUCK,
    'uav': TargetDetection.UAV,
}


def decode_bgr8(msg):
    """
    Decode a sensor_msgs/Image into an (H, W, 3) uint8 BGR array.

    The bytes are read with NumPy and the row padding of the message is
    dropped, so the result is what OpenCV and YOLO expect.
    """
    encoding = msg.encoding.lower()
    if encoding not in IMAGE_ENCODINGS:
        raise ValueError('unsupported image encoding: ' + msg.encoding)
    channels, rgb_order = IMAGE_ENCODINGS[encoding]

    row_bytes = msg.width * channels
    if msg.step < row_bytes:
        raise ValueError('image step is smaller than one row of pixels')

    data = np.frombuffer(msg.data, dtype=np.uint8)
    needed = msg.height * msg.step
    if data.size < needed:
        raise ValueError('image data is shorter than height * step')

    rows = data[:needed].reshape(msg.height, msg.step)
    pixels = rows[:, :row_bytes].reshape(msg.height, msg.width, channels)

    if rgb_order:
        if channels == 3:
            pixels = pixels[:, :, [2, 1, 0]]
        else:
            pixels = pixels[:, :, [2, 1, 0, 3]]
    if channels == 4:
        pixels = pixels[:, :, :3]
    elif channels == 1:
        pixels = np.repeat(pixels, 3, axis=2)

    return np.ascontiguousarray(pixels, dtype=np.uint8)


def encode_bgr8(image, header):
    """Wrap an (H, W, 3) uint8 BGR array into a sensor_msgs/Image."""
    height, width = image.shape[:2]
    message = Image()
    message.header = header
    message.height = height
    message.width = width
    message.encoding = 'bgr8'
    message.is_bigendian = 0
    message.step = width * 3
    message.data = np.ascontiguousarray(image, dtype=np.uint8).tobytes()
    return message


class TargetDetector(Node):
    """Runs YOLOv8 on the stereo left image and publishes 2D detections."""

    def __init__(self):
        super().__init__('target_detector')

        self.get_logger().info("============================================")
        self.get_logger().info("YOLO Target Detector Starting...")
        self.get_logger().info("============================================")

        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.3)
        # 'auto' picks the first CUDA GPU when PyTorch can see one, else the CPU
        self.declare_parameter('device', 'auto')
        # Classes the detector reports. The names match the class whitelists
        # of the tracker (follow.eligible_classes, intercept.eligible_classes).
        self.declare_parameter('target_classes', ['person', 'bicycle', 'car', 'truck'])

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = (
            self.get_parameter('confidence_threshold').get_parameter_value().double_value)
        requested_device = (
            self.get_parameter('device').get_parameter_value().string_value)
        wanted_classes = list(
            self.get_parameter('target_classes').get_parameter_value().string_array_value)

        self.class_names = {}
        for name in wanted_classes:
            if name not in CLASS_IDS:
                self.get_logger().warn(f"Unknown target class '{name}', ignored")
                continue
            self.class_names[CLASS_IDS[name]] = name
        if not self.class_names:
            raise ValueError('no valid target class requested')

        # COCO class id -> TargetDetection constant, for the wanted classes only
        self.target_classes = {}
        for coco_id, name in COCO_CLASS_NAMES.items():
            class_id = CLASS_IDS[name]
            if class_id in self.class_names:
                self.target_classes[coco_id] = class_id

        self.device = self.resolve_device(requested_device)

        # Initialize YOLO
        self.model = YOLO(model_path)
        self.model.to(self.device)

        # Subscribers
        self.subscription = self.create_subscription(
            Image,
            '/stereo/left/image_raw',
            self.image_callback,
            10)

        # Publishers
        self.publisher = self.create_publisher(TargetDetection, '/target/detection_2d', 10)
        self.viz_publisher = self.create_publisher(Image, '/target/detection_viz', 10)

        self.last_log_time_ = 0.0

        self.get_logger().info(f"YOLOv8 initialized on {self.device}")
        self.get_logger().info(
            "Monitoring for: " + ", ".join(sorted(self.class_names.values())))

    def resolve_device(self, requested):
        """
        Map the 'device' parameter to a torch device and log the reason.

        'auto' picks the GPU when PyTorch can see one and falls back to the
        CPU otherwise, so the node also runs in a container without a usable
        CUDA driver.
        """
        cuda_available = torch.cuda.is_available()
        torch_cuda = torch.version.cuda if torch.version.cuda else 'none'

        if requested == 'auto':
            device = 'cuda:0' if cuda_available else 'cpu'
            reason = 'auto'
        elif requested == 'cpu':
            device = 'cpu'
            reason = 'requested'
        elif requested.startswith('cuda'):
            if cuda_available:
                device = requested
                reason = 'requested'
            else:
                device = 'cpu'
                reason = 'requested CUDA, but no GPU is visible'
        else:
            raise ValueError('unknown device: ' + requested)

        self.get_logger().info(
            f"PyTorch {torch.__version__} (built for CUDA {torch_cuda}), "
            f"CUDA available: {cuda_available}")
        self.get_logger().info(f"Device '{requested}' -> '{device}' ({reason})")

        if device.startswith('cuda'):
            self.get_logger().info(
                f'Using GPU: {torch.cuda.get_device_name(torch.device(device))}')
        elif not cuda_available:
            self.get_logger().warn(
                f'PyTorch {torch.__version__} cannot see a CUDA GPU '
                f'(built for CUDA: {torch_cuda}); running on CPU')
        return device

    def image_callback(self, msg):
        """Detect the targets in one image and publish them."""
        try:
            image = decode_bgr8(msg)
        except ValueError as error:
            self.get_logger().error(f'Cannot decode image: {error}')
            return

        # Run Inference
        results = self.model(image, conf=self.conf_threshold, verbose=False,
                             device=self.device)

        found = []
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
                    found.append(det_msg)

            # Visualization (published only if there are subscribers)
            if self.viz_publisher.get_subscription_count() > 0:
                annotated_frame = result.plot()
                viz_msg = encode_bgr8(annotated_frame, msg.header)
                self.viz_publisher.publish(viz_msg)

        self.log_detections(found)

    def log_detections(self, found):
        """Log the detections of one image, at most once per second."""
        now = time.monotonic()
        if now - self.last_log_time_ < 1.0:
            return
        self.last_log_time_ = now
        if not found:
            self.get_logger().debug('No target detected')
            return
        summary = ', '.join(
            f"{det.class_name} {det.confidence:.2f} "
            f"({det.bbox_width}x{det.bbox_height} at {det.bbox_x},{det.bbox_y})"
            for det in found)
        self.get_logger().info(f"Detected: {summary}")


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
