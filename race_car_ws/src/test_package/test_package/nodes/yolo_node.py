import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
import os
class YoloConeDetector(Node):
    def __init__(self):
        super().__init__('yolo_cone_detector')
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        #node_dir = os.path.dirname(os.path.realpath(__file__))
        #model_path = os.path.join(node_dir, '../../runs/detect/train/weights/best.pt')
        self.model = YOLO('/home/myuser/Desktop/race-car-buggy-busters/race_car_ws/src/test_package/runs/detect/train/weights/best.pt')
        self.colors = {
            "yellow_cones": (0, 255, 255),  # Yellow
            "orange_cones": (0, 165, 255),  # Orange
            "blue_cones": (255, 0, 0)       # Blue
        }


    def camera_callback(self, msg):
        msg_str = f"""
        Header:
            Frame ID: {msg.header.frame_id}
            Timestamp: {msg.header.stamp.sec}s {msg.header.stamp.nanosec}ns
        Resolution:
            Width: {msg.width}
            Height: {msg.height}
        Distortion Model: {msg.distortion_model}
        Distortion Coefficients: {msg.d}
        Intrinsic Matrix (K): {msg.k.tolist()}
        Rectification Matrix (R): {msg.r.tolist()}
        Projection Matrix (P): {msg.p.tolist()}
        """
        self.get_logger().info(msg_str)

    def cameraImage_callback(self, msg):
        bridge = CvBridge()

        # Convert ROS Image message to OpenCV image
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        msg_str = f"""
         Header:
         Frame ID: {msg.header.frame_id}
        Timestamp: {msg.header.stamp.sec}s {msg.header.stamp.nanosec}ns
        Resolution:
         Width: {cv_image.shape[1]} pixels
         Height: {cv_image.shape[0]} pixels
        Channels: {cv_image.shape[2] if len(cv_image.shape) > 2 else 1}
        """
        self.get_logger().info(msg_str)
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        # Run YOLOv8 detection
        results = self.model(cv_image)
        
        # Diplay results
        self.visualize_results(cv_image, results[0])

    def visualize_results(self, image, results):
        # Draw bounding boxes on the cones
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            confidence = float(box.conf[0])
            cls = int(box.cls[0])
            label = results.names[cls]
            
            color = self.colors.get(label, (0, 255, 0))
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            text = f"{label} {confidence:.2f}"
            cv2.putText(image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Display image
        cv2.imshow('YOLO Detection', image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloConeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
