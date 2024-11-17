import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np


class YoloConeDetector(Node):
    def __init__(self):
        super().__init__('yolo_cone_detector')
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info("Cone Detector Node Initialized")


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
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect cones
        yellow_mask = self.detect_color(frame, lower_bound=(20, 100, 100), upper_bound=(30, 255, 255))
        red_mask = self.detect_color(frame, lower_bound=(0, 50, 50), upper_bound=(10, 255, 255))

        # Count non-zero pixels in masks (rough detection of objects)
        yellow_count = cv2.countNonZero(yellow_mask)
        red_count = cv2.countNonZero(red_mask)

        if yellow_count > 500:
            self.get_logger().info("Yellow cone detected!")
        if red_count > 500: 
            self.get_logger().info("Red cone detected!")
        else:
            self.get_logger().info("No cone is detected")

    @staticmethod
    def detect_color(frame, lower_bound, upper_bound):
        """Detect specific color in an image using HSV color space."""
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, np.array(lower_bound), np.array(upper_bound))
        return mask


def main(args=None):
    rclpy.init(args=args)
    node = YoloConeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
