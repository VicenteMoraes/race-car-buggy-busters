import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import torch
import numpy as np
import os

from test_package.yolov7.models.experimental import attempt_load
from test_package.yolov7.utils.general import non_max_suppression, scale_coords, xyxy2xywh
from test_package.yolov7.utils.datasets import letterbox

class YoloConeDetector(Node):
    def __init__(self):
        super().__init__('yolo_cone_detector')
        self.subscription = self.create_subscription(
            Image,
            '', # To add subscription topic
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, '/yolo', 10)
        self.bridge = CvBridge()

        """Load Own Model trained to detect cones"""
        model_path = os.path.join(
            os.path.dirname(__file__), '') # Add model path
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = attempt_load(model_path, map_location=self.device)
        self.stride = int(self.model.stride.max())
        self.imgsz = 640
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.get_logger().info('YOLOv7 model loaded')
        

    def listener_callback(self, msg):
        """Convert ROS Image message to OpenCV image that YOLO can handle with with 8-bit encoding"""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        """ Resize image to match the input size expected by YOLO"""
        img = letterbox(frame, self.imgsz, stride=self.stride)[0]

        """ Convert the image to PyTorch tensor"""
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3xHxW
        img = np.ascontiguousarray(img)

        img_tensor = torch.from_numpy(img).to(self.device)
        img_tensor = img_tensor.float()
        img_tensor /= 255.0  # Normalize to [0,1]
        if img_tensor.ndimension() == 3:
            img_tensor = img_tensor.unsqueeze(0)

        """ Run the model """
        with torch.no_grad():
            pred = self.model(img_tensor, augment=False)[0]

        """Apply NMS to filter the raw predictions to remove redundant detections""" 
        pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45, classes=None, agnostic=False)

        """ Process detections to rescale bounding boxes and annotate the image."""
        for det in pred:
            if len(det):
                # Rescale boxes from img_size to frame size
                det[:, :4] = scale_coords(img_tensor.shape[2:], det[:, :4], frame.shape).round()

                for *xyxy, conf, cls in reversed(det):
                    # Draw bounding boxes
                    label = f'{self.names[int(cls)]} {conf:.2f}'
                    cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
                    cv2.putText(frame, label, (int(xyxy[0]), int(xyxy[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        """Convert OpenCV image back to ROS Image message"""
        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        annotated_msg.header = msg.header
        """Publish the annotated image"""
        self.publisher.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloConeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
