#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from racecar_msgs.msg import DetectedCone, DetectedConeArray

from cv_bridge import CvBridge
from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration

class YoloConeDetectionNode(Node):
    """
    ROS2 node that uses YOLO to detect cones from camera sensors.
    """
    def __init__(self):
        super().__init__('yolo_cone_detection_node')

        self.declare_parameter('image_topic', '/camera/realsense2_camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/realsense2_camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/realsense2_camera/depth/image_rect_raw')
        self.declare_parameter('detection_topic', '/yolo_cones')
        self.declare_parameter('model_path', '/home/myuser/Desktop/race-car-buggy-busters/race_car_ws/src/test_package/runs/detect/train/weights/best.pt') 
        self.declare_parameter('confidence_threshold', 0.25) # Confidence threshold for filtering yolo detections
        self.declare_parameter('frame_id', 'base_link') # The target frame to which cone positions will be transformed.

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.frame_id = self.get_parameter('frame_id').value

        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)

        self.cones_pub = self.create_publisher(DetectedConeArray, self.detection_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_info = None
        self.depth_image = None

        self.get_logger().info("YOLO Cone Detection Node started...")

    def camera_info_callback(self, msg: CameraInfo):
        """
        Callback function for CameraInfo messages. Stores the message.

        :param msg: The incoming CameraInfo message.
        """
        self.camera_info = msg

    def depth_callback(self, msg: Image):
        """
        Callback function for depth image messages. Stores the message as an opencv message.

        :param msg: The incoming CameraInfo message.
        """
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg: Image):
        """
        Callback function for color image messages. Performs YOLO detection, transforms detected cones,
        and publishes them as a DetectedConeArray.

        :param msg: The incoming Image message.
        """
        # Return if camera info or depth image is not yet available
        if self.camera_info is None or self.depth_image is None:
            return

        # Run YOLO inference on the image and parse the results
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        results = self.model(cv_image)
        detections = self.parse_yolo_results(results)

        cone_array_msg = DetectedConeArray()
        cone_array_msg.header = msg.header
        camera_frame = self.camera_info.header.frame_id

        rgb_h, rgb_w = cv_image.shape[:2]
        depth_h, depth_w = self.depth_image.shape[:2]

        # Check if the depth and color image are of different sizes and calculate the scaling factor
        scale_depth_image = False
        if (rgb_w != depth_w) or (rgb_h != depth_h):
            scale_x = depth_w / float(rgb_w)
            scale_y = depth_h / float(rgb_h)
            scale_depth_image = True
        
        try:
            stamp_time = rclpy.time.Time.from_msg(msg.header.stamp)
            transform_stamped = self.tf_buffer.lookup_transform(
                self.frame_id,
                camera_frame,
                stamp_time,
                timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"Could not look up TF for laser scan: {e}")
            return

        for det in detections:
            
            x_min, y_min, x_max, y_max = det["bbox"]

            #Scale coordinates if necessary
            if scale_depth_image:
                x_min_scaled = x_min * scale_x
                x_max_scaled = x_max * scale_x
                y_min_scaled = y_min * scale_y
                y_max_scaled = y_max * scale_y
                u = int((x_min_scaled + x_max_scaled) / 2)
                v = int((y_min_scaled + y_max_scaled) / 2)
            else:
                u = int((x_min + x_max) / 2)
                v = int((y_min + y_max) / 2)

            # Convert pixel coordinates to 3D point in camera frame
            pt_camera = self.pixel_to_3d(u, v, self.depth_image, self.camera_info)
            if pt_camera is None:
                continue
            
            ps_x, ps_y, ps_z = pt_camera
            ps = PointStamped()
            ps.header.frame_id = camera_frame
            ps.header.stamp = msg.header.stamp
            ps.point.x = float(ps_x)
            ps.point.y = float(ps_y)
            ps.point.z = float(ps_z)
            
            try:
                ps_transformed = tf2_geometry_msgs.do_transform_point(ps, transform_stamped)
            except Exception as e:
                self.get_logger().warn(f"Error transforming camera point: {e}")
                continue

            cone_msg = DetectedCone()
            cone_msg.header = msg.header
            cone_msg.type = det["class_name"]  # "yellow_cone", etc.
            cone_msg.position.x = ps_transformed.point.x
            cone_msg.position.y = ps_transformed.point.y
            cone_msg.position.z = ps_transformed.point.z

            cone_array_msg.cones.append(cone_msg)

        self.cones_pub.publish(cone_array_msg)

    def parse_yolo_results(self, results):
        """
        Parses the results from the YOLO model and returns a list of detections.

        :param results: The results object returned by the YOLO model.
        :return: A list of dictionaries, where each dictionary represents a detection and contains the class name, confidence, and bounding box coordinates.
        """
        detections = []
        if len(results) == 0:
            return detections
        res = results[0]
        boxes = res.boxes

        for box in boxes:
            cls_id = int(box.cls[0])
            class_name = self.model.names[cls_id]
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].tolist()

            if conf < self.conf_threshold:
                continue
            detections.append({
                "class_name": class_name,
                "confidence": conf,
                "bbox": xyxy
            })
        return detections

    def pixel_to_3d(self, u, v, depth_image, camera_info):
        """
        Converts a pixel coordinate (u, v) and its corresponding depth value to a 3D point in the camera frame.

        :param u: The u-coordinate (horizontal) of the pixel.
        :param v: The v-coordinate (vertical) of the pixel.
        :param depth_image: The depth image.
        :param camera_info: The CameraInfo message containing camera intrinsics.
        :return: A tuple (X, Y, Z) representing the 3D point in the camera frame, or None if the conversion fails.
        """
        # Check for invalid pixel coordinates
        if (u < 0 or v < 0 or
            u >= camera_info.width or v >= camera_info.height):
            return None

        # Get the depth value at the given pixel
        depth = depth_image[v, u]
        if not np.isfinite(depth) or depth <= 0.0:
            return None

        # Get camera intrinsics from CameraInfo message
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        # Return None if extracted focal lengths are invalid
        if fx == 0.0 or fy == 0.0:
            return None

        # Calculate the 3D point in the camera frame using the extracted intrinsics
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth

        if not (math.isfinite(X) and math.isfinite(Y) and math.isfinite(Z)):
            return None

        return (X, Y, Z)


def main(args=None):
    rclpy.init(args=args)
    node = YoloConeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
