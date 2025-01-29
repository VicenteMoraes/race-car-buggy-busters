#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from racecar_msgs.msg import DetectedCone, DetectedConeArray
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration

class ObjectFusionNode(Node):
    """
    ROS2 node that fuses detected cones from LiDAR and camera data.
    """
    def __init__(self):
        super().__init__('object_fusion_node')

        self.declare_parameter('lidar_cones_topic', '/lidar_cones')
        self.declare_parameter('yolo_cones_topic', '/yolo_cones')
        self.declare_parameter('fused_cones_topic', '/fused_cones')
        self.declare_parameter('distance_threshold', 0.3) # Maximum distance (in meters) for matching cones
        self.declare_parameter('frame_id', 'odom') # Target frame for fused cone positions

        lidar_topic = self.get_parameter('lidar_cones_topic').value
        yolo_topic  = self.get_parameter('yolo_cones_topic').value
        self.fused_cones_topic = self.get_parameter('fused_cones_topic').value
        self.dist_thresh = self.get_parameter('distance_threshold').value
        self.frame_id = self.get_parameter('frame_id').value

        self.lidar_sub = self.create_subscription(
            DetectedConeArray,
            lidar_topic,
            self.lidar_callback,
            1
        )
        self.yolo_sub = self.create_subscription(
            DetectedConeArray,
            yolo_topic,
            self.yolo_callback,
            1
        )

        self.fused_pub = self.create_publisher(
            DetectedConeArray,
            self.fused_cones_topic,
            10
        )

        self.lidar_cones = []
        self.yolo_cones  = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Object Fusion Node started.")

        # Timer to publish fused cones periodically (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_fused_cones)

    def lidar_callback(self, msg: DetectedConeArray):
        """
        Callback function for LiDAR-detected cones. Stores the cones.

        :param msg: The incoming DetectedConeArray message from LiDAR.
        """
        self.lidar_cones = msg.cones

    def yolo_callback(self, msg: DetectedConeArray):
        """
        Callback function for camera-detected cones. Stores the cones.

        :param msg: The incoming DetectedConeArray message from the camera.
        """
        self.yolo_cones = msg.cones

    def publish_fused_cones(self):
        """
        Transforms detected cones to the target frame, fuses them, and publishes the fused cones.
        """
        lidar_odom_cones = []
        for cone in self.lidar_cones:
            transformed_cone = self.transform_point(
                    (cone.position.x, cone.position.y, cone.position.z),
                    cone.header.stamp,
                    cone.header.frame_id,
                    self.frame_id
                )
            if transformed_cone is not None:
                lidar_odom_cones.append((transformed_cone.point.x, transformed_cone.point.y, cone.type))

        yolo_odom_cones = []
        for cone in self.yolo_cones:
            transformed_cone = self.transform_point(
                    (cone.position.x, cone.position.y, cone.position.z),
                    cone.header.stamp,
                    cone.header.frame_id,
                    self.frame_id
                )
            if transformed_cone is not None:
                yolo_odom_cones.append((transformed_cone.point.x, transformed_cone.point.y, cone.type))

        fused_cones_data = self.fuse_detections(lidar_odom_cones, yolo_odom_cones, self.dist_thresh)

        fused_msg = DetectedConeArray()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = self.frame_id

        for (fx, fy, ftype) in fused_cones_data:
            cone = DetectedCone()
            cone.header = fused_msg.header
            cone.position.x = fx
            cone.position.y = fy
            cone.position.z = 0.0
            cone.type = ftype
            fused_msg.cones.append(cone)

        self.fused_pub.publish(fused_msg)
        
    def transform_point(self, xyz, stamp, from_frame, to_frame):
        """
        Transforms a point from one frame to another.

        :param xyz: A tuple (x, y, z) representing the point's coordinates.
        :param stamp: The timestamp associated with the point.
        :param from_frame: The frame in which the point is currently defined.
        :param to_frame: The target frame to which the point should be transformed.
        :return: A PointStamped object for the transformed point, or None if the transformation fails.
        """
        try:
            stamp_time = rclpy.time.Time.from_msg(stamp)
            trans = self.tf_buffer.lookup_transform(
                to_frame, from_frame, stamp_time, timeout=Duration(seconds=1.0)
            )
            ps = PointStamped()
            ps.header.frame_id = from_frame
            ps.header.stamp = stamp
            ps.point.x = xyz[0]
            ps.point.y = xyz[1]
            ps.point.z = xyz[2]

            ps_out = tf2_geometry_msgs.do_transform_point(ps, trans)
            return ps_out
        except Exception as e:
            self.get_logger().warn(f"Could not transform fused cone: {e}")
            return None

    def fuse_detections(self, lidar_cones, camera_cones, distance_threshold):
        """
        Fuses two sets of cone detections based on their proximity.

        :param lidar_cones: The list of cones detected by LiDAR in the target frame.
        :param camera_cones: The list of cones detected by camera in the target frame.
        :param distance_threshold: The maximum distance between two cones for them to be considered a match.
        :return: A list of fused cones, where each element is a tuple (x, y, type of cone).
        """
        fused_cones = []
        matched_1 = set()
        matched_2 = set()

        for i, (x1, y1, t1) in enumerate(lidar_cones):
            best_match_j = -1
            best_dist = distance_threshold
            for j, (x2, y2, t2) in enumerate(camera_cones):
                if j in matched_2:
                    continue
                dist = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
                if dist < best_dist:
                    best_dist = dist
                    best_match_j = j
            if best_match_j >= 0:
                matched_1.add(i)
                matched_2.add(best_match_j)
                mx, my, _ = camera_cones[best_match_j]
                mx = (x1 + mx) / 2.0
                my = (y1 + my) / 2.0
                fused_cones.append((mx, my, t2))

        for i, (x1, y1, t1) in enumerate(lidar_cones):
            if i not in matched_1:
                fused_cones.append((x1, y1, t1))

        for j, (x2, y2, t2) in enumerate(camera_cones):
            if j not in matched_2:
                fused_cones.append((x2, y2, t2))

        return fused_cones


def main(args=None):
    rclpy.init(args=args)
    node = ObjectFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
