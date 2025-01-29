#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np

from sensor_msgs.msg import LaserScan
import tf2_geometry_msgs
from racecar_msgs.msg import DetectedCone, DetectedConeArray

from sklearn.cluster import DBSCAN
import tf2_ros
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration

class LidarConeDetectionNode(Node):
    """
    ROS2 node that subscribes to LiDAR scan data, performs DBSCAN clustering to detect potential cones,
    transforms their positions to a target frame, and publishes the detected cones.
    """
    def __init__(self):
        super().__init__('lidar_cone_detection_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('detection_topic', '/lidar_cones')
        self.declare_parameter('cluster_eps', 0.15) # Maximum distance between two scans for them to be considered as in the same neighborhood.
        self.declare_parameter('min_samples', 3) # The number of scans in a neighborhood for a point to be considered as a core point.
        self.declare_parameter('frame_id', 'base_link') # The target frame to which cone positions will be transformed.
        self.declare_parameter('max_cluster_dim', 0.5) # Maximum size in meters of the cluster, to discard large / non-cone objets

        self.scan_topic = self.get_parameter('scan_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.cluster_eps = self.get_parameter('cluster_eps').value
        self.min_samples = self.get_parameter('min_samples').value
        self.frame_id = self.get_parameter('frame_id').value
        self.max_cluster_dim = self.get_parameter('max_cluster_dim').value

        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.cones_pub = self.create_publisher(DetectedConeArray, self.detection_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Lidar Cone Detection Node started...")

    def scan_callback(self, msg: LaserScan):
        """
        Callback function for LaserScan messages. Processes scan data, performs clustering, and publishes detected cones.

        :param msg: The incoming LaserScan message.
        """
        points = []
        angle = msg.angle_min
        # Extract valid points from the scan data and convert to Cartesian coordinates
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
            angle += msg.angle_increment

        if len(points) == 0:
            self.publish_empty_cones(msg)
            return

        points_array = np.array(points)
        db = DBSCAN(eps=self.cluster_eps, min_samples=self.min_samples).fit(points_array)
        labels = db.labels_

        unique_labels = set(labels)
        if -1 in unique_labels:
            unique_labels.remove(-1)

        cone_array_msg = DetectedConeArray()
        cone_array_msg.header.stamp = msg.header.stamp
        cone_array_msg.header.frame_id = self.frame_id

        laser_frame = msg.header.frame_id
        
        try:
            stamp_time = rclpy.time.Time.from_msg(msg.header.stamp)
            transform_stamped = self.tf_buffer.lookup_transform(
                self.frame_id,
                msg.header.frame_id,
                stamp_time,
                timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"Could not look up TF for laser scan: {e}")
            return

        for lbl in unique_labels:
            cluster_points = points_array[labels == lbl]
            min_x = np.min(cluster_points[:, 0])
            max_x = np.max(cluster_points[:, 0])
            min_y = np.min(cluster_points[:, 1])
            max_y = np.max(cluster_points[:, 1])

            cluster_width = max_x - min_x
            cluster_height = max_y - min_y
            max_dim = max(cluster_width, cluster_height)

            if max_dim > self.max_cluster_dim:
                continue
            
            # Calculate center of the cluster
            cx = (min_x + max_x) / 2.0
            cy = (min_y + max_y) / 2.0

            ps = PointStamped()
            ps.header.frame_id = laser_frame
            ps.header.stamp = msg.header.stamp
            ps.point.x = cx
            ps.point.y = cy
            ps.point.z = 0.0

            try:
                ps_transformed = tf2_geometry_msgs.do_transform_point(ps, transform_stamped)
            except Exception as e:
                self.get_logger().warn(f"Error transforming laser point: {e}")
                continue

            cone_msg = DetectedCone()
            cone_msg.header = cone_array_msg.header
            cone_msg.type = "cone"
            cone_msg.position.x = ps_transformed.point.x
            cone_msg.position.y = ps_transformed.point.y
            cone_msg.position.z = ps_transformed.point.z
            cone_array_msg.cones.append(cone_msg)

        self.cones_pub.publish(cone_array_msg)

    def publish_empty_cones(self, msg: LaserScan):
        """
        Publishes an empty DetectedConeArray message.

        :param msg: The incoming LaserScan message used for the header.
        """
        cone_array_msg = DetectedConeArray()
        cone_array_msg.header = msg.header
        self.cones_pub.publish(cone_array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarConeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
