#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType

import numpy as np
import numpy.typing as npt
from scipy.ndimage import label
from scipy.spatial.transform import Rotation

from avai_lab import enums, utils, msg_conversion

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from racecar_msgs.msg import SemanticGrid

class GlobalPlanningNode(Node):
    """
    Uses the immediate surroundings as orientation points to calculate target points with a short distance
    to the vehicle that lie inside of a track.
    The node subscribes to a semantic grid which holds the position of cones on a map alongside their label.
    The algorithm to calculate a new target point creates a projection point in front of the vehicle (in driving direction)
    and calculates the distances to all cones on the semantic grid. It then uses the closest blue cone to the left
    and the closest yellow cone to the right to calculate the gravitational center between the two and use that as
    a new target point.
    
    projection_point_distance: Distance from the projection point to the vehicle in meters
    """
    def __init__(self):
        super().__init__("global_planning_node") # "NodeName" will be displayed in rqt_graph
        self.declare_parameter("map_pose_topic", "/pose")
        self.declare_parameter("semantic_grid_topic", "/semantic_map")
        self.declare_parameter("target_point_topic", "/target_point")
        self.declare_parameter("start_point_epsilon", 1)
        self.declare_parameter("path_topic", "/path")

        self.map_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("map_pose_topic").value,
                                                         self.pose_callback, 10)
        self.semantic_grid_subscriber = self.create_subscription(SemanticGrid, self.get_parameter("semantic_grid_topic").value, 
                                                                 self.semantic_grid_callback, 10)
        self.target_point_subscriber = self.create_subscription(PoseStamped, self.get_parameter("target_point_topic").value, self.target_point_callback, 10)
        self.target_point_publisher = self.create_publisher(PoseStamped, self.get_parameter("target_point_topic").value, 10)
        self.path_publisher = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.start_point_epsilon = self.get_parameter("start_point_epsilon").value
        self.path = []
        self.left_starting_area = False
        self.start_position = None
        #self.cli = self.create_client(SetParameters, '/exploration_node/set_parameters')
        #while not self.cli.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('service not available, waiting again...')
        self.req = SetParameters.Request()

    def deactivate_exploration(self):
        param = Parameter()
        param.name = "active"
        param.value.type = ParameterType.PARAMETER_BOOL
        param.value = False
        self.req.parameters.append(param)

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def publish_path(self):
        msg = Path()
        t = self.get_clock().now()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = "map"
        msg.poses = self.path
        self.path_publisher.publish(msg)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        if self.start_position is None:
            self.start_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        vehicle_location = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.path.append(self.create_pose_stamped_from_pose_with_covariance(msg))
        if not self.left_starting_area:
            self.get_logger().info("Did not leave start")
            distance_to_start = np.linalg.norm((self.start_position, vehicle_location))
            self.left_starting_area = distance_to_start > self.start_point_epsilon
        else:
            self.get_logger().info("Left start")
        self.publish_path()


    def target_point_callback(self, msg: PoseStamped):
        pass

    def create_pose_stamped_from_pose_with_covariance(self, cov_msg: PoseWithCovarianceStamped) -> PoseStamped:
        msg = PoseStamped()
        msg.header = cov_msg.header
        msg.pose = cov_msg.pose.pose
        msg.pose = cov_msg.pose.pose
        return msg

    def create_target_point_msg(self, x: float, y: float) -> PoseStamped:
        msg = PoseStamped()
        t = self.get_clock().now()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        return msg

    def semantic_grid_callback(self, msg: SemanticGrid):
        """Convert the grid into a numpy array and calculate a new target point
        """
        return
        if self.last_pose is None:
            self.get_logger().info("Pose not initialized, skipping semantic grid callback")
            return
        grid = msg_conversion.semantic_grid_to_np(msg)
        valid_labels = [
            enums.YELLOW_CONE,
            enums.BLUE_CONE,
            enums.ORANGE_CONE,
            enums.UNKNOWN_CONE
        ]
        labels = grid
        cone_positions = []
        position_labels = []
        for cone_label in valid_labels:
            # Create a binary mask for this label.
            mask = (labels == cone_label)
            # Find connected clusters in the binary mask.
            labeled_mask, num_clusters = label(mask)
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            resolution = msg.info.resolution
            
            for cluster in range(1, num_clusters + 1):
                # Get indices (rows, cols) for cells belonging to this cluster.
                indices = np.where(labeled_mask == cluster)
                if len(indices[0]) == 0:
                    continue
                # Compute the centroid in grid coordinates.
                centroid_row = np.mean(indices[0])
                centroid_col = np.mean(indices[1])
                # Convert grid coordinates to world coordinates.
                centroid_x = origin_x + (centroid_col + 0.5) * resolution
                centroid_y = origin_y + (centroid_row + 0.5) * resolution
                cone_positions.append((centroid_x, centroid_y))
                position_labels.append(cone_label)
        position_labels = np.array(position_labels)
        cone_positions = np.array(cone_positions)
        if len(cone_positions) == 0:
            self.get_logger().info("No cones detected, skipping target point creation")
            return

def main(args=None):
    rclpy.init(args=args)

    node = GlobalPlanningNode()
    rclpy.spin(node) # used to loop the node

    rclpy.shutdown()

if __name__ == "__main__":
    main()
