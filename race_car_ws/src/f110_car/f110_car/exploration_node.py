#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node

import numpy as np
import numpy.typing as npt
from scipy.ndimage import label
from scipy.spatial.transform import Rotation

from avai_lab import enums, utils

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from racecar_msgs.msg import SemanticGrid

class ExplorationNode(Node):
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
        super().__init__("exploration_node") # "NodeName" will be displayed in rqt_graph
        self.declare_parameter("map_pose_topic", "/pose")
        self.declare_parameter("semantic_grid_topic", "/semantic_map")
        self.declare_parameter("target_point_topic", "/target_point")
        self.declare_parameter("projection_point_distance", 2)

        self.map_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("map_pose_topic").value,
                                                         self.pose_callback, 10)
        self.semantic_grid_subscriber = self.create_subscription(SemanticGrid, self.get_parameter("semantic_grid_topic").value, 
                                                                 self.semantic_grid_callback, 10)
        self.target_point_publisher = self.create_publisher(PoseStamped, self.get_parameter("target_point_topic").value, 10)
        self.forward_unit_vec = [1, 0, 0] # TODO: This will be wrong in the ROS2 coordinate system
        self.last_pose = None

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.last_pose = msg

    @staticmethod
    def semantic_grid_to_np(grid: SemanticGrid) -> npt.NDArray:
        width = grid.info.width
        height = grid.info.height
        labels = np.empty((height, width), dtype=int)
        for i, cell in enumerate(grid.cells):
            row = i // width
            col = i % width
            labels[row, col] = cell.label
        return labels

    def get_car_projection_vector(self) -> List:
        """Extract the current rotation from the last received pose message and create
        a vector pointing in the direction of the vehicle
        """
        assert self.last_pose is not None, "Pose not initialized"
        rotation_radians = utils.quat_to_rot_vec(self.last_pose.pose.pose.orientation.z, self.last_pose.pose.pose.orientation.w)
        rotation_axis = np.array([0, 0, 1])
         
        rotation_vector = rotation_radians * rotation_axis
        rotation = Rotation.from_rotvec(rotation_vector)
        return rotation.apply(self.forward_unit_vec)[:2]

    def get_projected_point(self) -> npt.NDArray:
        """Return a projected point in front of the vehicle that is in line with the current vehicle orientation"""
        assert self.last_pose is not None, "Pose not initialized"
        scaler = self.get_parameter("projection_point_distance").value
        projection_vector = self.get_car_projection_vector() * scaler
        vehicle_location = np.array([self.last_pose.pose.pose.position.x, self.last_pose.pose.pose.position.y])
        return projection_vector + vehicle_location

    def create_target_point_msg(self, x: float, y: float) -> PoseStamped:
        msg = PoseStamped()
        t = self.get_clock().now()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        return msg

    def get_left_cone(self, cone_positions: npt.NDArray, labels: npt.NDArray, 
                       projected_point: npt.NDArray) -> npt.NDArray | None:
        """Retrieve the closest blue cone that lies on the left side of the vehicle
        :param cone_positions: A 2D numpy array with the cones in the first dimension and the x,y coordinates in the
            second dimension
        :param labels: A numpy array holding the labels for the cone positions
        :param projected_point: A numpy array holding the x,y coordinates of the projected point in front of the vehicle
        :return: A numpy array with the x, y coordinates of the cone
        """
        assert len(cone_positions) == len(labels), "The amount of labels must equal the amount of cone positions"
        vehicle_location = np.array([self.last_pose.pose.pose.position.x, self.last_pose.pose.pose.position.y])
        for cone_pos in cone_positions[labels == enums.BLUE_CONE]:
            if utils.is_right(vehicle_location, projected_point, cone_pos):
                return cone_pos
        
        # If no colored cone is found, take the closest unknown cone
        for cone_pos in cone_positions[labels == enums.UNKNOWN_CONE]:
            if utils.is_right(vehicle_location, projected_point, cone_pos):
                return cone_pos
        return None


    def get_right_cone(self, cone_positions: npt.NDArray, labels: npt.NDArray, 
                       projected_point: npt.NDArray) -> npt.NDArray | None:
        """Retrieve the closest yellow cone that lies on the right side of the vehicle
        :param cone_positions: A 2D numpy array with the cones in the first dimension and the x,y coordinates in the
            second dimension
        :param labels: A numpy array holding the labels for the cone positions
        :param projected_point: A numpy array holding the x,y coordinates of the projected point in front of the vehicle
        :return: A numpy array with the x, y coordinates of the cone
        """
        assert len(cone_positions) == len(labels), "The amount of labels must equal the amount of cone positions"
        vehicle_location = np.array([self.last_pose.pose.pose.position.x, self.last_pose.pose.pose.position.y])
        for cone_pos in cone_positions[labels == enums.YELLOW_CONE]:
            if not utils.is_right(vehicle_location, projected_point, cone_pos):
                return cone_pos
        
        # If no colored cone is found, take the closest unknown cone
        for cone_pos in cone_positions[labels == enums.UNKNOWN_CONE]:
            if not utils.is_right(vehicle_location, projected_point, cone_pos):
                return cone_pos
        return None

    def semantic_grid_callback(self, msg: SemanticGrid):
        """Convert the grid into a numpy array and calculate a new target point
        """
        if self.last_pose is None:
            self.get_logger().info("Pose not initialized, skipping semantic grid callback")
            return
        grid = self.semantic_grid_to_np(msg)
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
        projected_point = self.get_projected_point()
        cone_positions = np.array(cone_positions)
        if len(cone_positions) == 0:
            self.get_logger().info("No cones detected, skipping target point creation")
            return
        # calculate the distances between the projected point and all cone positions
        distances = np.linalg.norm(cone_positions - projected_point, axis=1)
        # get the sorting index by distance
        dist_sort_idx = np.argsort(distances)
        sorted_labels = position_labels[dist_sort_idx]
        # Find the closest yellow cone that is to the right of the projection vector
        right_cone_position = self.get_right_cone(cone_positions[dist_sort_idx], sorted_labels, projected_point)
        if right_cone_position is None:
            self.get_logger().info("Could not locate a yellow cone to the right of the vehicle")
            return

        left_cone_position = self.get_left_cone(cone_positions[dist_sort_idx], sorted_labels, projected_point)
        if left_cone_position is None:
            self.get_logger().info("Could not locate a blue cone to the left of the vehicle")
            return
        target_point = right_cone_position + (left_cone_position - right_cone_position) / 2
        self.target_point_publisher.publish(self.create_target_point_msg(*target_point))

def main(args=None):
    rclpy.init(args=args)

    node = ExplorationNode()
    rclpy.spin(node) # used to loop the node

    rclpy.shutdown()

if __name__ == "__main__":
    main()
