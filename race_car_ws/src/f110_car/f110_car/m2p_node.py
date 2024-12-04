#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from avai_lab.utils import get_direction_vec, quat_to_rot_vec, rot_from_vec
from avai_lab.config import load_config

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class M2P(Node):
    """Node that subscribes to the "/drive" topic, collects all AckermannDriveStamped msgs and
    converts them to Twist msgs which are published to the "/cmd_vel" topic.
    """
    def __init__(self):
        super().__init__("AckermannToTwist") # "NodeName" will be displayed in rqt_graph
        self.odom_subscriber = self.create_subscription(Odometry, "/odom",
                                                         self.odom_callback, 10)
        self.point_subscriber = self.create_subscription(PoseStamped, "/target_points",
                                                         self.point_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        self.config = load_config()
        self.get_logger().info(f"Used config:\n{str(self.config)}")
        self.max_steering_angle = self.config.car_platform.max_steering_angle # radians
        self.max_speed = self.config.car_platform.max_speed # m/s
        self.max_acceleration = self.config.car_platform.max_acceleration # m/s
        self.max_steering = self.config.car_platform.max_steering_angle # radians/s

        self.target_stack = [] # Stack to hold upcoming target points
        self.current_target = None # Current target point (with optional rotation)
        self.last_point = None

    def point_callback(self, point_msg: PoseStamped):
        """
        Callback to handle new target points. Pushes received point to target stack.
        """
        target_position = np.array([point_msg.pose.position.x, point_msg.pose.position.y])
        target_orientation = quat_to_rot_vec(point_msg.pose.orientation.z, point_msg.pose.orientation.w)
        new_point = (target_position, target_orientation)
        if self.target_stack:
            if not np.array_equal(new_point[0], self.last_point[0]) or not (new_point[1] == self.last_point[1]):
                self.last_point = new_point
                self.target_stack.append(new_point)
                self.get_logger().info(f"Received new target point: {self.last_point[0]}, rotation: {self.last_point[1]}")
        else:
            self.last_point = new_point
            self.target_stack.append(new_point)
            self.get_logger().info(f"Received new target point: {self.last_point[0]}, rotation: {self.last_point[1]}")

    def odom_callback(self, odom_msg: Odometry):
        """
        Callback to process odometry updates and navigate to current target point.
        """

        # Handle target & target stack
        if not self.current_target and self.target_stack:
            self.current_target = self.target_stack.pop(0)

        if not self.current_target:
            self.get_logger().warn(f"No current target available.")
            return

        # Extract target and vehicle state
        pos = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]) # We only need to calculate in 2D
        direction_vec = get_direction_vec(pos, self.current_target[0])
        orientation_rot = quat_to_rot_vec(odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)

        # Calculate distance and steering angle
        distance = np.linalg.norm(direction_vec)
        direction_rot = rot_from_vec(direction_vec)
        target_orientation = self.current_target[1]
        delta_direction = direction_rot - orientation_rot

        if target_orientation is not None: # Point has defined rotation
            delta_rotation = target_orientation - orientation_rot
            w = max(0, min(1, 1 - distance / 5.0))
            steering_angle = (1 - w) * delta_direction + w * delta_rotation
        else: # Default behavior if no rotation given
            steering_angle = delta_direction

        # Target switching
        if distance < 1:
            if self.target_stack:
                self.get_logger().info("Switching to next target point.")
                self.current_target = self.target_stack.pop(0)
                return
            else:
                distance = 0.0 # Stop the car

        t = self.get_clock().now()
        msg = AckermannDriveStamped()
        msg.header.stamp = t.to_msg()
        #msg.header.seq = self.msg_id
        msg.header.frame_id = "0"
        msg.drive.steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        msg.drive.steering_angle_velocity = self.max_steering
        msg.drive.speed = min(float(distance), float(self.max_speed)) # Set the speed to the distance from the point (when we steer we should reduce that)
        msg.drive.jerk = self.max_acceleration
        msg.drive.acceleration = self.max_acceleration
        #msg.jerk = self.max_steering
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = M2P()
    rclpy.spin(node) # used to loop the node

    rclpy.shutdown()

if __name__ == "__main__":
    main()
