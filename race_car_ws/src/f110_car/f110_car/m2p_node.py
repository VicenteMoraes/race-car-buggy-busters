#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import numpy as np

from avai_lab.utils import get_direction_vec, quat_to_rot_vec, rot_from_vec
from avai_lab.config import load_config

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class M2P(Node):
    """
    Node that subscribes to the "/drive" topic, collects all AckermannDriveStamped msgs and
    converts them to Twist msgs which are published to the "/cmd_vel" topic.
    """
    def __init__(self):
        super().__init__("AckermannToTwist") # "NodeName" will be displayed in rqt_graph
        self.odom_subscriber = self.create_subscription(Odometry, "/odom",
                                                         self.odom_callback, 10)
        self.point_subscriber = self.create_subscription(PoseStamped, "/target_points",
                                                         self.point_callback, 100)
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        self.config = load_config()
        self.get_logger().info(f"Used config:\n{str(self.config)}")
        self.max_steering_angle = self.config.car_platform.max_steering_angle # radians
        self.max_speed = self.config.car_platform.max_speed # m/s
        self.max_acceleration = self.config.car_platform.max_acceleration # m/s
        self.max_steering = self.config.car_platform.max_steering_angle # radians/s

        self.target_stack = [] # Stack to hold upcoming target points
        self.current_target = None # Current target point
        self.last_point = None # Last received point
    
    def normalize_angle(self, angle):
        """
        Normalize an angle to the range [-pi, pi].
        """
        return math.remainder(angle, 2*math.pi)

    def point_callback(self, point_msg: PoseStamped):
        """
        Callback to handle new target points. Pushes received point to target stack.
        """
        new_point = np.array([point_msg.pose.position.x, point_msg.pose.position.y])
        if self.target_stack and (np.array_equal(new_point, self.last_point)):
            return
        else:
            self.last_point = new_point
            self.target_stack.append(new_point)
            self.get_logger().info(f"Received new target point: {self.last_point[0]}")

    def odom_callback(self, odom_msg: Odometry):
        """
        Callback to process odometry updates and navigate to current target point.
        """

        # Handle target & target stack
        if self.current_target is None and self.target_stack:
            self.current_target = self.target_stack.pop(0)

        if self.current_target is None:
            return

        # Extract target and vehicle state
        pos = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]) # We only need to calculate in 2D
        direction_vec = get_direction_vec(pos, self.current_target)
        orientation_rot = quat_to_rot_vec(odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)

        # Calculate distance and steering angle
        distance = np.linalg.norm(direction_vec)
        direction_rot = rot_from_vec(direction_vec)
        steering_angle = self.normalize_angle(direction_rot - orientation_rot)

        # Target switching
        if distance < 0.25:
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
