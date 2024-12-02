#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from avai_lab.utils import get_direction_vec, quat_to_rot_vec, rot_from_vec
from avai_lab.config import load_config

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class M2P(Node):
    """Node that subscribes to the "/drive" topic, collects all AckermannDriveStamped msgs and
    converts them to Twist msgs which are published to the "/cmd_vel" topic.
    """
    def __init__(self):
        super().__init__("AckermannToTwist") # "NodeName" will be displayed in rqt_graph
        self.odom_subscriber = self.create_subscription(Odometry, "/odom",
                                                         self.odom_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.config = load_config()
        self.get_logger().info(f"Used config:\n{str(self.config)}")
        self.max_steering_angle = self.config.car_platform.max_steering_angle # radians
        self.max_speed = self.config.car_platform.max_speed # m/s
        self.max_acceleration = self.config.car_platform.max_acceleration # m/s
        self.max_steering = self.config.car_platform.max_steering_angle # radians/s
        self.target = np.array([5, 5])

    def odom_callback(self, odom_msg: Odometry):
        pos = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]) # We only need to calculate in 2D
        direction_vec = get_direction_vec(pos, self.target)
        orientation_rot = quat_to_rot_vec(odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        steering_angle = rot_from_vec(direction_vec) - orientation_rot # The rotation the ego vehicle must undergo


        t = self.get_clock().now()
        msg = AckermannDriveStamped()
        msg.header.stamp = t.to_msg()
        #msg.header.seq = self.msg_id
        msg.header.frame_id = "0"
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = self.max_steering
        msg.drive.speed = 2.0
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
