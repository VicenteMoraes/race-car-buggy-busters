#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
#from gz.msgs10 import Twist

class MyNode(Node):
    def __init__(self):
        super().__init__("NodeName") # "NodeName" will be displayed in rqt_graph
        self.drive_subscriber = self.create_subscription(AckermannDriveStamped, "/drive",
                                                         self.drive_callback, 10)
        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def drive_callback(self, drive_msg: AckermannDriveStamped):
        msg = Twist()
        msg.linear.x = drive_msg.drive.speed
        msg.angular.z = drive_msg.drive.steering_angle
        self.twist_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()
    rclpy.spin(node) # used to loop the node

    rclpy.shutdown()

if __name__ == "__main__":
    main()
