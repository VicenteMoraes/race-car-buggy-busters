#!/usr/bin/env python3
import sys, termios, tty, select


import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

## Driving command for a car-like vehicle using Ackermann steering.
#  $Id$

# Assumes Ackermann front-wheel steering. The left and right front
# wheels are generally at different angles. To simplify, the commanded
# angle corresponds to the yaw of a virtual wheel located at the
# center of the front axle, like on a tricycle.  Positive yaw is to
# the left. (This is *not* the angle of the steering wheel inside the
# passenger compartment.)
#
# Zero steering angle velocity means change the steering angle as
# quickly as possible. Positive velocity indicates a desired absolute
# rate of change either left or right. The controller tries not to
# exceed this limit in either direction, but sometimes it might.
#
# float32 steering_angle          # desired virtual angle (radians)
# float32 steering_angle_velocity # desired rate of change (radians/s)

# Drive at requested speed, acceleration and jerk (the 1st, 2nd and
# 3rd derivatives of position). All are measured at the vehicle's
# center of rotation, typically the center of the rear axle. The
# controller tries not to exceed these limits in either direction, but
# sometimes it might.
#
# Speed is the desired scalar magnitude of the velocity vector.
# Direction is forward unless the sign is negative, indicating reverse.
#
# Zero acceleration means change speed as quickly as
# possible. Positive acceleration indicates a desired absolute
# magnitude; that includes deceleration.
#
# Zero jerk means change acceleration as quickly as possible. Positive
# jerk indicates a desired absolute rate of acceleration change in
# either direction (increasing or decreasing).
#
# float32 speed                   # desired forward speed (m/s)
# float32 acceleration            # desired acceleration (m/s^2)
# float32 jerk                    # desired jerk (m/s^3)

settings = termios.tcgetattr(sys.stdin)

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

class WASDControl(Node):
    def __init__(self):
        super().__init__("WASDControl") # "NodeName" will be displayed in rqt_graph
        self.timer = self.create_timer(0.5, self.publish_control)
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.get_logger().info("WASD control node started")
        self.get_logger().info("Press any other key to interrupt")
        self.max_steering_angle = 1.0 # radians
        self.max_speed = 0.25 # m/s
        self.max_acceleration = 0.05 # m/s
        self.max_acc = 0.05 # m/s
        self.max_steering = 0.05 # radians/s
        self.msg_id = 0

    def publish_control(self):
        # Keyboard logic
        t = self.get_clock().now()
        msg = AckermannDriveStamped()
        msg.header.stamp = t.to_msg()
        #msg.header.seq = self.msg_id
        msg.header.frame_id = "0"
        msg.drive.steering_angle = 0.0
        msg.drive.steering_angle_velocity = self.max_steering
        msg.drive.speed = 0.0
        msg.drive.jerk = self.max_acceleration
        msg.drive.acceleration = self.max_acceleration
        #msg.jerk = self.max_steering
        self.publisher.publish(msg)
        key = getKey()
        match key:
            case "w":
                self.get_logger().info("FORWARD")
                msg.drive.speed = self.max_speed
            case "d":
                self.get_logger().info("RIGHT")
                msg.drive.steering_angle = self.max_steering_angle
            case "a":
                self.get_logger().info("LEFT")
                msg.drive.steering_angle = -self.max_steering_angle
            case "s":
                self.get_logger().info("BACKWARDS")
                msg.drive.speed = -self.max_speed
            case _:
                raise KeyboardInterrupt("Program terminated by user")

        self.publisher.publish(msg)
        self.msg_id += 1


def main(args=None):
    rclpy.init(args=args)

    node = WASDControl()
    rclpy.spin(node) # used to loop the node

    rclpy.shutdown()

if __name__ == "__main__":
    main()
