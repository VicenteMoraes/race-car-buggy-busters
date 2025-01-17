import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TrackPlanner(Node):
    def __init__(self, points):
        super().__init__("TrackPlanner") # "NodeName" will be displayed in rqt_graph
        self.publisher = self.create_publisher(PoseStamped, "/target_points", 100)
        for (x, y, z) in points:
            t = self.get_clock().now()
            msg = PoseStamped()
            msg.header.stamp = t.to_msg()
            msg.header.frame_id = "gazebo_frame"
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    points = [(3.0, 2.0, 0.5)]
    node = TrackPlanner(points)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()