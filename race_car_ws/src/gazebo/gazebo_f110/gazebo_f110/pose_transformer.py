import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from gazebo_f110.transformer.transform_utils import transform_pose_gazebo_to_ros2

class GazeboPoseTransformer(Node):
    def __init__(self):
        super().__init__("gazebo_pose_transformer")
        self.sub_pose = self.create_subscription(
            PoseStamped,
            "/target_points",
            self.pose_callback,
            10
        )
        self.pub_pose = self.create_publisher(
            PoseStamped,
            "/transformed_points",
            10
        )

    def pose_callback(self, msg_in: PoseStamped):
        msg_in_pos = (msg_in.pose.position.x, msg_in.pose.position.y, msg_in.pose.position.z)

        pos_r = transform_pose_gazebo_to_ros2(msg_in_pos)

        msg_out = PoseStamped()
        msg_out.pose.position.x = pos_r[0]
        msg_out.pose.position.y = pos_r[1]
        msg_out.pose.position.z = pos_r[2]
    
        print("Gazebo Pose:")
        print("  Position G =", msg_in)
        print("\nROS2 Pose:")
        print("  Position R =", pos_r)


        self.pub_pose.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
