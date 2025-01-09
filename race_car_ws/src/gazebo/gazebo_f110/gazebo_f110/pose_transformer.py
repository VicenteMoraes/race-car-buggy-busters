import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from gazebo_f110.transformer.transform_utils import transform_pose_gazebo_to_ros2

class GazeboPoseTransformer(Node):
    def __init__(self):
        super().__init__("gazebo_pose_transformer")
        self.sub_pose = self.create_subscription(
            PoseArray,
            "/world/car_world/pose/info",
            self.pose_callback,
            10
        )
        self.pub_pose = self.create_publisher(
            PoseArray,
            "/transformed_pose",
            10
        )

    def pose_callback(self, msg_in):
        msg_pos_in = msg_in.pose[0]
        msg_quat_in = msg_in.pose[1]
        pos_g = (msg_pos_in.position.x, msg_pos_in.position.y, msg_pos_in.position.z)
        quat_g = (msg_quat_in.orientation.x, msg_quat_in.orientation.y,
                  msg_quat_in.orientation.z, msg_quat_in.orientation.w)

        pos_r, quat_r = transform_pose_gazebo_to_ros2(pos_g, quat_g)

        msg_out = PoseArray()
        msg_out.pose[0].position.x = pos_r[0]
        msg_out.pose[0].position.y = pos_r[1]
        msg_out.pose[0].position.z = pos_r[2]
        msg_out.pose[1].orientation.x = quat_r[0]
        msg_out.pose[1].orientation.y = quat_r[1]
        msg_out.pose[1].orientation.z = quat_r[2]
        msg_out.pose[1].orientation.w = quat_r[3]

        self.pub_pose.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
