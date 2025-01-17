import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped
from gazebo_f110.transformer.transform_utils import transform_pose_gazebo_to_ros2
import rclpy.time
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration

class GazeboPoseTransformer(Node):
    def __init__(self):
        super().__init__("gazebo_pose_transformer")
        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.sub_pose = self.create_subscription(
            PoseStamped,
            "/target_points",
            self.pose_callback_transformer,
            10
        )
        self.pub_pose = self.create_publisher(
            PoseStamped,
            "/transformed_points",
            10
        )
    def pose_callback_transformer(self, msg_in: PoseStamped):
        print("Transformer launched")
        now = rclpy.time.Time()
        trans = self.tf_buffer.lookup_transform('0', 'gazebo_frame', now, timeout=Duration(seconds=1.0))
        point_in = PointStamped()
        point_in.header.stamp = now.to_msg()
        point_in.header.frame_id = msg_in.header.frame_id
        point_in.point.x = msg_in.pose.position.x
        point_in.point.y = msg_in.pose.position.y
        point_in.point.z = msg_in.pose.position.z
        trasnformed_pose = tf2_geometry_msgs.do_transform_point(point_in, trans)
        print("Gazebo Pose:")
        print("  Position G =", point_in)
        print("\nROS2 Pose:")
        print("  Position R =", trasnformed_pose)

        transformed_pose_stamped = PoseStamped()
        transformed_pose_stamped.header = trasnformed_pose.header

        transformed_pose_stamped.pose.position.x = trasnformed_pose.point.x
        transformed_pose_stamped.pose.position.y = trasnformed_pose.point.y
        transformed_pose_stamped.pose.position.z = trasnformed_pose.point.z
        self.pub_pose.publish(transformed_pose_stamped)



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
