from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    wasd_node = Node(
            package="test_package",
            namespace="f110",
            executable="wasd_control_node",
            name="wasd_control"
            )
    ackermann_to_twist_node = Node(
            package="gazebo_f110",
            namespace="gazebo",
            executable="ackermann_to_twist",
            name="ackermann_to_twist"
            )
    ros_gz_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist", 
                "/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
                "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                "/camera@sensor_msgs/msg/Image@ignition.msgs.Image",
                "/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU"
                ],
            remappings=[
                ("/camera_info", "/camera/realsense2_camera/color/camera_info"),
                ("/camera", "/camera/realsense2_camera/color/image_raw")
                ],
            output='screen'
            )
    return LaunchDescription([
        wasd_node,
        ackermann_to_twist_node,
        ros_gz_bridge_node,
        ])
