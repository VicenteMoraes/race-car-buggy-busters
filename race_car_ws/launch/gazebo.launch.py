from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
            Node(
                package="test_package",
                namespace="f110",
                executable="wasd_control_node",
                name="wasd_control"
            ),
            Node(
                package="gazebo_f110",
                namespace="gazebo",
                executable="ackermann_to_twist",
                name="ackermann_to_twist"
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=["/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",],
                remappings=[],
                output='screen'
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=["/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",],
                remappings=[],
                output='screen'
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=["/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",],
                remappings=[("/camera_info", "/camera/realsense2_camera/color/camera_info")],
                output='screen'
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=["/camera@sensor_msgs/msg/Image@ignition.msgs.Image",],
                remappings=[("/camera", "/camera/realsense2_camera/color/image_raw")],
                output='screen'
            ),
        ])
