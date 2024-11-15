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
                )
        ])
