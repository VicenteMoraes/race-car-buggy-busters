
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gazebo_static_transform',
        output='screen',
        arguments=[
            '0', '0', '0',
            '-0.7071067811865475', '0.7071067811865475', '0.0', '0.0',
            'base_link', '0'
        ]
    )

    m2p_node = Node(
            package="f110_car",
            namespace="f110",
            executable="move_to_point",
            name="move_to_point"
        )
    
    transform_node = Node(
        package="gazebo_f110",
        namespace="gazebo",
        executable="transform_pose",
        name="transform_pose"
    )

    return LaunchDescription([
        static_transform_publisher,
        m2p_node,
        transform_node
    ])
