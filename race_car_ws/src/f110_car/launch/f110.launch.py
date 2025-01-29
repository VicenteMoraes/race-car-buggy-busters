from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    m2p_node = Node(
        package="f110_car",
        executable="move_to_point",
        name="move_to_point",
    )

    lidar_node = Node(
        package='test_package',
        executable='lidar_node',
        name='lidar_node',
    )

    yolo_node = Node(
        package='test_package',
        executable='yolo_node',
        name='yolo_node',
    )

    sensor_fusion_node = Node(
        package='test_package',
        executable='sensor_fusion_node',
        name='sensor_fusion_node',
    )

    mapping_node = Node(
        package='test_package',
        executable='mapping_node',
        name='mapping_node',
    )

    odom_tf_node = Node(
        package='test_package',
        executable='odom_tf_node',
        name='odom_tf_node',
    )

    static_tf_base_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_camera',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'base_link', 'camera_link'
        ],
    )

    return LaunchDescription([
        static_tf_base_camera,
        m2p_node,
        lidar_node,
        yolo_node,
        sensor_fusion_node,
        mapping_node,
        odom_tf_node,
    ])
