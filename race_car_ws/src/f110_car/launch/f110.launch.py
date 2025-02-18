from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    m2p_node = Node(
        package="f110_car",
        executable="move_to_point",
        name="move_to_point",
        parameters=[{'use_sim_time': True}],
    )

    lidar_node = Node(
        package='test_package',
        executable='lidar_node',
        name='lidar_node',
        parameters=[{'use_sim_time': True}],
    )

    yolo_node = Node(
        package='test_package',
        executable='yolo_node',
        name='yolo_node',
        parameters=[{'use_sim_time': True}],
    )

    sensor_fusion_node = Node(
        package='test_package',
        executable='sensor_fusion_node',
        name='sensor_fusion_node',
        parameters=[{'use_sim_time': True}],
    )

    mapping_node = Node(
        package='test_package',
        executable='mapping_node',
        name='mapping_node',
        parameters=[{'use_sim_time': True}],
    )

    odom_tf_node = Node(
        package='test_package',
        executable='odom_tf_node',
        name='odom_tf_node',
        parameters=[{'use_sim_time': True}],
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
        parameters=[{'use_sim_time': True}],
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
