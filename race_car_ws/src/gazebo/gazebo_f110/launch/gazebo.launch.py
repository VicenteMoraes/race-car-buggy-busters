from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    pkg_gazebo_f110 = get_package_share_directory('gazebo_f110')
    
    wasd_node = Node(
            package="test_package",
            namespace="f110",
            executable="wasd_control_node",
            name="wasd_control",
            parameters=[{'use_sim_time': True}],
            )
    m2p_node = Node(
            package="f110_car",
            namespace="f110",
            executable="move_to_point",
            name="move_to_point",
            parameters=[{'use_sim_time': True}],
            )
    ackermann_to_twist_node = Node(
            package="gazebo_f110",
            namespace="gazebo",
            executable="ackermann_to_twist",
            name="ackermann_to_twist",
            parameters=[{'use_sim_time': True}],
            )
    world_pose_to_odom_node = Node(
            package="gazebo_f110",
            namespace="gazebo",
            executable="world_pose_to_odom",
            name="world_pose_to_odom",
            parameters=[{'use_sim_time': True}],
            )
    lidar_node = Node(
            package="test_package",
            namespace="f110",
            executable="lidar_node",
            name="lidar_node",
            parameters=[{'use_sim_time': True}],
        )
    yolo_node = Node(
            package="test_package",
            namespace="f110",
            executable="yolo_node",
            name="yolo_node",
            parameters=[{'use_sim_time': True}],
        )
    sensor_fusion_node = Node(
            package="test_package",
            namespace="f110",
            executable="sensor_fusion_node",
            name="sensor_fusion_node",
            parameters=[{'use_sim_time': True}],
        )
    mapping_node = Node(
            package="test_package",
            namespace="f110",
            executable="mapping_node",
            name="mapping_node",
            parameters=[{'use_sim_time': True}],
        )
    odom_tf_node = Node(
            package='test_package',
            namespace="f110",
            executable='odom_tf_node',
            name='odom_tf_node',
            parameters=[{'use_sim_time': True}],
        )
    ros_gz_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist", 
                "/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
                "/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
                "/rgbd_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
                "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                "/rgbd_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
                "/rgbd_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
                "/world/car_world/pose/info@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V",
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                ],
            remappings=[
                ("/rgbd_camera/camera_info", "/camera/realsense2_camera/color/camera_info"),
                ("/rgbd_camera/image", "/camera/realsense2_camera/color/image_raw"),
                ("/rgbd_camera/depth_image", "/camera/realsense2_camera/depth/image_rect_raw"),
                ],
            output='screen',
            parameters=[{'use_sim_time': True}],
            )

    gazebo_launch_group = GroupAction(
            actions = [
                DeclareLaunchArgument(
                    'world',
                    default_value='plane',
                    choices=['plane', 'circle'],
                    description='World to load into Gazebo'
                    ),
                SetLaunchConfiguration(name='world_file', 
                                       value=[LaunchConfiguration('world'), 
                                              TextSubstitution(text='.sdf')]),
               IncludeLaunchDescription(
                   PythonLaunchDescriptionSource(gz_launch_path),
                   launch_arguments={
                       'ign_args': [PathJoinSubstitution([pkg_gazebo_f110, "world", LaunchConfiguration('world_file')])],
                       'on_exit_shutdown': 'True'
                       }.items(),
                   )
               ]
            )
    
    #TF Transforms
    static_tf_base_link_lidar_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_lidar_link',
        arguments=['0.12', '0', '0.055', '0', '0', '0', 'base_link', 'f110_car/lidar_link'],
        parameters=[{'use_sim_time': True}],
    )

    static_tf_lidar_link_gpu_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar_link_gpu_lidar',
        arguments=['0', '0', '0', '0', '0', '0', 'f110_car/lidar_link', 'f110_car/lidar_link/gpu_lidar'],
        parameters=[{'use_sim_time': True}],
    )
    static_tf_base_link_realsense_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_realsense_link',
        arguments=['0', '0', '0.1', '0', '0', '0', 'f110_car/lidar_link', 'f110_car/realsense_link'],
        parameters=[{'use_sim_time': True}],
    )
    static_tf_realsense_link_realsense_d435 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_realsense_link_realsense_d435',
        arguments=['0', '0', '0', '0', '0', '0', 'f110_car/realsense_link', 'f110_car/realsense_link/realsense_d435'],
        parameters=[{'use_sim_time': True}],
    )
    static_tf_realsense_link_depth_camera1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_realsense_link_depth_camera1',
        arguments=['0', '0', '0', '0', '0', '0', 'f110_car/realsense_link', 'f110_car/realsense_link/depth_camera1'],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo_launch_group,
        wasd_node,
        #m2p_node,
        lidar_node,
        yolo_node,
        sensor_fusion_node,
        mapping_node,
        world_pose_to_odom_node,
        odom_tf_node,
        static_tf_base_link_lidar_link,
        static_tf_lidar_link_gpu_lidar,
        static_tf_base_link_realsense_link,
        static_tf_realsense_link_realsense_d435,
        static_tf_realsense_link_depth_camera1,
        ackermann_to_twist_node,
        ros_gz_bridge_node,
        ])
