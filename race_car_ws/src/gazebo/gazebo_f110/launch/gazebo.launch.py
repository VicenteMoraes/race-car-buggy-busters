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
    slam_toolbox_config = PathJoinSubstitution([pkg_gazebo_f110, "mapper_params_online_async.yaml"])
    
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
    transform_node = Node(
        package="gazebo_f110",
        namespace="gazebo",
        executable="transform_pose",
        name="transform_pose",
        parameters=[{'use_sim_time': True}],
    )
    rviz = Node(
            package="rviz2",
            namespace="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", PathJoinSubstitution([pkg_gazebo_f110, "rviz_config.rviz"])],
            parameters=[{'use_sim_time': True}],
            )
    slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory("slam_toolbox"),
                                                                "launch", "online_async_launch.py"])),
            launch_arguments={
                "slam_params_file": slam_toolbox_config,
                "use_sim_time": "true"
                }.items()
            )

    ros_gz_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist", 
                "/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
                "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                "/camera@sensor_msgs/msg/Image@ignition.msgs.Image",
                "/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
                "/depth_camera@sensor_msgs/msg/Image@ignition.msgs.Image",
                #"/model/f110_car/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                "/world/car_world/pose/info@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V",
                ],
            remappings=[
                ("/camera_info", "/camera/realsense2_camera/color/camera_info"),
                ("/camera", "/camera/realsense2_camera/color/image_raw"),
                ("/depth_camera", "/camera/realsense2_camera/depth/image_rect_raw"),
                #("/model/f110_car/odometry", "/odom"),
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
                       'on_exit_shutdown': 'True',
                       'use_sim_time': 'True'
                       }.items(),
                   )
               ]
            )
    transforms = GroupAction(
            actions = [
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='gpu_lidar_tf',
                    output='screen',
                    arguments=[
                        '0', '0', '0', '0.0', '0.0', '0.0',
                        'lidar_link', 'gpu_lidar'
                        ],
                    parameters=[{'use_sim_time': True}],
                    ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='map_scan_tf',
                    output='screen',
                    arguments=[
                        '0', '0', '0', '0.0', '0.0', '0.0',
                        'map', 'scan'
                       ],
                    parameters=[{'use_sim_time': True}],
                    ),
                ])
    return LaunchDescription([
        gazebo_launch_group,
        #m2p_node,
        world_pose_to_odom_node,
        wasd_node,
        transforms,
        transform_node,
        ackermann_to_twist_node,
        ros_gz_bridge_node,
        slam_launch,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher',
             output='screen',
             arguments=[PathJoinSubstitution([pkg_gazebo_f110, "model", "f110_car.sdf"])]),
        rviz
        ])
