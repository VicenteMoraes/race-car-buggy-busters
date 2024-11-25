## Simulation

The simulation package incorporates a translation layer that maps the topics from the f110 car platform to the gazebo topics and back.

### Basic Gazebo commands

Every command in gazebo fortress starts with `ign`. They changed that behavior between gazebo fortress and gazebo garden so be aware when you see someone using `gz` instead of `ign`. Both should expose somewhat the same interface.

- List all gazebo topics `ign topic -l`
- Subscribe to a gazebo topic `ign topic -e -t <topic>`
- Check the message type of a topic `ign topic -i -t <topic>`

### Topics available through the simulation

The bridge will map the following topics and convert the message types from gazebo to ros2. The following list are the topics as they are visible in ROS2 (using `ros2 topic list`). This does not cover all the topics that are available on the f110 car platform but the most important ones.

- `/camera/realsense2_camera/color/camera_info`
- `/camera/realsense2_camera/color/image_raw`
- `/camera/realsense2_camera/depth/image_rect_raw`
- `/imu`
- `/scan`

### Start simulation

The simulation can be started using the `gazebo_f110 gazebo.launch.py` launch file which is installed alongside the `gazebo_f110` package.

```sh
ros2 launch gazebo_f110 gazebo.launch.py
```

You can run the simulation on its own using the `race-car-buggy-busters/race_car_ws/src/gazebo/gazebo_f110/world/plane.sdf` world.

```sh
ign gazebo -v 4 race_car_ws/src/gazebo/gazebo_f110/world/plane.sdf
```


### Dependencies

- ROS 2 Humble
[Gazebo Fortress](https://gazebosim.org/docs/fortress/ros_installation/)

```sh
sudo apt-get update
sudo apt-get install lsb-release gnupg
```

```sh
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
```

[ROS2 Gazebo pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs)

```sh
sudo apt install ros-humble-gazebo-ros-pkgs
```

[Ros GZ Bridge](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md)

```sh
sudo apt install ros-humble-ros-gz
```

Install all of the repository packages

```sh
pip install -e avai_lab
```

Install the `race_car_ws`

```sh
cd race_car_ws
colcon build
source install/setup.bash
```

### Tutorials/Docs

- [Bounding Box Camera](https://gazebosim.org/api/sensors/9/boundingbox_camera.html)
- [SDF format](https://osrf-distributions.s3.amazonaws.com/sdformat/api/1.5.html)
- [Gazebo for Beginners](https://github.com/scole02/Guide2Gazebo)
- [Very useful StackOverflow about plugins](https://robotics.stackexchange.com/questions/103881/gazebo-plugin-location-and-documentation/103884#103884)
- [GZ Sim Repository](https://github.com/gazebosim/gz-sim/tree/gz-sim7)
    - Documentation for plugins is in the respective Header file
    - Check `/examples/worlds` for example implementations in SDF


### Message Types

The conversion from the `AckermannDriveStamped` message to the `Twist` message is done via a custom node that translates the `AckermannDriveStamped` into a `Twist` message.

| Gazebo | ROS2 |
| :--- | :--- |
| ignition.msgs.Twist | AckermannDriveStamped | 
| ignition.msgs.IMU | sensor_msgs/msg/Imu |
| ignition.msgs.CameraInfo | sensor_msgs/msg/CameraInfo |
| ignition.msgs.Image | sensor_msgs/msg/Image |


## Plugins

Some plugins are required to get access to certain services and functions in Gazebo

### ignition-gazebo-user-commands-system

[Spawn URDF Tutorial](https://gazebosim.org/docs/fortress/spawn_urdf/)

This plugin is required to get access to the `/world/<world_name>/create` service which can be used to spawn `.sdf` or `.urdf` files.

Example:

```sh
ign service -s /world/car_world/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path/to/model.sdf", name: "my_cone"
```
