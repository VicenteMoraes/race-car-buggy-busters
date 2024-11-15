# Sensor Data

## Published Topics

Here are all published topics from the racecar platform.

### Color (RGB Module)

#### /camera/realsense2_camera/color/camera_info

#### /camera/realsense2_camera/color/image_raw

#### /camera/realsense2_camera/color/metadata

### Depth (Stereo Module)

#### /camera/realsense2_camera/depth/camera_info

#### /camera/realsense2_camera/depth/image_rect_raw

#### /camera/realsense2_camera/depth/metadata

### Extrinsics

Enable Infra1 (left), Infra2 (right), Color first.

Cross-stream extrinsics: encodes the topology describing how the different devices are oriented

* float64[9] rotation Column - major 3x3 rotation matrix
* float64[3] translation Three-element translation vector, in meters

#### /camera/realsense2_camera/extrinsics/depth_to_color

#### /camera/realsense2_camera/extrinsics/depth_to_infra1

#### /camera/realsense2_camera/extrinsics/depth_to_infra2

### Left Infra Red

Enable Infra1 first.

#### /camera/realsense2_camera/infra1/camera_info

#### /camera/realsense2_camera/infra1/image_rect_raw

### Right Infra Red

Enable Infra2 first.

#### /camera/realsense2_camera/infra2/camera_info

#### /camera/realsense2_camera/infra2/image_rect_raw

### LiDAR

#### /laser_status

#### /scan

### Sensors

#### /sensors/core

#### /sensors/imu

#### /sensors/imu/core

#### /sensors/servo_position_command

### Other

#### /diagnostics

#### /tf

#### /tf_static
