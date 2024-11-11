# Sensor Data

## HOKUYO UST-10 (2D LiDAR)

Moved to a temp section. 

## Intel RealSense Depth Camera D435i (Depth + RGB Data + IR Data)

### Published Topics

#### Color (RGB Module)

##### /camera/camera/color/camera_info

##### /camera/camera/color/image_raw

##### /camera/camera/color/metadata

#### Depth (Stereo Module)

##### /camera/camera/depth/camera_info

##### /camera/camera/depth/image_rect_raw

##### /camera/camera/depth/metadata

#### Extrinsics

Enable Infra1 (left), Infra2 (right), Color first.

Cross-stream extrinsics: encodes the topology describing how the different devices are oriented

* float64[9] rotation Column - major 3x3 rotation matrix
* float64[3] translation Three-element translation vector, in meters

##### /camera/camera/extrinsics/depth_to_color

##### /camera/camera/extrinsics/depth_to_infra1

##### /camera/camera/extrinsics/depth_to_infra2

##### /camera/camera/extrinsics/depth_to_accel

##### /camera/camera/extrinsics/depth_to_gyro

#### Left Infra Red

Enable Infra1 first.

##### /camera/camera/infra1/camera_info

##### /camera/camera/infra1/image_rect_raw

#### Right Infra Red

Enable Infra2 first.

##### /camera/camera/infra2/camera_info

##### /camera/camera/infra2/image_rect_raw

#### Point Cloud

Enable pointcloud first.

##### /camera/camera/depth/color/points

#### Aligned Streams

Enable align_depth filter first.

##### /camera/camera/aligned_depth_to_color/camera_info

##### /camera/camera/aligned_depth_to_color/image_raw

#### IMU

After setting the parameter unite_imu_method new topics are published in /camera/imu

header.frame_id is either set to "imu_accel" or "imu_gyro" to distinguish between "accel" and "gyro" info.

* std_msgs/Header header
* float64[12] data
* float64[3] noise_variances
* float64[3] bias_variances

##### /camera/camera/gyro/imu_info

##### /camera/camera/gyro/metadata

##### /camera/camera/gyro/sample

##### /camera/camera/accel/imu_info

##### /camera/camera/accel/metadata

##### /camera/camera/accel/sample

##### /camera/camera/imu

#### Other

##### /diagnostics

##### /tf

##### /tf_static

### Compression packages

The compression packages apply to all the camera Modules (SKUs). To enable compressed topics, user should install image-transport-plugin package (For example, for humble distro run this command for installation ```sudo apt install ros-humble-image-transport```)

### RGBD Topic

RGBD is a new topic, publishing (RGB + Depth) in the same message. For now, works only with depth aligned to color images, as color and depth images are synchronized by frame time tag.

RGBD Message
* std_msgs/Header header
* sensor_msgs/CameraInfo rgb_camera_info
* sensor_msgs/CameraInfo depth_camera_info
* sensor_msgs/Image rgb
* sensor_msgs/Image depth
