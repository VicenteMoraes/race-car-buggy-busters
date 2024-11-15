# Sensor Data

## Published Topics

Here are all published topics from the racecar platform.

### Color (RGB Module)

#### /camera/camera/color/camera_info

#### /camera/camera/color/image_raw

#### /camera/camera/color/metadata

### Depth (Stereo Module)

#### /camera/camera/depth/camera_info

#### /camera/camera/depth/image_rect_raw

#### /camera/camera/depth/metadata

### Extrinsics

Enable Infra1 (left), Infra2 (right), Color first.

Cross-stream extrinsics: encodes the topology describing how the different devices are oriented

* float64[9] rotation Column - major 3x3 rotation matrix
* float64[3] translation Three-element translation vector, in meters

#### /camera/camera/extrinsics/depth_to_color

#### /camera/camera/extrinsics/depth_to_infra1

#### /camera/camera/extrinsics/depth_to_infra2

### Left Infra Red

Enable Infra1 first.

#### /camera/camera/infra1/camera_info

#### /camera/camera/infra1/image_rect_raw

### Right Infra Red

Enable Infra2 first.

#### /camera/camera/infra2/camera_info

#### /camera/camera/infra2/image_rect_raw

### Other

#### /diagnostics

#### /tf

#### /tf_static
