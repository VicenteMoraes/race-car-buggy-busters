# Sensor Data

## Published Topics

Here are all published topics from the racecar platform.

### Color (RGB Module)

#### /camera/realsense2_camera/color/camera_info

[sensor_msgs/msg/CameraInfo](#camerainfo)

- frame_id: camera_color_opical_frame
- height: 720
- width: 1280
- disortion_model: plumb_bob
- d: 
	- 0.0 
	- 0.0 
	- 0.0 
	- 0.0 0
	- .0
- k: 
	- 905.5966796875 
	- 0.0 
	- 644.2418823242188 
	- 0.0 
	- 905.8685302734375 
	- 378.13494873046875 
	- 0.0 
	- 0.0 
	- .0
- r: 
	- 1.0 
	- 0.0 
	- 0.0 
	- 0.0 
	- 1.0 
	- 0.0 
	- 0.0 
	- 0.0 
	- 1.0
- p: 
	- 905.5966796875 
	- 0.0 644.2418823242188 
	- 0.0 0.0 905.8685302734375 
	- 378.13494873046875 
	- 0.0 
	- 0.0 
	- 0.0 
	- 1.0 
	- 0.0
- binning_x: 0
- binning_y: 0
- roi: 
	- x_offset: 0 
	- y_offset: 0 
	- height: 0 
	- width: 0 
	- do_rectify: false

#### /camera/realsense2_camera/color/image_raw

[sensor_msgs/msg/Image](#image)

- frame_id: camera_color_optical_frame
- height: 720
- width: 1280
- encoding: rgb8 -> 8-bit rgb
- is_bigendian: 0
- data: values **should** lie within the range of [0, 127]
	- no indication on what is "black" and what is "white" or the ordering of data (RBG per pixel (red value, then blue, then green) or RBG per array (red array, then blue, then green)
	- data is too big to be displayed in an echo (it's 720 x 1280 x 3 data points)

#### /camera/realsense2_camera/color/metadata

### Depth (Stereo Module)

#### /camera/realsense2_camera/depth/camera_info

[sensor_msgs/msg/CameraInfo](#camerainfo)

- frame_id: camera_depth_optical_frame
- height: 480
- width: 848
- distortion_model: plumb_bob
- d: 
	- 0.0 
	- 0.0 
	- 0.0 
	- 0.0 
	- 0.0
- k: 
	- 428.4230651855469 
	- 0.0 
	- 426.15936279296875 
	- 0.0 
	- 428.4230651855469 
	- 235.37384033203125 
	- 0.0 
	- 0.0 
	- 1.0
- r: 
	- 1.0
	- 0.0
	- 0.0
	- 0.0
	- 1.0
	- 0.0
	- 0.0
	- 0.0
	- 1.0
- p:
	- 428.4230651855469
	- 0.0
	- 426.15936279296875
	- 0.0
	- 0.0
	- 428.4230651855469
	- 235.37384033203125
	- 0.0
	- 0.0
	- 0.0
	- 1.0
	- 0.0
- binning_x: 0
- binning_y: 0
- roi:
 	- x_offset: 0
 	- y_offset: 0
 	- height: 0
 	- width: 0
 	- do_rectify: false

#### /camera/realsense2_camera/depth/image_rect_raw

[sensor_msgs/msg/Image](#image)

- frame_id: camera_depth_optical_frame
- height: 480
- width: 848
- encoding: 16UC1 -> cv_16uc1 (16-bit unsigned greyscale)
- is_bigendian: 0
- data: values **should** lie within the range of [0, 255]
	- data is too big to be displayed in an echo (it's 480 x 848 data points)
	- here the values (at least the echo'ed part) ist zero'ed out!


#### /camera/realsense2_camera/depth/metadata

### Extrinsics

- Enable Infra1 (left), Infra2 (right), Color first.

Cross-stream extrinsics: encodes the topology describing how the different devices are oriented

```
float64[9] rotation Column - major 3x3 rotation matrix
float64[3] translation Three-element translation vector, in meters
```

#### /camera/realsense2_camera/extrinsics/depth_to_color

#### /camera/realsense2_camera/extrinsics/depth_to_infra1

#### /camera/realsense2_camera/extrinsics/depth_to_infra2

### Left Infra Red

- Enable Infra1 first.

#### /camera/realsense2_camera/infra1/camera_info

[sensor_msgs/msg/CameraInfo](#camerainfo)

- frame_id: camera_infra1_optical_frame
- height: 480
- width: 848
- distortion_model: plumb_bob
- d:
	- 0.0
	- 0.0
	- 0.0
	- 0.0
	- 0.0
- k:
	- 428.4230651855469
	- 0.0
	- 426.15936279296875
	- 0.0
	- 428.4230651855469
	- 235.37384033203125
	- 0.0
	- 0.0
	- 1.0
- r:
	- 1.0
	- 0.0
	- 0.0
	- 0.0
	- 1.0
	- 0.0
	- 0.0
	- 0.0
	- 1.0
- p:
	- 428.4230651855469
	- 0.0
	- 426.15936279296875
	- 0.0
	- 0.0
	- 428.4230651855469
	- 235.37384033203125
	- 0.0
	- 0.0
	- 0.0
	- 1.0
	- 0.0
- binning_x: 0
- binning_y: 0
- roi:
 	- x_offset: 0
 	- y_offset: 0
 	- height: 0
 	- width: 0
 	- do_rectify: false

#### /camera/realsense2_camera/infra1/image_rect_raw

[sensor_msgs/msg/Image](#image)

- frame_id: camera_infra1_optical_frame
- height: 480
- width: 848
- encoding: mono8 -> 8-bit greyscale
- is_bigendian: 0
- data: values **should** lie within the range of [0, 127]
	- data is too big to be displayed in an echo (it's 480 x 848 data points)

### Right Infra Red

- Enable Infra2 first.

#### /camera/realsense2_camera/infra2/camera_info

[sensor_msgs/msg/CameraInfo](#camerainfo)

- frame_id: -camera_infra1_optical_frame-
- height: 480
- width: 848
- distortion_model: plumb_bob
- d:
	- 0.0
	- 0.0
	- 0.0
	- 0.0
	- 0.0
- k:
	- 428.4230651855469
	- 0.0
	- 426.15936279296875
	- 0.0
	- 428.4230651855469
	- 235.37384033203125
	- 0.0
	- 0.0
	- 1.0
- r:
	- 1.0
	- 0.0
	- 0.0
	- 0.0
	- 1.0
	- 0.0
	- 0.0
	- 0.0
	- 1.0
- p:
	- 428.4230651855469
	- 0.0
	- 426.15936279296875
	- -21.4457950592041
	- 0.0
	- 428.4230651855469
	- 235.37384033203125
	- 0.0
	- 0.0
	- 0.0
	- 1.0
	- 0.0
- binning_x: 0
- binning_y: 0
- roi:
  	- x_offset: 0
  	- y_offset: 0
  	- height: 0
  	- width: 0
  	- do_rectify: false

#### /camera/realsense2_camera/infra2/image_rect_raw

[sensor_msgs/msg/Image](#image)

- frame_id: camera_infra2_optical_frame
- height: 480
- width: 848
- encoding: mono8 -> 8-bit greyscale
- is_bigendian: 0
- step: 848
- data: values **should** lie within the range of [0, 127]
	- data is too big to be displayed in an echo (it's 480 x 848 data points)

### LiDAR

#### /laser_status

#### /scan

[sensor_msgs/msg/LaserScan](#laserscan)

- frame_id: laser
- angle_min: -2.356194496154785 -> in radians
- angle_max: 2.356194496154785 -> in radians
- angle_increment: 0.004363323096185923 -> in radians
- time_increment: 1.736111516947858-e-05- -> time between measurements (angle increments in seconds)
- scan_time: 0.02500000037252903 -> in seconds
- range_min: 0.019999999552965164 -> in meters
- range_max: 30.0 -> in meters
- ranges: 
	- **should** lie between ```range_min``` and ```range_max```
	- other values should be discarded
	- data is too big to be displayed in an echo ( ~ 1030 measurements in one scan)
- intensities: [] -> not provided

### Sensors

#### /sensors/core

#### /sensors/imu

#### /sensors/imu/core

#### /sensors/servo_position_command

[--std_msgs/msg/Float64](#float64)

- data: some floating point number starting with ```0.``` (range probably between 0 and 1)


### Other

#### /diagnostics

[diagnostic_msgs/msg/DiagnosticArray](#diagnosticarray)

##### Ackermann MUX

- frame_id: ''
- status:
	- level: "\0"
	- name: 'ackermann_mux: Ackermann mux status'
	- message: ok
	- hardware_id: none
	- values:
		  - key: velocity topics.joystick
		  - value: ' unmasked (listening to teleop @ 0.200000s with priority #100)'
		  - key: velocity topics.navigation
		  - value: ' masked (listening to drive @ 0.200000s with priority #10)'
		  - key: current priority
		  - value: '0'
		  - key: loop time in sec
		  - value: '0'
		  - key: data age in sec
		    value: '0'

##### LiDAR

- frame_id: ''
- status:
	- level: "\0"
  	- name: 'urg_node: Hardware Status'
  	- message: Streaming
  	- hardware_id: B2227176
  	- values:
		  - key: IP Address
		  - value: 192.168.0.10
		  - key: IP Port
		  - value: '10940'
		  - key: Vendor Name
		  - value: Hokuyo Automatic Co., Ltd.
		  - key: Product Name
		  - value: UST-10LX
		  - key: Firmware Version
		  - value: **receive error.**
		  - key: Firmware Date
		  - value: **receive error.**
		  - key: Protocol Version
		  - value: SCIP 2.2
		  - key: Device ID
		  - value: B2227176
		  - key: Computed Latency
		  - value: '0'
		  - key: User Time Offset
		  - value: '0'
		  - key: Device Status
		  - value: sensor is working normally
		  - key: Scan Retrieve Error Count
		  - value: '0'
		  - key: Lidar Error Code
		  - value: '0'
		  - key: Locked out
		  - value: 'False'
	- level: "\0"
	- name: 'urg_node: Laser Scan topic status'
	- message: ''
	- hardware_id: B2227176
	- values:
		  - key: Events in window
		  - value: '91'
		  - key: Events since startup
		  - value: '21456'
		  - key: Duration of window (s)
		  - value: '2.287878'
		  - key: Actual frequency (Hz)
		  - value: '39.774852'
		  - key: Target frequency (Hz)
		  - value: '40.000000'
		  - key: Minimum acceptable frequency (Hz)
		  - value: '38.000000'
		  - key: Maximum acceptable frequency (Hz)
		  - value: '42.000000'

#### /tf

[tf2_msgs/msg/TFMessage](#tfmessage)

- frame_id: odom
- child_frame_id: base_link
- transform:
	- translation:
		- x: -5.116210464579059
		- y: 0.47674305243408466
		- z: 0.0
	- rotation:
		- x: 0.0
		- y: 0.0
		- z: -0.15724700085429322
		- w: -0.9875593049140542

#### /tf_static

[tf2_msgs/msg/TFMessage](#tfmessage)

##### Camera Tranforms

Static relations between the different image data (camera, depth, infra1, infra2)

###### Camera Link to Infra1 Frane

- frame_id: camera_link
- child_frame_id: camera_infra1_frame
- transform:
	- translation:
		- x: 0.0
		- y: -0.0
		- z: -0.0
	- rotation:
		- x: 0.0
		- y: 0.0
		- z: 0.0
		- w: 1.0

###### Aligned Depth to Infra1 Frame to Infra1 Opical Frame

- frame_id: camera_aligned_depth_to_infra1_frame
- child_frame_id: camera_infra1_optical_frame
- transform:
	- translation:
		- x: 0.0
		- y: -0.0
		- z: -0.0
	- rotation:
		- x: -0.5
		- y: 0.4999999999999999
		- z: -0.5
		- w: 0.5000000000000001

###### Camera Link to Aligned Depth to Infra1 Frame

- frame_id: camera_link
- child_frame_id: camera_aligned_depth_to_infra1_frame
- transform:
	- translation:
		- x: 0.0
		- y: -0.0
		- z: -0.0
	- rotation:
		- x: 0.0
		- y: 0.0
		- z: 0.0
		- w: 1.0

###### Camera Link to Infra2 Frame

- frame_id: camera_link
- child_frame_id: camera_infra2_frame
- transform:
	- translation:
		- x: 0.0
		- y: -0.05005751922726631
		- z: -0.0
	- rotation:
		- x: 0.0
		- y: 0.0
		- z: 0.0
		- w: 1.0

###### Infra2 Frame to Infra2 Optical Frame

- frame_id: camera_infra2_frame
- child_frame_id: camera_infra2_optical_frame
- transform:
	- translation:
		- x: 0.0
		- y: -0.0
		- z: -0.0
	- rotation:
		- x: -0.5
		- y: 0.4999999999999999
		- z: -0.5
		- w: 0.5000000000000001

###### Camera Link to Depth Frame

- frame_id: camera_link
- child_frame_id: camera_depth_frame
- transform:
	- translation:
		- x: 0.0
		- y: -0.0
		- z: -0.0
	- rotation:
		- x: 0.0
		- y: 0.0
		- z: 0.0
		- w: 1.0

###### Depth Frame to Depth Optical Frame

- frame_id: camera_depth_frame
- child_frame_id: camera_depth_optical_frame
- transform:
	- translation:
		- x: 0.0
		- y: -0.0
		- z: -0.0
	- rotation:
		- x: -0.5
		- y: 0.4999999999999999
		- z: -0.5
		- w: 0.5000000000000001

###### Camera Link to Color Frame

- frame_id: camera_link
- child_frame_id: camera_color_frame
- transform:
	- translation:
		- x: -0.0004415937000885606
		- y: 0.014996647834777832
		- z: -1.0729166206147056 **e-05**
	- rotation:
		- x: 0.006116042379289865
		- y: 0.00262435688637197
		- z: -0.000178089045220986
		- w: 0.9999778270721436

###### Color Frame to Color Optical Frame

- frame_id: camera_color_frame
- child_frame_id: camera_color_optical_frame
- transform:
	- translation:
		- x: 0.0
		- y: -0.0
		- z: -0.0
	- rotation:
		- x: -0.5
		- y: 0.4999999999999999
		- z: -0.5
		- w: 0.5000000000000001

##### Laser

- frame_id: base_link
- child_frame_id: laser
- transform:
	- translation:
		- x: 0.27
		- y: 0.0
		- z: 0.11
	- rotation:
		- x: 0.0
		- y: 0.0
		- z: 0.0
		- w: 1.0


## Message Formats

### CameraInfo

sensor_msgs/msg/CameraInfo

This message defines meta information for a camera. It should be in a camera namespace on topic ```"camera_info"``` and accompanied by up to five image topics named:

```
  image_raw - raw data from the camera driver, possibly Bayer encoded
  image            - monochrome, distorted
  image_color      - color, distorted
  image_rect       - monochrome, rectified
  image_rect_color - color, rectified 
```
 
The ```image_pipeline``` contains packages (image_proc, stereo_image_proc) for producing the four processed image topics from ```image_raw``` and ```camera_info```. The meaning of the camera parameters are described in detail at [ROS CameraInfo](http://www.ros.org/wiki/image_pipeline/CameraInfo).

The ```image_geometry``` package provides a user-friendly interface to common operations using this meta information. If you want to, e.g., project a 3d point into image coordinates, we strongly recommend using image_geometry.

If the camera is uncalibrated, the matrices D, K, R, P should be left zeroed out. In particular, clients may assume that ```K[0] == 0.0``` indicates an uncalibrated camera.

#### Image Acquistion Info

Time of image acquisition, camera coordinate frame ID

```
std_msgs/Header header  Header timestamp should be acquisition time of image
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
                             Header frame_id should be optical frame of camera
                             origin of frame should be optical center of camera
                             +x should point to the right in the image
                             +y should point down in the image
                             +z should point into the plane of the image
```

#### Calibration Parameters

These are fixed during camera calibration. Their values will be the  same in all messages until the camera is recalibrated. Note that self-calibrating systems may "recalibrate" frequently.  
                                                                               
The internal parameters can be used to warp a raw (distorted) image to:            
  1. An undistorted image (requires D and K)                        
  2. A rectified image (requires D, K, R)                           
The projection matrix P projects 3D points into the rectified image.

The image dimensions with which the camera was calibrated.
Normally this will be the full camera resolution in pixels.

```
uint32 height
uint32 width
```

The distortion model used. Supported models are listed in ```sensor_msgs/distortion_models.hpp```. For most cameras, ```"plumb_bob"``` - a simple model of radial and tangential distortion - is sufficent. 

```
string distortion_model
```

The distortion parameters, size depending on the distortion model. For ```"plumb_bob"```, the 5 parameters are: ```(k1, k2, t1, t2, k3)```.

```
float64[] d
```

Intrinsic camera matrix for the raw (distorted) images.

```
    [fx  0 cx]
K = [ 0 fy cy]
    [ 0  0  1]
```

Projects 3D points in the camera coordinate frame to 2D pixel coordinates using the focal lengths ```(fx, fy)``` and principal point ```(cx, cy)```.

```
float64[9]  k  3x3 row-major matrix
```

Rectification matrix (stereo cameras only)

A rotation matrix aligning the camera coordinate system to the ideal
stereo image plane so that epipolar lines in both stereo images are
parallel.

```
float64[9]  r  3x3 row-major matrix
```

Projection/camera matrix

```
    [fx'  0  cx' Tx]
P = [ 0  fy' cy' Ty]
    [ 0   0   1   0]
```

By convention, this matrix specifies the intrinsic (camera) matrix of the processed (rectified) image. That is, the left 3x3 portion is the normal camera intrinsic matrix for the rectified image.

It projects 3D points in the camera coordinate frame to 2D pixel coordinates using the focal lengths ```(fx', fy')``` and principal point ```(cx', cy')``` - these may differ from the values in K.
For monocular cameras, ```Tx = Ty = 0```. Normally, monocular cameras will also have ```R = the identity and P[1:3,1:3] = K```.

For a stereo pair, the fourth column ```[Tx Ty 0]'``` is related to the position of the optical center of the second camera in the first camera's frame. We assume ```Tz = 0``` so both cameras are in the same stereo image plane. The first camera always has ```Tx = Ty = 0```. For the right (second) camera of a horizontal stereo pair, ```Ty = 0``` and ```Tx = -fx' - B```, where ```B``` is the baseline between the cameras.

Given a 3D point ```[X Y Z]'```, the projection ```(x, y)``` of the point onto the rectified image is given by:

```
[u v w]' = P - [X Y Z 1]'
       x = u / w
       y = v / w
```

This holds for both images of a stereo pair.

```
float64[12] p 3x4 row-major matrix
```

#### Operational Parameters

These define the image region actually captured by the camera driver. Although they affect the geometry of the output image, they  may be changed freely without recalibrating the camera.             

Binning refers here to any camera setting which combines rectangular neighborhoods of pixels into larger "super-pixels." It reduces the resolution of the output image to ```(width / binning_x) x (height / binning_y)```.

The default values ```binning_x = binning_y = 0``` is considered the same as ```binning_x = binning_y = 1``` (no subsampling).

```
uint32 binning_x
uint32 binning_y
```

Region of interest (subwindow of full camera resolution), given in full resolution (unbinned) image coordinates. A particular ROI always denotes the same window of pixels on the camera sensor, regardless of binning settings.

The default setting of roi (all values 0) is considered the same as full resolution (```roi.width = width, roi.height = height```).

```
RegionOfInterest roi
	
	uint32 x_offset  
	                  (0 if the ROI includes the left edge of the image)
	uint32 y_offset  
	                  (0 if the ROI includes the top edge of the image)
	uint32 height    
	uint32 width     
	bool do_rectify
```

### Image

sensor_msgs/msg/Image

This message contains an uncompressed image ```(0, 0)``` is at top-left corner of image

```
std_msgs/Header header  Header timestamp should be acquisition time of image
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
                             Header frame_id should be optical frame of camera
                             origin of frame should be optical center of cameara
                             +x should point to the right in the image
                             +y should point down in the image
                             +z should point into to plane of the image
                             If the frame_id here and the frame_id of the CameraInfo
                             message associated with the image conflict
                             the behavior is undefined

uint32 height                image height, that is, number of rows
uint32 width                 image width, that is, number of columns
```

The legal values for encoding are in file ```src/image_encodings.cpp```

```
string encoding       Encoding of pixels -- channel meaning, ordering, size
                      taken from the list of strings in include/sensor_msgs/image_encodings.hpp

uint8 is_bigendian    is this data bigendian?
uint32 step           Full row length in bytes
uint8[] data          actual matrix data, size is (step - rows)
```

### LaserScan

sensor_msgs/msg/LaserScan

Single scan from a planar laser range-finder

If you have another ranging device with different behavior (e.g. a sonar array), please find or create a different message, since applications will make fairly laser-specific assumptions about this data

```
std_msgs/Header header  timestamp in the header is the acquisition time of
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
                             the first ray in the scan.
                             in frame frame_id, angles are measured around
                             the positive Z axis (counterclockwise, if Z is up)
                             with zero angle being forward along the x axis

float32 angle_min            start angle of the scan [rad]
float32 angle_max            end angle of the scan [rad]
float32 angle_increment      angular distance between measurements [rad]

float32 time_increment       time between measurements [seconds] - if your scanner
                             is moving, this will be used in interpolating position
                             of 3d points
float32 scan_time            time between scans [seconds]

float32 range_min            minimum range value [m]
float32 range_max            maximum range value [m]

float32[] ranges             range data [m]
                             (Note: values < range_min or > range_max should be discarded)
float32[] intensities        intensity data [device-specific units].  If your
                             device does not provide intensities, please leave
                             the array empty.                             
```

### Float64

std_msgs/msg/Float64

This was originally provided as an example message. It is deprecated as of Foxy. It is recommended to create your own semantically meaningful message. However if you would like to continue using this please use the equivalent in example_msgs.

```
float64 data
```

### DiagnosticArray

diagnostic_msgs/msg/DiagnosticArray

This message is used to send diagnostic information about the state of the robot.

```
std_msgs/Header header for timestamp
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
DiagnosticStatus[] status an array of components being reported on
	byte OK=0
	byte WARN=1
	byte ERROR=2
	byte STALE=3
	byte level
	string name
	string message
	string hardware_id
	KeyValue[] values
		string key
		string value
```

### TFMessage

tf2_msg/msg/TFMessage

```
geometry_msgs/TransformStamped[] transforms
	
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	string child_frame_id
	Transform transform
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
```
