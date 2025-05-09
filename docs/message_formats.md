# Message Formats

Here are all relevant message formats of the car platform.

## CameraInfo

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

### Image Acquistion Info

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

### Calibration Parameters

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

## #Operational Parameters

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

## Image

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

## LaserScan

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

## IMU

sensor_msgs/msg/Imu

This is a message to hold data from an IMU (Inertial Measurement Unit).

Accelerations should be in ```m/s^2 (not in g's)```, and rotational velocity should be in ```rad/sec```.

If the covariance of the measurement is known, it should be filled in (if all you know is the variance of each measurement, e.g. from the datasheet, just put those along the diagonal).
A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the data a covariance will have to be assumed or gotten from some other source.

If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation estimate), please set element ```0``` of the associated covariance matrix to ```-1```.
If you are interpreting this message, please check for a value of ```-1``` in the first element of each covariance matrix, and disregard the associated estimate.

```
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

geometry_msgs/Quaternion orientation
	float64 x 0
	float64 y 0
	float64 z 0
	float64 w 1
float64[9] orientation_covariance Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
	float64 x
	float64 y
	float64 z
float64[9] angular_velocity_covariance Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
	float64 x
	float64 y
	float64 z
float64[9] linear_acceleration_covariance Row major x, y z
```


## Float64

std_msgs/msg/Float64

This was originally provided as an example message. It is deprecated as of Foxy. It is recommended to create your own semantically meaningful message. However if you would like to continue using this please use the equivalent in example_msgs.

```
float64 data
```

## DiagnosticArray

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

## TFMessage

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

## Extrinsics

realsense2_camera_msgs/msg/Extrinsics

Cross-stream extrinsics: encodes the topology describing how the different devices are oriented

```
float64[9] rotation  Column - major 3x3 rotation matrix
float64[3] translation  Three-element translation vector, in meters
```

## Status

urg_node_msgs/msg/Status

Normal vs setting in the UAM manual.

```
uint16 NORMAL=0
uint16 SETTING=1
uint16 operating_mode

uint16 area_number The configured area number the stop occurred in.
bool error_status If the laser is reporting an error or not.
uint16 error_code The error code the laser is reporting.
bool lockout_status Does the laser report that it is locked out.
uint16 distance Distance in mm the stop was reported at.
float32 angle The reported angle of the stop in deg.
```

## VESC State Stamped

Timestamped VESC open source motor controller state (telemetry)

```
std_msgs/Header  header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
VescState state
	int32 FAULT_CODE_NONE=0
	int32 FAULT_CODE_OVER_VOLTAGE=1
	int32 FAULT_CODE_UNDER_VOLTAGE=2
	int32 FAULT_CODE_DRV8302=3
	int32 FAULT_CODE_ABS_OVER_CURRENT=4
	int32 FAULT_CODE_OVER_TEMP_FET=5
	int32 FAULT_CODE_OVER_TEMP_MOTOR=6
	float64 temp_fet             
	float64 temp_motor           
	float64 current_motor        
	float64 current_input        
	float64 avg_id
	float64 avg_iq
	float64 duty_cycle           
	float64 speed                
	float64 voltage_input        
	float64 charge_drawn         
	float64 charge_regen         
	float64 energy_drawn         
	float64 energy_regen         
	int32   displacement         
	int32   distance_traveled    
	int32   fault_code
	float64 pid_pos_now
	int32 controller_id
	float64 ntc_temp_mos1
	float64 ntc_temp_mos2
	float64 ntc_temp_mos3
	float64 avg_vd
	float64 avg_vq
```

## VESC IMU Stamped

```
std_msgs/Header  header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
VescImu imu
	geometry_msgs/Vector3  ypr
		float64 x
		float64 y
		float64 z
	geometry_msgs/Vector3  linear_acceleration
		float64 x
		float64 y
		float64 z
	geometry_msgs/Vector3  angular_velocity
		float64 x
		float64 y
		float64 z
	geometry_msgs/Vector3  compass
		float64 x
		float64 y
		float64 z
	geometry_msgs/Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1
```
