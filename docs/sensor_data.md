# Sensor Data

## Published Topics

Here are example echos of all relevant published topics from the racecar platform.

All echos starting with frame_id have a header of the shape

```
header
	stamp
		sec
		nanosec
	frame_id
```

### Color (RGB Module)

#### /camera/realsense2_camera/color/camera_info

[sensor_msgs/msg/CameraInfo](message_formats.md#camerainfo)

- frame_id: camera_color_opical_frame
- height: 720
- width: 1280
- disortion_model: plumb_bob
- d: 
	- 0.0 
	- 0.0 
	- 0.0 
	- 0.0 
	- 0.0
- k: 
	- 905.5966796875 
	- 0.0 
	- 644.2418823242188 
	- 0.0 
	- 905.8685302734375 
	- 378.13494873046875 
	- 0.0 
	- 0.0 
	- 0.0
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
	- 0.0 
	- 644.2418823242188 
	- 0.0 
	- 0.0 
	- 905.8685302734375 
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

[sensor_msgs/msg/Image](message_formats.md#image)

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

[sensor_msgs/msg/CameraInfo](message_formats.md#camerainfo)

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

[sensor_msgs/msg/Image](message_formats.md#image)

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

[realsense2_camera_msgs/msg/Extrinsics](message_formats.md#extrinsics)

- rotation:
	- 0.9999251365661621
	- -0.012232748791575432
	- 0.0003240688529331237
	- 0.012230878695845604
	- 0.9999114274978638
	- 0.005250775720924139
	- -0.0003882715536747128
	- -0.005246418993920088
	- 0.9999861717224121
- translation:
	- 0.01499522291123867
	- -0.0001964952243724838
	- 0.000446391204604879

#### /camera/realsense2_camera/extrinsics/depth_to_infra1

[realsense2_camera_msgs/msg/Extrinsics](message_formats.md#extrinsics)

- rotation:
	- 1.0
	- 0.0
	- 0.0
	- 0.0
	- 1.0
	- 0.0
	- 0.0
	- 0.0
	- 1.0
- translation:
	- 0.0
	- 0.0
	- 0.0

#### /camera/realsense2_camera/extrinsics/depth_to_infra2

[realsense2_camera_msgs/msg/Extrinsics](message_formats.md#extrinsics)

- rotation:
	- 1.0
	- 0.0
	- 0.0
	- 0.0
	- 1.0
	- 0.0
	- 0.0
	- 0.0
	- 1.0
- translation:
	- -0.05005751922726631
	- 0.0
	- 0.0

### Left Infra Red

- Enable Infra1 first.

#### /camera/realsense2_camera/infra1/camera_info

[sensor_msgs/msg/CameraInfo](message_formats.md#camerainfo)

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

[sensor_msgs/msg/Image](message_formats.md#image)

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

[sensor_msgs/msg/CameraInfo](message_formats.md#camerainfo)

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

[sensor_msgs/msg/Image](message_formats.md#image)

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

[urg_node_msgs/msg/Status](message_formats.md#status)

#### /scan

[sensor_msgs/msg/LaserScan](message_formats.md#laserscan)

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

[vesc_msgs/msg/VescStateStamped](message_formats.md#vescstatestamped)

- frame_id: ''
- state:
	  - temp_fet: 0.0
	  - temp_motor: 0.0
	  - current_motor: 0.0
	  - current_input: 0.0
	  - avg_id: 0.0
	  - avg_iq: 0.0
	  - duty_cycle: 0.001
	  - speed: 0.0
	  - voltage_input: 11.9
	  - charge_drawn: 0.0
	  - charge_regen: 0.0
	  - energy_drawn: 0.0
	  - energy_regen: 0.0
	  - displacement: -3
	  - distance_traveled: 5
	  - fault_code: 0
	  - pid_pos_now: 315.0
	  - controller_id: 114
	  - ntc_temp_mos1: 0.0
	  - ntc_temp_mos2: 0.0
	  - ntc_temp_mos3: 0.0
	  - avg_vd: 0.008
	  - avg_vq: 0.006

#### /sensors/imu

[vesc_msgs/msg/VescImuStamped](message_formats.md#vescimustamped)

- frame_id: ''
- imu:
	- ypr:
		- x: -1.5143272172990507
		- y: 0.6758916679632196
		- z: 24.65624695086296
	- linear_acceleration:
		- x: -0.01318359375
		- y: 0.02587890625
		- z: 0.99755859375
	- angular_velocity:
		- x: 0.0
		- y: -0.3662109375
		- z: 0.3662109375
	- compass:
		- x: 0.0
		- y: 0.0
		- z: 0.0
	- orientation:
		- x: 0.014168892987072468
		- y: 0.0029403199441730976
		- z: -0.2135637253522873
		- w: 0.9768219590187073


#### /sensors/imu/raw

[sensor_msgs/msg/Imu](message_formats.md#imu)

Not a published topic, lol.

#### /sensors/servo_position_command

[std_msgs/msg/Float64](message_formats.md#float64)

- data: some floating point number starting with ```0.``` (range probably between 0 and 1)

### Other

#### /diagnostics

[diagnostic_msgs/msg/DiagnosticArray](message_formats.md#diagnosticarray)

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
		  - value: '0'

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

[tf2_msgs/msg/TFMessage](message_formats.md#tfmessage)

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

[tf2_msgs/msg/TFMessage](message_formats.md#tfmessage)

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
