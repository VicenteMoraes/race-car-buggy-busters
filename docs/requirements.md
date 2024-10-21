# Requirements

This section will hold the requirements for the project.

## Functional

This section will hold the functional requirements for the project.

\# | ID | Name | Text | Autonomous Driving Task(s)
---|----|------|------|---------------------------
1 | F1 | Cone Detection | When on track the vehicle must detect cones marking the track boundaries in real-time using the camera and LiDAR sensors. | Perception
2 | F2 | Obstacle Detection | When approaching obstacles the vehicle must detect static obstacles on the track (with a detection range of at least 10 meters). | Perception
3 | F2 | Sensor Fusion | When collecting data the perception system must integrate LiDAR and camera data using a sensor fusion algorithm to improve obstacle detection accuracy. | Perception
4 | F4 | Distance Estimation | While driving the vehicle must estimate the disctance to the obstacles and track boundaries continuously (with less than 5% error margin). | Perception
5 | F5 | Self-Localization | When the race begins the vehicle must localize its position relative to track boundaries using fused data from LiDAR, camera, and odometry. | Localization
6 | F6 | Map Updates | When obstacles are detected the vehicle must update its internal map of the track dynamically in real-time. | Mapping, Planning
7 | F7 | Optimal trajectory | When planning the path the vehicle must generate an optimal trajectory within track boundaries using the detected cone positions and real-time sensor data. | Planning
8 | F8 | Obstacle Avoidance | When an obstacle is detected the vehicle must replan its path to avoid the obstacle (while maintaining speed). | Planning
9 | F9 | Speed Maintenance | When on track the vehicle must maintain a speed of max. 30-40 km/h depending on the track layout and detected obstacles. | Planning
10 | F10 | Sharp Turns | While approaching sharp turns the vehicle must adjust its speed to ensire safe cornering without exceeding the track boundaries. | Planning, Control
11 | F11 | Smooth Acceleration | When accelreating the vehicle mist limit its accelaration to avoid jerky movements using a sof acceleration curve, ensuring smooth control. | Control
12 | F12 | Controlled Braking | When braking the vehicle must decelerate within a controlled distance to avoid overshooting turns or track boundaries (, with braking distance adjusted to 5-8/ max. 10 meters at maximum speed). | Control
13 | F13 | Steering | When controlling the steering the vehicle must calculate the optimal steering angle based on the track curvature and current speed. | Control
14 | F14 | Obstacles & Track Boundaries | When obstacles are present the vehicle must continue following the track boundaries while adjusting its path to avoid the obstacle. | Planning
15 | F15 | Failsafe Mode | When a critical system failure is detected the vehicle must engage the emergency braking system (within 100 ms) to bring the vehicle to a stop (within 5-8 meters). | Monitoring, Control
16 | F16 | System Health | When system health is checked the vehicle must monitor the status of all sensors and acutators in real-time to ensure continuous operation. | Control

## Quality

This section will hold the quiality requirements for the project.

\# | ID | Name | Text | Autonomous Driving Task(s)
---|----|------|------|---------------------------

## Constraints

This section will hold the constraints for the project.

\# | ID | Name | Text | Autonomous Driving Task(s)
---|----|------|------|---------------------------

