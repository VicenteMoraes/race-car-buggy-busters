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
1 | Q1 | Sensor Processing Latency | The system must process sensor data in real-time with latency under 50 ms to ensure timely decision-making and control. | Decision Making, Control
2 | Q2 | Control Loop Frequency | The vehicle must maintain a control loop update frequency of 20-30Hz to ensure smooth and responsive driving at max. 30-40 km/h. | Control
3 | Q3 | Localization Accuracy | The localization system must maintain position ±10 cm to ensure the vehicle stays within the track boundaries. | Localization
4 | Q4 | Perception Radius | The perception system must detect cones and obstacles at least 10 meters ahead to provide sufficient time for obstacle avoidance and path adjustments. | Perception
5 | Q5 | Emergency Braking Distance | The vehicle must decelerate within 5-8/ max. 10 meters when the emergency braking system is engaged to ensure safe stopping. | Monitoring, Control, Safety
6 | Q6 | Emergency Stop | The vehicle must engage the emergency braking system within 100 ms of detecting critical system failures to avoid accidents. | Control, Monitoring
7 | Q7 | Obstacle Avoidance Rate | The obstacle avoidance system must successfully avoid at least 95% of detected obstacles to ensure a high level of safety. | Perception, Planning, Control
8 | Q8 | Detection Rate | The perception system must maintain a continuous detection rate of at least 95% for cones and obstacles to prevent boundary violations or collisions. | Perception
9 | Q9 | Control System | The control system must operate continuously for the duration of the race without any system crashes or interruptions. | Monitoring, Control
10 | Q10 | Fallback | The vehicle must recover from sensor or communication failures within 200 ms using fallback systems to ensure continuous operation. | Monitoring
11 | Q11 | Logs | The system must log sensor and control data with timestamps for a post-race analysis without exceeding storage limits. | Monitoring
12 | Q12 | Hardware Temperature | The compute hardware must maintain stable operation withing a temperature range of 0-50°C to prevent overheating or shutdowns. | Monitoring
13 | Q13 | Real-Time Data | The system must provide real-time telemetry data to the race team at least once every 100 ms to allow for remote monitoring during the race. | Monitoring
14 | Q14 | Reconfiguration & Updates | The system must support easy reconfiguration and updates of perception and control algorithms without requiring major code overhauls. | Perception, Control
15 | Q15 | Diagnostics | The user interface must allow for quick diagnostic checks of system health before and during the race to facilitate rapid troubleshooting. | Monitoring

## Constraints

This section will hold the constraints for the project.

\# | ID | Name | Text | Autonomous Driving Task(s)
---|----|------|------|---------------------------

