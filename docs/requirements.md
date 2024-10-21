# Requirements

This section will hold the requirements for the project.

## Functional

This section will hold the functional requirements for the project.

\# | ID | Name | Text | Autonomous Driving Task(s)
---|----|------|------|---------------------------
1 | F1 | General Perception | The vehicle must capture real-time 3D depth data and high-resolution mono visual data to perceive the environment. | Perception
2 | F2 | Object Detection & Classification | The system must detect and classify objects such as track boundaries and obstacles. | Perception
1 | F1 | Cone Detection | When on track the vehicle must detect cones marking the track boundaries in real-time using the camera and LiDAR sensors. | Perception
2 | F2 | Obstacle Detection | When approaching obstacles the vehicle must detect static obstacles on the track (with a detection range of at least 10 meters). | Perception
3 | F2 | Sensor Fusion | When collecting data the perception system must integrate LiDAR and camera data using a sensor fusion algorithm to improve obstacle detection accuracy. | Perception
4 | F4 | Distance Estimation | While driving the vehicle must estimate the disctance to the obstacles and track boundaries continuously (with less than 5% error margin). | Perception
4 | F4 | Point Cloud | The vehicle must collect 3D LiDAR piont cloud data to detect and map its surroundlings in real-time. | Perception
5 | F5 | Visual Data | The camera system must provide input for visual odometry and SLAM, ensuring accurate self-localization. | Perception, Localization
5 | F5 | Self-Localization | The vehicle must localize its position relative to track boundaries using fused data from LiDAR, camera, and odometry. | Localization, Perception
5 | F5 | Real-Time Map | The vehicle must generate a real-time map of the environment using LiDAR and visual sensors, aiding in obstacle detection and path planning. | Perception, Planning, Localization
6 | F6 | Map Updates | When obstacles are detected the vehicle must update its internal map of the track dynamically in real-time. | Mapping, Planning
6 | F6 | Environments | The system must be able to handle both static and dynamic environments, where obstacles or the track layout may change.
7 | F7 | Optimal trajectory | When planning the path the vehicle must generate an optimal trajectory within track boundaries using the detected cone positions and real-time sensor data, maintaining the desired speed. | Planning
7 | F7 | Path Planning | The vehicle must implement a planning algorithm that works in real time, adjusting paths as the environment changes. | Planning
7 | F7 | Path Updates | The path planner must compute new trajectories dynamically to handle unexpected obstacles or sudden track changes. | Planning
8 | F8 | Obstacle Avoidance | When an obstacle is detected the vehicle should replan its path to avoid the obstacle (while maintaining speed), but the system may choose not to avoid obstacles that do not interfere with the vehicle's path. | Planning
8 | F8 | Obstacle Avoidance | The vehicle mist detect obstacles in its path and plan a safe trajectory to avoid them while maintaining speed and stability. | Perception, Planning, Control
9 | F9 | Acceleration & Deceleration | The vehicle must decide when to accelerate, decelerate, or stop, depending on the environment. | Perception, Planning, Control
9 | F9 | Driving Modes | The vehicle must be able between different driving modes (e.g. cruising, obstacle avoidance, emergency stop) based on the context. | Planning, Control, Monitoring
9 | F9 | Speed Maintenance | When on track the vehicle must maintain a speed of max. 30-40 km/h depending on the track layout and detected obstacles. | Planning
9 | F9 | Acceleration Adaptation | The system must adapt acceleration to keep the vehicle within speed limits, adjusting for factors like road slope, friction, and other dynamic conditions. | Control
10 | F10 | Sharp Turns | While approaching sharp turns the vehicle must adjust its speed to ensire safe cornering without exceeding the track boundaries. | Planning, Control
11 | F11 | Smooth Acceleration | When accelreating the vehicle mist limit its accelaration to avoid jerky movements using a sof acceleration curve, ensuring smooth control. | Control
12 | F12 | Braking | The vehicle must be able to brake in real-time to slow down or stop based on planned behavior or unexpected events. | Control
12 | F12 | Controlled Emergency Braking | When braking the vehicle must decelerate within a controlled distance to avoid overshooting turns or track boundaries (, with braking distance adjusted to 5-8/ max. 10 meters at maximum speed). | Control
13 | F13 | Steering Angle | When controlling the steering the vehicle must calculate the optimal steering angle based on the track curvature and current speed. | Control
13 | F13 | Steering | The vehicle must control the Ackerman steering system to follow the planned trajectory accurately. | Control
13 | F13 | Smooth Steering | The control system must ensure stable and smooth steering, especially at high speeds. | Control
14 | F14 | Obstacles & Track Boundaries | When obstacles are present the vehicle must continue following the track boundaries while adjusting its path to avoid the obstacle. | Planning
15 | F15 | External Signals | The system must respond appropriately to external signals and traffic rules defined for the track. | Control, Perception, Planning
15 | F15 | Failsafe Mode | When a critical system failure is detected the vehicle must engage the emergency braking system (within 100 ms) to bring the vehicle to a stop (within 5-8 meters). | Monitoring, Control
16 | F16 | System Health | When system health is checked the vehicle must monitor the status of all sensors, compute hardware, and acutators in real-time to ensure continuous operation. | Control
16 | F16 | Odometry | The vehicle must collect odometry data to track its speed, orientation and position over time. | Control, Monitoring
16 | F16 | Health Checks | The system must implement watchdog timers and health-check nodes ro detect failures or malfunctions in any part of the system. | Monitoring
17 | F17 | Vehicle Tracking | The system must provide continuous and real-time feedback on the vehicle's movement and position to maintain trajectory tracking. | Monitoring, Planning, Control
18 | F18 | Internal Communication | All sensors, acutators, and the onboard compute hardware must communicate via ROS2 Humble, using DDS for real-time message passing. | Communication
19 | F19 | Manual Override Mode | The system should be able to transition to a manual override mode in case of critical failures, allowing the race team to take control if neccessary. | Monitoring, Control
19 | F19 | Partial System Failures | The system must include mutliple levels of safety checks, and emergency overrides must be functional even in the event of partial system failures. | Control, Monitoring
20 | F20 | Testing | The vehicle must be tested in both simulation and real-world environments to validate its perception, planning, and control systems. | Testing

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
16 | Q16 | Real-Time Internal Communication | The system must support reliable, low-latency communication between sensor nodes, perception nodes, and control nodes to ensure real-time response. | Communication
17 | Q17 | Energy Consumption | The software stack should optimize CPU and GPU usage to minimize energy consumption during intensive tasks like real-time perception and path planning. | Perception, Planning
18 | Q18 | Sensor Efficiency | The vehicle must use its sensors efficiently, ensuring that data redundancy is minimizes, and sensor processing loads are balances to avoid system bottlenecks. | Perception
19 | Q19 | Data Fitlering | The system should implement intelligent data filtering to reduce unnecessary sendor data while preserving critical information needed for accurate perception and planning. | Perception, Planning
20 | Q20 | Trajectory Updates | The path planning system must generate new trajectories within a strict time frame/ every 50 ms to adapt to changes in the environment, avoiding delays that could cause the vehicle to miss turns or collide with obstacles. | Planning

## Constraints

This section will hold the constraints for the project.

\# | ID | Name | Text | Autonomous Driving Task(s)
---|----|------|------|---------------------------
1 | C1 | Speed Limit | The vehicle must not exceed a speed of 40 km/h due to track limitations and safety considerations. | Control
2 | C2 | Jetson Limitiations | The system must operate within the limitations of the NVIDIA Jetson NX ensuring all processes can run in real-time without overloading the CPU, GPU, and memory resources. | Perception, Planning, Control, Monitoring
3 | C3 | Real-Time Performance | The system must process sensor inputs at 20-30Hz to maintain real-time performance within the computational limits of the vehicle. | Perception
4 | C4 | Remote Emergency Stop | The vehicle must include a Remote Emergency Stop system that can be triggered by the race team/ race officials at any time during the race. | Control
5 | C5 | System Monitoring | The autonomous system must continuously monitor critical safety components and transition to a safe state if a failure is detected. | Monitoring
6 | C6 | Power Budget | The sensor suite must operate within a power budget defined by the F1TENTH platform, ensuring no component exceeds the power capacity available to the vehicle. | Monitoring
7 | C7 | Ackerman Steering | The vehicle must use the Ackerman steering system for turning, which constains the types of maneuvers the vehicle can perform, preventing mechanical failure or loss of control, especially at high speeds. | Control, Planning
8 | C8 | On Track Testing | Vehicle testing on track must be done within the practice tracks provided. | Testing
9 | C9 | Formula Student Test Scenarios | The vehicle must pass all Formula Student test scenarios, such as obstacle avoidance, lane following, and speed control, to ensure it's race-ready.
10 | C10 | Processor Limitations | Algorithms for perception, planning, and control must be designed to avoid overloading the processor, ensuring real-time performance without exceeding the power and thermal limits of the hardware. | Perception, Planning, Control
11 | C11 | Formula Student Constraints | The vehicle must meet all Formula Student 2025 regulations, including specific requirements on safety, system redundancy, maximum speed, and obstacle avoidance systems. | Control, Planning, Perception
12 | C12 | 4-Wheel Drive | The vehicle's control system must optimize the distribution of torque across all four wheels in a manner that ensures both optimal acceleration and stability, without causing skidding or loss of traction. | Control
13 | C13 | Energy Consumption | The vehicles's energy consumption must be managed to ensure that it can complete the entire race or testing session within the available battery capacity, taking into account the energy demands of high-speed racing and intensice computations. | Perception, Planning, Control, Monitoring
14 | C14 | Data Priorization | The system must ensure that critical sensor data is prioritized in high-speed scenarions, where rapid decision-making is essential.
