# AVAI Lab

This is the repository for the course "Autonomous Vehicles And Artificial Intelligence" (AVAI).

Take a look at the [Documentation](https://rub-avai.github.io/race-car-buggy-busters/)

## IMPORTANT UPDATES

In order to run the simulation please update your IGN GAZEBO RESOURCE PATH so it points to the race car models:

```sh
export IGN_GAZEBO_RESOURCE_PATH=/home/$USER/race-car-buggy-busters/race_car_ws/install/gazebo_f110/share/gazebo_f110/model
```

- Run simple driving server:

```sh
colcon build
source install/setup.bash
ros2 run simple_driving driving_node
```
Action Server: /simple_driving
Control Driving Topic: /control_driving


- Action Server Message:

```
# Request (First Message)
string connect
---
# Result (Final Message)
string disconnect
---
# Feedback
string is_driving
```

- Send goal to action Server through the terminal:

```sh
ros2 action send_goal --feedback /simple_driving racecar_msgs/action/Server "{connect: 'connect'}"
```

- Start navigation:

```sh
ros2 topic pub /control_driving std_msgs/msg/String "data: start" --once
```

- Stop navigation:

```sh
ros2 topic pub /control_driving std_msgs/msg/String "data: 'stop'" --once
```

## Structure

The respository contains:
- folders for ROS2 packages (`_ws` - suffix)
- the `avai_lab` python package
- [mkdocs](https://squidfunk.github.io/mkdocs-material/getting-started/) documentation

All important documentation and information will be compiled in the mkdocs pages.
You can find them in the `docs` folder.

## Developer Installation

In order to use this repository you will need [ROS2 Humble](https://docs.ros.org/en/humble/index.html) (ships with python3.10).

First make sure you have the latest and greatest installation tools:

```sh
python3 -m pip install --upgrade pip
pip install --upgrade packaging setuptools
```

Install the `avai_lab` package as editable. The `[docs]` installs the optional dependencies for `mkdocs`

```sh
pip install -e avai_lab[docs]
```

Now go and build the ROS2 `test_package`.

```sh
cd race_car_ws
colcon build
source install/setup.bash
```

Now you should be able to view `mkdocs`
(In the repositories base directory)
```sh
mkdocs serve
```
