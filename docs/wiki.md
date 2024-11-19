# Wiki

This section will act as a knowledge base for all topics not covered by other documentation.

## External Docs

This section will link to all external documentation needed for this project. This section will be updated regularly.

### Dev Docs

This section will cover all documentation for the development process.

* [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
* [Formula Student Rules 2025 v1.0](https://www.formulastudent.de/fileadmin/user_upload/all/2025/rules/FS-Rules_2025_v1.0.pdf)
* [F1TENTH System](https://github.com/f1tenth/f1tenth_system)
* [F1TENTH Build Documentation](https://f1tenth.readthedocs.io/en/stable/index.html)

### Hardware Docs

This section will cover all hardware-related documentation.

* [NVIDIA Jetson Orin NX 16GB](https://developer.nvidia.com/embedded/downloads#?search=Data%20Sheet&tx=$product,jetson_agx_orin,jetson_orin_nx,jetson_orin_nano)
* [HOKUYO UST-10/20LX](https://www.hokuyo-aut.jp/search/single.php?serial=167#download)
* [Programming Guides for HOKUYO Hardware](https://sourceforge.net/p/urgnetwork/wiki/top_en/)
* [Intel RealSense Depth Camera D435i](https://www.intelrealsense.com/depth-camera-d435i/)
* [Intel RealSense & ROS 2](https://dev.intelrealsense.com/docs/ros2-wrapper)

### Methodical Docs

This section will cover all documentation for the agile process.

* [The Scrum Guide](https://scrumguides.org/docs/scrumguide/v2020/2020-Scrum-Guide-US.pdf#zoom=100)
* [Agile Manifesto Principles](https://agilemanifesto.org/principles.html)
* [A Self-Driving Car Architecture in ROS2](https://ieeexplore.ieee.org/document/9041020)

## ROS2 Overview


### Tutorials

- [Nice playlist crash course](https://www.youtube.com/watch?v=c5DRTN2b2kY&list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy&index=2) (uses python3.10)

### Tools

- `sudo apt install ros-humble-<package>` to install a package
- `rqt_graph` is a tool that visualizes ROS topics
- `Ros` vscode-extension by microsoft (autocompletion)
- `ros2 node list` lists all nodes that are running
- `ros2 node info /node_name` will display information about a certain node
- `ros2 topic list` lists all the active topics
- `ros2 topic info <topic>` prints out the type of the msg and the publisher/subscriber count
- `ros2 interface show </path/to/ros2/class>` shows the interface for a certain ros2 object like `ros2 interface show ackermann_msgs/msg/AckermannDrive`
- `ros2 topic echo <topic>` will print out all messages published over that topic
- `ros2 bag play <rosbag-file>` replay ros2 bag

#### Build Tools

- Colcon

```sh
sudo apt install python3-colcon-common-extensions
```

in order to enable autocomplete for colcon source the `/usr/share/colcon_argcomplete/hook/colcon_argcomplete.bash` script.

### Workspace

- A workspace is just a folder...
- ros2 workspaces tend to have a `_ws` suffix
- has an `src` folder
- workspaces act as an addition to the global installation
- `colcon build` in the workspace directory
    - creates a `setup.bash` script in `<workspace>/install/setup.bash` that needs to be sourced in order for the nodes to be used
    - creates `log` and `build` folder


### Create packages

- a package can contain many nodes
- a package is language specific which means you can not mix python and C++
- `rclpy` is a package for python ros2
- Use `colcon build` in the workspace folder to build the packages (downgrade setuptools to 58.2 if there are errors)
- In order to avoid rebuilding every time a change is made use `colcon build --symlink-install` which will just refer to the package source
- Might need to `source ~/.bashrc` again

Go to the `src` folder in the ros2 workspace and type:

```sh
ros2 pkg create <package_name> --build-type ament_<cmake or python> --dependencies rclpy <more space separated pkgs>
```

This will create a new folder `src/<package_name>` containing:
- `package.xml`
    - <depend> &rarr; dependencies for the package specified by `--dependencies`
    - <build-type> &rarr; build type we specified with `--build-type`
- `setup.cfg` python packaging file
- `<package_name>/
- set cli commands to start nodes in the `setup.cfg` files so that these commands can be executed using `ros2 run`

```setup.py
from setuptools import setup

package_name = "package_name"
setup(
# stuff
entry_points={
    "console_scripts": [
        "my_node" = "package_name.node_file_name:main" # path to function starting the node
    ]
})
```

Afterwards we can call `ros2 run package_name my_node`.


### Nodes

- Nodes are programs that have access to the ROS2 bridge.
- Nodes can do anything, they can even be a webserver.
- Nodes are modules/files in the package folder (`src/<package>/<package>/node.py`)
- The first line of a python file containing a node should be how the file can be executed `#!/usr/bin/env python3` (path to python I guess)
- Always import `rclpy`
- Ros will execute the script with `__name__ == "__main__"` function:
- Ros nodes are python classes inheriting from `rclpy.node.Node`
- To keep ROS2 nodes alive use `rclpy.spin(node)`
- logging is available through `Node.get_logger().<log_level>("my log text")` &rarr; `Node.get_logger().info("my info log")`

Node template:

```py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("NodeName") # "NodeName" will be displayed in rqt_graph

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()
    rclpy.spin(node) # used to loop the node

    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

#### Timers and Callbacks

- `Node.create_timer` creates a periodic timer that can be used to execute callbacks
- `Node.counter_` hält die Anzahl an Timer Durchläufen (was ist bei mehreren Timern?)

```py
class MyNode(Node):
    def __init__(self):
        super().__init__("NodeName")
        timer_period_s = 1.0
        self.create_timer(timer_period_s, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("callback executed")
```


### Command structure

```sh
ros2 run <package> <node>
```

### Publisher Node

The `create_publisher` method needs the parameters:
- `msg_type`, the actual type of the message
- `topic` a `str` with an url like structure like `"/my/car/topic"`
- `queue_size` an `int` with the size of the message queue

```py
class MockMsgType: # this is not a real message type
    pass

class MyNode(Node):
    def __init__(self):
        super().__init__("NodeName") # "NodeName" will be displayed in rqt_graph
        self.cmd_publisher = self.create_publisher() # Needs a msg type and the parameters (msg_type, topic: str, queue_size: int)
        self.timer = self.create_timer(0.5, self.publish_something)

    def publish_something(self):
        msg = MockMsgType()
        msg.attr = "some_value"
        self.cmd_publisher.publish()

```
