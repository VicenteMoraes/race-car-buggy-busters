
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class TestNode(Node):
    """Test node for Installation and docs"""
    def __init__(self):
        super().__init__("NodeName") # "NodeName" will be displayed in rqt_graph
        timer_period_s = 1.0
        self.create_timer(timer_period_s, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello from the test node!")

def main(args=None):
    rclpy.init(args=args)

    node = TestNode()
    rclpy.spin(node) # used to loop the node

    rclpy.shutdown()

if __name__ == "__main__":
    main()
