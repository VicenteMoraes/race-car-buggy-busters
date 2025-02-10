import unittest
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
import launch_testing.markers
import launch_testing.asserts
from ackermann_msgs.msg import AckermannDriveStamped

@pytest.mark.launch_test
def generate_test_description():
    """Launch both nodes under test"""
    return LaunchDescription([
        Node(
            package='f110_car',
            executable='move_to_point',
            output='screen'
        ),
        
        Node(
            package='test_package',
            executable='wasd_control_node',
            output='screen'
        ),
        
        ReadyToTest()
    ])

class TestM2PControl(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Initialize ROS context once for all tests"""
        rclpy.init()
        cls.node = rclpy.create_node('test_node')

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS context after all tests"""
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_target_reception_and_movement(self):
        """Test that sending a target results in movement commands"""
        
        # Create publisher for target points
        target_pub = self.node.create_publisher(
            PoseStamped,
            '/target_points',
            10
        )
        
        # Create subscriber for movement commands
        received_cmd = None
        def cmd_vel_callback(msg):
            nonlocal received_cmd
            received_cmd = msg
            
        self.node.create_subscription(
            AckermannDriveStamped,
            '/drive',
            cmd_vel_callback,
            10
        )
        
        # Send test target point (2 meters forward)
        target_msg = PoseStamped()
        target_msg.header.stamp = self.node.get_clock().now().to_msg()
        target_msg.pose.position.x = 2.0
        target_pub.publish(target_msg)
        
        # Wait for movement command (10 second timeout)
        start_time = self.node.get_clock().now().nanoseconds
        while self.node.get_clock().now().nanoseconds - start_time < int(10e9):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if received_cmd is not None:
                break
        
        # Verify results
        self.assertIsNotNone(received_cmd, "No movement command received")
        self.assertGreater(received_cmd.linear.x, 0.0, 
                           "Should move forward for positive X target")

@post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Verify node exit codes"""
        launch_testing.asserts.assertExitCodes(proc_info)
