import pytest
import rclpy
from rclpy.node import Node
from unittest.mock import MagicMock, patch
import numpy as np
import time
from f110_car.m2p_node import M2P
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


@pytest.fixture(scope="session")
def ros_setup():
    """Ensure rclpy is initialized only once per test session."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def ros_node(ros_setup):
    """Fixture to create and return an M2P node with proper setup."""
    node = M2P()
    yield node
    node.destroy_node()


@patch("avai_lab.config.load_config", return_value=MagicMock(
    car_platform=MagicMock(
        max_steering_angle=0.4,
        max_speed=3.0,
        min_speed=0.5,
        max_acceleration=1.0,
        max_steering=1.0
    )
))
def test_normalize_angle(mock_config):
    """Test angle normalization function"""
    rclpy.init()
    node = M2P()
    assert node.normalize_angle(0) == 0
    assert node.normalize_angle(np.pi) == np.pi
    assert node.normalize_angle(-np.pi) == -np.pi
    assert node.normalize_angle(3 * np.pi) == -np.pi
    assert node.normalize_angle(-3 * np.pi) == np.pi
    node.destroy_node()
    rclpy.shutdown()




def test_point_callback(ros_node):
    """Test target point handling"""
    point_msg = PoseStamped()
    point_msg.pose.position.x = 1.0
    point_msg.pose.position.y = 2.0

    ros_node.point_callback(point_msg)
    assert len(ros_node.target_stack) == 1
    assert np.array_equal(ros_node.target_stack[0], np.array([1.0, 2.0]))

    ros_node.point_callback(point_msg)
    assert len(ros_node.target_stack) == 1 


@patch("avai_lab.utils.get_direction_vec", return_value=np.array([1, 1]))
@patch("avai_lab.utils.quat_to_rot_vec", return_value=0.0)
@patch("avai_lab.utils.rot_from_vec", return_value=np.pi / 4)
def test_odom_callback(mock_get_dir, mock_quat_rot, mock_rot_vec, ros_node):
    """Test odometry handling and drive message publishing"""
    odom_msg = Odometry()
    odom_msg.pose.pose.position.x = 0.0
    odom_msg.pose.pose.position.y = 0.0
    odom_msg.pose.pose.orientation.z = 0.0
    odom_msg.pose.pose.orientation.w = 1.0

    ros_node.target_stack.append(np.array([1.0, 1.0]))
    ros_node.publisher.publish = MagicMock()
    ros_node.odom_callback(odom_msg)

    ros_node.publisher.publish.assert_called_once()
    published_msg = ros_node.publisher.publish.call_args[0][0]

    # Check if steering angle is correctly set
    assert -ros_node.max_steering_angle <= published_msg.drive.steering_angle <= ros_node.max_steering_angle
    assert published_msg.drive.speed >= ros_node.min_speed


def test_drive_stopping(ros_node):
    """Test stopping behavior when reaching the target"""
    odom_msg = Odometry()
    odom_msg.pose.pose.position.x = 1.0
    odom_msg.pose.pose.position.y = 1.0

    ros_node.target_stack.append(np.array([1.0, 1.0]))
    ros_node.target_stack.append(np.array([2.0, 2.0]))  # another target

    ros_node.publisher.publish = MagicMock()

    ros_node.odom_callback(odom_msg) 
    time.sleep(0.1)  
    ros_node.odom_callback(odom_msg) 

    # Check if a new target was set
    assert ros_node.current_target is not None, "The car did not switch to the next target"

    if ros_node.publisher.publish.call_count > 0:
        published_msg = ros_node.publisher.publish.call_args[0][0]
        assert published_msg.drive.speed >= ros_node.min_speed
    else:
        pytest.fail("Expected AckermannDriveStamped message but none was published")





