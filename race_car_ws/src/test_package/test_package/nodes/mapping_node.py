#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PointStamped
from std_msgs.msg import Header

import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration

from racecar_msgs.msg import DetectedConeArray
from scipy.spatial.transform import Rotation

def bresenham_line(x0, y0, x1, y1):
    """
    Calculate a list of grid cells that form a line from (x0, y0) to (x1, y1)
    using the Bresenham algorithm.

    :param x0: Starting x-coordinate
    :param y0: Starting y-coordinate
    :param x1: Ending x-coordinate
    :param y1: Ending y-coordinate
    :return: A list of (x, y) tuples representing cells on the line path."""
    points = []
    steep = abs(y1 - y0) > abs(x1 - x0)
    if steep:
        x0, y0 = y0, x0
        x1, y1 = y1, x1
    
    swapped = False
    if x0 > x1:
        x0, x1 = x1, x0
        y0, y1 = y1, y0
        swapped = True
    
    ystep = 1 if y0 < y1 else -1

    dx = x1 - x0
    dy = abs(y1 - y0)
    error = -dx // 2
    y = y0

    for x in range(x0, x1 + 1):
        if steep:
            points.append((y, x))
        else:
            points.append((x, y))
        error += dy
        if error > 0:
            y += ystep
            error -= dx

    if swapped:
        points.reverse()
    return points


class MappingNode(Node):
    """
    ROS2 node that creates an occupancy grid map based on laser scans,
    detected cones and and odometry.
    """
    def __init__(self):
        super().__init__('mapping_node')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('occupancy_grid_topic', '/occupancy_grid')
        self.declare_parameter('fused_cones_topic', '/fused_cones')

        self.declare_parameter('frame_id', 'odom')
        
        #E.g. 20x20 meter
        self.declare_parameter('grid_width', 400)
        self.declare_parameter('grid_height', 400)
        self.declare_parameter('grid_resolution', 0.05)
        
        #Define a mapping from grid cell to odom/world coordinate -> Here (-10, -10) would be the leftmost corner of the grid (0,0)
        self.declare_parameter('origin_x', -10.0)
        self.declare_parameter('origin_y', -10.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.occupancy_grid_topic = self.get_parameter('occupancy_grid_topic').value
        self.fused_cones_topic = self.get_parameter('fused_cones_topic').value

        self.frame_id = self.get_parameter('frame_id').value
        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )
        self.cones_sub = self.create_subscription(
            DetectedConeArray,
            self.fused_cones_topic,
            self.cones_callback,
            10
        )

        self.occupancy_pub = self.create_publisher(OccupancyGrid, self.occupancy_grid_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header = Header()
        self.occupancy_grid.header.frame_id = self.frame_id

        info = MapMetaData()
        info.resolution = self.grid_resolution
        info.width = self.grid_width
        info.height = self.grid_height
        origin_pose = Pose()
        origin_pose.position.x = self.origin_x
        origin_pose.position.y = self.origin_y
        origin_pose.position.z = 0.0
        info.origin = origin_pose
        self.occupancy_grid.info = info
        #Init Grid completely unknown
        self.occupancy_grid.data = [-1] * (self.grid_width * self.grid_height)

        # Keep track of the latest odometry pose (x, y, yaw)
        self.robot_pose = (0.0, 0.0, 0.0)

        self.get_logger().info(f"Mapping Node started in frame: {self.frame_id}")

    def odom_callback(self, msg: Odometry):
        """
        Callback for odometry updates. Extracts current robot pose and 
        orientation (converted to yaw) from the Odometry message.

        :param msg: The incoming Odometry message.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        rot = Rotation.from_quat([qx, qy, qz, qw])
        _, _, yaw = rot.as_euler('xyz')
        self.robot_pose = (x, y, yaw)

    def scan_callback(self, scan_msg: LaserScan):
        """
        Callback for laser scan messages. Updates the occupancy grid by
        clearing free cells along the beam, and marking the end of the
        beam as occupied (if it is not at max range).

        :param scan_msg: The incoming LaserScan message.
        """
        try:
            stamp_time = rclpy.time.Time.from_msg(scan_msg.header.stamp)
            transform_stamped = self.tf_buffer.lookup_transform(
                self.frame_id,
                scan_msg.header.frame_id,
                stamp_time,
                timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"Could not look up TF for laser scan: {e}")
            return

        angle = scan_msg.angle_min
        xr, yr, _ = self.robot_pose
        robot_gx, robot_gy = self.world_to_grid(xr, yr)

        for r in scan_msg.ranges:
            # If range is inf / nothing was hit set it to max range
            if not math.isfinite(r):
                r = scan_msg.range_max

            # Calculate the end point of the laser beam
            x_laser = r * math.cos(angle)
            y_laser = r * math.sin(angle)
            
            angle += scan_msg.angle_increment

            #Transforming from laser -> map (odom) frame
            ps = PointStamped()
            ps.header.frame_id = scan_msg.header.frame_id
            ps.header.stamp = scan_msg.header.stamp
            ps.point.x = x_laser
            ps.point.y = y_laser
            ps.point.z = 0.0

            try:
                ps_odom = tf2_geometry_msgs.do_transform_point(ps, transform_stamped)
                X_global = ps_odom.point.x
                Y_global = ps_odom.point.y
            except Exception as e:
                self.get_logger().warn(f"Error transforming laser point: {e}")
                continue
            
            #Converting the point to grid coordinates and calculating a line from the robot to the point in the grid
            gx, gy = self.world_to_grid(X_global, Y_global)
            line_cells = bresenham_line(robot_gx, robot_gy, gx, gy)

            #If range is max (nothing was hit) mark all cells in the path as free, otherwise mark last cell as occupied
            if r >= scan_msg.range_max:
                for c in line_cells:
                    self.set_cell(c[0], c[1], 0)
            else:
                for c in line_cells[:-1]:
                    self.set_cell(c[0], c[1], 0)
                if line_cells:
                    last_c = line_cells[-1]
                    self.set_cell(last_c[0], last_c[1], 100)

        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.occupancy_pub.publish(self.occupancy_grid)

    def cones_callback(self, msg: DetectedConeArray):
        """
        Callback for detected cones. For each cone in the array,
        mark the corresponding occupancy grid cells as occupied.

        :param msg: The incoming DetectedConeArray message containing a list of cones.
        """
        try:
            stamp_time = rclpy.time.Time.from_msg(msg.header.stamp)
            transform_stamped = self.tf_buffer.lookup_transform(
                self.frame_id,
                msg.header.frame_id,
                stamp_time,
                timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"Could not look up TF for cones: {e}")
            return
        
        for cone in msg.cones:
            from_frame = cone.header.frame_id
            
            ps = PointStamped()
            ps.header.frame_id = from_frame
            ps.header.stamp = msg.header.stamp
            ps.point.x = cone.position.x
            ps.point.y = cone.position.y
            ps.point.z = cone.position.z
            
            try:
                pt_transformed = tf2_geometry_msgs.do_transform_point(ps, transform_stamped)
            except Exception as e:
                self.get_logger().warn(f"Error transforming cone position: {e}")
                continue
            
            x_world = pt_transformed.point.x
            y_world = pt_transformed.point.y

            #Convert cone point to grid and mark it + surrounding cells as occupied
            #TODO: Should probably make this configurable & dependant on grid resolution
            gx, gy = self.world_to_grid(x_world, y_world)
            self.set_cell(gx, gy, 100)
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    self.set_cell(gx + dx, gy + dy, 100)

        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.occupancy_pub.publish(self.occupancy_grid)

    def world_to_grid(self, x_world, y_world):
        """
        Convert world/odom coordinates (x_world, y_world) to grid indices (gx, gy)
        based on the origin and resolution of the occupancy grid.

        :param x_world: World x-coordinate
        :param y_world: World y-coordinate
        :return: A tuple (gx, gy) representing grid indices.
        """
        gx = int((x_world - self.origin_x) / self.grid_resolution)
        gy = int((y_world - self.origin_y) / self.grid_resolution)
        return (gx, gy)

    def set_cell(self, gx, gy, value):
        """
        Set the occupancy value of a grid cell if it lies within 
        the grid boundaries.

        :param gx: Grid cell x-index
        :param gy: Grid cell y-index
        :param value: Occupancy value (e.g., 0 for free, 100 for occupied).
        """
        if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
            idx = gy * self.grid_width + gx
            self.occupancy_grid.data[idx] = value


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
