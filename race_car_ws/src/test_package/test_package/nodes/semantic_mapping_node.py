#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from racecar_msgs.msg import DetectedConeArray, SemanticGrid, SemanticCell
from geometry_msgs.msg import PointStamped

import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
from scipy.ndimage import label

class SemanticMappingNode(Node):
    """Node for building a semantic map by integrating cone detections into a SLAM map."""
    
    def __init__(self):
        super().__init__('semantic_mapping_node')
        
        # Declare parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cones_topic', '/yolo_cones')
        self.declare_parameter('semantic_grid_topic', '/semantic_map')
        self.declare_parameter('filtered_map_topic', '/filtered_map')
        self.declare_parameter('max_cone_cells', 30)  # Maximum cluster size for a cone (used for filtering too large objects)
        self.declare_parameter('cone_merge_distance', 0.10)  # Maximum distance (in m) for which to merge detected yolo cones 
        self.declare_parameter('cone_size', 0.1)  # Cone size in meters (default 10cm x 10cm) (used for the detected yolo cones)
        
        # Retrieve parameters
        self.map_topic = self.get_parameter('map_topic').value
        self.cones_topic = self.get_parameter('cones_topic').value
        self.semantic_grid_topic = self.get_parameter('semantic_grid_topic').value
        self.filtered_map_topic = self.get_parameter('filtered_map_topic').value
        self.max_cone_cells = self.get_parameter('max_cone_cells').value
        self.cone_merge_distance = self.get_parameter('cone_merge_distance').value
        self.cone_size = self.get_parameter('cone_size').value
        
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 1)
        self.cones_sub = self.create_subscription(DetectedConeArray, self.cones_topic, self.cones_callback, 1)
        
        self.filtered_map_pub = self.create_publisher(OccupancyGrid, self.filtered_map_topic, 1)
        self.semantic_pub = self.create_publisher(SemanticGrid, self.semantic_grid_topic, 1)
        
        # Variables to store the current map and known cone detections
        self.current_map = None
        self.known_cones = []  # List of tuples: (x, y, label)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("Semantic Mapping Node started.")

    def map_callback(self, msg):
        """
        Callback for receiving the occupancy grid map.
        
        :param msg: The occupancy grid message from SLAM Toolbox.
        :return: None
        """
        self.current_map = msg
        
        filtered_map = self.filter_large_objects(msg)
        self.filtered_map_pub.publish(filtered_map)
        
        semantic_msg = self.build_semantic_grid(filtered_map)
        self.semantic_pub.publish(semantic_msg)

    def cones_callback(self, msg):
        """
        Callback for receiving new cone detections.
        
        :param msg: The DetectedConeArray message.
        :return: None
        """
        if not self.current_map:
            return
        self.process_new_cones(msg)

    def process_new_cones(self, cone_array_msg):
        """
        Transform each newly detected cone into the map frame and merge it into known_cones.
        Duplicate detections (i.e. cones of the same type within a threshold distance) are merged.
        
        :param cone_array_msg: The DetectedConeArray message.
        :return: None
        """
        map_frame = self.current_map.header.frame_id
        
        for cone in cone_array_msg.cones:
            trans_point= self.transform_point_to_frame(cone.position, cone.header, map_frame)
            if trans_point is None:
                continue
            mx = trans_point.point.x
            my = trans_point.point.y
            
            merged = False
            # Check for an existing cone detection with the same label within the merge threshold
            for i, (kx, ky, k_label) in enumerate(self.known_cones):
                if k_label != cone.type:
                    continue
                distance = math.sqrt((mx - kx) ** 2 + (my - ky) ** 2)
                if distance < self.cone_merge_distance:
                    # Merge: average the coordinates
                    new_x = (kx + mx) / 2.0
                    new_y = (ky + my) / 2.0
                    self.known_cones[i] = (new_x, new_y, k_label)
                    merged = True
                    break
            if not merged:
                self.known_cones.append((mx, my, cone.type))

    def build_semantic_grid(self, filtered_map):
        """
        Build a semantic grid by overlaying known cone detections onto the filtered map.
        Each cone is drawn as a square with the size defined by the 'cone_size' parameter.
        
        :param filtered_map: The filtered occupancy grid.
        :return: A SemanticGrid message with overlaid cone detections.
        """
        semantic_grid = SemanticGrid()
        semantic_grid.header.stamp = self.get_clock().now().to_msg()
        semantic_grid.header.frame_id = filtered_map.header.frame_id
        semantic_grid.info = filtered_map.info

        width = filtered_map.info.width
        height = filtered_map.info.height
        data_len = width * height
        data_in = list(filtered_map.data)

        semantic_cells = []
        for i in range(data_len):
            cell_val = data_in[i]
            scell = SemanticCell()
            scell.occupancy = cell_val
            scell.label = -1
            if cell_val == 0:
                scell.label = 0  # free
            elif cell_val == -1:
                scell.label = -1  # unknown
            elif cell_val == 100:
                scell.label = -1  # occupied but unknown type
            semantic_cells.append(scell)

        origin_x = filtered_map.info.origin.position.x
        origin_y = filtered_map.info.origin.position.y
        resolution = filtered_map.info.resolution

        # Determine how many grid cells cover the cone size (cone_size in meters)
        num_cells = max(1, int(round(self.cone_size / resolution)))
        
        #Mark cone + label at location of yolo cone
        for (cx, cy, label_val) in self.known_cones:
            gx, gy = self.world_to_grid(cx, cy, origin_x, origin_y, resolution)
            start_x = gx - num_cells // 2
            start_y = gy - num_cells // 2
            for dx in range(num_cells):
                for dy in range(num_cells):
                    cell_x = start_x + dx
                    cell_y = start_y + dy
                    if 0 <= cell_x < width and 0 <= cell_y < height:
                        idx = cell_y * width + cell_x
                        semantic_cells[idx].occupancy = 100
                        semantic_cells[idx].label = label_val

        semantic_grid.cells = semantic_cells
        return semantic_grid

    def filter_large_objects(self, occupancy_map):
        """
        Filter out large clusters of occupied cells from the occupancy map.
        Connected clusters with a size greater than self.max_cone_cells are marked with -1.
        
        :param occupancy_map: The raw OccupancyGrid.
        :return: A new OccupancyGrid with large clusters filtered.
        """
        width = occupancy_map.info.width
        height = occupancy_map.info.height
        # Convert grid data into a 2D NumPy array
        data_array = np.array(occupancy_map.data).reshape((height, width))
        
        # Create a mask where occupied cells are True and free/unknown cells are False
        occupied_mask = (data_array == 100)
        
        # Find/Label connected regions in the 2d array
        labeled_array, num_connected_regions = label(occupied_mask)
        
        # Iterate over each connected region
        for region in range(1, num_connected_regions + 1):
            #Get all cells from the current region/cluster
            region_indices = np.where(labeled_array == region)
            #Get total amount of cells in cluster/region
            region_size = len(region_indices[0])
            if region_size > self.max_cone_cells:
                # Mark all cells in this region as -1 if the size of the cluster is too big
                data_array[region_indices] = -1
        
        filtered_map = OccupancyGrid()
        filtered_map.header = occupancy_map.header
        filtered_map.info = occupancy_map.info
        filtered_map.data = data_array.flatten().tolist()
        
        return filtered_map

    def transform_point_to_frame(self, point, header, target_frame):
        """
        Transform a Point from its original frame to a target frame using TF.
        
        :param point: The geometry_msgs/Point to transform.
        :param header: The header (containing frame_id and stamp) of the point.
        :param target_frame: The target frame to transform the point into.
        :return: The transformed point or None if the transform failed.
        """
        try:
            stamp_time = rclpy.time.Time.from_msg(header.stamp)
            transform_stamped = self.tf_buffer.lookup_transform(
                target_frame,
                header.frame_id,
                stamp_time,
                timeout=Duration(seconds=0.5)
            )
            ps = PointStamped()
            ps.header = header
            ps.point = point
            pt_map = tf2_geometry_msgs.do_transform_point(ps, transform_stamped)
            return pt_map
        except Exception as e:
            self.get_logger().warn(f"Failed to transform cone to {target_frame}: {e}")
            return None

    def world_to_grid(self, x, y, origin_x, origin_y, resolution):
        """
        Convert world/odom coordinates (x_world, y_world) to grid indices (gx, gy)
        based on the origin and resolution of the occupancy grid.
        
        :param x: The world x-coordinate.
        :param y: The world y-coordinate.
        :param origin_x: The x-coordinate of the map origin.
        :param origin_y: The y-coordinate of the map origin.
        :param resolution: The map resolution.
        :return: A tuple (grid_x, grid_y).
        """
        gx = int((x - origin_x) / resolution)
        gy = int((y - origin_y) / resolution)
        return (gx, gy)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
