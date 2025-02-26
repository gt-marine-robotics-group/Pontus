import numpy as np
import queue

import rclpy
from rclpy.node import Node
import tf_transformations
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

'''
This is intended to mark areas that are considered explored. 
Define exploration regions.
Give next interested region if not found. 

Be able to update the bounds of the map based on our sonar

The exploration map is represneted through an occupancy grid
'''

class ExplorationMapManager(Node):
    def __init__(self):
        super().__init__('exploration_map_manager')

        self.exploration_map_publisher = self.create_publisher(
            OccupancyGrid,
            '/pontus/exploration_map',
            10,
        )

        self.odometry_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odometry_callback,
            10,
        )

        self.timer = self.create_timer(
            0.2,
            self.exploration_map_update
        )

        self.declare_parameter('perception_fov', 0.8)
        self.declare_parameter('perception_distance', 5.0)
        self.declare_parameter('map_resolution', 5.0)
        self.declare_parameter('map_width', 50)
        self.declare_parameter('map_height', 50)

        self.perception_fov = self.get_parameter('perception_fov').value
        self.perception_distance = self.get_parameter('perception_distance').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value

        self.current_odometry = None

        self.map_width_cells = int(self.map_width / self.map_resolution)
        self.map_height_cells = int(self.map_height / self.map_resolution)

        # Set basic information
        self.exploration_map: OccupancyGrid = OccupancyGrid()
        self.exploration_map.header.frame_id = 'map'
        self.exploration_map.info.resolution = self.map_resolution
        self.exploration_map.info.width = self.map_width_cells
        self.exploration_map.info.height = self.map_height_cells
        self.exploration_map.info.origin.position.x = -self.map_width / 2 
        self.exploration_map.info.origin.position.y = -self.map_height / 2
        self.exploration_map.info.origin.position.z = 0.0
        self.exploration_map.info.origin.orientation.x = 0.0
        self.exploration_map.info.origin.orientation.y = 0.0
        self.exploration_map.info.origin.orientation.z = 0.0
        self.exploration_map.info.origin.orientation.w = 1.0
        self.exploration_map.data = [0] * int(self.map_height_cells * self.map_width_cells)
        self.previous_odometry = None
    

    # Callbacks
    def odometry_callback(self, msg: Odometry):
        if not self.current_odometry:
            self.previous_odometry = msg
        self.current_odometry = msg
    

    def convert_odom_point_to_map(self, current_point: tuple, map_width: float, map_height: float, map_resolution: float) -> int:
        """
        Takes a point from odometry and converts it to map frame

        Parameters:
        current_point (tuple(float, float)) : point we want to convert
        map_width (float) : the width of the map in meters
        map_height (float) : the height of the map in meters
        map_resolution (float) : the resolution of the map in meters

        Returns:
        int : the index in the map corresponding to the point
        """
        map_x = self.my_floor(current_point[1], map_resolution) / map_resolution - map_width / 2
        map_y = self.my_floor(current_point[0], map_resolution) / map_resolution - map_height / 2
        return int(map_x * map_width + map_y)


    def calculate_angle_difference(self, angle1: float, angle2: float) -> float:
        """
        Returns the absolute difference in radians between the two angles

        Parameters:
        angle1 (float) : first angle in radians
        angle2 (float) : second angle in radians

        Returns:
        float : the difference in radians between angle1 and angle2
        """
        q1 = tf_transformations.quaternion_from_euler(0.0, 0.0, angle1)
        q2 = tf_transformations.quaternion_from_euler(0.0, 0.0, angle2)

        q1_inv = tf_transformations.quaternion_inverse(q1)
        q_diff = tf_transformations.quaternion_multiply(q2, q1_inv)

        angle = 2 * np.arccos(np.clip(q_diff[3], -1.0, 1.0))

        return angle
        

    def within_bounds(self,
                      current_x: float,
                      current_y: float,
                      current_yaw: float,
                      new_x: float,
                      new_y: float,
                      perception_fov: float,
                      perception_distance: float) -> bool:
        """
        Checks whether a given point is within our expected FOV

        Parameters:
        current_x (float) : current_x position
        current_y (float) : current_y position
        current_yaw (float) : Our current yaw
        new_x (float) : new_x position we want to check if is perception bounds
        new_y (float) : new_y position we want to check if is perception bounds
        perception_fov (float) : The field of view of our perception in radians
        perception_distance (float) : The max distance of our perception in meters

        Returns:
        bool : whether or not this point falls in this bound
        """
        # Calculate distance
        new_distance = ((new_x - current_x) ** 2 + (new_y - current_y) ** 2) ** 0.5
        # Calculate angle
        new_yaw = np.arctan2(new_y - current_y, new_x - current_x)
        angle_diff = self.calculate_angle_difference(new_yaw, current_yaw)
        # self.get_logger().info(f"{new_x, new_y, new_yaw, current_yaw}")
        return (- perception_fov / 2 <= angle_diff <= perception_fov / 2) \
            and (new_distance <= perception_distance) 
        

    def my_floor(self, value: float, resolution: float) -> float:
        """
        Performs a custom floor to a specific resolution

        Parameters:
        value (float) : the value to be floored
        resolution (float) : the resolution at which the value should be floored to

        Returns:
        (float) : floored value
        """
        return value - (value % resolution)


    def get_new_visited_squares(self,
                                current_odom: Odometry,
                                perception_fov: float,
                                perception_distance: float,
                                map_resolution: float) -> list:
        """
        Performs breadth first search to generates points within the perception FOV 
        and distance from the current pose

        Parameters:
        current_odom (nav_msgs/Odometry) : The current pose of the sub
        perception_fov (float) : The field of view of our perception in radians
        perception_distance (float) : The max distance of our perception in meters
        map_resolution (float) : The resolution of our map in meters

        Returns:
        list: A list of (x,y) coordinates at the map_resolution that should be set as
        visited
        """
        # Quantisize these values
        current_x = self.my_floor(current_odom.pose.pose.position.x, map_resolution)
        current_y = self.my_floor(current_odom.pose.pose.position.y, map_resolution)

        q = queue.Queue()
        visited_set = []
        starting_points = [(-map_resolution, map_resolution),
                           (map_resolution, -map_resolution),
                           (map_resolution, map_resolution),
                           (-map_resolution, -map_resolution),
                           (0.0, map_resolution),
                           (0.0, -map_resolution),
                           (map_resolution, 0.0),
                           (-map_resolution, 0.0)]

        _, _, current_yaw = tf_transformations.euler_from_quaternion([current_odom.pose.pose.orientation.x,
                                                                      current_odom.pose.pose.orientation.y,
                                                                      current_odom.pose.pose.orientation.z,
                                                                      current_odom.pose.pose.orientation.w])

        # Get starting points for breadth first search
        for starting_point in starting_points:
            new_x = current_x + starting_point[0]
            new_y = current_y + starting_point[1]
            if self.within_bounds(current_x, current_y, current_yaw, new_x, new_y, perception_fov, perception_distance):
                q.put((new_x, new_y))
        
        # Run breadth first search
        neighbors = [(0.0, map_resolution), (map_resolution, 0.0), (-map_resolution, 0.0), (0.0, -map_resolution)]
        while not q.empty():
            current_node = q.get()
            visited_set.append(current_node)
            for neighbor in neighbors:
                # If already visited or invalid point, skip
                new_point = (current_node[0] + neighbor[0], current_node[1] + neighbor[1])
                if new_point in visited_set or \
                    not self.within_bounds(current_x, current_y, current_yaw, new_point[0], new_point[1], perception_fov, perception_distance):
                    continue
                visited_set.append(new_point)
                q.put(new_point)
        
        return visited_set


    def exploration_map_update(self):
        """
        Timer callback function to update the exploration map with our current FOV
        """
        if not self.current_odometry:
            self.get_logger().info("Waiting on odometry callback, skipping")
            return
        # Calculate which squares should be marked visited 
        new_visited = self.get_new_visited_squares(self.current_odometry, self.perception_fov, self.perception_distance, self.map_resolution)
        for point in new_visited:
            visited_square_location_index = self.convert_odom_point_to_map(point, self.map_width_cells, self.map_height_cells, self.map_resolution)
            self.exploration_map.data[visited_square_location_index] = 100
        
        # Update on map where the robot is
        prev_robot_map_index = self.convert_odom_point_to_map(
            (self.previous_odometry.pose.pose.position.x, self.previous_odometry.pose.pose.position.y),
            self.map_width_cells,
            self.map_height_cells,
            self.map_resolution
        )
        self.exploration_map.data[prev_robot_map_index] = 100

        current_robot_map_index = self.convert_odom_point_to_map(
            (self.current_odometry.pose.pose.position.x, self.current_odometry.pose.pose.position.y),
            self.map_width_cells,
            self.map_height_cells,
            self.map_resolution
        )
        self.exploration_map.data[current_robot_map_index] = -1
        
        
        self.exploration_map.header.stamp = self.get_clock().now().to_msg()
        self.exploration_map_publisher.publish(self.exploration_map)
        self.previous_odometry = self.current_odometry


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationMapManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()