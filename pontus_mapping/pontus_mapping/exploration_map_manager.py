import queue
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Polygon

from pontus_msgs.action import SearchRegion
from pontus_mapping.helpers import get_fov_polygon, polygon_contained

'''
This is intended to mark areas that are considered explored.
Define exploration regions.
Give next interested region if not found.

Be able to update the bounds of the map based on our sonar

The exploration map is represneted through an occupancy grid

Set desired region
TODO:
Create Action Server
'''


class ExplorationMapManager(Node):
    def __init__(self):  # noqa: D107
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

        self.search_action_server = ActionServer(
            self,
            SearchRegion,
            '/pontus/search_region',
            execute_callback=self.search_execute_callback
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
    def odometry_callback(self, msg: Odometry) -> None:
        """
        Handle incoming odometry messages.

        Updates the current odometry variables so it can be accessed later

        Args:
        ----
            msg (Odometry): current odometry

        Return:
        ------
            None

        """
        if not self.current_odometry:
            self.previous_odometry = msg
        self.current_odometry = msg

    async def search_execute_callback(self, goal_handle: Any) -> None:
        """
        Search execute callback to handle request to search region.

        Execute callback for /pontus/search_region action. This is intended to receive a polygon,
        then search that polygon and conclude when that polygon is fully searched. The feedback
        should be the next point the AUV should search.

        Args:
        ----
            goal_handle (Any): goal_handle from callback

        Return:
        ------
            None

        """
        request = goal_handle.request
        interested_points = self.calculate_polygon_region_points(request.search_zone,
                                                                 self.map_resolution)
        unvisited_points = set()
        for interested_point in interested_points:
            interested_points_indx = self.convert_odom_point_to_map(interested_point,
                                                                    self.map_width_cells,
                                                                    self.map_height_cells,
                                                                    self.map_resolution)
            if self.exploration_map.data[interested_points_indx] == 100:
                continue
            self.exploration_map.data[interested_points_indx] = 50
            unvisited_points.add(interested_point)
        goal_handle.succeed()
        result = SearchRegion.Result()
        result.completed = True
        return result

    def convert_odom_point_to_map(self,
                                  current_point: tuple,
                                  map_width_cells: float,
                                  map_height_cells: float,
                                  map_resolution: float) -> int:
        """
        Convets a point from odometry to map frame.

        Args:
        ----
            current_point (tuple(float, float)): point we want to convert
            map_width_cells (float): the width of the map in cells
            map_height_cells (float): the height of the map in cells
            map_resolution (float): the resolution of the map in meters

        Return:
        ------
            int: the index in the map corresponding to the point

        """
        map_x = self.my_floor(current_point[1], map_resolution) \
            / map_resolution - map_width_cells / 2
        map_y = self.my_floor(current_point[0], map_resolution) \
            / map_resolution - map_height_cells / 2
        return int(map_x * map_width_cells + map_y)

    def my_floor(self, value: float, resolution: float) -> float:
        """
        Perform a custom floor to a specific resolution.

        Args:
        ----
            value (float): the value to be floored
            resolution (float): the resolution at which the value should be floored to

        Return:
        ------
            (float) : floored value

        """
        return value - (value % resolution)

    def calculate_polygon_region_points(self, polygon: Polygon,
                                        map_resolution: float) -> list[tuple[int, int]]:
        """
        Return points within a polygon.

        Given a geometry_msg/Polygon, return a list of points that are contained within that
        polygon at a given resolution This is done using a simple Breadth-First Search.

        Args:
        ----
            polygon (geometry_msgs/Polygon): Polygon in map coordinates of requested region
            map_resolution (float): The resolution of the map in meters

        Return:
        ------
            list[tuple[int, int]]: a list of points that are within this region

        """
        bfs_queue = queue.Queue()
        visited_set = set()
        starting_point = (self.my_floor(polygon.points[0].x, map_resolution),
                          self.my_floor(polygon.points[0].y, map_resolution))

        neighbor_list = [(-map_resolution, map_resolution),
                         (map_resolution, -map_resolution),
                         (map_resolution, map_resolution),
                         (-map_resolution, -map_resolution),
                         (0.0, map_resolution),
                         (0.0, -map_resolution),
                         (map_resolution, 0.0),
                         (-map_resolution, 0.0)]

        # Initial check to see which points to queue
        for neighbor in neighbor_list:
            new_starting_point = (starting_point[0] + neighbor[0], starting_point[1] + neighbor[1])
            if polygon_contained(polygon, new_starting_point):
                bfs_queue.put(new_starting_point)

        # Run BFS
        while not bfs_queue.empty():
            current_node = bfs_queue.get()
            if current_node in visited_set:
                continue
            visited_set.add(current_node)
            for neighbor in neighbor_list:
                new_point = (current_node[0] + neighbor[0], current_node[1] + neighbor[1])
                if new_point in visited_set or not polygon_contained(polygon, new_point):
                    continue
                bfs_queue.put(new_point)

        return list(visited_set)

    def exploration_map_update(self) -> None:
        """
        Timer callback function to update the exploration map with our current FOV.

        Args:
        ----
            None

        Return:
        ------
            None

        """
        if not self.current_odometry:
            self.get_logger().info("Waiting on odometry callback, skipping")
            return
        # Calculate which squares should be marked visited
        fov_polygon = get_fov_polygon(self.current_odometry,
                                      self.perception_fov,
                                      self.perception_distance)
        new_visited = self.calculate_polygon_region_points(fov_polygon, self.map_resolution)
        for point in new_visited:
            visited_square_location_index = self.convert_odom_point_to_map(point,
                                                                           self.map_width_cells,
                                                                           self.map_height_cells,
                                                                           self.map_resolution)
            self.exploration_map.data[visited_square_location_index] = 100

        # Update on map where the robot is
        prev_robot_map_index = self.convert_odom_point_to_map(
            (self.previous_odometry.pose.pose.position.x,
             self.previous_odometry.pose.pose.position.y),
            self.map_width_cells,
            self.map_height_cells,
            self.map_resolution
        )
        self.exploration_map.data[prev_robot_map_index] = 100

        current_robot_map_index = self.convert_odom_point_to_map(
            (self.current_odometry.pose.pose.position.x,
             self.current_odometry.pose.pose.position.y),
            self.map_width_cells,
            self.map_height_cells,
            self.map_resolution
        )
        self.exploration_map.data[current_robot_map_index] = -1

        # Publish map
        self.exploration_map.header.stamp = self.get_clock().now().to_msg()
        self.exploration_map_publisher.publish(self.exploration_map)
        self.previous_odometry = self.current_odometry


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationMapManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
