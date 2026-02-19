import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer

from pontus_msgs.srv import GetPathToObject

import numpy as np
from queue import PriorityQueue
import math

# PSA: Times are currently set based on the message time. This may cause issues during real running. Change the times to current time


class path_planner(Node):
    def __init__(self) -> None:
        super().__init__('occupancy_grid_manager')

        self.get_path_to_object_srv = self.create_service(
            GetPathToObject,
            'path_planning_service',
            self.get_path_to_pose_callback
        )

        self.occupancy_grid = self.create_subscription(
            OccupancyGrid,
            '/pontus/occupancy_grid',
            self.occupancy_grid_callback,
            10
        )

        self.current_pose = self.create_subscription(
            PoseStamped,
            '/pontus/odometry',
            self.current_pose_callback,
            10
        )

        #self.timer = self.create_timer(
        #    0.1,
        #    self.occupancy_grid_update
        #) Uncomment this line to use a timer update/remove comment in pointcloud_callback

        self.latest_occupancy_grid: np.ndarray = None
        self.current_position: tuple = None

        self.path: Path = Path()
        self.path.header.frame_id = 'map'

        self.height_min = 0.0 # if robot below height, path to target height
        self.height_max = 100.0
        self.target_height = (self.height_max + self.height_min) / 2 # currently avg b/w min and max height

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Occupancy Grid started")

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        """
        Saves received occupancy grid to latest_occupancy_grid
        Args:
        ----
        msg (OccupancyGrid): subscribed OccupancyGrid
        """

        if msg.header.frame_id != 'map':
            #log an error
            self.get_logger().info("Occupancy grid is not in map frame")
        else:
            # Comment this line if current time is desired
            self.path.header.stamp = msg.header.stamp

            self.latest_occupancy_grid = msg.data
            self.current_path_update() #comment this line if you want to use a timer update time

    def current_pose_callback(self, msg: PoseStamped) -> None:
        """
        Saves received PoseStamped to current_position
        Args:
        ----
        msg (PoseStamped): subscribed PoseStamped
        """

        if msg.header.frame_id != 'map':
            # transform into map frame
            try:
                msg = self.tf_buffer.transform(
                    msg,
                    target_frame='map',
                )
                # self.get_logger().info("SUCCESS ON TRANSFORM")
            except Exception as e:
                self.get_logger().warn("failure to transform pose to map frame")
                return
            
        # Comment this line if current time is desired
        self.path.header.stamp = msg.header.stamp

        self.current_position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.current_path_update() #comment this line if you want to use a timer update time
           

    def get_path_to_pose_callback(self, request: GetPathToObject.Request, response: GetPathToObject.Response) -> GetPathToObject.Response:
        """
        Uses occupancy grid to update current_path
        Args:
        ----
        occupancy_grid (np.ndarray): np array of all 2d points from occupancy grid
        """
        open_cells = PriorityQueue() # scale float by 1000 then turn to int, open_cells.put((int(priority*1000), index))
        closed_cells = set()
        cell_record = dict() #

        #if self.current_position[2] < min_height or self.current_position[2] > max_height: # just to be safe
        #   add (current_position[0], current_position[1], target_height) to path 

        # convert indices to PoseStamped msgs TODO

        return response



    def get_adjacency(index: tuple[int], array: np.ndarray, eight_way: bool = False) -> list[tuple[int]]:
        """_summary_

        Args:
            index (tuple[int]): index of cell in array to find other 
            array (np.ndarray): occupancy grid, assumes unoccupied spaces have a value of 0 at index
            eight_way (bool, optional): if true, check cells diagonal to index as well. Defaults to False.

        Returns:
            list[tuple[int]]: _description_
        """
        # if occupied cell is within (agent size * clearance multiplier) of cell, cell is untravesable, check with nathan if necessary
        adjacent_cells = []
        directions = [[0,1], [1,0], [0,-1], [-1,0]] # cardinal directions
        if eight_way:
            extra_directions = [[1,1], [-1,1], [1,-1], [-1,-1]] # diagonal directions
            directions.extend(extra_directions)

        max_width = array.shape[0]
        max_height = array.shape[1]

        for direction in directions:
            if index[0] + direction[0] >= 0 and \
            index[1] + direction[1] >= 0 and \
                index[0] + direction[0] < max_width and \
                    index[1] + direction[1] < max_height and \
                        array[index[0] + direction[0], index[1] + direction[1]] == 0: # unoccupied cell
                adjacent_cells.append((index[0] + direction[0], index[1]+direction[1]))
        return adjacent_cells
    
    def get_cost(self, start_index: tuple[int], adjacent_index: tuple[int]): # euclidean distance
        return math.sqrt( (start_index[0] - adjacent_index[0])**2 + (start_index[1] - adjacent_index[1])**2 )
    
    def get_heuristic(self, adjacent_index: tuple[int], goal_index: tuple[int]): #also euclidean distance, but we can add other heuristics
        return math.sqrt( (adjacent_index[0] - goal_index[0])**2 + (adjacent_index[1] - goal_index[1])**2 )
        

def main(args=None):
    rclpy.init(args=args)
    node = path_planner()
    rclpy.spin(node)
    rclpy.shutdown()
