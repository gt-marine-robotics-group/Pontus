import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer

from pontus_msgs.srv import GetPathToObject

import numpy as np
import math
from copy import copy
import heapq

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
        self.map_info: MapMetaData = None
        self.current_position: tuple = None
        self.current_stamp = None
        
        self.height_min = 0.0 # if robot below height, path to target height
        self.height_max = 100.0
        self.target_height = (self.height_max + self.height_min) / 2 # currently avg b/w min and max height
        self.threshold = 0.1 # occupancy grid applies score to cells with cluster points inside, if score is higher than threshold, we consider it occupied

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
            self.current_stamp = msg.header.stamp

            self.latest_occupancy_grid = msg.data
            self.map_info = msg.info
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
        self.current_stamp = msg.header.stamp

        self.current_position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.current_path_update() #comment this line if you want to use a timer update time
           
    def get_path_to_pose_callback(self, request: GetPathToObject.Request, response: GetPathToObject.Response) -> GetPathToObject.Response:
        """
        Uses occupancy grid to update current_path
        Args:
        ----
        request (geometry_msgs/PoseStamped):  pose of goal
        response (nav_msgs/Path):  path of poses to goal
        """
        open_cells = [] # scale float by 1000 then turn to int, heapq.heappush(open_cells, (int(priority*1000), index) )
        closed_cells = set()
        cell_record = dict()
        response.found = False

        response_path = Path()
        response_path.header.frame_id = "map"
        response_path.header.stamp = self.current_stamp

        current_index = self.position_to_index(self.current_position[0], self.current_position[1])
        goal_position = (request.goal.pose.position.x, request.goal.pose.position.y)
        goal_index = self.position_to_index(goal_position)

        try:
            self.latest_occupancy_grid[goal_index]
        except IndexError as e:
            self.get_logger().warn("Index out of bounds: ", e)
            response.path_to_object = response_path
            return response

        path_list = []

        if self.current_position[2] < self.min_height or self.current_position[2] > self.max_height: # just to be safe, adjust height
            path_list.append(self.current_position[0], self.current_position[1], self.target_height)
         
        current_node = pathSearchRecord(current_index, path_list, 0.0, self.get_heuristic(current_index, goal_index))

        heapq.heappush(open_cells,(0, current_index.node_index))
        cell_record[current_node.node_index] = current_node

        goal_not_explored = True

        while goal_not_explored and (len(open_cells) > 0):
            current_index = heapq.heappop(open_cells)[1]
            current_node = cell_record[current_index]

            closed_cells.add(current_index)

            if current_index == goal_index:
                goal_not_explored = False
            else:
                adjacent_indices = self.get_adjacency(current_index, self.latest_occupancy_grid, True)
                for index in adjacent_indices:
                    if not closed_cells.__contains__(index):
                        cost_so_far = current_node.cost_to_reach + self.get_cost(current_node.node_index, index) # float
                        if not cell_record.__contains__(index):
                            f_value = cost_so_far + self.get_heuristic(index, goal_index)

                            temp_list = copy(current_node)
                            temp_list.append(index)

                            temp_node = pathSearchRecord(index, temp_list, cost_so_far, f_value)

                            heapq.heappush(open_cells,(int(f_value*1000), index))
                            cell_record[temp_node.node_index] = temp_node

                        elif cost_so_far < cell_record[index].cost_to_reach:
                            f_value = cost_so_far + self.get_heuristic(index, goal_index)

                            temp_list = copy(current_node)
                            temp_list.append(index)

                            temp_node = pathSearchRecord(index, temp_list, cost_so_far, f_value)

                            open_cells = self.remove_item_from_heap(open_cells, index)
                            heapq.heappush(open_cells, (int(f_value*1000), index))

                            cell_record[index] = temp_node

        if goal_not_explored:
            self.get_logger().warn("Goal index not found")
        else:

            for index in cell_record[goal_index].path_to_cell:

                # get position from indices and target height, skip orientation
                position = self.index_to_position(index)

                temp_pose = PoseStamped()
                temp_pose.pose.position.x = position[0]
                temp_pose.pose.position.y = position[1]
                temp_pose.pose.position.z = self.target_height
                temp_pose.header = response_path.header

                response_path.poses.append(temp_pose)

            response.path_to_object = response_path
            response.found = True
        return response

    def get_adjacency(self, index: tuple[int], array: np.ndarray, eight_way: bool = False) -> list[tuple[int]]:
        """_summary_

        Args:
            index (tuple[int]): index of cell in array to find other 
            array (np.ndarray): occupancy grid, assumes unoccupied spaces have a value of 0 at index
            eight_way (bool, optional): if true, check cells diagonal to index as well. Defaults to False.

        Returns:
            list[tuple[int]]: _description_
        """
        # modify occupancy grid to mark occupied spaces. easier and useful outside of path planning TODO
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
                        array[index[0] + direction[0], index[1] + direction[1]] <= self.threshold: # unoccupied cell
                adjacent_cells.append((index[0] + direction[0], index[1]+direction[1]))
        return adjacent_cells
    
    def get_cost(self, start_index: tuple[int], adjacent_index: tuple[int]): # euclidean distance
        return math.sqrt( (start_index[0] - adjacent_index[0])**2 + (start_index[1] - adjacent_index[1])**2 )
    
    def get_heuristic(self, adjacent_index: tuple[int], goal_index: tuple[int]): #also euclidean distance, but we can add other heuristics
        return math.sqrt( (adjacent_index[0] - goal_index[0])**2 + (adjacent_index[1] - goal_index[1])**2 )
    
    def position_to_index(self, position: tuple[float]) -> tuple[int]: # TODO
        x_index = (position[0] - self.map_info.origin.position.x) // self.map_info.resolution
        y_index = (position[1] - self.map_info.origin.position.y) // self.map_info.resolution
        return (x_index, y_index)
    
    def index_to_position(self, index: tuple[int]) -> tuple[float]:
        x_pos = float(self.map_info.resolution*index[0] + self.map_info.origin.position.x)
        y_pos = float(self.map_info.resolution*index[1] + self.map_info.origin.position.y)
        return (x_pos, y_pos)
    
    def remove_item_from_heap(self, item_list, item): # lin search, sue me
        for i in range(len(item_list)):
            if item == item_list[i][1]:
                del item_list[i]
                break
        return item_list

class pathSearchRecord:
    def __init__(self, node_index: tuple[int], path_to_cell: list[tuple[int]], cost_to_reach: float, estimated_total_cost: float):
        self.node_index = node_index
        self.path_to_cell = path_to_cell
        self.cost_to_reach = cost_to_reach
        self.estimated_total_cost = estimated_total_cost
        pass

def main(args=None):
    rclpy.init(args=args)
    node = path_planner()
    rclpy.spin(node)
    rclpy.shutdown()