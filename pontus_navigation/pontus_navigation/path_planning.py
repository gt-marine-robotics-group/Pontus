import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer

from pontus_msgs.srv import GetPathToObject
# from pontus_tools.pontus_tools import transform_helpers # what's the correct way to do this?

import tf2_ros  # replace this and helper function with above import
from tf2_geometry_msgs import do_transform_pose
from rclpy.logging import get_logger
from typing import Optional
from rclpy.time import Time
from geometry_msgs.msg import Pose

import numpy as np
import math
from copy import copy
import heapq

# PSA: Times are currently set based on the message time. This may cause issues during real running. Change the times to current time
LOGGER = get_logger(__name__)  # replace this when import is fixed


class path_planner(Node):
    def __init__(self) -> None:
        super().__init__('path_planner')

        self.get_path_to_object_srv = self.create_service(
            GetPathToObject,
            '/pontus/path_planning_service',
            self.get_path_to_pose_callback
        )

        self.occupancy_grid = self.create_subscription(
            OccupancyGrid,
            '/pontus/occupancy_grid',
            self.occupancy_grid_callback,
            10
        )

        self.current_odom = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.current_pose_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/pontus/path',
            10
        )

        self.latest_path = None

        # self.timer = self.create_timer(
        #    0.1,
        #    self.occupancy_grid_update
        # ) Uncomment this line to use a timer update/remove comment in pointcloud_callback

        self.latest_occupancy_grid: np.ndarray = None
        self.map_info: MapMetaData = None
        self.current_position: tuple = None
        self.current_stamp = None

        self.height_min = -2.0  # if robot below height, path to target height
        self.height_max = 1.0
        # currently avg b/w min and max height
        self.target_height = (self.height_max + self.height_min) / 2
        # occupancy grid applies score to cells with cluster points inside, if score is higher than threshold, we consider it occupied
        self.score_threshold = 25

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub_timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info("Path Planning Service started")

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        """
        Saves received occupancy grid to latest_occupancy_grid
        Args:
        ----
        msg (OccupancyGrid): subscribed OccupancyGrid
        """

        if msg.header.frame_id != 'map':
            # log an error
            self.get_logger().info("Occupancy grid is not in map frame")
        else:
            # Comment this line if current time is desired
            self.current_stamp = msg.header.stamp

            self.latest_occupancy_grid = np.array(msg.data).reshape(
                # needs to be transposed for some reason
                (msg.info.width, msg.info.height)).transpose()

            self.map_info = msg.info

    def current_pose_callback(self, msg: Odometry) -> None:
        """
        Saves received PoseStamped to current_position
        Args:
        ----
        msg (PoseStamped): subscribed PoseStamped
        """
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose

        if msg.header.frame_id != 'map':
            # transform into map frame
            try:
                temp_pose_msg = self.convert_to_map_frame(
                    pose_msg, self.tf_buffer)
                pose_msg.pose = temp_pose_msg
                # self.get_logger().info("SUCCESS ON TRANSFORM")
            except Exception as e:
                self.get_logger().warn(
                    f"failure to transform pose to map frame, current pose frame: {msg.header.frame_id}, error: {str(e)}")
                return

        # Comment this line if current time is desired
        self.current_stamp = msg.header.stamp

        self.current_position = (
            pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)

    def get_path_to_pose_callback(self, request: GetPathToObject.Request, response: GetPathToObject.Response) -> GetPathToObject.Response:
        """
        Uses occupancy grid to update current_path
        Args:
        ----
        request (geometry_msgs/PoseStamped):  pose of goal
        response (nav_msgs/Path):  path of poses to goal
        """
        open_cells = [
            # scale float by 1000 then turn to int, heapq.heappush(open_cells, (int(priority*1000), index) )
        ]
        closed_cells = set()
        cell_record = dict()
        response.found = False

        response_path = Path()
        response_path.header.frame_id = "map"
        if self.current_stamp is not None:
            response_path.header.stamp = self.current_stamp

        if self.latest_occupancy_grid is None:
            self.get_logger().warn("Occupancy Grid not initialized")
            response.path_to_object = response_path
            return response
        elif self.current_position is None:
            self.get_logger().warn("Position not initialized")
            response.path_to_object = response_path
            return response

        current_index = self.position_to_index(
            (self.current_position[0], self.current_position[1]))
        goal_position = (request.goal.pose.position.x,
                         request.goal.pose.position.y)
        goal_index = self.position_to_index(goal_position)

        try:
            self.latest_occupancy_grid[goal_index[0], goal_index[1]]
        except IndexError as e:
            self.get_logger().warn(
                f"Index out of bounds: {e}. Goal not within map")
            response.path_to_object = response_path
            return response

        path_list = []

        # just to be safe, adjust height
        if self.current_position[2] < self.height_min or self.current_position[2] > self.height_max:
            path_list.append(
                self.current_position[0], self.current_position[1], self.target_height)

        current_node = pathSearchRecord(
            current_index, path_list, 0.0, self.get_heuristic(current_index, goal_index))

        heapq.heappush(open_cells, (0, current_node.node_index))
        cell_record[current_node.node_index] = current_node

        goal_not_explored = True

        while goal_not_explored and (len(open_cells) > 0):
            current_index = heapq.heappop(open_cells)[1]
            current_node = cell_record[current_index]

            closed_cells.add(current_index)

            if current_index == goal_index:
                goal_not_explored = False
            else:
                adjacent_indices = self.get_adjacency(
                    current_index, self.latest_occupancy_grid, True)
                for index in adjacent_indices:
                    if not closed_cells.__contains__(index):
                        cost_so_far = current_node.cost_to_reach + \
                            self.get_cost(
                                current_node.node_index, index)  # float
                        if not cell_record.__contains__(index):
                            f_value = cost_so_far + \
                                self.get_heuristic(index, goal_index)

                            temp_list = copy(current_node.path_to_cell)
                            temp_list.append(index)

                            temp_node = pathSearchRecord(
                                index, temp_list, cost_so_far, f_value)

                            heapq.heappush(
                                open_cells, (int(f_value*1000), index))
                            cell_record[temp_node.node_index] = temp_node

                        elif cost_so_far < cell_record[index].cost_to_reach:
                            f_value = cost_so_far + \
                                self.get_heuristic(index, goal_index)

                            temp_list = copy(current_node.path_to_cell)
                            temp_list.append(index)

                            temp_node = pathSearchRecord(
                                index, temp_list, cost_so_far, f_value)

                            open_cells = self.remove_item_from_heap(
                                open_cells, index)
                            heapq.heappush(
                                open_cells, (int(f_value*1000), index))

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

        self.latest_path = response.path_to_object

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
        adjacent_cells = []
        directions = [[0, 1], [1, 0], [0, -1], [-1, 0]]  # cardinal directions
        if eight_way:
            extra_directions = [[1, 1], [-1, 1],
                                [1, -1], [-1, -1]]  # diagonal directions
            directions.extend(extra_directions)

        max_width = array.shape[0]
        max_height = array.shape[1]

        for direction in directions:
            if index[0] + direction[0] >= 0 and \
                    index[1] + direction[1] >= 0 and \
                    index[0] + direction[0] < max_width and \
                index[1] + direction[1] < max_height and \
                    array[index[0] + direction[0], index[1] + direction[1]] <= self.score_threshold:  # unoccupied cell
                adjacent_cells.append(
                    (index[0] + direction[0], index[1]+direction[1]))
        return adjacent_cells

    # euclidean distance
    def get_cost(self, start_index: tuple[int], adjacent_index: tuple[int]):
        return math.sqrt((start_index[0] - adjacent_index[0])**2 + (start_index[1] - adjacent_index[1])**2)

    # also euclidean distance, but we can add other heuristics
    def get_heuristic(self, adjacent_index: tuple[int], goal_index: tuple[int]):
        return math.sqrt((adjacent_index[0] - goal_index[0])**2 + (adjacent_index[1] - goal_index[1])**2)

    def position_to_index(self, position: tuple[float]) -> tuple[int]:  # TODO
        x_index = int(
            (position[0] - self.map_info.origin.position.x) // self.map_info.resolution)
        y_index = int(
            (position[1] - self.map_info.origin.position.y) // self.map_info.resolution)
        return (x_index, y_index)

    def index_to_position(self, index: tuple[int]) -> tuple[float]:
        x_pos = float(self.map_info.resolution *
                      index[0] + self.map_info.origin.position.x)
        y_pos = float(self.map_info.resolution *
                      index[1] + self.map_info.origin.position.y)
        return (x_pos, y_pos)

    def remove_item_from_heap(self, item_list, item):  # lin search, sue me
        for i in range(len(item_list)):
            if item == item_list[i][1]:
                del item_list[i]
                break
        return item_list

    def convert_to_map_frame(self,
                             pose_stamped: PoseStamped,
                             tf_buffer: tf2_ros.Buffer,
                             target_frame: str = 'map',
                             ) -> Optional[Pose]:
        """Transform a pose into the target frame (defaults to ``map``).

        Args:
            pose_stamped: Pose to transform.
            tf_buffer: TF2 buffer used to look up transforms.
            target_frame: Desired frame for the resulting pose.

        Returns:
            The transformed pose in the target frame, or ``None`` if the
            transform was unavailable.
        """
        try:
            transform = tf_buffer.lookup_transform(
                target_frame,
                pose_stamped.header.frame_id,
                Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            LOGGER.warning(
                'Failed to lookup transform from %s to %s: %s',
                pose_stamped.header.frame_id,
                target_frame,
                exc,
            )
            return None
        if type(pose_stamped) is PoseStamped:
            pose_stamped = pose_stamped.pose
        transformed = do_transform_pose(pose_stamped, transform)
        return transformed.pose if hasattr(transformed, 'pose') else transformed

    def publish_path(self):
        """
        Publishes the latest path followed to /pontus/path
        """

        if self.latest_path:
            self.path_pub.publish(self.latest_path)


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
