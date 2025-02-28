##
# Abstracted
# go_to_pose
# current_state
# at_pose

import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

from pontus_msgs.action import SearchRegion


class SearchRegionClient:
    def __init__(self, node):
        self.action_client = ActionClient(
            node,
            SearchRegion,
            '/pontus/search_region',
        )

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error("SearchRegion action server not available.")
        
        self.completed = False
        self.is_in_progress = False
        self.next_pose = None
    
    # Ros architecture
    def search_region(self, search_region: Polygon):
        self.completed = False
        self.is_in_progress = True
        goal_msg = SearchRegion.Goal()
        goal_msg.search_zone = search_region
        
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback):
        self.next_pose = feedback.feedback.next_pose
    

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        completed = result.completed
        # Idk what to do here
        if not completed:
            pass
        self.completed = True

    # Abstraction Layer
    def get_next_pose(self):
        return self.next_pose

    def finished_searching(self):
        return self.completed

    def in_progress(self):
        return self.is_in_progress


class Test(Node):
    def __init__(self):
        super().__init__('test_searchregion')
        polygon = Polygon()
        polygon.points.append(Point32(x=1.0,y=1.0))
        polygon.points.append(Point32(x=1.0,y=5.0))
        polygon.points.append(Point32(x=5.0,y=5.0))
        polygon.points.append(Point32(x=5.0,y=1.0))
        search_region_client = SearchRegionClient(self)
        search_region_client.search_region(polygon)

def main(args=None):
    rclpy.init(args=args)
    node = Test()
    rclpy.spin(node)
    

