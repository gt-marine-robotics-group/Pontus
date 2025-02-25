##
# Abstracted
# go_to_pose
# current_state
# at_pose

from rclpy.action import ActionClient
from geometry_msgs.msg import Pose

from pontus_msgs.action import GoToPose
from pontus_controller.position_controller import PositionControllerState

class GoToPoseClient:
    def __init__(self, node):
        self.action_client = ActionClient(
            node,
            GoToPose,
            '/pontus/go_to_pose',
        )

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error("GoToPose action server not available.")
        self.current_state = PositionControllerState.Maintain_position
        self.completed = False
        self.is_in_progress = False
    
    # Ros architecture
    def go_to_pose(self, goal_pose: Pose):
        self.completed = False
        self.is_in_progress = True
        goal_msg = GoToPose.Goal()
        goal_msg.desired_pose = goal_pose
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback):
        self.current_state = PositionControllerState(feedback.feedback.current_state)
    

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
    def get_current_state(self):
        return self.current_state

    def at_pose(self):
        return self.completed

    def in_progress(self):
        return self.is_in_progress