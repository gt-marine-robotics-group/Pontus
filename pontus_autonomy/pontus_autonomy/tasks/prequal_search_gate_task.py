import rclpy
import math
import tf_transformations

from geometry_msgs.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj

from pontus_msgs.msg import SemanticMap
from pontus_msgs.srv import AddSemanticObject


class PrequalSearchTask(BaseTask):

    def __init__(self, fallback_points=None):
        super().__init__("prequal_search_gate_task")

        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pool_depth', 1.8),
                ('height_from_bottom', 1.4)
            ]
        )

        self.pool_depth: float = float(self.get_parameter('pool_depth').value)
        self.height_from_bottom_m: float = float(self.get_parameter('height_from_bottom').value)

        self.fallback_points = fallback_points

        self.search_angles = [math.radians(45), math.radians(-45)]
        self.search_index = 0
        self.start_turn = False

        self.turn_interval = 3.0  # seconds
        self.last_turn_time = self.get_clock().now()

        # ROS Subscriptions
        self.semantic_map_sub = self.create_subscription(
            SemanticMap,
            '/pontus/semantic_map',
            self.semantic_map_callback,
            10
        )

        # Action/Service Clients
        self.go_to_pose_client = GoToPoseClient(self)

        self.add_semantic_object_client = self.create_client(
            AddSemanticObject,
            '/pontus/add_semantic_object',
        )

        self.get_logger().info("Finished Setting Up")

        # Timer for turning behavior
        self.turn_timer = self.create_timer(
            0.5,
            self.turn_callback,
            self.service_callback_group
        )

    def semantic_map_callback(self, msg: SemanticMap) -> None:
        """
        If a gate is detected, stop the search immediately.
        """
        if msg.meta_gate.header.frame_id != "":
            self.get_logger().info("Gate pair detected in semantic map")
            self.complete(True)

    def turn_callback(self) -> None:
        """
        Alternates between +45° and -45° relative turns.
        """

        # Wait for previous motion to finish
        if not self.start_turn or self.go_to_pose_client.at_pose():

            now = self.get_clock().now()
            dt = (now - self.last_turn_time).nanoseconds * 1e-9
            if dt < self.turn_interval:
                return

            self.last_turn_time = now
            self.start_turn = True

            # Select next angle (+45 or -45)
            target_angle = self.search_angles[self.search_index]

            self.get_logger().info(
                f"Turning {'right' if target_angle > 0 else 'left'} "
                f"{math.degrees(abs(target_angle))} degrees"
            )

            self.turn_command(target_angle)

            # Alternate index
            self.search_index = (self.search_index + 1) % len(self.search_angles)

    def _send_semantic_object(self) -> None:
        """
        Sends fallback semantic objects if needed.
        """
        return

        # if not self.add_semantic_object_client.service_is_ready():
        #     self.get_logger().warn("AddSemanticObject service not available")
        #     return

        # req = AddSemanticObject.Request()
        # req.ids = [self.fallback_points[0][0], self.fallback_points[1][0]]
        # req.positions = [self.fallback_points[0][1], self.fallback_points[1][1]]

        # self.add_semantic_object_client.call_async(req)

    def turn_command(self, relative_yaw: float) -> None:
        """
        Sends a RELATIVE rotation command.
        """

        cmd_pose = Pose()

        # Convert yaw to quaternion
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(
            0.0, 0.0, relative_yaw
        )

        # No positional movement
        cmd_pose.position.x = 0.0
        cmd_pose.position.y = 0.0
        cmd_pose.position.z = -self.pool_depth + self.height_from_bottom_m

        cmd_pose.orientation.x = qx
        cmd_pose.orientation.y = qy
        cmd_pose.orientation.z = qz
        cmd_pose.orientation.w = qw

        self.go_to_pose_client.go_to_pose(
            pose_obj=PoseObj(
                cmd_pose=cmd_pose,
                use_relative_position=True,
                skip_orientation=False
            )
        )


# import rclpy

# import math
# from math import fabs
# import tf_transformations

# from geometry_msgs.msg import Pose
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# from pontus_autonomy.tasks.base_task import BaseTask
# from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj

# from pontus_msgs.msg import SemanticMap
# from pontus_msgs.srv import AddSemanticObject

# class PrequalSearchTask(BaseTask):
    
#     def __init__(self, fallback_points = None):
#         super().__init__("prequal_search_gate_task")
#         self.service_callback_group = MutuallyExclusiveCallbackGroup()
        
#         # Parameters
#         self.declare_parameters(
#             namespace='',
#             parameters=[
#                 ('number_of_spins', 0),
#                 ('pool_depth', 2.0),
#                 ('height_from_bottom', 0.5)
#             ]
#         )
        
#         self.number_of_spins = self.get_parameter('number_of_spins').value
#         self.pool_depth : float = float(self.get_parameter('pool_depth').value)
#         self.height_from_bottom_m : float = float(self.get_parameter('height_from_bottom').value)
#         self.fallback_points = fallback_points
        
#         # Local Variables
#         self.total_rad = self.number_of_spins * 2 * math.pi
#         self.start_turn = False
#         self.current_rad = 0
            
#         # ROS Subscriptions
#         self.semantic_map_sub = self.create_subscription(
#             SemanticMap,
#             '/pontus/semantic_map',
#             self.semantic_map_callback,
#             10
#         )
        
        
#         # ROS Action/Service Manager
#         self.go_to_pose_client = GoToPoseClient(self)
        
#         self.add_semantic_object_client = self.create_client(
#             AddSemanticObject,
#             '/pontus/add_semantic_object',
#         )
        
#         self.get_logger().info("Finished Setting Up")
        
#         self.turn_timer = self.create_timer(1, self.turn_callback, self.service_callback_group)
    
#     def semantic_map_callback(self, msg: SemanticMap) -> None:
#         """
#         When semantic map is updated, we need to check if there is a valid gate pair,
#         if there is, we complete the autonomy.
        
#         Args:
#             msg [SemanticMap] : contains label and position data of objects like gate sides on the course
        
#         Returns:
#             N/A
#         """
#         if msg.meta_gate.header.frame_id != "":
#             #self.complete(True)
#             self.get_logger().info("Gate pair detected in semantic map")
#             self.complete(True)
        
    
#     def turn_callback(self) -> None:
#         """
#         Checks if we have completed turn, If we have, we turn.
#         We turn in increments since sending a full 360 turn gets simplified to 0.
#         If we turn more than the total_rad, we have passed tolerance and use fallback points
        
#         Returns:
#             N/A
        
#         """
#         if not self.start_turn or self.go_to_pose_client.at_pose():
#             self.start_turn = True 
#             self.current_rad = self.current_rad + (0.25 * 2 * math.pi)
            
#             if self.total_rad == 0:
#                 self.get_logger().info("We've completed all rotations and found no poles")
#                 self._send_semantic_object()
#                 self.complete(True)
#             else:
#                 self.turn_command(self.current_rad)
#                 self.total_rad -= (0.25 * 2 * math.pi)
    
#     def _send_semantic_object(self) -> None:
#         """
#         Sends a semantic objects to be added to the semantic map.
#         This is used for adding the fallback points to the semantic map
        
#         Returns:
#             N/A
#         """
#         if not self.add_semantic_object_client.service_is_ready():
#             self.get_logger().warn(
#                 "AddSemanticObject service not available"
#             )
#             return
        
#         req = AddSemanticObject.Request()
#         req.ids = [self.fallback_points[0][0], self.fallback_points[1][0]]
#         req.positions = [self.fallback_points[0][1], self.fallback_points[1][1]]
        
#         self.add_semantic_object_client.call_async(req)
    
#     def turn_command(self, target_angle_rad: float) -> None:
#         """
#         Issues a turn command to the Pontus so that it turns a certain amount of radians
        
#         Args:
#             target_angle_rad [float] : The total amount of radians to turn
        
#         Returns:
#             N/A
#         """
#         cmd_pose = Pose()
        
#         yaw_abs = target_angle_rad
#         qx, qy, qz, qw = tf_transformations.quaternion_from_euler(
#             0.0, 0.0, yaw_abs)

#         #How can we provide the current position so that it doens't change?
#         cmd_pose.position.x = 0.0
#         cmd_pose.position.y = 0.0
#         cmd_pose.position.z = 0.0
        
#         cmd_pose.orientation.x = qx
#         cmd_pose.orientation.y = qy
#         cmd_pose.orientation.z = qz
#         cmd_pose.orientation.w = qw
        
#         self.go_to_pose_client.go_to_pose(pose_obj=PoseObj(cmd_pose=cmd_pose, use_relative_position=True, skip_orientation=False))
        

