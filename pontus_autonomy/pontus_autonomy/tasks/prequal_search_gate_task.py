import rclpy

import math
from math import fabs
import tf_transformations
import numpy as np

from geometry_msgs.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj

from pontus_msgs.msg import SemanticMap
from pontus_msgs.srv import AddSemanticObject
from pontus_autonomy.helpers.run_info_helpers import SemanticMapObject

class PrequalSearchTask(BaseTask):
    
    def __init__(self, fallback_points = None):
        super().__init__("prequal_search_gate_task")
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('number_of_spins', 2),
                ('gate_width', 3.048),
                ('gate_width_tolerance', 0.3),
                ('pool_depth', 2.0),
                ('height_from_bottom', 0.5)
            ]
        )
        
        self.number_of_spins = self.get_parameter('number_of_spins').value
        self.pool_depth : float = float(self.get_parameter('pool_depth').value)
        self.height_from_bottom_m : float = float(self.get_parameter('height_from_bottom').value)
        self.gate_width_m : float = float(self.get_parameter('gate_width').value)
        self.gate_width_tolerance_m : float = float(self.get_parameter('gate_width_tolerance').value)
        self.fallback_points = fallback_points[0]
        
        # Local Variables
        self.total_rad = self.number_of_spins * 2 * math.pi
        self.start_turn = False
        self.current_rad = 0
            
        # ROS Subscriptions
        self.semantic_map_sub = self.create_subscription(
            SemanticMap,
            '/pontus/semantic_map',
            self.semantic_map_callback,
            10
        )
        
        
        # ROS Action/Service Manager
        self.go_to_pose_client = GoToPoseClient(self)
        
        self.add_semantic_object_client = self.create_client(
            AddSemanticObject,
            '/pontus/add_semantic_object',
        )
        
        self.get_logger().info("Finished Setting Up")
        
        self.turn_timer = self.create_timer(1, self.turn_callback, self.service_callback_group)
    
    def semantic_map_callback(self, msg: SemanticMap) -> None:
        """
        When semantic map is updated, we need to check if there is a valid gate pair,
        if there is, we complete the autonomy.
        
        Args:
            msg [SemanticMap] : contains label and position data of objects like gate sides on the course
        
        Returns:
            N/A
        """
        if self.detect_gate_pair(msg):
            self.complete(True)
            pass
    
    def detect_gate_pair(self, sem_map : SemanticMap) ->  bool:
        """
        Given the semantic map. Check all gate side detections to see if
        any pair meet the conditions to be a gate pair
        
        - Distance between the two gate sides is very close to the known 
          expected value as given in the RoboSub Team Handbook

        Once the candidate pair is found, we return True. Otherwisse, False

        Args:
            sem_map [SemanticMap] : Map containing all detected semantic objects
        
        Return:
            bool : If a gate pair exists 
        """

        gate_list = sem_map.gate_left

        if len(gate_list) < 2:
            return False

        pair_exists = False

        for i in range(len(gate_list) - 1):
            for j in range(i+1, len(gate_list)):
                gate_i_vec : np.ndarray = self._pose_to_nparray(gate_list[i].pose.pose)
                gate_j_vec : np.ndarray = self._pose_to_nparray(gate_list[j].pose.pose)

                gate_width_est : float = float(np.linalg.norm(gate_i_vec - gate_j_vec))

                if abs(gate_width_est - self.gate_width_m) <= self.gate_width_tolerance_m:
                    pair_exists = True

        return pair_exists
        
    
    def turn_callback(self) -> None:
        """
        Checks if we have completed turn, If we have, we turn.
        We turn in increments since sending a full 360 turn gets simplified to 0.
        If we turn more than the total_rad, we have passed tolerance and use fallback points
        
        Returns:
            N/A
        
        """
        if not self.start_turn or self.go_to_pose_client.at_pose():
            self.start_turn = True 
            self.current_rad = self.current_rad + (0.25 * 2 * math.pi)
            
            if self.total_rad == 0:
                self.get_logger().info("We've completed all rotations and found no poles")
                self._send_semantic_object()
                self.complete(True)
            else:
                self.turn_command(self.current_rad)
                self.total_rad -= (0.25 * 2 * math.pi)
    
    def _send_semantic_object(self) -> None:
        """
        Sends a semantic objects to be added to the semantic map.
        This is used for adding the fallback points to the semantic map
        
        Returns:
            N/A
        """
        if not self.add_semantic_object_client.service_is_ready():
            self.get_logger().warn(
                "AddSemanticObject service not available"
            )
            return
        
        req = AddSemanticObject.Request()
        self.get_logger().info(f"{self.fallback_points}")
        req.ids = [self.fallback_points[0].label, self.fallback_points[1].label]
        req.positions = [self.fallback_points[0].get_pose_stamped(), self.fallback_points[1].get_pose_stamped()]
        
        self.add_semantic_object_client.call_async(req)
    
    def turn_command(self, target_angle_rad: float) -> None:
        """
        Issues a turn command to the Pontus so that it turns a certain amount of radians
        
        Args:
            target_angle_rad [float] : The total amount of radians to turn
        
        Returns:
            N/A
        """
        cmd_pose = Pose()
        
        yaw_abs = target_angle_rad
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(
            0.0, 0.0, yaw_abs)

        #How can we provide the current position so that it doens't change?
        cmd_pose.position.x = 0.0
        cmd_pose.position.y = 0.0
        cmd_pose.position.z = -self.pool_depth + self.height_from_bottom_m
        
        cmd_pose.orientation.x = qx
        cmd_pose.orientation.y = qy
        cmd_pose.orientation.z = qz
        cmd_pose.orientation.w = qw
        
        self.go_to_pose_client.go_to_pose(pose_obj=PoseObj(cmd_pose=cmd_pose, skip_orientation=False))
    
    def _pose_to_nparray(self, msg: Pose) -> np.ndarray:
        """
        Convert a Pose into a 2D numpy array
        
        Args:
            msg [Pose]
        Return:
            [np.ndarray] : Vector version of the ROS Pose for easy math
        """
        return np.array([
            msg.position.x,
            msg.position.y],
            dtype=float
        )
        

