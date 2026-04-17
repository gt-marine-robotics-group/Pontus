import rclpy
import math
import tf_transformations
from enum import Enum
import numpy as np
from rclpy.duration import Duration

from geometry_msgs.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Float64MultiArray

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj

from pontus_msgs.msg import SemanticMap
from pontus_msgs.srv import AddSemanticObject

from pontus_mapping.cluster_coord import CandidateTrack
from nav_msgs.msg import Odometry


from tf2_ros.buffer import Buffer
from tf2_ros import TransformListener
import tf2_geometry_msgs

class SearchConditions(Enum):
    GATE = 0
    SLALOM = 1
    TARGET = 2
    BIN = 3
    OCTAGON = 4

#Notes:
#Angle threshold should be tuned since it's relativtly low right now. Could increase
#Currently, it iterates clusters in groups.
# It takes the message and does all the clusters in that message before doing a new one
# This was done so that it would be consistent and wouldn't have constantly updating or neverending clusters or be using outdated clusters where we see something but already recieved the message for it's arival
# This may be unecessary? 
# Also, the angle print statements are incorrect, but I'm pretty sure everything is working since it turns how I expect with the current layout
# Minor Optimization: Currently, it sorts the angles from smallest to biggest. This does cause issues where it prioritizes angles which are smaller, but farther from the current position. Could be optimized, but may be unecessary

class ScanTask(BaseTask):

    def __init__(
        self, 
        args):
        super().__init__("prequal_search_gate_task")

        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        self.terminating_condition = args[2]

        #self.fallback_points = args[3]
        # ----- Parameters -----
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_iterations', 2),
                ('max_cluster_iterations', 3),
            ]
        )

        # ----- Search Configuration -----
        self.search_angles = [args[0], args[1]]
        self.search_index = 0
        self.start_turn = False
        
        self.cluster_angles : np.array = np.array([])

        self.turn_interval = 3.0  # seconds
        self.last_turn_time = self.get_clock().now()
        self.angle_merge_threshold = 5 #degrees
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.max_iterations = self.get_parameter('max_iterations').value
        self.max_cluster_iterations = self.get_parameter('max_cluster_iterations').value #Max number of cluster iterations before stopping
        self.cur_cluster_iteration = 0
        self.cur_iteration = 0
        self.saved_pose : Pose = None 
        self.finished_turn = True

        # ROS Subscriptions
        self.semantic_map_sub = self.create_subscription(
            SemanticMap,
            '/pontus/semantic_map',
            self.semantic_map_callback,
            10
        )
        
        self.unlabled_candidate_tracks_sub = self.create_subscription(
            Float64MultiArray,
            '/pontus/unlabled_candidate_tracks',
            self.unlabled_tracks_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
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
    
    def odom_callback(self, msg: Odometry) -> None:
        """
        Saves the current position of the sub
        
        Return:
            N/A
        """
        
        if self.saved_pose is None:
            self.saved_pose = msg.pose.pose
            #self.get_logger().info(f"Saved Pose: {self.saved_pose}")
        
     
    def unlabled_tracks_callback(self, msg: Float64MultiArray) -> None:
        """
        Calculate the necessary rotations to face the cluster points
        
        args:
            msg [Float64MultiArray] : A msg with a flattened positions of the unrevealed clusters
        
        Returns:
            N/A
        """
        # Only take new cluster points if we have finished our current search
        if len(self.cluster_angles) == 0:
            self.get_logger().info(f"Current Cluster Iterations: {self.cur_cluster_iteration}")       
            if self.cur_cluster_iteration >= self.max_cluster_iterations:
                self.complete(False)
            cluster_positions = np.array(msg.data).reshape(-1, 3)
            for position in cluster_positions:
                transformed_cluster_pos = self.transform_cluster_positions(position)
                if transformed_cluster_pos is None:
                    continue
                
                cluster_angle = math.atan2(transformed_cluster_pos[1], transformed_cluster_pos[0]) #radians
                self.cluster_angles = np.append(self.cluster_angles, cluster_angle)
            self.cur_cluster_iteration += 1
            self.cluster_angles = np.sort(self.cluster_angles)
            
            index = 0
            while(index < len(self.cluster_angles) - 1):
                if abs(math.degrees(self.cluster_angles[index]) - math.degrees(self.cluster_angles[index + 1])) <= self.angle_merge_threshold:
                    self.cluster_angles = np.delete(self.cluster_angles, index + 1)
                else:
                    index += 1
                    
                

    def semantic_map_callback(self, msg: SemanticMap) -> None:
        """
        If a gate is detected, stop the search immediately.
        
        args:
            msg [SemanticMap] : A message of all objects on the semantic map
        
        Return:
            N/A
        """

        if self.terminating_condition is SearchConditions.GATE: 
            if msg.meta_gate.header.frame_id != "":
                self.get_logger().info("Gate pair detected in semantic map")
                self.complete(True)

        elif self.terminating_condition is SearchConditions.SLALOM:
            if msg.meta_slalom.header.frame_id != "":
                self.get_logger().info("Slalom Pair detected in semantic map")
                self.complete(True)

    def turn_callback(self) -> None:
        """
        Follows the given target angles 
        
        Prefers to follow cluster angles over the base turn angles
        
        Return:
            N/A
        """

        # Wait for previous motion to finish
        if (not self.start_turn or self.go_to_pose_client.at_pose()) and len(self.cluster_angles) > 0 and self.saved_pose is not None:
            now = self.get_clock().now()
            if not self.finished_turn:
                self.get_logger().info(f"Setting Time and Pose is {self.go_to_pose_client.at_pose()}")
                self.last_turn_time = now
                self.finished_turn = True
            dt = now - self.last_turn_time
            if dt < Duration(seconds=self.turn_interval):
                return 

            self.last_turn_time = now
            self.start_turn = True
            self.finished_turn = False
            
            if len(self.cluster_angles) > 0:
                target_angle = self.cluster_angles[0]
                self.cluster_angles = np.delete(self.cluster_angles, 0)
            
            self.get_logger().info(
                f"Cluster Unlabled Turning {'right' if target_angle > 0 else 'left'} "
                f"{math.degrees(abs(target_angle))} degrees"
            )
            self.turn_command(target_angle) 
        elif (not self.start_turn or self.go_to_pose_client.at_pose()) and self.saved_pose is not None: 
            # Enforce dwell time between turns
            now = self.get_clock().now()
            if not self.finished_turn:
                self.last_turn_time = now
                self.finished_turn = True
            dt = now - self.last_turn_time
            if dt < Duration(seconds=self.turn_interval):
                return
            
            if self.cur_iteration == self.max_iterations:
                self.complete(False)
                return
            

            self.last_turn_time = now
            self.start_turn = True
            self.finished_turn = False

            
            target_angle = self.search_angles[self.search_index] + self.convert_quaternion_to_euler(self.saved_pose.orientation)[1]

            self.get_logger().info(
                f"Base Turn Turning {'right' if target_angle > 0 else 'left'} "
                f"{math.degrees(abs(target_angle))} degrees"
                f"{target_angle} radians"
            )

            # Send turn command (ABSOLUTE)
            self.turn_command(target_angle)

            # Alternate index
            if (self.search_index + 1) >= len(self.search_angles):
                self.cur_iteration += 1
            self.search_index = (self.search_index + 1) % len(self.search_angles)
        
    def convert_quaternion_to_euler(self, quaternion) -> tuple[float, float, float]:
        """
        Converts a Quaternion object to tuple of euler angles
        
        arg:
            quaternion [Quaternion] : the quaternion object to be converted

        Return:
            tuple[float, float, float] : tuple of euler angles
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        euler = tf_transformations.euler_from_quaternion((x, y, z, w))
        return euler
        

    def turn_command(self, absolute_yaw: float) -> None:
        """
        Sends a ABSOLUTE rotation command.
        
        arg:
            absolute_yaw [float] : Absolute angle to turn to
        """

        cmd_pose = Pose()

        # Convert yaw to quaternion
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(
            0.0, 0.0, absolute_yaw
        )

        # No positional movement
        cmd_pose.position.x = self.saved_pose.position.x
        cmd_pose.position.y = self.saved_pose.position.y
        cmd_pose.position.z = self.saved_pose.position.z #May need to change in the future if constant depth is desired
        
        cmd_pose.orientation.x = qx
        cmd_pose.orientation.y = qy
        cmd_pose.orientation.z = qz
        cmd_pose.orientation.w = qw

        self.go_to_pose_client.go_to_pose(
            pose_obj=PoseObj(
                cmd_pose=cmd_pose,
            )
        )
    
    def transform_cluster_positions(self, position_map : np.ndarray) -> np.ndarray | None:
        """
        Transforms the cluster positions from map to base_link
        
        arg:
            position_map [np.ndarray] : array of x,y and z position
        
        Returns:
            np.ndarray | None : returns np.ndarray if successful transform, otherwise, None is returned
        """
        
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame="base_link",
                source_frame="map",
                time = rclpy.time.Time()
            )
            
            cluster_pose = Pose()
            
            cluster_pose.position.x = position_map[0]
            cluster_pose.position.y = position_map[1]
            cluster_pose.position.z = 0
            
            cluster_pose.orientation.x = 0
            cluster_pose.orientation.y = 0
            cluster_pose.orientation.z = 0
            cluster_pose.orientation.w = 1.0
            
            cluster_pose_transformed = tf2_geometry_msgs.do_transform_pose(
                cluster_pose, transform
            )
            
            return [cluster_pose_transformed.position.x, cluster_pose_transformed.position.y, cluster_pose_transformed.position.z]
        except:
            self.get_logger().warn("Transform to convert cluster position to base_link failed")
            return None        
