import rclpy
import math
import tf_transformations
from enum import Enum
import numpy as np

from geometry_msgs.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Float64MultiArray

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj

from pontus_msgs.msg import SemanticMap
from pontus_msgs.srv import AddSemanticObject

from pontus_mapping.cluster_coord import CandidateTrack

from tf2_ros.buffer import Buffer
from tf2_ros import TransformListener
import tf2_geometry_msgs

class SearchConditions(Enum):
    GATE = 0
    SLALOM = 1
    TARGET = 2
    BIN = 3
    OCTAGON = 4


class ScanTask(BaseTask):

    def __init__(
        self, 
        args):
        super().__init__("prequal_search_gate_task")

        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        self.terminating_condition = args[2]

        self.fallback_points = args[3]
        self.get_logger().info(f"Terminating Condition: {self.terminating_condition}")
        # ----- Parameters -----
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_iterations', 2),
                ('max_cluster_iterations', 3)
            ]
        )

        # ----- Search Configuration -----
        self.search_angles = [args[0], args[1]]
        self.search_index = 0
        self.start_turn = False
        self.reset_time = False
        
        self.start_track_turn = False
        self.last_track_turn = self.get_clock().now()
        
        self.track_angles : np.array = np.array([])

        self.turn_interval = 10.0  # seconds
        self.last_turn_time = self.get_clock().now()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.max_iterations = self.get_parameter('max_iterations').value
        self.max_cluster_iterations = self.get_parameter('max_cluster_iterations').value #Max number of cluster iterations before stopping
        self.cur_cluster_iteration = 0
        self.cur_iteration = 0

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
        
    def unlabled_tracks_callback(self, msg: Float64MultiArray) -> None:
        """
        Calculate the necessary rotations to face the cluster points
        """
        # Unsure of how to handle getting new cluster points while searching for cluster points. 
        if len(self.track_angles) == 0:
            if self.cur_cluster_iteration >= self.max_cluster_iterations:
                self.complete(False)
            track_positions = np.array(msg.data).reshape(-1, 3)
            for position in track_positions:
                transformed_track_pos = self.transform_track_positions(position)
                if transformed_track_pos is None:
                    continue
                
                track_angle = math.atan2(transformed_track_pos[1], transformed_track_pos[0])
                self.track_angles = np.append(self.track_angles, track_angle)
                self.start_track_turn = False
            self.cur_cluster_iteration += 1
            self.get_logger().info(f"Current Cluster Iterations: {self.cur_cluster_iteration}")       
            self.track_angles = np.sort(self.track_angles)
    
    #TODO: There might be instances where track_turn_callback completes, but turn goes first and steals the next movement.
    # def track_turn_callback(self) -> None:
    #     if not self.start_track_turn or self.go_to_pose_client.at_pose():
    #         now = self.get_clock().now()
    #         dt = (now - self.last_track_turn).nanoseconds * 1e-9
    #         if dt  < self.track_interval:
    #             return 

    #         self.last_track_turn = now
    #         self.start_track_turn = True
    #         self.start_turn = False
    #         self.get_logger().info(f"Angles: {self.track_angles}")
            
    #         if len(self.track_angles) > 0:
    #             target_angle = self.track_angles[0]
    #             self.track_angles = np.delete(self.track_angles, 0)
    #         else:
    #             self.get_logger().info("No more unlabled clusters found")
    #             self.track_timer.destroy()
    #             self.start_turn = False
            
    #         self.get_logger().info(
    #             f"Cluster Unlabled Turning {'right' if target_angle > 0 else 'left'} "
    #             f"{math.degrees(abs(target_angle))} degrees"
    #         )
    #         self.turn_command(target_angle)
            
    #         #TODO: Change the target radians from the original turn as well.
    #         self.track_angles -= target_angle    

    def semantic_map_callback(self, msg: SemanticMap) -> None:
        """
        If a gate is detected, stop the search immediately.
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
        """

        # Wait for previous motion to finish
        if (not self.start_track_turn or self.go_to_pose_client.at_pose()) and len(self.track_angles) > 0:
            now = self.get_clock().now()
            dt = (now - self.last_track_turn).nanoseconds * 1e-9
            if dt  < self.turn_interval:
                return 

            self.last_track_turn = now
            self.start_track_turn = True
            self.get_logger().info(f"Angles: {self.track_angles}")
            
            if len(self.track_angles) > 0:
                target_angle = self.track_angles[0]
                self.track_angles = np.delete(self.track_angles, 0)
            
            self.get_logger().info(
                f"Cluster Unlabled Turning {'right' if target_angle > 0 else 'left'} "
                f"{math.degrees(abs(target_angle))} degrees"
            )
            self.turn_command(target_angle) 
        elif (not self.start_turn or self.go_to_pose_client.at_pose()):
            # Enforce dwell time between turns
            now = self.get_clock().now()
            if self.go_to_pose_client.at_pose():
                self.last_turn_time = now
            dt = (now - self.last_turn_time).nanoseconds * 1e-9
            if dt < self.turn_interval:
                return
            
            if self.cur_iteration == self.max_iterations:
                self.complete(False)
            

            self.last_turn_time = now
            self.start_turn = True

            target_angle = self.search_angles[self.search_index]

            self.get_logger().info(
                f"Base Turn Turning {'right' if target_angle > 0 else 'left'} "
                f"{math.degrees(abs(target_angle))} degrees"
            )

            # Send turn command (RELATIVE)
            self.turn_command(target_angle)

            # Alternate index
            if (self.search_index + 1) >= len(self.search_angles):
                self.cur_iteration += 1
            self.search_index = (self.search_index + 1) % len(self.search_angles)
            

    def _send_fallback_semantic_objects(self) -> None:
        """
        Sends fallback semantic objects if needed.
        """
        if not self.add_semantic_object_client.service_is_ready():
            self.get_logger().warn("AddSemanticObject service not available")
            return

        req = AddSemanticObject.Request()
        req.ids = [self.fallback_points[0][0], self.fallback_points[1][0]]
        req.positions = [self.fallback_points[0][1], self.fallback_points[1][1]]

        self.add_semantic_object_client.call_async(req)

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
        cmd_pose.position.z = 0.0

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
    
    def transform_track_positions(self, position_map : np.ndarray) -> np.ndarray | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame="base_link",
                source_frame="map",
                time = rclpy.time.Time()
            )
            
            track_pose = Pose()
            
            track_pose.position.x = position_map[0]
            track_pose.position.y = position_map[1]
            track_pose.position.z = 0
            
            track_pose.orientation.x = 0
            track_pose.orientation.y = 0
            track_pose.orientation.z = 0
            track_pose.orientation.w = 1.0
            
            track_pose_transformed = tf2_geometry_msgs.do_transform_pose(
                track_pose, transform
            )
            
            return [track_pose_transformed.position.x, track_pose_transformed.position.y, track_pose_transformed.position.z]
        except:
            self.get_logger().warn("Transform to convert track position to base_link failed")
            return None        
