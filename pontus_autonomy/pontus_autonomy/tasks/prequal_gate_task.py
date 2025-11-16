import numpy as np
from enum import Enum 
from dataclasses import dataclass

import rclpy
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry

from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
import tf2_geometry_msgs

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj

from pontus_msgs.msg import SemanticMap, SemanticObject
from pontus_msgs.srv import AddMetaGate

class GateSide(Enum):
    RIGHT = 0
    LEFT  = 1
    
@dataclass
class GatePair:
    """
    Holds information about gate objects selected to be a gate pair
    """
    left_gate  : SemanticObject
    right_gate : SemanticObject

class PrequalGateTask(BaseTask):
    
    def __init__(self):
        super().__init__("prequal_gate_task")
        
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        
        # ------ Parameters ------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('height_from_bottom', 0.5),
                ('gate_side', 0),  # Gate side right
                ('waypoint_dist_from_gate', 0.6),  # How far should the apparoach and pass through points be to the gate
                ('gate_width', 3.048),  # Distance between gate poles according to the team handbook
                ('gate_width_tolerance', 0.3),  # Tolerance of how close the estimated gate_width is to expected to be accepted
                ('follow_path_period', 0.25),
                ('pool_depth', 2.0)
            ]
        )

        self.height_from_bottom_m      : float    = float(self.get_parameter('height_from_bottom').value)
        self.gate_side                 : GateSide = GateSide(self.get_parameter('gate_side').value)
        self.waypoint_dist_from_gate_m : float    = float(self.get_parameter('waypoint_dist_from_gate').value)
        self.gate_width_m              : float    = float(self.get_parameter('gate_width').value)
        self.gate_width_tolerance_m    : float    = float(self.get_parameter('gate_width_tolerance').value)
        self.follow_path_period        : float    = float(self.get_parameter('follow_path_period').value)
        self.pool_depth                : float    = float(self.get_parameter('pool_depth').value)

        
        # ------ Variables ------
        self.waypoints_are_created : bool = False
        self.path : list[np.ndarray] = []

        self.curr_waypoint: Pose = None 

        self.execute_path : bool = False
        
        self.detected_gate_pair : GatePair = None

        self.latest_odom = None

        # ------ ROS Subscriptions ------
        self.semantic_map_sub = self.create_subscription(
            SemanticMap,
            '/pontus/semantic_map',
            self.semantic_map_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
            10
        )

        # ------ ROS Action / Service Managers ------
        self.go_to_pose_client = GoToPoseClient(self)

        self.add_meta_gate_client = self.create_client(
            AddMetaGate,
            '/pontus/add_meta_gate'
        )

        # ------ TF Transforms ------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------ Timers ------
        self.follow_path_timer = self.create_timer(
            self.follow_path_period,
            self.follow_path,
            self.service_callback_group
        )

    def odom_callback(self, msg: Odometry) -> None:
        """
        Keep track of current position of the robot
        """

        self.latest_odom = msg

    def semantic_map_callback(self, msg: SemanticMap) -> None:
        """
        When the semantic map is updated check for the conditions to show
        the Gate has been detected.
        
        If it has been detected calculate the waypoints to travel through
        the gate.
        
        Args:
            msg [SemanticMap] : contains semantic information about position
                                and labels of relavant obstacles in the course 
        
        Return:
            N/A
        """

        if self.waypoints_are_created:
            return

        self.detected_gate_pair = self.detect_gate_pair(msg)

        if self.detected_gate_pair is not None:
            path = self.generate_waypoints(self.detected_gate_pair)

            if path is not None:
                self.waypoints_are_created = True
                self.path = path
                self.execute_path = True

                self._send_meta_gate(self.detected_gate_pair)
                

    def detect_gate_pair(self, sem_map : SemanticMap) -> GatePair | None:
        """
        Given the semantic map. Check all gate side detections to see if
        any pair meet the conditions to be a gate pair
        
        - Distance between the two gate sides is very close to the known 
          expected value as given in the RoboSub Team Handbook

        Once the candidate pair is found we convert the pose to the body frame
        to determine which one is the "left" and "right" gates (relative
        to our robots current position)
            - We need to know this for selecting the correct gate side

        Args:
            sem_map [SemanticMap] : Map containing all detected semantic objects
        
        Return:
            [GatePair] : Set of gates we are confident pair together or None if 
                         no valid pairs found 
        """

        gate_list = sem_map.gate_left

        if len(gate_list) < 2:
            return None

        candidate_pair : list[SemanticObject] = None

        for i in range(len(gate_list) - 1):
            for j in range(i+1, len(gate_list)):
                gate_i_vec : np.ndarray = self._pose_to_nparray(gate_list[i].pose.pose)
                gate_j_vec : np.ndarray = self._pose_to_nparray(gate_list[j].pose.pose)

                gate_width_est : float = float(np.linalg.norm(gate_i_vec - gate_j_vec))

                if abs(gate_width_est - self.gate_width_m) <= self.gate_width_tolerance_m:
                    candidate_pair = [gate_list[i], gate_list[j]]

        if candidate_pair is not None:
            # Now we need to set which one is left and which one is right
            body_frame_gate : list[Pose] = []
            for side in candidate_pair:
                body_frame_side_pose = self._transform_sem_obj_to_body(side)
                body_frame_gate.append(body_frame_side_pose)

            if body_frame_gate[0].position.y > body_frame_gate[1].position.y:
                gate_pair = GatePair(
                    left_gate=candidate_pair[0], 
                    right_gate=candidate_pair[1]
                )
            else:
                gate_pair = GatePair(
                    left_gate=candidate_pair[1], 
                    right_gate=candidate_pair[0]
                )
            
            return gate_pair

        return None

    def generate_waypoints(self, gate_pair : GatePair) -> list[np.ndarray]:
        """
        Once we have a GatePair we can generate the waypoints for the sub
        to pass through with some basic linalg.

        We set the point where we want the sub to pass through the gate.
        This is the midpoint of the gate midpoint and one of the gate sides.
            - Which gate side is decided before the run
            
        We then find the perpendicular unit vector to generate two waypoints
        and return these as the path for the robot to follow.
        
        Args:
            gate_pair [GatePair] : detected pair from the semantic map
            
        Return:
            list[np.ndarray] : Path for the robot to follow as 2D np.array 
                               vectors
        """

        g1 : np.ndarray = self._pose_to_nparray(gate_pair.left_gate.pose.pose)
        g2 : np.ndarray = self._pose_to_nparray(gate_pair.right_gate.pose.pose)

        # gate passthrough is the point the sub actually goes through the gate
        if self.gate_side == GateSide.LEFT:
            # This is the midpoint between the midpoint and on of the gate sides
            passthrough_point =  (3*g1 + g2) / 4.0
        else:
            passthrough_point =  (3*g2 + g1) / 4.0

        # Gate vector from left side to right side
        gate_vec = g2 - g1

        # Perpendicular Vector
        perp_vec = np.array([-gate_vec[1], gate_vec[0]])
        perp_unit_vec = perp_vec / np.linalg.norm(perp_vec)


        waypoint_delta = self.waypoint_dist_from_gate_m * perp_unit_vec

        waypoint_1 = passthrough_point + waypoint_delta
        waypoint_2 = passthrough_point - waypoint_delta

        robot_xy = self._pose_to_nparray(self.latest_odom.pose.pose)

        wp1_dist = np.linalg.norm(waypoint_1 - robot_xy)
        wp2_dist = np.linalg.norm(waypoint_2 - robot_xy)

        if wp1_dist < wp2_dist:
            return [waypoint_1, waypoint_2]
        else:
            return [waypoint_2, waypoint_1]

    def follow_path(self) -> None:
        """
        After we generate the waypoints we switch to execute mode and follow
        the path we have created.
        
        Once we reach the end of this path we exit the task.
        """

        if not self.execute_path:
            return

        if self.curr_waypoint is None or self.go_to_pose_client.at_pose():
            if not self.path:
                self.complete(True)
                return

            target_pos_xy = self.path.pop(0)
            self._send_waypoint_command(target_pos_xy)

    
    def _send_waypoint_command(self, target_pos_xy: np.ndarray) -> None:
        """
        Convert a np.ndarray 2D vector to a command pose and send to pos_controller
        """
        cmd_pose = Pose()
        
        cmd_pose.position.x = target_pos_xy[0]
        cmd_pose.position.y = target_pos_xy[1]
        cmd_pose.position.z = -self.pool_depth + self.height_from_bottom_m

        self.curr_waypoint = cmd_pose

        self.go_to_pose_client.go_to_pose(PoseObj(cmd_pose=cmd_pose,
                                                  skip_orientation=True))

    def _send_meta_gate(self, gate_pair: GatePair) -> None:
        """
        Call ROS client to add the GatePair as a meta object to the semantic
        map
        """

        if not self.add_meta_gate_client.service_is_ready():
            self.get_logger().warn(
                "AddMetaGate service not available"
            )
            return

        req = AddMetaGate.Request()
        req.left_gate = gate_pair.left_gate
        req.right_gate = gate_pair.right_gate

        self.add_meta_gate_client.call_async(req)

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
        
    def _transform_sem_obj_to_body(self, obj : SemanticObject) -> Pose:
        """
        Transform a Pose to the body_frame of the robot
        
        Args:
            msg [Pose] : Pose we want to transform to body_frame
            
        Return:
            [Pose] : Input pose in body frame
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = obj.header
        pose_stamped.pose = obj.pose.pose

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='body_frame',
                source_frame=pose_stamped.header.frame_id,
                time=Time(seconds=pose_stamped.header.stamp.sec,
                          nanoseconds=pose_stamped.header.stamp.nanosec)
            )

            pose_transformed_stamped = tf2_geometry_msgs.do_transform_pose(
                pose_stamped,
                transform
            )

            return pose_transformed_stamped.pose

        except Exception as e:
            self.get_logger().warn(
                f"failure to transform pose to body frame, current frame: {obj.header.frame_id}")
            self.get_logger().warn(f"exception: {e}")
            return pose_stamped.pose
