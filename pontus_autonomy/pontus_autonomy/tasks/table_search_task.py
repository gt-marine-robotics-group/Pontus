import numpy as np
import math
import tf_transformations
from image_geometry import PinholeCameraModel
import rclpy
import tf2_geometry_msgs

from pontus_autonomy.tasks.base_task import BaseTask
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo
from tf2_ros.buffer import Buffer
from tf2_ros import TransformListener
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from pontus_msgs.msg import SemanticMap, SemanticObject

class TableSearchTask(BaseTask):
    def __init__(self, fallback_point : Pose = None):
        super().__init__("table_search_task")
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
    
        #Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ("number_of_spins", 2),
                ("pool_depth", 2.0),
                ('height_from_bottom', 0.5),
                ('identification_threshold', 0.4),
                ('pose_threshold', 5),
                ("ray_update_timer", 0.4),
                ("pos_update_timer", 2)
            ]
        )
        
        self.number_of_spins = self.get_parameter("number_of_spins").value
        self.pool_depth = self.get_parameter("pool_depth").value
        self.height_from_bottom = self.get_parameter("height_from_bottom").value
        self.id_threshold = self.get_parameter("identification_threshold").value
        self.pose_threshold = self.get_parameter("pose_threshold").value
        self.ray_update_timer = self.get_parameter("ray_update_timer").value
        self.pos_update_timer = self.get_parameter("pos_update_timer").value
        
        #Local Variables
        self.total_rad = self.number_of_spins * 2 *  math.pi
        self.start_turn = False
        self.current_rad = 0
        self.yolo_detections : Detection2DArray = None
        self.camera_model = PinholeCameraModel()
        self.camera_initialized = False
        self.approaching_object = False
        self.rays = []
        self.curr_waypoint : Pose = None
        self.fallback_point : Pose = fallback_point
        self.vectors = [np.array([0.0, 1.0, 0.0]), np.array([0.0, -2.0, 0.0])]
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.go_to_pose_client = GoToPoseClient(self)
        
        #Subscriptions
        self.yolo_detections_sub = self.create_subscription(
            Detection2DArray,
            'pontus/camera_front/yolo_results',
            self.yolo_callback,
            10
        )
        
        self.camera_info = self.create_subscription(
            CameraInfo,
            '/pontus/camera_front/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.semantic_map_sub = self.create_subscription(
            SemanticMap,
            '/pontus/semantic_map',
            self.semantic_map_callback,
            10
        )
        
        #Timers
        self.turn_timer = None
        self.position_timer = None
        self.ray_timer = None
        self.data_timer = None
        
        self.get_logger().info("Finished Setting Up")

    
    def camera_info_callback(self, msg: CameraInfo) -> None:
        """
        Initializes PinholeCameraModel with camera information
        
        Args:
            msg (CameraInfo): message with camera information
        """
        if not self.camera_initialized:
            self.camera_model.fromCameraInfo(msg)
            self.camera_initialized = True
            self.get_logger().info("Set up camera with information")
            self.turn_timer = self.create_timer(2, self.turn_callback, self.service_callback_group)
    
    def yolo_callback(self, msg: Detection2DArray) -> None:
        """
        Saves most recent YOLO detection and checks if table has been seen
        
        Args:
            msg (Detection2DArray): message with YOLO results
        """
        self.yolo_detections = msg
        if not self.approaching_object and self.camera_initialized:
            self.object_detection()
    
    def object_detection(self):
        """
        iterates through YOLO results for if table has been detected
        """
        #For now, using 4 as a stand-in for slalom red. TODO: Change this to a actual constant.
        #self.get_logger().info("Starting Object Detection")
        for detection in self.yolo_detections.detections:
            for result in detection.results:
                #self.get_logger().info(f"Class id: {result.hypothesis.class_id}, comparison: {SemanticObject.GATE_IMAGE_FISH}, score: {result.hypothesis.score}, threshold: {self.id_threshold}")
                if result.hypothesis.class_id == "vertical_marker" and result.hypothesis.score >= self.id_threshold:
                    #self._begin_approach()
                    self.approaching_object = True
                    self.turn_timer.destroy()
                    self.data_timer = self.create_timer(5, self._gather_more_data, self.service_callback_group)
                    self.get_logger().info("Found Object")
    
    def _gather_more_data(self):
        """
        Moves the sub left adn right to gather more diverse ray data
        """
        #self.get_logger().info("Starting Data Gathering Stage")

        if self.curr_waypoint is None or self.go_to_pose_client.at_pose():
            self.update_rays()
            if len(self.vectors) == 0:
                self._begin_approach()
                self.data_timer.destroy()
                self.curr_waypoint = None
                return
            curr_vector = self.vectors.pop(0)
            curr_vector = curr_vector * 1

            self.curr_waypoint = curr_vector
            self.move_command(curr_vector)
    
    def _begin_approach(self):
        """
        Begins approach stage and ends the turn stage
        """
        self.get_logger().info("Begun Approach")
        self.approaching_object = True
        self.position_timer = self.create_timer(self.pos_update_timer, self.calculate_obj_location, self.service_callback_group)
        self.ray_timer = self.create_timer(self.ray_update_timer, self.update_rays, self.service_callback_group)
        self.update_rays()
                
    def semantic_map_callback(self, msg: SemanticMap):
        """
        Checks if semantic map has detected table. If it has, the task is complete
        
        Args:
            msg (SemanticMap): Semantic Map message
        """
        #TODO: Change to value of Table object
        #if msg.gate_image_fish.header.frame_id != "":
        #    self.get_logger().info("Object detected")
        #    self.complete(True)
            
    
    def calculate_obj_location(self):
        """
        calculates approximate table location with rays and updates the current traveled position.
        """
        A = np.zeros((3, 3))
        B = np.zeros((3, 1))
        
        self.get_logger().info("Calculating Object Location")
        
        for ray in self.rays:
            projection = np.identity(3) - np.matrix(ray[1]).T @ np.matrix(ray[1])
            A += projection
            B += projection @  self._pose_to_matrix(ray[0]).T
        
        estimated_position = np.linalg.lstsq(A, B)[0].T[0]
        self.get_logger().info(f"{estimated_position}")
        if not self.curr_waypoint is None:
            #self.get_logger().info(f"self waypoint: {self.curr_waypoint} matrix: {self._pose_to_matrix(self.curr_waypoint)}")
            curr_waypoint = self._pose_to_array(self.curr_waypoint) #3x1 matrix 
            self.get_logger().info(f"curr_waypoint: {curr_waypoint}")
        
            estimated_position[2] = curr_waypoint[2]
        
            distance = np.linalg.norm(estimated_position - curr_waypoint)
            if distance >= self.pose_threshold or self.go_to_pose_client.at_pose():
                self._send_waypoint_command(estimated_position)
        else:
            self._send_waypoint_command(estimated_position)

    
    def _pose_to_matrix(self, pose: Pose) -> np.matrix:
        """
        converts a give pose to a 1x3 np.matrix
        
        Args:
            pose (Pose): message with camera information
        
        Returns:
            np.matrix: A matrix of the pose position
        """
        return np.matrix([pose.position.x, pose.position.y, pose.position.z])
    
    def _pose_to_array(self, pose: Pose) -> np.array:
        return np.array([pose.position.x, pose.position.y, pose.position.z])

    
    def _send_waypoint_command(self, target_pos: np.ndarray) -> None:
        """
        sends new command for sub to travel to new position
        
        Args:
            target_pos (np.ndarray): position to travel to
        """
        self.get_logger().info(f"Going to location: {target_pos}")
        cmd_pose = Pose()
        cmd_pose.position.x = target_pos[0]
        cmd_pose.position.y = target_pos[1]
        cmd_pose.position.z = -self.pool_depth + self.height_from_bottom
        self.get_logger().info(f"Sending the location pose: {cmd_pose}")
    
        self.go_to_pose_client.go_to_pose(pose_obj=PoseObj(cmd_pose=cmd_pose, skip_orientation=True))
        self.curr_waypoint = cmd_pose
        
        
    def update_rays(self):
        """
        adds a new ray if it exists.
        """
        for detection in self.yolo_detections.detections:
            for result in detection.results:
                #TODO: Replace with actual table enumeration
                if result.hypothesis.class_id == "vertical_marker" and result.hypothesis.score >= self.id_threshold:
                    center_position = detection.bbox.center.position
                    point = np.array([[center_position.x, center_position.y]])
                    point = np.expand_dims(point, 1)
                    rectified_point = self.camera_model.rectify_point(point)
                    ray_unit_vector = self.camera_model.project_pixel_to_3d_ray(rectified_point)
                    
                    camera_pos = self.transform_camera()
                    if camera_pos is None:
                        self.get_logger().warn("Camera Transform Returned None")
                        continue
                    
                    
                    #self.get_logger().info(f"Before Ray Transform: {ray_unit_vector}")
                    ray_unit_vector = self.transform_ray(ray_unit_vector)
                    #self.get_logger().info(f"After Ray Transform {ray_unit_vector}")
                    
                    self.rays.append([camera_pos, ray_unit_vector])
                    
    
    def transform_ray(self, ray) -> Pose | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame = "map",
                source_frame = "camera_front_optical_frame",
                time = rclpy.time.Time()
                )
            
            vec_stamped = Vector3Stamped()
            vec_stamped.header.stamp = rclpy.time.Time()
            vec_stamped.header.frame_id = "camera_front_optical_frame"
            
            vec_stamped.vector.x = ray[0]
            vec_stamped.vector.y = ray[1]
            vec_stamped.vector.z = ray[2]
            
            pose_transformed = tf2_geometry_msgs.do_transform_vector3(vec_stamped, transform)
            
            return np.array([pose_transformed.vector.x, pose_transformed.vector.y, pose_transformed.vector.z])
            
        except:
            self.get_logger().warn("Transform Ray to World Pose failed")
            return None
        
    
    def transform_camera(self) -> Pose | None:
        """
        transforms the camera position to the world position
        
        Returns:
            (Pose None) : If a transform exists, this returns the Pose. Otherwise, it returns None
        """
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame = "map",
                source_frame = "camera_front_optical_frame",
                time = rclpy.time.Time()
                )

            camera_pose = Pose()
            camera_pose.position.x = 0
            camera_pose.position.y = 0
            camera_pose.position.z = 0
            
            camera_pose.orientation.x = 0
            camera_pose.orientation.y = 0
            camera_pose.orientation.z = 0
            camera_pose.orientation.w = 1.0
            
            pose_transformed = tf2_geometry_msgs.do_transform_pose(
                camera_pose, transform) 
            
            return pose_transformed
            
        except:
            self.get_logger().warn("Camera Transform failed")
            return None
        
        
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
                self.get_logger().info("We've completed all rotations and found no table")
                self._send_waypoint_command(self.fallback_point)
                self._begin_approach()
                #TODO: Include backup point logic and movement
            else:
                self.turn_command(self.current_rad)
                self.total_rad -= (0.25 * 2 * math.pi)
                
    def move_command(self, movement_vector: np.ndarray) -> None:
        """
        Issues a movement command in the movement vector direction
        """
        self.get_logger().info(f"Movement Vector: {movement_vector}")
    
        
        current_pose = Pose()
        current_pose.position.x = movement_vector[0]
        current_pose.position.y = movement_vector[1]
        current_pose.position.z = movement_vector[2]
        
        current_pose.orientation.x = 0.0
        current_pose.orientation.y = 0.0
        current_pose.orientation.z = 0.0
        current_pose.orientation.w = 1.0
        
        self.get_logger().info(f"Transformed Pose: {current_pose.position}")
        self.get_logger().info(f"Transformed Rotation: {current_pose.orientation}")
        
        self.go_to_pose_client.go_to_pose(pose_obj=PoseObj(cmd_pose=current_pose, use_relative_position=True, skip_orientation=True))
        
    
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

        # HTODO: ow can we provide the current position so that it doens't change?
        cmd_pose.position.x = 0.0
        cmd_pose.position.y = 0.0
        cmd_pose.position.z = -self.pool_depth + self.height_from_bottom
        
        cmd_pose.orientation.x = qx
        cmd_pose.orientation.y = qy
        cmd_pose.orientation.z = qz
        cmd_pose.orientation.w = qw
        
        self.go_to_pose_client.go_to_pose(pose_obj=PoseObj(cmd_pose=cmd_pose, skip_orientation=False))