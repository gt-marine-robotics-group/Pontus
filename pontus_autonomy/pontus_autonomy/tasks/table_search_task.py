import numpy as np
import math
import tf_transformations
from image_geometry import PinholeCameraModel
import rclpy
import tf2_geometry_msgs

from pontus_autonomy.tasks.base_task import BaseTask
from rclpy.clock import Clock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose, Vector3Stamped, PoseStamped
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo
from tf2_ros.buffer import Buffer
from tf2_ros import TransformListener
from pontus_msgs.msg import SemanticMap, SemanticObject
from pontus_msgs.srv import AddSemanticObject

class TableSearchTask(BaseTask):
    def __init__(self, fallback_point : Pose = None, additional_rays: bool = False, target_object: str = "gate_shark"):
        super().__init__("table_search_task")
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
    
        #Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ("number_of_spins", 2),
                ('identification_threshold', 0.4),
                ("distance_thresh", 0.8),
                ("convergence_thresh", 0.01),
                ("min_rays_to_exit", 5),
                ("min_distance_exit", 0.2)
            ]
        )
        
        self.number_of_spins = self.get_parameter("number_of_spins").value
        self.id_threshold = self.get_parameter("identification_threshold").value
        self.distance_thresh = self.get_parameter("distance_thresh").value #The distance before getting another ray data
        self.convergance_thresh = self.get_parameter("convergence_thresh").value #Threshold of ray convergence
        self.min_rays_to_exit = self.get_parameter("min_rays_to_exit").value #Minimum rays to capture before exiting is possible
        self.min_distance_exit = self.get_parameter("min_distance_exit").value #min distance from current point before forfetting
        self.additional_rays = additional_rays
        self.target_object = target_object
        
        
        #Local Variables
        self.total_rad = self.number_of_spins * 2 *  math.pi
        self.ascent = False
        self.gather_data = False
        self.start_turn = False
        self.current_rad = 0
        self.yolo_detections : Detection2DArray = None
        self.camera_model = PinholeCameraModel()
        self.camera_initialized = False
        self.rays = []
        self.curr_waypoint : Pose = None
        self.fallback_point : Pose = fallback_point
        self.vectors = [np.array([0.0, 0.707, 0.707]), np.array([0.0, -0.707, -0.707])]
        self.prev_position : np.array = None
        self.finding_ray : bool = False
        self.convergance : bool = False
        
        self.name_map = {
            'gate_side': SemanticObject.GATE_LEFT,
            'red_slalom': SemanticObject.SLALOM_RED,
            'reef_shark': SemanticObject.GATE_IMAGE_SHARK,
            'saw_shark': SemanticObject.GATE_IMAGE_FISH,
            'white_slalom': SemanticObject.SLALOM_WHITE,
            'path_marker': SemanticObject.PATH_MARKER,
            'vertical pole': SemanticObject.VERTICAL_MARKER,

            # Name from the sim yolo model is different from real apparently
            'slalom_red': SemanticObject.SLALOM_RED,
            'slalom_white': SemanticObject.SLALOM_WHITE,
            'gate_shark': SemanticObject.GATE_IMAGE_SHARK,
            'gate_fish': SemanticObject.GATE_IMAGE_FISH,
            'left_gate': SemanticObject.GATE_LEFT,
            'right_gate': SemanticObject.GATE_LEFT,
            'vertical_marker' : SemanticObject.VERTICAL_MARKER
        }
        
        self.field_map = {
            'gate_side': 'gate_left',
            'red_slalom': 'slalom_red',
            'reef_shark': 'gate_image_shark',
            'saw_shark': 'gate_image_fish',
            'white_slalom': 'slalom_white',
            'path_marker': 'path_marker',
            'vertical pole': 'vertical_marker',

            # Name from the sim yolo model is different from real apparently
            'slalom_red': 'slalom_red',
            'slalom_white': 'slalom_white',
            'gate_shark': 'gate_image_shark',
            'gate_fish': 'gate_image_fish',
            'left_gate': 'gate_left',
            'right_gate':  'gate_left',
            'vertical_marker' : 'vertical_marker'
        }
        
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
        
        self.add_semantic_object_client = self.create_client(
            AddSemanticObject,
            '/pontus/add_semantic_object',
        )
        
        #Timers
        self.turn_timer = None
        self.data_timer = None
        self.position_timer = None
        self.delay_timer = None
        
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
        if not self.gather_data and self.camera_initialized:
            self.object_detection()
    
    def object_detection(self):
        """
        iterates through YOLO results for if table has been detected
        """
        #self.get_logger().info("Starting Object Detection")
        for detection in self.yolo_detections.detections:
            for result in detection.results:
                #self.get_logger().info(f"Class id: {result.hypothesis.class_id}, comparison: {SemanticObject.GATE_IMAGE_FISH}, score: {result.hypothesis.score}, threshold: {self.id_threshold}")
                if result.hypothesis.class_id == self.target_object and result.hypothesis.score >= self.id_threshold:
                    self.gather_data = True
                    self.turn_timer.destroy()
                    #self.data_timer = self.create_timer(0.3, self._gather_more_data, self.service_callback_group)
                    self.delay_timer = self.create_timer(1, self._create_timer, self.service_callback_group)
                    self.get_logger().info("Found Object")
    
    def _create_timer(self):
        """
        Exists just for debugging since often times the object is detected before the sub fully sinks. 
        
        For real scenarios, comment the delay_timer line and uncomment the data_timer above
        """
        self.data_timer = self.create_timer(0.3, self._gather_more_data, self.service_callback_group)
        self.delay_timer.destroy() 
        
    
    def _gather_more_data(self):
        """
        Moves the sub left and right to gather more diverse ray data
        """
        #self.get_logger().info("Starting Data Gathering Stage")
        if not self.additional_rays:
            if self.update_rays():
                self.begin_approach()
                return
        if self.curr_waypoint is None or self.go_to_pose_client.at_pose():
            if not self.update_rays():
                return
            if len(self.vectors) == 0:
                self.begin_approach()
                return
            curr_vector = self.vectors.pop(0)
            if self.curr_waypoint is not None:
                curr_vector = curr_vector * 2

            self.curr_waypoint = curr_vector
            self.move_command(curr_vector)
    
    def begin_approach(self):
        """
        Transitions from finding object to moving toward estimated points
        """
        self.data_timer.destroy()
        self.curr_waypoint = None
        self.calculate_obj_location()
        self.position_timer = self.create_timer(0.2, self.approach, self.service_callback_group)
        self.prev_position = self._pose_to_array(self._get_current_position())
        
    def approach(self):
        """
        Attempts to approach object and gather new ray data
        """
        if len(self.rays) <= 2:
            #For the beginning, we need data. If we immediatly start moving, we need more than 1 ray and will add two for quick access
            self.update_rays()
            self.calculate_obj_location()
        
        curr_position = self._get_current_position()
        if not curr_position is None:
            if self.finding_ray:
                self.finding_ray = self.update_rays()
                self.calculate_obj_location()
            else:
                distance = np.linalg.norm(self._pose_to_array(curr_position) - self.prev_position)
                if distance >= self.distance_thresh:
                    self.prev_position = self._pose_to_array(curr_position)
                    self.finding_ray = self.update_rays()
                    self.calculate_obj_location()
                    
        if len(self.rays) <= self.min_rays_to_exit:
            self._close_to_object()

    
    def _close_to_object(self):
        """
        Detects if the sub is close to the final object position. Ifthe difference is less than the threshold, we assume the point is wrong and exit.
        TODO: This threshold needs to be tuned as the range of the semantic map vs how many iterations we need to recalculate the approximate convergence is unknonw
        
        Commented code block used for simualtion object
        """
        
        if self.curr_waypoint is None:
            return
        
        curr_waypoint = self._pose_to_array(self.curr_waypoint)
        curr_pose = self._get_current_position()
        if curr_pose is None:
            return
        curr_position = self._pose_to_array(curr_pose)
        diff = np.linalg.norm(curr_waypoint - curr_position)
        if diff < self.min_distance_exit:
            self.complete(False)
        
        '''       
        if not self.ascent:
            curr_waypoint = self._pose_to_array(self.curr_waypoint)
            curr_pose = self._get_current_position()
            if curr_pose is None:
                return
            curr_position = self._pose_to_array(curr_pose)
            
            diff = np.linalg.norm(curr_waypoint - curr_position)
            if (diff < 1.5):
                self.move_command(np.array([0.0, 0.0, 1.0]))
                self.ascent = True
                self.get_logger().info("Starting Ascent")
        elif self.ascent and self.go_to_pose_client.at_pose():
            self.move_command(np.array([0.0, 0.0, -2.0]))
            self.position_timer.destroy()
        '''
        
    
                
    def semantic_map_callback(self, msg: SemanticMap):
        """
        Checks if semantic map has detected table. If it has, the task is complete
        
        Args:
            msg (SemanticMap): Semantic Map message
        """
        #TODO: Find object using the string
        field = self.field_map[self.target_object]
        object = getattr(msg, field)
        #self.get_logger().info(f"field: {field} type: {type(object)[0]} type: {type(msg.gate_image_fish[0].header.frame_id)}")
        if object[0].header.frame_id != "":
            self.get_logger().info("Object detected in semantic map")
            self.complete(True)
            
    
    def calculate_obj_location(self):
        """
        calculates approximate table location with rays and updates the current traveled position.
        """
        A = np.zeros((3, 3))
        B = np.zeros((3, 1))
        
        #self.get_logger().info("Calculating Object Location")
        
        for ray in self.rays:
            projection = np.identity(3) - np.matrix(ray[1]).T @ np.matrix(ray[1])
            A += projection
            B += projection @  self._pose_to_matrix(ray[0]).T
        
        estimated_position = np.linalg.lstsq(A, B)[0].T[0]
        #self.get_logger().info(f"{estimated_position}")
        if not self.curr_waypoint is None:
            #self.get_logger().info(f"self waypoint: {self.curr_waypoint} matrix: {self._pose_to_matrix(self.curr_waypoint)}")
            curr_waypoint = self._pose_to_array(self.curr_waypoint) #3x1 matrix 
        
            distance = np.linalg.norm(estimated_position - curr_waypoint)
            #self.get_logger().info(f"Distance: {distance}")
            cmd_pose = Pose()
            cmd_pose.position.x = estimated_position[0]
            cmd_pose.position.y = estimated_position[1]
            cmd_pose.position.z = estimated_position[2]
            self.curr_waypoint = cmd_pose
            if self.go_to_pose_client.at_pose():
                self._send_waypoint_command(estimated_position)
            if distance < self.convergance_thresh and not self.convergance:
                self.convergance = True
                #self.get_logger().info("Convergance Achieved")
                self._send_semantic_object()
        else:
            cmd_pose = Pose()
            cmd_pose.position.x = estimated_position[0]
            cmd_pose.position.y = estimated_position[1]
            cmd_pose.position.z = estimated_position[2]
            self.curr_waypoint = cmd_pose
            self._send_waypoint_command(estimated_position)

    
    def _pose_to_matrix(self, pose: Pose) -> np.matrix:
        """
        converts a give pose to a 1x3 np.matrix
        
        Args:
            pose (Pose): Pose of information
        
        Returns:
            np.matrix: A matrix of the pose position
        """
        return np.matrix([pose.position.x, pose.position.y, pose.position.z])
    
    def _pose_to_array(self, pose: Pose) -> np.ndarray:
        """
        Converts a given pose to np.array
        
        Args:
            pose (Pose): Pose containing position data
        
        Returns:
            np.array: A array of the position data
        """
        return np.array([pose.position.x, pose.position.y, pose.position.z])

    def _send_semantic_object(self) -> None:
        """
        Sends a semantic object based on the current waypoint
        """
        req = AddSemanticObject.Request()
        req.ids = [self.name_map[self.target_object]]
        
        pose_stamp = PoseStamped()
        pose_stamp.pose = self.curr_waypoint
        pose_stamp.header.frame_id = "world"
        pose_stamp.header.stamp = Clock().now().to_msg()
        
        req.positions = [pose_stamp]
        self.add_semantic_object_client.call_async(req)
        #self.get_logger().info("Sent object to semantic map")
    
    def _send_waypoint_command(self, target_pos: np.ndarray) -> None:
        """
        sends new command for sub to travel to new position based on absolute point.
        
        Args:
            target_pos (np.ndarray): position to travel to
        """
        #self.get_logger().info(f"Going to location: {target_pos}")
        cmd_pose = Pose()
        cmd_pose.position.x = target_pos[0]
        cmd_pose.position.y = target_pos[1]
        cmd_pose.position.z = target_pos[2] -0.44 #Adding a constant value
        #self.get_logger().info(f"Sending the location pose: {cmd_pose}")
    
        self.go_to_pose_client.go_to_pose(pose_obj=PoseObj(cmd_pose=cmd_pose, skip_orientation=True))
        
        
    def update_rays(self) -> bool:
        """
        adds a new ray if it exists.
        
        Returns:
            bool: whether a new ray has been added
        """
        new_ray = False
        for detection in self.yolo_detections.detections:
            for result in detection.results:
                #TODO: Replace with actual table enumeration
                if result.hypothesis.class_id == self.target_object and result.hypothesis.score >= self.id_threshold:
                    #self.get_logger().info(f"Rays Number: {len(self.rays)}")
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
                    new_ray = True
        return new_ray
    
    def transform_ray(self, ray : np.array) -> np.ndarray | None:
        """
        Transforms the ray from camera frame to map frame
        
        Args:
            ray (np.array) : the unit vector ray

        Returns:
            np.array | None : Returns the transformed ray as np.array or None if transform fails
        """
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
    
    def _get_current_position(self) -> Pose | None:
        """
        Returns the current position of the sub
        
        Returns:
            Pose | None : Pose of the current position or None if transform fails
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame="map",
                source_frame="base_link",
                time = rclpy.time.Time()
            )
            
            current_pose = Pose()
            current_pose.position.x = 0
            current_pose.position.y = 0
            current_pose.position.z = 0
            
            current_pose.orientation.x = 0
            current_pose.orientation.y = 0
            current_pose.orientation.z = 0
            current_pose.orientation.w = 1.0
            
            pose_transformed = tf2_geometry_msgs.do_transform_pose(
                current_pose, transform
            )
            
            return pose_transformed
            
        except:
            self.get_logger().warn("Transform to get current position failed")
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
                if self.fallback_point is None:
                    self.complete(False)
                self._send_waypoint_command(self.fallback_point)
                self.begin_approach()
            else:
                self.turn_command(self.current_rad)
                self.total_rad -= (0.25 * 2 * math.pi)
                
    def move_command(self, movement_vector: np.ndarray) -> None:
        """
        Issues a movement command in the movement vector direction with relative movement
        """
        #self.get_logger().info(f"Movement Vector: {movement_vector}")
    
        
        current_pose = Pose()
        current_pose.position.x = movement_vector[0]
        current_pose.position.y = movement_vector[1]
        current_pose.position.z = movement_vector[2] - 0.44
        
        current_pose.orientation.x = 0.0
        current_pose.orientation.y = 0.0
        current_pose.orientation.z = 0.0
        current_pose.orientation.w = 1.0
        
        #self.get_logger().info(f"Transformed Pose: {current_pose.position}")
        #self.get_logger().info(f"Transformed Rotation: {current_pose.orientation}")
        
        self.go_to_pose_client.go_to_pose(pose_obj=PoseObj(cmd_pose=current_pose, use_relative_position=True, skip_orientation=False))
        
    
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

        cmd_pose.position.x = 0.0
        cmd_pose.position.y = 0.0
        cmd_pose.position.z = -0.44
        
        cmd_pose.orientation.x = qx
        cmd_pose.orientation.y = qy
        cmd_pose.orientation.z = qz
        cmd_pose.orientation.w = qw
        
        self.go_to_pose_client.go_to_pose(pose_obj=PoseObj(cmd_pose=cmd_pose, use_relative_position=True, skip_orientation=False))