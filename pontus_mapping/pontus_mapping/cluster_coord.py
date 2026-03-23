# from pontus_msgs.srv import AddSemanticObject
from dataclasses import dataclass
import geometry_msgs
from pontus_msgs.msg import SemanticObject, SemanticMap
from pontus_msgs.srv import AddSemanticObject
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3Stamped, Vector3
from sensor_msgs.msg import PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_geometry_msgs

import numpy as np
from image_geometry import PinholeCameraModel
from math import acos, pi

from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2 as pc2

import tf_transformations
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2D, Detection2DArray

from pontus_mapping.semantic_map_manager import SemanticMapDC

@dataclass
class SlalomRowCandidate:
    slalom_red: SemanticObject
    slalom_white: SemanticObject

class ImageCoordinator(Node):
    def __init__(self) -> None:
        super().__init__('image_to_cluster_coordinator')

        # TODO: This is copied and pasted from sm manager, grab the parameters from there or use common parameters in a config file here
        self.declare_parameters(
            namespace='',
            parameters=[
                ("slalom_white_to_white_width", 3.0), 
                ("slalom_white_to_red_width", 1.5), 
                ("slalom_width_tolerance", 0.3), # slalom pairing tolerance
                ("slalom_row_tolerance", 0.3) # red slalom row deviation tolerance
            ]
        )
        self.slalom_white_to_white_width_m = float(self.get_parameter('slalom_white_to_white_width').value)
        self.slalom_white_to_red_width_m = float(self.get_parameter('slalom_white_to_red_width').value)
        self.slalom_width_tolerance_m = float(self.get_parameter("slalom_width_tolerance").value)
        self.slalom_row_tolerance_m = float(self.get_parameter("slalom_row_tolerance").value)

        self.latest_pointcloud: np.array = None
        # (m), width of line pointing toward object detected by yolo
        self.line_projection_width = .1
        # object score must be at least this to be added to semantic map
        self.confidence_min = 0.5
        self.vector_projection_dist = 10  # (m), how far the line is projected
        self.cam_model = PinholeCameraModel()
        self.camera_frame_name = 'camera_front'
        self.cam_initialized = False

        self.min_bbox_width = 0
        self.max_cluster_dist_m = 8.0

        # self.red_list = None
        self.white_list = []

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
            'vertical_marker': SemanticObject.VERTICAL_MARKER,

            # Names from simulator bounding box cameras have to be integers so we also map the
            # label numbers
            str(SemanticObject.SLALOM_RED): SemanticObject.SLALOM_RED,
            str(SemanticObject.SLALOM_WHITE): SemanticObject.SLALOM_WHITE,
            str(SemanticObject.GATE_IMAGE_SHARK): SemanticObject.GATE_IMAGE_SHARK,
            str(SemanticObject.GATE_IMAGE_FISH): SemanticObject.GATE_IMAGE_FISH,
            str(SemanticObject.GATE_LEFT): SemanticObject.GATE_LEFT,
            # we currently don't distinguish between gate sides
            str(SemanticObject.GATE_RIGHT): SemanticObject.GATE_LEFT,
            str(SemanticObject.VERTICAL_MARKER): SemanticObject.VERTICAL_MARKER,
            str(SemanticObject.BIN): SemanticObject.BIN,
            str(SemanticObject.OCTAGON): SemanticObject.OCTAGON,
            str(SemanticObject.TARGET): SemanticObject.TARGET,
            str(SemanticObject.PATH_MARKER): SemanticObject.PATH_MARKER
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.slalom_row_candidates = []

        self.cli = self.create_client(
            AddSemanticObject, '/pontus/add_semantic_object')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.cluster = self.create_subscription(
            PointCloud2,
            '/pontus/sonar/clustercloud',
            self.pointcloud_callback,
            10
        )

        self.yolo = self.create_subscription(
            Detection2DArray,
            '/pontus/{}/yolo_results'.format(self.camera_frame_name),
            self.yolo_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            # Replace with your camera info topic
            '/pontus/{}/camera_info'.format(self.camera_frame_name),
            self.info_callback,
            10
        )

        self.sm_sub = self.create_subscription(
            SemanticMap,
            '/pontus/semantic_map',
            self.semantic_map_callback,
            10
        )

        self.debug_line_pub = self.create_publisher(
            MarkerArray,
            '/pontus/{}/debug_lines'.format(self.camera_frame_name),
            10
        )

        self.slalom_debug_pub = self.create_publisher(
            String,
            '/pontus/slalom_completion_debug',
            10
        )


        self.debug_id = 0

    def yolo_callback(self, msg: Detection2DArray) -> None:
        """Assigns results from yolo to points in latest_pointcloud and publishes all points as list

        Args:
            msg (Detection2DArray): _description_
        """
        if not self.cam_initialized:
            self.get_logger().warn("camera info not initialized")
            return
        elif self.latest_pointcloud is None:
            self.get_logger().warn("No point cloud found")
            return

        # TODO: This should be removed when not testing on old bag data
        msg.header.frame_id = "camera_front_optical_frame"

        array_msg = MarkerArray()
        marker_msg = Marker()
        marker_msg.header = msg.header
        marker_msg.ns = "lines"
        marker_msg.action = Marker.DELETEALL
        array_msg.markers.append(marker_msg)
        self.debug_line_pub.publish(array_msg)

        for object in msg.detections:
            if object.bbox.size_x >= self.min_bbox_width:
                temp, point_found = self.associate_object_with_point(
                    object, self.latest_pointcloud)
                if point_found and temp[2] > self.confidence_min:
                    success = self.send_request(temp[0], temp[1])

    def semantic_map_callback(self, msg: SemanticMap) -> None:
        """
        Builds a list of candidate red and white poles that could be completed with detected cluster point cloud

        
        """

        # slalom_row_candidates = []
        # red_list = msg.slalom_red
        white_list = msg.slalom_white

        existing_rows = msg.meta_slalom.meta_slalom_rows 

        # self.red_list = []
        self.white_list = []
        
        # for red in red_list:
        #     dup_flag = False 
        #     for cur_row in existing_rows:
        #         if (SemanticMapDC.check_semantic_object_duplicant(cur_row.slalom_red, red)):
        #             dup_flag = True
        #             break
        #     if dup_flag:
        #         continue
                
        #     self.red_list.append(red)
        
        # we don't consider poles that are already part of a row when completing to prevent duplicates
        for white in white_list:
            dup_flag = False 
            for cur_row in existing_rows:
                if (SemanticMapDC.check_semantic_object_duplicant(cur_row.slaloms_white[0], white)
                        or SemanticMapDC.check_semantic_object_duplicant(cur_row.slaloms_white[1], white)):
                    dup_flag = True
                    break
            if dup_flag:
                continue
                
            self.white_list.append(white)
            



        # for red in red_list:
        #     dup_flag = False 
        #     for cur_row in existing_rows:
        #         if (SemanticMapDC.check_semantic_object_duplicant(cur_row.slalom_red, red)):
        #             dup_flag = True
        #             break
            
        #     if dup_flag:
        #         continue

        #     red_vec = self._pose_to_vec2(red.pose.pose)
        #     for white in white_list:
        #         white_vec = self._pose_to_vec2(white.pose.pose)

        #         red_to_white = np.linalg.norm(red_vec - white_vec)

        #         if (abs(red_to_white - self.slalom_white_to_red_width_m) <= self.slalom_width_tolerance_m):
        #             slalom_row_candidate = SlalomRowCandidate(slalom_red=red, slalom_white=white)
        #             slalom_row_candidates.append(slalom_row_candidate)

        # self.slalom_row_candidates = slalom_row_candidates
    def pointcloud_callback(self, msg: PointCloud2) -> None:
        """
        Saves recieved pointcloud to latest_pointcloud, taken from occupency_grid_manager.py
        Args:
        ----
        msg (PointCloud2): subscribed pointcloud
        """

        # clustered_cloud is already in correct space, optionally add transform logic here just in case
        if msg.header.frame_id != 'map':
            transformed_msg = self.transform_sonar(msg)
        else:
            transformed_msg = msg

        points = point_cloud2.read_points_numpy(
            transformed_msg, field_names=('x', 'y', 'z'),
            skip_nans=True
        )
        # points[:, 1] = -points[:, 1]  # TODO ask why do this?

        self.latest_pointcloud = points
        
        self.complete_slalom_rows()
        # self.publish_slalom_completion_debug()

    # def publish_slalom_completion_debug(self):
        
    #     debug_msg = String()
    #     debug_msg.data = (
    #         f"num slalom row candidates: {len(self.slalom_row_candidates)},"
    #     )

    #     self.slalom_debug_pub.publish(debug_msg)

    def info_callback(self, msg):
        """
        Callback for CameraInfo messages. Updates the PinholeCameraModel.
        """
        if not self.cam_initialized:  # Only update if not initialized or if parameters change
            self.cam_model.fromCameraInfo(msg)
            self.cam_initialized = True
            self.get_logger().info('Camera model initialized with new info.')

    def complete_slalom_rows(self):
        
        point2_array = self.latest_pointcloud.copy()[:, :2]

        for white_pole in self.white_list:
            white_vec = self._pose_to_vec2(white_pole.pose.pose)

            # distances of all clusters points from the white pole
            dists = np.linalg.norm(point2_array - white_vec, axis=1)

            # boolean masks for points that satisfy distance tolerances for red pole and other white pole in a row
            possible_valid_to_red = np.abs(dists - self.slalom_white_to_red_width_m) <= self.slalom_width_tolerance_m
            possible_valid_to_white = np.abs(dists - self.slalom_white_to_white_width_m) <= self.slalom_width_tolerance_m

            # cluster points that satisfy distances to white_vec
            possible_reds =  point2_array[possible_valid_to_red]
            possible_whites =  point2_array[possible_valid_to_white]

            # check possible reds and other whites against each other
            possible_reds_to_whites_dists = np.linalg.norm(possible_reds[:, np.newaxis, :] - possible_whites[np.newaxis, :, :], axis=2)
            possible_reds_to_whites_mask = np.abs(possible_reds_to_whites_dists - self.slalom_white_to_red_width_m) <= self.slalom_width_tolerance_m
            possible_reds_idx, possible_whites_idx = np.asarray(possible_reds_to_whites_mask).nonzero()

            possible_reds = possible_reds[possible_reds_idx]
            possible_whites = possible_whites[possible_whites_idx]

            white_vec_to_possible_white = white_vec - possible_whites

            # array of vectors perpendicular to the vector between white_vec and possible_whites
            perp_vecs = np.stack([-white_vec_to_possible_white[:, 1], white_vec_to_possible_white[:, 0]], axis=1)
            perp_unit_vecs = perp_vecs / np.linalg.norm(perp_vecs, axis=1, keepdims=True)

            white_to_red_vecs = possible_reds - white_vec
            off_dists = np.abs(np.sum(perp_unit_vecs * white_to_red_vecs, axis=1))

            valid_row_mask = off_dists <= self.slalom_row_tolerance_m
            final_reds = possible_reds[valid_row_mask]
            final_whites = possible_whites[valid_row_mask]

            n = len(final_reds)

            if (n == 1):
                class_ids = [SemanticObject.SLALOM_WHITE, SemanticObject.SLALOM_RED]

                red_pose = PoseStamped()
                red_pose.header = white_pole.header
                red_point = Point()
                red_point.x = float(final_reds[0][0])
                red_point.y = float(final_reds[0][1])
                red_point.z = float(white_pole.pose.pose.position.z)
                red_pose.pose.position = red_point

                white_pose = PoseStamped()
                white_pose.header = white_pole.header
                white_point = Point()
                white_point.x = float(final_whites[0][0])
                white_point.y = float(final_whites[0][1])
                white_point.z = float(white_pole.pose.pose.position.z)
                white_pose.pose.position = white_point

                object_poses = [white_pose, red_pose]

                self.send_request(class_ids, object_poses)
                self.get_logger().info("completed slalom row using cluster points")

            
        # for slalom_row_candidate in self.slalom_row_candidates:
        #     red_vec = self._pose_to_vec2(slalom_row_candidate.slalom_red.pose.pose)
        #     white_vec = self._pose_to_vec2(slalom_row_candidate.slalom_white.pose.pose)

        #     red_dists = np.linalg.norm(point2_array - red_vec, axis=1)
        #     white_dists = np.linalg.norm(point2_array - white_vec, axis=1)

        #     possible_valid_to_red = np.abs(red_dists - self.slalom_white_to_red_width_m) <= self.slalom_width_tolerance_m
        #     possible_valid_to_white = np.abs(white_dists - self.slalom_white_to_white_width_m) <= self.slalom_width_tolerance_m

        #     possible_points = point2_array[np.asarray(possible_valid_to_red & possible_valid_to_white).nonzero()]


        #     for point in possible_points:
                
        #         white_to_point = white_vec - point

        #         perp_vec = np.array([-white_to_point[1], white_to_point[0]])
        #         perp_unit_vec = perp_vec / np.linalg.norm(perp_vec)

        #         off_dist = np.abs(np.dot(perp_unit_vec, red_vec - white_vec))

        #         if (off_dist <= self.slalom_row_tolerance_m):
        #             object_pose = PoseStamped()
        #             object_pose.header = slalom_row_candidate.slalom_red.header
        #             selected_point = Point()
        #             selected_point.x = float(point[0])
        #             selected_point.y = float(point[1])
        #             selected_point.z = float(slalom_row_candidate.slalom_red.pose.pose.position.z)
        #             object_pose.pose.position = selected_point

                    


    def associate_object_with_point(self, object_msg: Detection2D, point_array: np.ndarray) -> tuple:
        """
        finds the tracetory to an object from the camera and associates the first point in pointcloud on the line defined
        by that trajectory with that object

        Args:
            object_msg (Detection2D): dectected object message
            point_array (np.ndarray): pointcloud to search for matching points in as an array, in map frame

        Returns:
            placeholder: _description_
        """

        camera_pose = PoseStamped()
        camera_pose.header = object_msg.header
        camera_pose.header.frame_id = "camera_front_optical_frame"

        # naively find highest confidence object
        center_position = object_msg.bbox.center.position
        max_confidence = -1.0
        highest_class = -1.0
        for result in object_msg.results:
            if result.hypothesis.score > max_confidence:
                max_confidence = result.hypothesis.score
                highest_class = self.name_map[result.hypothesis.class_id]
        center_position = object_msg.bbox.center.position

        # self.get_logger().warn("Detected {}, {}".format(result.hypothesis.class_id, highest_class))

        # generate pose in object frame ID to define line, starting point at 0
        point = np.array([[center_position.x, center_position.y]])
        point = np.expand_dims(point, 1)
        rectified_point = self.cam_model.rectify_point(point)
        ray_unit_vector = self.cam_model.project_pixel_to_3d_ray(
            rectified_point)
        pose_to_object = self.align_pose_x_with_vector(
            camera_pose, ray_unit_vector)

        # transform pose to match point cloud frame from camera frame
        transformed_pose = self.transform_camera(pose_to_object)

        # Debug publish lines
        array_msg = MarkerArray()
        marker_msg = Marker()
        marker_msg.header = transformed_pose.header
        marker_msg.pose = transformed_pose.pose
        marker_msg.ns = "lines"
        marker_msg.id = self.debug_id
        self.debug_id += 1
        marker_msg.type = Marker.ARROW
        marker_msg.action = Marker.ADD
        marker_msg.frame_locked = True
        marker_msg.scale.x = 15.0
        marker_msg.scale.y = 0.05
        marker_msg.scale.z = 0.05
        marker_msg.color = self.get_debug_line_color(highest_class)
        array_msg.markers.append(marker_msg)
        self.debug_line_pub.publish(array_msg)

        # find first point lying on line from pose
        relevant_points = self.points_on_line(point_array, transformed_pose)

        if (relevant_points.shape[0] == 0):
            # no points on line
            # self.get_logger().info("no objects on line")
            return ([], [], 0.0), False
            # relevant_points = point_array # for testing

        # relevant_points is sorted by distance from the point to the line
        selected_point = Point()
        selected_point.x = float(relevant_points[0, 0])
        selected_point.y = float(relevant_points[0, 1])
        selected_point.z = float(relevant_points[0, 2])

        # Get the camera ray direction in map frame (same x-axis you used to build the line)
        vector_to_object = self.get_x_axis_vector_from_pose(transformed_pose)

        # Camera position in map frame
        cam_pos = np.array([
            transformed_pose.pose.position.x,
            transformed_pose.pose.position.y,
            transformed_pose.pose.position.z
        ], dtype=float)

        # Ray direction in map frame
        dir_vec = np.array([
            vector_to_object.vector.x,
            vector_to_object.vector.y,
            vector_to_object.vector.z
        ], dtype=float)

        dir_xy = dir_vec[:2]
        cam_xy = cam_pos[:2]
        target_xy = np.array([selected_point.x, selected_point.y], dtype=float)

        denom = float(dir_xy.dot(dir_xy))
        if denom > 1e-8:
            # Best t such that ray’s (x, y) is closest to sonar (x, y)
            t = dir_xy.dot(target_xy - cam_xy) / denom

            if t < 0.0:
                t = 0.0

            z_est = cam_pos[2] + t * dir_vec[2]
            selected_point.z = float(z_est)

        # create object stamped pose
        object_pose = PoseStamped()
        object_pose.header = transformed_pose.header
        object_pose.pose.position = selected_point

        return ([highest_class], [object_pose], max_confidence), True

    def align_pose_x_with_vector(self, pose_msg: PoseStamped, target_vector: list) -> PoseStamped:
        """
        Aligns the x-axis of a given Pose orientation with a target 3D vector.

        Args
            :param pose_msg: The original geometry_msgs.msg.PoseStamped message.
            :param target_vector: The desired direction as a list unit vector.
        Returns
            :return: A new Pose message with the updated orientation.
        """
        # 1. Define the initial X-axis vector (1, 0, 0)
        initial_x_axis = np.array([1.0, 0.0, 0.0])

        target_array = np.array(target_vector)

        # 2. Calculate the rotation axis and angle
        # The axis of rotation is the cross product of the initial and target vectors
        rotation_axis = np.cross(initial_x_axis, target_array)
        rotation_axis_norm = np.linalg.norm(rotation_axis)

        if rotation_axis_norm == 0:
            # Vectors are parallel (either same or opposite direction)
            if np.dot(initial_x_axis, target_array) > 0:
                # Same direction, no rotation needed
                return pose_msg
            else:
                # Opposite direction (180 degrees rotation around any orthogonal axis, e.g., Y-axis)
                rotation_axis = np.array([0.0, 1.0, 0.0])
                angle = pi
        else:
            # Normalize the rotation axis
            rotation_axis = rotation_axis / rotation_axis_norm
            # The angle of rotation can be found using the dot product (angle = acos(dot(u, v)))
            angle = acos(np.dot(initial_x_axis, target_array))

        # 3. Convert the axis-angle representation to a quaternion
        quat_new = tf_transformations.quaternion_about_axis(
            angle, rotation_axis)

        # 4. Update the Pose message
        new_pose = PoseStamped()
        new_pose.header = pose_msg.header
        # Keep the original position, should be all zeros
        new_pose.pose.position = pose_msg.pose.position
        new_pose.pose.orientation.x = quat_new[0]
        new_pose.pose.orientation.y = quat_new[1]
        new_pose.pose.orientation.z = quat_new[2]
        new_pose.pose.orientation.w = quat_new[3]

        return new_pose

    def get_x_axis_vector_from_pose(self, pose_msg: PoseStamped) -> Vector3Stamped:
        """
        Extracts the x-axis direction vector from a geometry_msgs/Pose message.
        """
        # 1. Define the unit vector along the local x-axis
        # local_x_axis_vector = Vector3Stamped(x=1.0, y=0.0, z=0.0)

        local_x_axis_vector = Vector3Stamped()
        local_x_axis_vector.header = pose_msg.header
        local_x_axis_vector.vector = Vector3(x=1.0, y=0.0, z=0.0)

        # 2. Create a transform using only the orientation of the pose
        # We use a dummy transform with zero translation for this
        transform = geometry_msgs.msg.Transform(
            translation=Point(x=0.0, y=0.0, z=0.0),
            rotation=pose_msg.pose.orientation
        )

        # 3. Create a TransformStamped message for the tf2 utility
        transform_stamped = geometry_msgs.msg.TransformStamped()
        transform_stamped.transform = transform

        # 4. Transform the local x-axis vector to the pose's frame of reference
        # do_transform_vector3 applies the rotation defined by the transform
        global_x_axis_vector = tf2_geometry_msgs.do_transform_vector3(
            local_x_axis_vector, transform_stamped)

        return global_x_axis_vector

    def points_on_line(self, point_array: np.ndarray, pose: PoseStamped) -> np.ndarray:
        """
        Checks that provided point exists on line defined by pose, this could be improved

        Args:
            point_array (Point): array of points to be checked against
            pose (Pose): pose that defines line of interest

        Returns:
            np.ndarray: structured numpy array of all points on the line defined by pose
        """

        point2_array = point_array.copy()
        point2_array[:, 2] = 0

        vector_to_object = self.get_x_axis_vector_from_pose(pose)

        start_point = np.array(
            [pose.pose.position.x, pose.pose.position.y, 0.0], dtype=point_array.dtype)

        # Range filtering
        ranges = np.linalg.norm(point2_array - start_point, axis=1)
        in_range = ranges <= self.max_cluster_dist_m

        point_array = point_array[in_range]
        point2_array = point2_array[in_range]

        if point_array.size == 0:
            return point_array.reshape(0, point_array.shape[1])

        end_point = np.array([vector_to_object.vector.x*self.vector_projection_dist + pose.pose.position.x,
                              vector_to_object.vector.y*self.vector_projection_dist + pose.pose.position.y,
                              0.0], dtype=point_array.dtype)

        ba = start_point - end_point
        bc = point2_array - end_point

        distance = np.sum(np.abs(np.cross(ba, bc)), axis=1) / \
            (np.sum(np.abs(ba)) + 0.000000001)

        keys = np.argsort(distance)
        sorted_point_array = point_array[keys]
        sorted_distance_array = np.sort(distance)

        return sorted_point_array[sorted_distance_array <= self.line_projection_width, :]

    @staticmethod
    def _canonicalize_cloud(cloud: PointCloud2) -> PointCloud2:
        pts = np.array([(x, y, z) for x, y, z in pc2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True)],
                       dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
        fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        new_cloud = pc2.create_cloud(cloud.header, fields, pts)
        return new_cloud

    def transform_sonar(self, msg: PointCloud2) -> PointCloud2:
        """
        transforms the frame from sonar to map

        Args:
        ----
        msg (PointCloud2): subscribed pointcloud

        Return:
        ----
        (PointCloud2): The pointedcloud with transformed data
        """
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame=msg.header.frame_id,
                time=Time(seconds=msg.header.stamp.sec,
                          nanoseconds=msg.header.stamp.nanosec)
            )
            # self.get_logger().info("SUCCESS ON TRANSFORM")
        except Exception as e:
            self.get_logger().warn(
                "failure to transform pointcloud to map frame, current frame: {}".format(msg.header.frame_id))
            self.get_logger().warn(f"exception: {e}")
            return msg

        msg_canon = self._canonicalize_cloud(msg)
        transformed_msg = do_transform_cloud(msg_canon, transform)

        return transformed_msg

    def transform_camera(self, msg: PoseStamped) -> PoseStamped:
        """
        transforms the frame from sonar to map

        Args:
        ----
        msg (PoseStamped): posestamped in camera frame

        Return:
        ----
        (PoseStamped): The posestamped with transformed data
        """
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame=msg.header.frame_id,
                time=Time(seconds=msg.header.stamp.sec,
                          nanoseconds=msg.header.stamp.nanosec)
            )
            pose_transformed = tf2_geometry_msgs.do_transform_pose(
                msg.pose, transform)
            # self.get_logger().info("SUCCESS ON TRANSFORM")
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header = msg.header
            pose_stamped_msg.header.frame_id = "map"
            pose_stamped_msg.pose = pose_transformed
            return pose_stamped_msg
        except Exception as e:
            self.get_logger().warn(
                "failure to transform pose to map frame, current frame: {}".format(msg.header.frame_id))
            self.get_logger().warn(f"exception: {e}")
            return msg

    def send_request(self, class_id, pose):
        req = AddSemanticObject.Request()
        req.ids = class_id
        req.positions = pose
        self.cli.call_async(req)

    def get_debug_line_color(self, class_id):
        match class_id:
            case SemanticObject.SLALOM_RED:
                return ColorRGBA(r=1.0, a=1.0)
            case SemanticObject.SLALOM_WHITE:
                return ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            case SemanticObject.VERTICAL_MARKER:
                return ColorRGBA(a=1.0)
            case default:
                return ColorRGBA(g=1.0, a=1.0)

    def _pose_to_vec2(self, pose: Pose) -> np.ndarray:
        """Convert a Pose into 2D np.array [x, y] for planar distance calculations."""
        return np.array([pose.position.x, pose.position.y], dtype=float)


def main(args=None):
    rclpy.init(args=args)
    node = ImageCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
