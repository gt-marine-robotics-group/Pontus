# from pontus_msgs.srv import AddSemanticObject
from pontus_msgs.msg import SemanticObject
import rclpy
from rclpy.node import Node
import sensor_msgs_py
import sensor_msgs_py.point_cloud2
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import PointCloud2
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
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    BoundingBox2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose
)

class ImageCoordinator(Node):
    def __init__(self) -> None:
        super().__init__('image_to_cluster_coordinator')

        self.latest_pointcloud: np.array = None
        self.line_projection_width = .2 # (m), width of line pointing toward object detected by yolo
        self.confidence_min = 0.5 # object score must be at least this to be added to semantic map
        self.cam_model = PinholeCameraModel()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.semantic_object_publisher = self.create_publisher(
            SemanticObject,
            '/pontus/add_semantic_object',
            10
        )

        self.cluster = self.create_subscription(
            PointCloud2,
            '/pontus/sonar/clustercloud',
            self.pointcloud_callback,
            10
        )

        self.yolo = self.create_subscription(
            Detection2DArray,
            'results',
            self.yolo_callback,
            10
        )

    def yolo_callback(self, msg: Detection2DArray) -> None:
        """Assigns results from yolo to points in latest_pointcloud and publishes all points as list

        Args:
            msg (Detection2DArray): _description_
        """
        for object in msg.detections:
            temp, point_found = self.associate_object_with_point(object, self.latest_pointcloud)
            if point_found and temp.confidence > self.confidence_min:
                self.semantic_object_publisher.publish(temp)

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
        points[:, 1] = -points[:, 1] #TODO ask why do this?

        # Comment this line if current time is desired
        self.occupancy_grid.header.stamp = msg.header.stamp

        self.latest_pointcloud = points


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
        camera_pose.header.frame_id = 'camera_front'

        # naively find highest confidence object
        center_position = object_msg.bbox.center.position
        max_confidence = -1.0
        highest_class = -1.0
        for result in object_msg.results:
            if result.score > max_confidence:
                max_confidence = result.score
                highest_class = result.id
        center_position = object_msg.bbox.center.position

        # generate pose in object frame ID to define line, starting point at 0
        rectified_point = self.cam_model.rectify_point((center_position.x, center_position.y))
        ray_unit_vector = self.cam_model.project_pixel_to_3d_ray(rectified_point)
        pose_to_object = self.align_pose_x_with_vector(camera_pose, ray_unit_vector)

        # transform pose to match point cloud frame from camera frame
        transformed_pose = self.transform_camera(pose_to_object)

        # find first point lying on line from pose
        relevant_points = self.points_on_line(point_array, transformed_pose)
        named_point = SemanticObject()

        if (relevant_points == []).all():
            #no points on line
            return named_point, False

        pose_array = np.empty(1,dtype=point_array.dtype)

        pose_array['x'] = transformed_pose.position.x
        pose_array['y'] = transformed_pose.position.y
        pose_array['z'] = transformed_pose.position.z

        dim_diff = relevant_points - pose_array
        index_of_closest_point = np.argmin(np.sum(np.abs(dim_diff), axis= 1))

        # TODO check if point already exists in map (use distance tolerance, as objects are far apart), if so: don't add to map (return false)

        selected_point = Point()
        selected_point.x = relevant_points[index_of_closest_point,'x']
        selected_point.y = relevant_points[index_of_closest_point,'y']
        selected_point.z = relevant_points[index_of_closest_point,'z']

        # create named point
        named_point = SemanticObject()
        named_point.header = transformed_pose.header
        named_point.object_type = highest_class
        named_point.pose.position = selected_point

        named_point.confidence = max_confidence
        named_point.last_updated = object_msg.header.stamp

        return named_point, True
    
    def align_pose_x_with_vector(pose_msg: PoseStamped, target_vector: list) -> PoseStamped:
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
        quat_new = tf_transformations.quaternion_about_axis(angle, rotation_axis)

        # 4. Update the Pose message
        new_pose = PoseStamped()
        new_pose.header = pose_msg.header
        new_pose.pose.position = pose_msg.position # Keep the original position, should be all zeros
        new_pose.pose.orientation.x = quat_new[0]
        new_pose.pose.orientation.y = quat_new[1]
        new_pose.pose.orientation.z = quat_new[2]
        new_pose.pose.orientation.w = quat_new[3]

        return new_pose
    
    def points_on_line(self, point_array: np.ndarray, pose: PoseStamped) -> np.ndarray:
        """
        Checks that provided point exists on line defined by pose

        Args:
            point_array (Point): array of points to be checked against
            pose (Pose): pose that defines line of interest

        Returns:
            np.ndarray: structured numpy array of all points on the line defined by pose
        """

        # get indices based on pose orientation, atan2, math.abs(target_x/a) % 1 < self.line_projection_width
        # convert quaternion to euler, get x using pitch and roll
        # TODO

        # 
        point_array['x'] / pose.orientation % 1.0 < self.line_projection_width # something like this for xyz
        on_line = False
        return on_line

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
                time=rclpy.time.Time()
            )
            # self.get_logger().info("SUCCESS ON TRANSFORM")
        except:
            self.get_logger().warn("failure to transform pointcloud to map frame, current frame: {}".format(msg.header.frame_id))
            return msg

        msg_canon = self._canonicalize_cloud(msg)
        transformed_msg = do_transform_cloud(msg_canon, transform)

        return transformed_msg
    
    def transform_camera(self, msg: PoseStamped) -> PoseStamped:
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
                time=rclpy.time.Time()
            )
            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, transform)
            # self.get_logger().info("SUCCESS ON TRANSFORM")
            return pose_transformed
        except:
            self.get_logger().warn("failure to transform pointcloud to map frame, current frame: {}".format(msg.header.frame_id))
            return msg


def main(args=None):
    rclpy.init(args=args)
    node = ImageCoordinator()
    rclpy.spin(node)
    rclpy.shutdown()