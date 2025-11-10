# from pontus_msgs.srv import AddSemanticObject
from pontus_msgs.msg import NamedPoint
import rclpy
from rclpy.node import Node
import sensor_msgs_py
import sensor_msgs_py.point_cloud2
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np

from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2 as pc2

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

        # self.srv = self.create_service(AddSemanticObject, 'add_three_ints', self.add_three_ints_callback)  

        self.coordinated_cloud_publisher = self.create_publisher(
            PointCloud2,
            '/pontus/coordinated_cloud',
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
        """Assigns results from yolo to points in latest_pointcloud

        Args:
            msg (Detection2DArray): _description_
        """
        1

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        """
        Saves recieved pointcloud to latest_pointcloud, taken from occupency_grid_manager.py
        Args:
        ----
        msg (PointCloud2): subscribed pointcloud
        """

        transformed_msg = self.transform_sonar(msg)

        points = point_cloud2.read_points_numpy(
            transformed_msg, field_names=('x', 'y'),
            skip_nans=True
        )
        points[:, 1] = -points[:, 1]

        # Comment this line if current time is desired
        self.occupancy_grid.header.stamp = msg.header.stamp

        self.latest_pointcloud = points


    def associate_object_with_point(self, object_msg: Detection2D, point_cloud_msg: PointCloud2) -> NamedPoint:
        """
        finds the tracetory to an object from the camera and associates the first point in pointcloud on the line defined
        by that trajectory with that object

        Args:
            object_msg (Detection2D): dectected object message
            point_cloud_msg (PointCloud2): pointcloud to search for matching points in

        Returns:
            placeholder: _description_
        """
        # generate pose in object frame ID to define line, starting point at 0
        # TODO

        # transform pose to match point cloud frame
        # TODO
        transformed_pose = Pose()

        # find first point lying on line from pose
        point_array = sensor_msgs_py.point_cloud2.read_points(point_cloud_msg, field_names=['x', 'y', 'z'], skip_nans=True)
        relevant_points = self.points_on_line(point_array, transformed_pose)
        if (relevant_points == []).all():
            #no points on line
            named_point = NamedPoint()
            named_point.label = -1 # use as failure indicator
            named_point.point = Point()
            return named_point

        pose_array = np.empty(1,dtype=point_array.dtype)

        pose_array['x'] = transformed_pose.position.x
        pose_array['y'] = transformed_pose.position.y
        pose_array['z'] = transformed_pose.position.z

        dim_diff = relevant_points - pose_array
        index_of_closest_point = np.argmin(np.sum(np.abs(dim_diff), axis= 1))

        # TODO check if point already exists, if so: take next closest

        selected_point = Point()
        selected_point.x = relevant_points[index_of_closest_point,'x']
        selected_point.y = relevant_points[index_of_closest_point,'y']
        selected_point.z = relevant_points[index_of_closest_point,'z']

        # create named point
        named_point = NamedPoint()
        named_point.label = object_msg.results.id
        named_point.point = selected_point

        return named_point
    
    def points_on_line(self, point_array: np.ndarray, pose: Pose) -> np.ndarray:
        """
        Checks that provided point exists on line defined by pose

        Args:
            point_array (Point): array of points to be checked against
            pose (Pose): pose that defines line of interest

        Returns:
            np.ndarray: structured numpy array of all points on the line defined by pose
        """

        # get indices based on pose orientation, math.abs(target_x/a) % 1 < self.line_projection_width
        # TODO

        # 
        point_array['x'] / pose.orientation % 1 < self.line_projection_width # something like this for xyz
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
            self.get_logger().warn("failure to transform pointcloud to map frame")
            return msg

        msg_canon = self._canonicalize_cloud(msg)
        transformed_msg = do_transform_cloud(msg_canon, transform)

        return transformed_msg


def main(args=None):
    rclpy.init(args=args)
    node = ImageCoordinator()
    rclpy.spin(node)
    rclpy.shutdown()