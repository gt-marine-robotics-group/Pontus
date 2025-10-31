import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np

from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2 as pc2

# PSA: Times are currently set based on the message time. This may cause issues during real running. Change the times to current time


class OccupancyGridManager(Node):
    def __init__(self) -> None:
        super().__init__('occupancy_grid_manager')
        self.occupancy_grid_publisher = self.create_publisher(
            OccupancyGrid,
            '/pontus/occupancy_grid',
            10
        )

        self.sonar = self.create_subscription(
            PointCloud2,
            'pontus/sonar/pointcloud',
            self.pointcloud_callback,
            10
        )

        #self.timer = self.create_timer(
        #    0.1,
        #    self.occupancy_grid_update
        #) Uncomment this line to use a timer update/remove comment in pointcloud_callback

        
        self.declare_parameter('map_resolution_m', 0.1)
        self.declare_parameter('map_width_cell', 500)
        self.declare_parameter('map_height_cell', 500)

        self.resolution = self.get_parameter('map_resolution_m').value
        self.map_width = self.get_parameter('map_width_cell').value
        self.map_height = self.get_parameter('map_height_cell').value
        self.latest_pointcloud: np.array = None
        self.point_weight = 6
        self.decay_rate = 3

        self.occupancy_grid: OccupancyGrid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = 'map'
        self.occupancy_grid.info.resolution = self.resolution
        self.occupancy_grid.info.height = self.map_height
        self.occupancy_grid.info.width = self.map_width
        self.occupancy_grid.info.origin.position.x = - \
            (self.map_width * self.resolution) / 2
        self.occupancy_grid.info.origin.position.y = - \
            (self.map_height * self.resolution) / 2
        self.occupancy_grid.info.origin.position.z = 0.0
        self.occupancy_grid.info.origin.orientation.x = 0.0
        self.occupancy_grid.info.origin.orientation.y = 0.0
        self.occupancy_grid.info.origin.orientation.z = 0.0
        self.occupancy_grid.info.origin.orientation.w = 1.0

        self.occupancy_grid.data = [0] * (self.map_width * self.map_height)
        self.occupancy_ndarray = np.zeros(self.map_width * self.map_height, dtype=np.int16)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Occupancy Grid started")

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        """
        Saves recieved pointcloud to latest_pointcloud
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
        self.occupancy_grid_update() #comment this line if you want to use a timer update time
        
        

    def occupancy_grid_update(self) -> None:
        """
        Publishes most recent Occupancy Grid update
        """
        if self.latest_pointcloud is not None and self.latest_pointcloud.size != 0:
            self.process_data(self.latest_pointcloud)
        # else:
            # self.get_logger().info("Something is wrong with the Point Cloud Data")

        # Uncomment this line to set time to current time instead of message time
        # self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.occupancy_grid.data = self.occupancy_ndarray.tolist()
        self.occupancy_grid_publisher.publish(self.occupancy_grid)

    def process_data(self, points) -> None:
        """
        Uses points to update Occupancy Grid
        Args:
        ----
        points (np.ndarray): np array of all 2d points from PointCloud2d
        """
        bin = np.floor(points.copy() / self.resolution)
        # Account for the shifting from begininng
        bin[:, 0] += self.map_width // 2
        bin[:, 1] = self.map_height // 2 - bin[:, 1]

        bin = bin[np.all(((bin[:, :2] >= [0, 0]) & (
            bin[:, :2] <= [self.map_width, self.map_height])), axis=1)]

        score = np.full(self.map_width * self.map_height, -self.decay_rate, dtype=np.int16)

        #For each (x,y) pair in bin, we perform x + (y - 1) * map_width to convert from pointgrid to occupancy_grid location
        bin = bin[:, 0] + (bin[:, 1] - 1) * self.map_width
        
        bin = bin.astype(np.int32)
        
        #Some bin locations are negative and must be removed
        bin = bin[bin > 0]
        

        bin = np.bincount(bin,minlength=self.map_height * self.map_width)
        
        #For every cell with 1 or more point, we calculate additional score as (#points * point_weight)
        bin[bin > 0] *= self.point_weight
        #Subtract decay_rate as decay rate is added to all cells in score
        bin[bin > 0] += self.decay_rate
        
        score += self.occupancy_ndarray + bin

        #Scores are clamped to between 0-100
        score[score<0] = 0
        score[score>100] = 100
        self.occupancy_ndarray = score

    def calculate_values(self, cell_count, curr_score) -> int:
        """
        updates score given the number of cells in the count.
        Args:
        ----
        cell_count (int): number of points in the cell
        curr_score (int): the current score of the cell
        Return:
        ----
        (int): the updated score
        """
        if cell_count <= 0:
            return max(curr_score - self.decay_rate, 0)
        new_score = cell_count * self.point_weight
        return min(curr_score + new_score, 100)

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
    node = OccupancyGridManager()
    rclpy.spin(node)
    rclpy.shutdown()
