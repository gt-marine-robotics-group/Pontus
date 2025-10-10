import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PointStamped, Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np

# The Occupancy Grid values sometimes don't align. Might be a non-issue/noise problem
# PSA: Times are currently set based on the message time. This may cause issues during real running. Change the times to current time
# grid size may have undue effects on the accuracy. Unsure

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
            'pontus/sonar/rect',
            self.pointcloud_callback,
            10
        )
        
        self.timer = self.create_timer(
            0.1,
            self.occupancy_grid_update
        )
        
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('width', 250)
        self.declare_parameter('height', 250)
        
        self.resolution = self.get_parameter('resolution').value
        self.map_width = self.get_parameter('width').value
        self.map_height = self.get_parameter('height').value
        self.latest_pointcloud: np.array = None
        self.point_weight = 6
        self.decay_rate = 3
        
        
        self.occupancy_grid: OccupancyGrid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = 'map'
        self.occupancy_grid.info.resolution = self.resolution
        self.occupancy_grid.info.height = self.map_height
        self.occupancy_grid.info.width = self.map_width
        self.occupancy_grid.info.origin.position.x = -(self.map_width * self.resolution) / 2 
        self.occupancy_grid.info.origin.position.y = -(self.map_height * self.resolution) / 2
        self.occupancy_grid.info.origin.position.z = 0.0
        self.occupancy_grid.info.origin.orientation.x = 0.0
        self.occupancy_grid.info.origin.orientation.y = 0.0
        self.occupancy_grid.info.origin.orientation.z = 0.0
        self.occupancy_grid.info.origin.orientation.w = 1.0
        
        self.occupancy_grid.data = [0] * (self.map_width * self.map_height)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("Occupancy Grid started")
    
    def pointcloud_callback(self, msg: PointCloud2) -> None:
        
        transformed_msg = self.transform_sonar(msg)
        
        points = np.array( [[i[0], -i[1]] for i in point_cloud2.read_points(transformed_msg, field_names=('x', 'y'), skip_nans = True)] )
        
        self.occupancy_grid.header.stamp = msg.header.stamp #Comment this line if current time is desired
        
        self.latest_pointcloud = points
    
    def occupancy_grid_update(self) -> None:
        if self.latest_pointcloud is not None and self.latest_pointcloud.size != 0:
            self.process_data(self.latest_pointcloud)
        #else:
            #self.get_logger().info("Something is wrong with the Point Cloud Data")
        
        #Uncomment this line to set time to current time instead of message time
        #self.occupancy_grid.header.stamp = self.get_clock().now().to_msg() 
        self.occupancy_grid_publisher.publish(self.occupancy_grid)
        
    
    def process_data(self, points) -> None:
        bin = np.floor(points.copy() / self.resolution)
        bin[:, 0] += self.map_width // 2 #Account for the shifting from begininng
        bin[:, 1] = self.map_height // 2 - bin[:, 1]
        
        bin = bin[np.all(((bin[:, :2] >= [0, 0]) & (bin[:, :2] <= [self.map_width, self.map_height])), axis=1)]
        
        scan = np.zeros(self.map_width * self.map_height, dtype=np.int8)
        for cell in bin:
            cell_index = int(cell[0] + (cell[1] - 1) * (self.map_width))
            scan[cell_index] += 1
        
        
        for index, curr_val in enumerate(self.occupancy_grid.data):
            self.occupancy_grid.data[index] = int(self.calculate_values(scan[index], curr_val))
    
    def calculate_values(self, cell_count, curr_score) -> int:
        if cell_count <= 0:
            return max(curr_score - self.decay_rate, 0)
        new_score = cell_count * self.point_weight
        return min(curr_score + new_score, 100)
    
    def transform_sonar(self, msg: PointCloud2):
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame = 'map',
                source_frame = msg.header.frame_id,
                time=rclpy.time.Time()
            )
            #self.get_logger().info("SUCCESS ON TRANSFORM")
        except:
            self.get_logger().warn("failure to transform pointcloud to map frame")
            return msg
        
        transformed_msg = do_transform_cloud(msg, transform)
        
        return transformed_msg


        
    
def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridManager()
    rclpy.spin(node)
    rclpy.shutdown()
        