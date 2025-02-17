import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from pontus_perception.gate_detection.stereo_gate_detection import StereoGateDetction
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

class GateDetection(Node):
    def __init__(self):
        super().__init__('gate_detection')

        self.front_stereo_sub = self.create_subscription(
            PointCloud2,
            '/pontus/camera_2/depth',
            self.stereo_callback,
            10
        )

        self.point_cloud_debug = self.create_publisher(
            PointCloud2,
            '/gate_pc_debug',
            10
        )

        self.depth_sub = self.create_subscription(
            Odometry,
            '/pontus/depth_0',
            self.depth_callback,
            10
        )

        self.point_cloud = None
        self.timer = self.create_timer(
            0.2,
            self.update_callback
        )

        self.current_depth = None
        self.pool_depth = -2.1336
    
    def depth_callback(self, msg: Odometry):
        self.current_depth = msg.pose.pose.position.z


    def stereo_callback(self, msg):
        self.point_cloud = msg


    def get_gate_from_yolo(self):
        return None, None


    def get_gate_from_stereo(self):
        if self.point_cloud is None or self.current_depth is None:
            return None, None
        points = pc2.read_points(self.point_cloud, field_names=("x", "y", "z"), skip_nans=True)
        point_array = np.array(list(points))
        converted = point_array.view(np.float32).reshape(len(point_array), -1)
        left_gate, right_gate, debug_pc = StereoGateDetction.detect_gate(converted, self.current_depth, self.pool_depth)
        self.publish_debug_point_cloud(debug_pc)
        return left_gate, right_gate

    def update_callback(self):
        # Check YOLO model
        left_gate, right_gate = self.get_gate_from_yolo()
        if left_gate is not None and right_gate is not None:
            # Publish Gate detection
            return
        left_gate, right_gate = self.get_gate_from_stereo()
        self.get_logger().info(f"Left gate: {left_gate}, Right gate: {right_gate}")
        if left_gate is not None and right_gate is not None:
            # Publish Stereo detection
            return
        
        # If no gate found, say no gate found and return

    def publish_debug_point_cloud(self, point_cloud):
        header = Header()
        header.frame_id = 'camera_2'
        header.stamp = self.get_clock().now().to_msg()
        downsampled_msg = pc2.create_cloud_xyz32(header, point_cloud)
        self.point_cloud_debug.publish(downsampled_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GateDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()