import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from pontus_perception.gate_detection.stereo_gate_detection import StereoGateDetction
from pontus_perception.gate_detection.yolo_gate_detection import YoloGateDetection
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from pontus_msgs.srv import GetGateLocation
from geometry_msgs.msg import Point

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

        self.service = self.create_service(
            GetGateLocation,
            '/pontus/get_gate_detection',
            self.handle_get_gate_detection
        )

        self.point_cloud = None
        self.current_depth = None
        self.pool_depth = -2.1336
        self.detect_functions = [self.get_gate_from_yolo, self.get_gate_from_stereo]

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


    def handle_get_gate_detection(self, request, response):
        response.left_location = Point()
        response.right_location = Point()
        response.found = False
        # Iterate through a list of detection functions
        for detect_gate in self.detect_functions:
            left_gate_detection, right_gate_detection = detect_gate()
            # If detection not found, move on to next detection function
            if left_gate_detection is None or right_gate_detection is None:
                continue
            self.get_logger().info(f"Left gate: {left_gate_detection}, Right gate: {right_gate_detection}")
            # Publish Gate detection
            response.left_location.x = left_gate_detection[0]
            response.left_location.y = left_gate_detection[1]
            response.left_location.z = left_gate_detection[2]
            response.right_location.x = right_gate_detection[0]
            response.right_location.y = right_gate_detection[1]
            response.right_location.z = right_gate_detection[2]
            response.found = True
            return response
        # If no gate found, say no gate found and return
        return response
        

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