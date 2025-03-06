import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from pontus_perception.gate_detection.stereo_gate_detection import StereoGateDetection, StereoGateDetectionParams
from pontus_perception.gate_detection.yolo_gate_detection import YoloGateDetection
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from pontus_msgs.srv import GetGateLocation
from geometry_msgs.msg import Point
from pontus_msgs.msg import YOLOResultArray
from sensor_msgs.msg import CameraInfo

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

        self.yolo_sub_left = self.create_subscription(
            YOLOResultArray,
            '/pontus/camera_2/yolo_results',
            self.yolo_results_left_callback,
            10,
        )

        self.yolo_sub_right = self.create_subscription(
            YOLOResultArray,
            '/pontus/camera_3/yolo_results',
            self.yolo_results_right_callback,
            10,
        )

        # The right camera should provide the necessary intrinsic and extrinsic parameters
        self.right_camera_info = self.create_subscription(
            CameraInfo,
            '/pontus/camera_3/camera_info',
            self.camera_info_callback,
            10
        )

        self.service = self.create_service(
            GetGateLocation,
            '/pontus/get_gate_detection',
            self.handle_get_gate_detection
        )

        self.declare_parameter('pool_depth', -2.1336)
        self.declare_parameter('gate_size', 3.0)
        self.declare_parameter('gate_size_tolerance', 0.3)
        self.declare_parameter('remove_statistical_outlier_nb_neighbors', 30)
        self.declare_parameter('remove_statistical_outlier_std_ratio', 3.0)
        self.declare_parameter('dbscan_eps', 0.3)
        self.declare_parameter('dbscan_min_points', 40)
        self.declare_parameter('remove_floor_tolerance', 0.4)
        self.declare_parameter('side_pole_height_min', 1.3)
        self.declare_parameter('side_pole_height_max', 1.7)
        self.declare_parameter('tx_override', -1.0)

        self.stereo_gate_detection_params = StereoGateDetectionParams(
            self.get_parameter('pool_depth').value,
            self.get_parameter('gate_size').value,
            self.get_parameter('gate_size_tolerance').value,
            self.get_parameter('remove_statistical_outlier_nb_neighbors').value,
            self.get_parameter('remove_statistical_outlier_std_ratio').value,
            self.get_parameter('dbscan_eps').value,
            self.get_parameter('dbscan_min_points').value,
            self.get_parameter('remove_floor_tolerance').value,
            self.get_parameter('side_pole_height_min').value,
            self.get_parameter('side_pole_height_max').value
        )

        self.tx_override = self.get_parameter('tx_override').value
        self.point_cloud = None
        self.current_depth = None
        self.left_yolo_result = None
        self.right_yolo_result = None
        self.camera_info = None
        self.gate_width = self.get_parameter('gate_size').value
        # self.detect_functions = [self.get_gate_from_yolo, self.get_gate_from_stereo]
        self.detect_functions = [self.get_gate_from_yolo]

    def depth_callback(self, msg: Odometry):
        self.current_depth = msg.pose.pose.position.z


    def stereo_callback(self, msg):
        self.point_cloud = msg


    def yolo_results_left_callback(self, msg):
        self.left_yolo_result = msg

    
    def yolo_results_right_callback(self, msg):
        self.right_yolo_result = msg


    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def get_gate_from_yolo(self):
        if None in [self.left_yolo_result, self.right_yolo_result, self.camera_info]:
            return None, None, None
        left_gate, right_gate, scaling_factor = YoloGateDetection.detect_gate(self.left_yolo_result, self.right_yolo_result, self.camera_info, self.gate_width, self.tx_override)
        return left_gate, right_gate, scaling_factor


    def get_gate_from_stereo(self):
        if self.point_cloud is None or self.current_depth is None:
            return None, None
        points = pc2.read_points(self.point_cloud, field_names=("x", "y", "z"), skip_nans=True)
        point_array = np.array(list(points))
        converted = point_array.view(np.float32).reshape(len(point_array), -1)
        left_gate, right_gate, debug_pc = StereoGateDetection.detect_gate(converted, self.current_depth, self.stereo_gate_detection_params)
        self.publish_debug_point_cloud(debug_pc)
        return left_gate, right_gate


    def handle_get_gate_detection(self, request: GetGateLocation.Request, response: GetGateLocation.Response):
        response.left_location = Point()
        response.right_location = Point()
        response.found = False
        # Iterate through a list of detection functions
        for detect_gate in self.detect_functions:
            left_gate_detection, right_gate_detection, scaling_factor = detect_gate()
            # If detection not found, move on to next detection function
            if left_gate_detection is None or right_gate_detection is None:
                continue
            self.get_logger().info(f"Left gate: {left_gate_detection}, Right gate: {right_gate_detection}")
            self.get_logger().info(f"Scaling factor: {scaling_factor}")
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