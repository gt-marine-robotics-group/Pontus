import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point

from pontus_perception.vertical_marker_detection.yolo_vertical_marker_detection import YoloVerticalMarkerDetection
from pontus_msgs.msg import YOLOResultArray
from pontus_msgs.srv import GetVerticalMarkerLocation

class VerticalMarkerDetection(Node):
    def __init__(self):
        super().__init__('vertical_marker_detection')

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
            GetVerticalMarkerLocation,
            '/pontus/get_vertical_marker_detection',
            self.handle_get_vertical_marker_location
        )
        self.tx_override = -1.0
        self.detect_functions = [self.get_vertical_marker_from_yolo]

    # Callbacks
    def yolo_results_left_callback(self, msg):
        self.left_yolo_result = msg

    
    def yolo_results_right_callback(self, msg):
        self.right_yolo_result = msg
    
    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
    
    def handle_get_vertical_marker_location(self, request: GetVerticalMarkerLocation.Request, response: GetVerticalMarkerLocation.Response):
        response.location = Point()
        response.found = False
        for detect_vertical_marker in self.detect_functions:
            vertical_marker = detect_vertical_marker()
            if vertical_marker is None:
                continue
            self.get_logger().info(f"Vertical Marker location: {vertical_marker}")
            response.location.x = vertical_marker[0]
            response.location.y = vertical_marker[1]
            response.location.z = vertical_marker[2]
            response.found = True
            return response
        self.get_logger().info(f"Unable to find vertical marker")
        return response

    # Detection
    def get_vertical_marker_from_yolo(self):
        if None in [self.left_yolo_result, self.right_yolo_result, self.camera_info]:
            return None
        vertica_marker = YoloVerticalMarkerDetection.detect(self.left_yolo_result, self.right_yolo_result, self.camera_info, self.tx_override)
        return vertica_marker

    

def main(args=None):
    rclpy.init(args=args)
    node = VerticalMarkerDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()