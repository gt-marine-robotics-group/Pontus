import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
from sensor_msgs.msg import CameraInfo
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from pontus_msgs.msg import YOLOResultArray
from pontus_msgs.msg import YOLOResult
from pontus_msgs.srv import AddSemanticObject

class YoloPoseDetection(Node):
    def __init__(self):
        super().__init__('yolo_pose_detection')

        self.left_yolo_sub = Subscriber(
            self,
            YOLOResultArray,
            '/pontus/camera_2/yolo_results',
        )

        self.right_yolo_sub = Subscriber(
            self,
            YOLOResultArray,
            '/pontus/camera_3/yolo_results'
        )
        self.ts = ApproximateTimeSynchronizer(
            [self.left_yolo_sub, self.right_yolo_sub], 5, 0.1
        )

        self.camera_info_right_subscriber = self.create_subscription(
            CameraInfo,
            '/pontus/camera_3/camera_info',
            self.camera_info_callback_right,
            10,
        )

        self.camera_info_left_subscriber = self.create_subscription(
            CameraInfo,
            '/pontus/camera_2/camera_info',
            self.camera_info_callback_left,
            10,
        )

        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        self.add_semantic_object_client = self.create_client(
            AddSemanticObject,
            '/pontus/add_semantic_object',
            callback_group=self.service_callback_group
        )
        while not self.add_semantic_object_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for add semantic object service")

        self.ts.registerCallback(self.yolo_callback)
        self.Tx = None
        self.cx = None
        self.cy = None
        self.f = None


    def camera_info_callback_right(self, msg: CameraInfo):
        self.Tx = -msg.p[3]
    

    def camera_info_callback_left(self, msg: CameraInfo):
        self.f = msg.k[0]
        self.cx = msg.k[2]
        self.cy = msg.k[5]


    def pair_detections(self,
                        left_result: YOLOResultArray, right_result: YOLOResultArray,
                        min_distance: float, max_distance: float, Tx: float) -> list[tuple[YOLOResult, YOLOResult]]:
        """
        Use nearest neighbors algorithm to pair detections together to calculate disparity.

        Parameters:
        left_result (YOLOResultArray) : the detecitons from the left camera
        right_result (YOLOResultArray) : the detections from the right camera
        min_distance (float) : the minimum distance to be considered a detection
        max_distance (float) : the maximum distance to be considered a detection
        Tx (float) : the translation of the left and right frame center in pixels 

        Returns:
        list[tuple[YOLOResult, YOLOResult]] : pairs of YOLOResults representing the same object
        """
        valid_pairs = []
        for left_yolo_result in left_result.results:
            # Arbitrarly large value for initialization
            current_min_distance = 100000
            left_yolo_result_center = np.array([
                (left_yolo_result.x1 + left_yolo_result.x2) / 2,
                (left_yolo_result.y1 + left_yolo_result.y2) / 2
                ]).T
            # Look for closest detection in right camera
            current_pair = None
            abs_disparity = None
            for right_yolo_result in right_result.results:
                if left_yolo_result.class_id != right_yolo_result.class_id:
                    continue
                right_yolo_result_center = np.array([
                    (right_yolo_result.x1 + right_yolo_result.x2) / 2,
                    (right_yolo_result.y1 + right_yolo_result.y2) / 2
                ]).T
                distance = np.linalg.norm(left_yolo_result_center - right_yolo_result_center)
                if distance < current_min_distance:
                    current_min_distance = distance
                    current_pair = (left_yolo_result, right_yolo_result)
                    abs_disparity = left_yolo_result_center[0] - right_yolo_result_center[0]
            # Only add to list if found a pair and within dispairty

            if current_pair and min_distance <= Tx / abs_disparity <= max_distance:
                valid_pairs.append(current_pair)
            
        return valid_pairs


    def calculate_3d_pose(self, 
                          pair: tuple[YOLOResult, YOLOResult],
                          Tx: float, cx: float, cy: float, f: float) -> np.ndarray:
        """
        Given two detections, calculate its 3D pose using camera intrsics and stereo

        Parameters:
        pair (tuple[YOLOResult, YOLOResult]) : pair of detections of the same object we want to calculate pose for
        Tx (float) : the translation in pixels between the left and right camera frame center
        cx (float) : the x coordiante of the frame center of the left image
        cy (float) : the y coordiante of the frame center of the left image
        f (float) : the focal length in pixels of the left camera

        Returns:
        np.ndarray : a numpy array of size 3 containing the pose of the object in the relative body frame
        """
        left_cam_center_x = (pair[0].x1 + pair[0].x2) / 2
        left_cam_center_y = (pair[0].y1 + pair[0].y2) / 2
        right_cam_center_x = (pair[1].x1 + pair[1].x2) / 2
        z = Tx / (left_cam_center_x - right_cam_center_x)
        x = (left_cam_center_x - cx) * z / f
        y = (left_cam_center_y - cy) * z / f

        detection_optical_frame = np.array([x, y, z])
        R = np.array([[0, 0, 1],
                        [-1, 0, 0],
                        [0, -1, 0]])
        detection_body_frame = np.dot(detection_optical_frame, R.T)
        return detection_body_frame


    def yolo_callback(self, left_result: YOLOResultArray, right_result: YOLOResultArray) -> None:
        """
        Approximate synchronized callback for left and right yolo results. This will do the following:
        1. Calculate 3D pose of all the detections
        2. Delineat between left and right gate
        3. Call AddSemanticObject service to add detection to semantic map
        
        Parameters:
        left_result (YOLOResultArray) : the yolo results from the left camera
        right_result (YOLOResultArray) : the yolo results from the right camera

        Returns:
        None
        """
        if not self.Tx or not self.f:
            self.get_logger().info("Waiting to receive camera info topic")
            return
        paired_detections = self.pair_detections(left_result, right_result, 0.5, 6, self.Tx)
        for pair in paired_detections:
            detection_body_frame = self.calculate_3d_pose(pair, self.Tx, self.cx, self.cy, self.f)
            request = AddSemanticObject.Request()
            request.id = pair[0].class_id
            request.position.header.frame_id = "camera_2"
            # request.position.header.stamp = self.get_clock().now()
            request.position.pose.position.x = detection_body_frame[0]
            request.position.pose.position.y = detection_body_frame[1]
            request.position.pose.position.z = detection_body_frame[2]
            future = self.add_semantic_object_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() is None:
                self.get_logger().info("Failed to call add semantic object service")
            self.get_logger().info(f'{pair[0].class_id} {detection_body_frame}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloPoseDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
