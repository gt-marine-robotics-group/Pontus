import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
from sensor_msgs.msg import CameraInfo
from stereo_msgs.msg import DisparityImage
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from cv_bridge import CvBridge

from pontus_msgs.msg import YOLOResultArray
from pontus_msgs.msg import YOLOResult
from pontus_msgs.srv import AddSemanticObject

from enum import Enum

class SamplingMethod(Enum):
    AVERAGE = 0
    MEDIAN = 0


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

        self.disparity_sub = self.create_subscription(
            DisparityImage,
            '/disparity',
            self.disparity_callback,
            10
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
        self.disparity_msg = None
        self.bridge = CvBridge()


    def disparity_callback(self, msg):
        self.disparity_msg = msg


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


    def calculate_3d_pose_stereo(self, 
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


    def calculate_3d_pose_disparity_map(self, left_result: YOLOResult, disparity_msg: DisparityImage, 
                                        Tx: float, cx: float, cy: float, f: float,
                                        sampling_method: SamplingMethod = SamplingMethod.MEDIAN) -> np.ndarray:
        """
        Given yolo detection and a disparity map, return the 3d pose

        Parameters:
        left_result (YOLOResult) : The yolo detection
        disparity_image (np.ndarray) : The disparity image created by the left and right camera
        Tx (float) : the translation in pixels between the left and right camera frame center
        cx (float) : the x coordiante of the frame center of the left image
        cy (float) : the y coordiante of the frame center of the left image
        f (float) : the focal length in pixels of the left camera
        sampling_method (SamplingMethod) : the sampling method to sample the bounding box of disparity values

        Returns:
        np.ndarray : the 3d pose of the detection in body coordinates
        """
        disparity_image = self.bridge.imgmsg_to_cv2(disparity_msg.image)
        x_min, y_min, x_max, y_max = int(left_result.x1), int(left_result.y1), int(left_result.x2), int(left_result.y2)
        disparity_cropped = disparity_image[y_min:y_max, x_min:x_max]
        
        # Filter out invalid distances
        valid_disparities = disparity_cropped[disparity_cropped > 0]
        
        if len(valid_disparities) == 0:
            return np.nan

        match sampling_method:
            case SamplingMethod.MEDIAN:
                disparity = np.median(valid_disparities)
            case SamplingMethod.AVERAGE:
                disparity = np.mean(valid_disparities)
        
        left_cam_center_x = (left_result.x1 + left_result.x2) / 2
        left_cam_center_y = (left_result.y1 + left_result.y2) / 2
        z = Tx / disparity
        x = (left_cam_center_x - cx) * z / f
        y = (left_cam_center_y - cy) * z / f

        detection_optical_frame = np.array([x, y, z])
        R = np.array([[0, 0, 1],
                        [-1, 0, 0],
                        [0, -1, 0]])
        detection_body_frame = np.dot(detection_optical_frame, R.T)
        return detection_body_frame


    def add_to_semantic_map(self, class_id: int, detection_body_frame: np.ndarray) -> None:
        """
        Adds a detection to the semantic map

        Parameters:
        class_id (int) : the class id of the object
        detection_body_frame (np.ndarray) : the detection pose in body_frame coordaintes

        Returns:
        None
        """
        request = AddSemanticObject.Request()
        request.id = class_id
        request.position.header.frame_id = "camera_2"
        request.position.pose.position.x = detection_body_frame[0]
        request.position.pose.position.y = detection_body_frame[1]
        request.position.pose.position.z = detection_body_frame[2]
        future = self.add_semantic_object_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            self.get_logger().info("Failed to call add semantic object service")
        self.get_logger().info(f'{class_id} {detection_body_frame}')


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
        if not self.Tx or not self.f or not self.disparity_msg:
            self.get_logger().info("Waiting to receive camera info topic")
            return
        # This calculates the disparity by calculating the disparity between the center of the detection in the left camera
        # and the center of the detection in the right camera
        # Not robust, but doesn't heavily depend on stereo/calibration
        # START
        # paired_detections = self.pair_detections(left_result, right_result, 0.5, 6, self.Tx)
        # for pair in paired_detections:
        #     detection_body_frame = self.calculate_3d_pose_stereo(pair, self.Tx, self.cx, self.cy, self.f)
        #     self.add_to_semantic_map(pair[0].class_id, detection_body_frame)
        # END
        
        # This calculates the disparity by referencing the disparity map
        # More robust, but need to have a good disparity map creating
        # These also need to be aligned
        # START
        for detection in left_result.results:
            detection_body_frame = self.calculate_3d_pose_disparity_map(detection, self.disparity_msg, self.Tx, self.cx, self.cy, self.f, SamplingMethod.MEDIAN)
            # Will be nan if the object is out of the field of view of the camera
            if np.isnan(detection_body_frame).any() or detection_body_frame[0] > 5:
                continue
            self.add_to_semantic_map(detection.class_id, detection_body_frame)
        # END


def main(args=None):
    rclpy.init(args=args)
    node = YoloPoseDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
