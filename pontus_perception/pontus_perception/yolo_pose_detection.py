from enum import Enum
import numpy as np
import cv2
from typing import Optional, List

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import CameraInfo
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from pontus_msgs.srv import AddSemanticObject
from pontus_msgs.msg import YOLOResult
from pontus_msgs.msg import YOLOResultArray


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
            [self.left_yolo_sub, self.right_yolo_sub], 3, 0.1
        )

        self.camera_info_right_subscriber = self.create_subscription(
            CameraInfo,
            '/pontus/camera_3/camera_info',
            self.camera_info_callback_right,
            10,
        )

        self.disparity_sub = self.create_subscription(
            Image,
            '/disparity_image',
            self.disparity_callback,
            10
        )

        self.camera_info_left_subscriber = self.create_subscription(
            CameraInfo,
            '/pontus/camera_2/camera_info',
            self.camera_info_callback_left,
            10,
        )

        self.left_camera_sub = self.create_subscription(
            Image,
            '/pontus/camera_2/image_raw',
            self.left_camera_callback,
            10
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
        self.image_width = None
        self.disparity_msg = None
        self.bridge = CvBridge()
        self.left_camera = None

    def left_camera_callback(self, msg: Image) -> None:
        """
        Handle callback for the left camera.

        This is used to see the pixel intesities of gate detections to determine if it is a
        left or right gate.

        Args:
        ----
        msg (Image): image message from topic

        Return:
        ------
        None

        """
        self.left_camera = self.bridge.imgmsg_to_cv2(msg)

    def disparity_callback(self, msg: Image) -> None:
        """
        Handle callback for disparity image of forward facing cameras.

        This is used to get the disparity of yolo detections so that we can calculate its 3D pose.

        Args:
        ----
        msg (Image): image message from topic

        Return:
        ------
        None

        """
        self.disparity_msg = msg

    def camera_info_callback_right(self, msg: CameraInfo) -> None:
        """
        Handle callback for camera info of right camera.

        The right camera info is used to determine the baseline in pixels. This is important
        for converting disparity to real distance in meters.

        Args:
        ----
        msg (CameraInfo): camera info message from topic

        Return:
        ------
        None

        """
        self.Tx = -msg.p[3]

    def camera_info_callback_left(self, msg: CameraInfo) -> None:
        """
        Handle callback for camera info of left camera.

        The left camera info topics will be used for calculating the x and y location
        of the object in the camera frame.

        Args:
        ----
        msg (CameraInfo): camera info message from topic

        Return:
        ------
        None

        """
        self.f = msg.k[0]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.image_width = msg.width

    def at_bounds(self, detection_pair: tuple[YOLOResult, YOLOResult], image_width: int) -> bool:
        """
        Return whether or not a detection box is close to the boundary of the image.

        If so, this will give us a inaccuracte depth estimation so we need to reject this pair.

        Args:
        ----
        detection_pair (tuple[YOLOResult, YOLOResult]): the left and right camera detection
                                                        pair
        image_width (int): the width of the image

        Return:
        ------
        bool: whether any of the bounding boxes are near the frame of FOV

        """
        return (detection_pair[0].x1 <= 5 or detection_pair[0].x2 >= (image_width - 5)
                or detection_pair[1].x1 <= 5 or detection_pair[1].x2 >= (image_width - 5))

    def pair_detections(self,
                        left_result: YOLOResultArray, right_result: YOLOResultArray,
                        min_distance: float, max_distance: float,
                        Tx: float, image_width: int) -> list[tuple[YOLOResult, YOLOResult]]:
        """
        Use nearest neighbors algorithm to pair detections together to calculate disparity.

        Args:
        ----
        left_result (YOLOResultArray): the detecitons from the left camera
        right_result (YOLOResultArray): the detections from the right camera
        min_distance (float): the minimum distance to be considered a detection
        max_distance (float): the maximum distance to be considered a detection
        Tx (float): the translation of the left and right frame center in pixels
        image_width (int): the width of the image

        Return:
        ------
        list[tuple[YOLOResult, YOLOResult]]: pairs of YOLOResults representing the same object

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

            if current_pair and min_distance <= Tx / abs_disparity <= max_distance \
                    and not self.at_bounds(current_pair, image_width):
                valid_pairs.append(current_pair)

        return valid_pairs

    def calculate_3d_pose_stereo(self,
                                 pair: tuple[YOLOResult, YOLOResult],
                                 Tx: float, cx: float, cy: float, f: float,
                                 sampling_method: SamplingMethod = SamplingMethod.AVERAGE
                                 ) -> np.ndarray:
        """
        Return the pose using stereo bounding boxes.

        Given two detections, calculate its 3D pose using camera intrsics and stereo.
        This will use a series of points to get a collection of disparities. Then using the
        sampling method it will use that disparity to calculate its distance.

        Args:
        ----
        pair (tuple[YOLOResult, YOLOResult]): pair of detections of the same object we want
                                              to calculate pose for
        Tx (float): the translation in pixels between the left and right camera frame center
        cx (float): the x coordiante of the frame center of the left image
        cy (float): the y coordiante of the frame center of the left image
        f (float): the focal length in pixels of the left camera
        sampling_method (SamplingMethod): sampling method to determine disparity

        Return:
        ------
        np.ndarray: a numpy array of size 3 containing the pose of the object in
                    the relative body frame

        """
        valid_disparities = []

        # Centers
        left_cam_center_x = (pair[0].x1 + pair[0].x2) / 2
        left_cam_center_y = (pair[0].y1 + pair[0].y2) / 2

        # Left Bound
        valid_disparities.append(pair[0].x1 - pair[1].x1)

        valid_disparities = np.array(valid_disparities)

        match sampling_method:
            case SamplingMethod.MEDIAN:
                disparity = np.median(valid_disparities)
            case SamplingMethod.AVERAGE:
                disparity = np.mean(valid_disparities)

        z = Tx / disparity
        x = (left_cam_center_x - cx) * z / f
        y = (left_cam_center_y - cy) * z / f

        detection_optical_frame = np.array([x, y, z])
        R = np.array([[0, 0, 1],
                      [-1, 0, 0],
                      [0, -1, 0]])
        detection_body_frame = np.dot(detection_optical_frame, R.T)
        return detection_body_frame

    def calculate_3d_pose_disparity_map(self,
                                        left_result: YOLOResult,
                                        disparity_msg: DisparityImage,
                                        Tx: float, cx: float, cy: float, f: float,
                                        sampling_method: SamplingMethod = SamplingMethod.MEDIAN
                                        ) -> np.ndarray:
        """
        Return 3D pose of a YOLO detection.

        This relies on using the disparity map.

        Args:
        ----
        left_result (YOLOResult): The yolo detection
        disparity_msg (DisparityImage): The disparity image created by the left
                                        and right camera
        Tx (float): the translation in pixels between the left and right camera frame center
        cx (float): the x coordiante of the frame center of the left image
        cy (float): the y coordiante of the frame center of the left image
        f (float): the focal length in pixels of the left camera
        sampling_method (SamplingMethod): the sampling method to sample the bounding
                                              box of disparity values

        Return:
        ------
        np.ndarray: the 3d pose of the detection in body coordinates

        """
        disparity_image = self.bridge.imgmsg_to_cv2(disparity_msg)
        x_min, y_min = int(left_result.x1), int(left_result.y1)
        x_max, y_max = int(left_result.x2), int(left_result.y2)
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

    def add_to_semantic_map(self, detection_array: tuple[int, np.ndarray]) -> None:
        """
        Add detections to the semantic map.

        Args:
        ----
        detection_array (tuple[int, np.ndarray]): an array of class_ids and their detection
                                                    pose

        Return:
        ------
        None

        """
        request = AddSemanticObject.Request()
        for detection in detection_array:
            request.ids.append(detection[0])
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "camera_2"
            pose_stamped.pose.position.x = detection[1][0]
            pose_stamped.pose.position.y = detection[1][1]
            pose_stamped.pose.position.z = detection[1][2]
            request.positions.append(pose_stamped)
        future = self.add_semantic_object_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            self.get_logger().info("Failed to call add semantic object service")
        self.get_logger().info(f'Adding to semantic map {detection_array}')

    def determine_if_left_gate(self, yolo_result: YOLOResult, image: np.ndarray) -> None:
        """
        Determine if a detection is left or right.

        Given a yolo result that is a gate_side return whether or not the gate detection represents
        the left gate or the right gate.

        Args:
        ----
        yolo_result (YOLOResult): the yolo result of the gate_side
        image (np.ndarray): the left camera feed that correlates to the yolo_result

        Return:
        ------
        bool: if the gate_side detection is the left side or right side

        """
        x_min, x_max = int(yolo_result.x1), int(yolo_result.x2)
        y_min, y_max = int(yolo_result.y1), int(yolo_result.y2)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        gate_image_slice = hsv[y_min: y_max, x_min: x_max]
        mid_point = int(gate_image_slice.shape[0] / 2)
        upper_half = gate_image_slice[:mid_point]
        lower_half = gate_image_slice[mid_point:]

        upper_half_hue_mean = np.mean(upper_half, axis=(0, 1))[0]
        lower_half_hue_mean = np.mean(lower_half, axis=(0, 1))[0]

        smaller_half_hue_mean = lower_half_hue_mean if lower_half_hue_mean < upper_half_hue_mean \
            else upper_half_hue_mean
        larger_half_hue_mean = upper_half_hue_mean if lower_half_hue_mean < upper_half_hue_mean \
            else lower_half_hue_mean

        # If the color is not drastically different, probably not the gate and a fake detection
        self.get_logger().info(f"{smaller_half_hue_mean / larger_half_hue_mean}")
        if smaller_half_hue_mean / larger_half_hue_mean > 0.6:
            return None
        return smaller_half_hue_mean == upper_half_hue_mean

    def yolo_callback(self, left_result: YOLOResultArray, right_result: YOLOResultArray) -> None:
        """
        Use YOLO to update semantic map.

        Approximate synchronized callback for left and right yolo results.
        This will do the following:
        1. Calculate 3D pose of all the detections
        2. Delineat between left and right gate
        3. Call AddSemanticObject service to add detection to semantic map

        Args:
        ----
        left_result (YOLOResultArray): the yolo results from the left camera
        right_result (YOLOResultArray): the yolo results from the right camera

        Return:
        ------
        None

        """
        # if not self.Tx or not self.f:
        if not self.Tx or not self.f or not self.disparity_msg:
            self.get_logger().info("Waiting to receive camera info topic")
            return
        # The following two for loops are different ways to calculate the stereo depth
        # of a detection. You should be using the disparity map, but currently our jetson
        # can't handle calculating it on the GPU

        # This calculates the disparity by calculating the disparity between the center of the
        # detection in the left camera and the center of the detection in the right camera
        # Not robust, but doesn't heavily depend on stereo/calibration
        # START
        # paired_detections = self.pair_detections(left_result,
        #                                          right_result,
        #                                          1, 6,
        #                                          self.Tx,
        #                                          self.image_width)
        # for pair in paired_detections:
        #     detection_body_frame = self.calculate_3d_pose_stereo(pair,
        #                                                          self.Tx,
        #                                                          self.cx,
        #                                                          self.cy,
        #                                                          self.f)
        #     if pair[0].class_id == 0:
        #         left_side = self.determine_if_left_gate(pair[0], self.left_camera)
        #         if left_side is None:
        #             continue
        #         pair[0].class_id = 0 if left_side else 10
        #     self.get_logger().info(f"{detection_body_frame}")
        #     self.add_to_semantic_map(pair[0].class_id, detection_body_frame)
        # END

        # This calculates the disparity by referencing the disparity map
        # More robust, but need to have a good disparity map creating
        # These also need to be aligned
        # START
        detections = []
        for detection in left_result.results:
            detection_body_frame = self.calculate_3d_pose_disparity_map(detection,
                                                                        self.disparity_msg,
                                                                        self.Tx,
                                                                        self.cx,
                                                                        self.cy,
                                                                        self.f,
                                                                        SamplingMethod.MEDIAN)
            # Will be nan if the object is out of the field of view of the camera
            if np.isnan(detection_body_frame).any() or detection_body_frame[0] > 10:
                continue

            # If detected gate size
            # if detection.class_id == 0:
            #     left_side = self.determine_if_left_gate(detection, self.left_camera)
            #     if left_side is None:
            #         continue
            #     detection.class_id = 0 if left_side else 10

            detections.append((detection.class_id, detection_body_frame))
        # Every loop we will add to the semantic map, this is used to keep track of confidences
        self.add_to_semantic_map(detections)
        # END


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = YoloPoseDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
