import rclpy
import numpy as np
import cv2 as cv
import tf_transformations
import math

from enum import Enum
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from pontus_msgs.msg import YOLOResultArray

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient, PoseObj
from pontus_perception.color_threshold_detection import *

from typing import List, Optional

class PathMarkerTask(BaseTask):
    class State(Enum):
        Searching = 0
        Centering = 1
        Aligning = 2
        Done = 3

    def __init__(self):
        super().__init__("Path_Marker_Task")
        
        self.go_to_pose_client = GoToPoseClient(self)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
            10
        )

        self.cv_bridge = CvBridge()
        self.autonomy_loop = self.create_timer(0.2, self.state_machine_callback)
        self.state = self.State.Searching
        self.previous_state = self.state

        self.btm_camera_sub = self.create_subscription(
            Image,
            '/pontus/camera_1/image_raw',
            self.camera_callback,
            10
        )

        self.contour = None

        # prob need to tune for real images (in sim the path marker is green?????)
        self.threshold = ColorThreshold(
            np.array([44, 34, 93]), 
            np.array([119, 255, 182])
        )

        # self.yolo_tilted_sub = self.create_subscription(
        #     YOLOResultArray,
        #     '/camera_2/yolo/yolo_results',
        #     self.yolo_callback,
        #     10
        # )

        # self.yolo_bottom_sub = self.create_subscription(
        #     YOLOResultArray,
        #     '/camera_1/yolo/yolo_results',
        #     self.yolo_callback,
        #     10
        # )
        # self.yolo_tilted_result = None
        # self.yolo_down_result = None

        
        self.current_pose = None
        self.current_yaw = 0.0
        self.img = None
        self.hsv_img = None
        self.pixel_to_meter = 50 # idk yet
        self.marker_x = None
        self.marker_y = None

    # <---- Callbacks ---- >
    def odom_callback(self,msg: Odometry) -> None:
        if self.current_pose is None:
            self.desired_depth = msg.pose.pose.position.z
        self.current_pose = msg.pose.pose
        quat = [self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w]
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(quat)

    def camera_callback(self, msg: Image) -> None:
        self.img = self.cv_bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding='passthrough')
        self.hsv_img = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)


    # def yolo_tilted_callback(self, msg: YOLOResultArray) -> None:

    #     for tilted_result in msg.results:
    #         if (tilted_result.label == "path_marker"):
    #             self.yolo_tilted_result = tilted_result
    #             return
    #     self.yolo_tilted_result = None

    # def yolo_down_callback(self, msg: YOLOResultArray) -> None:
    #     for down_result in msg.results:
    #         if (down_result.label == "path_marker"):
    #             self.yolo_down_result = down_result
    #             return

    #     self.yolo_down_result = None
    
    def state_machine_callback(self):

        self.state_machine_debugger()

        cmd_pose_obj = None
        match self.state:
            case self.State.Searching:
                cmd_pose_obj = self.search()
            case self.State.Centering:
                cmd_pose_obj = self.center()
            case self.State.Aligning:
                cmd_pose_obj = self.align()
            case self.State.Done:
                self.done()

        # if cmd_pose_obj:
        #     self.go_to_pose_client.go_to_pose(cmd_pose_obj)

    # <---- Autonomy ----> 
    def state_machine_debugger(self):
        if self.previous_state != self.state:
            self.get_logger().info(f"State changed from [{self.previous_state}] to [{self.state}]")
            self.previous_state = self.state

    def search(self) -> Optional[PoseObj]:
        
        
        detected_parallel = self.detect_path_marker()

        if (len(detected_parallel) == 0):
            # something's wrong, can't detect path marker, maybe too close
            self.get_logger().error("Unable to detect path marker!")
        else:
            line_pair = min(detected_parallel, key= lambda x: x[2])
            # self.get_logger().info("DETECTED PATH MARKER lksjdflksdjflkdsjflksdfljdslfkj")
            self.state = self.State.Centering
            self.marker_x, self.marker_y = self.get_contour_center(line_pair[3])
            
            # self.get_logger().info(f"Center of detected path marker contour: ({self.marker_x}, {self.marker_y})")
            # cv.imwrite("raw_downward.png", self.img)
            # cv.circle(self.img, (self.marker_x, self.marker_y), 10, (0,0,255), -1)
            # cv.drawContours(self.img, [line_pair[3]], -1, (0, 0, 255), 10)
            # cv.imwrite("contour and center.png", self.img)


        return None

    def center(self):
        """
        cmd velocity so that the sub moves to center the path marker in the sub
        """
        if self.marker_x == None or self.marker_y == None:
            self.marker_x, self.marker_y = self.get_contour_center()
        


        

        self.marker_x = self.marker_y = None
        center_x, center_y = (self.img.shape[0] // 2, self.img.shape[1] // 2)

        x_diff = center_x - self.marker_x
        y_diff = center_y - self.marker_y


        




        
    def align(self):
        """
        """
        pass

        
    
    def done(self):
        pass

    # <---- Vision ---->
    def detect_path_marker(self):

        parallel_lines = []
        if (self.hsv_img is None):
            self.get_logger().error("hsv_img is EMPTY")
            if (self.img is None):
                self.get_logger().error("img is also EMPTY")
            return parallel_lines

        contours = get_color_threshold_contours(self.hsv_img, self.threshold)
        contour_mask = None
        for contour in contours:

            contour_mask = np.zeros(shape=(self.hsv_img.shape[0], self.hsv_img.shape[1]), dtype=np.uint8)
            cv.drawContours(contour_mask, [contour], -1, 255, 1)
            

            lines = cv.HoughLines(contour_mask, 1, np.pi / 180, threshold=100)
            n = len(lines)
            if n > 0:
                parallel_threshold = 5
                for i in range(n):
                    for j in range(i + 1, n):
                        angle_diff = math.degrees(abs(lines[i][0][1] - lines[j][0][1]))
                        
                        if angle_diff < parallel_threshold:
                            parallel_lines.append((lines[i], lines[j], angle_diff, contour))

                if len(parallel_lines) > 0:
                    break
        
        return parallel_lines

    def get_corner(self, contour_mask: cv.Mat):
        pixels = list(zip(*np.where(contour_mask == 255)))

        if len(pixels) > 0:
            return pixels[0]
        return None

    def get_contour_center(self, contour: cv.Mat):
        M = cv2.moments(contour)
        if (M["m00"] != 0):
            return int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])

    def get_pca_orientation(self) -> float:
        
        contours = get_color_threshold_contours(self.hsv_img, self.threshold)

        contours = sorted(contours, key=cv.contourArea, reverse=True)
        pts = contours[0]
        sz = len(pts)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts[i,0] = pts[i,0,0]
            data_pts[i,1] = pts[i,0,1]
 
        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv.PCACompute2(data_pts, mean)

        return math.atan2(eigenvectors[0,1], eigenvectors[0,0])


    
