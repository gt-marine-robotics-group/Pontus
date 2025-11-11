from pontus_controller.position_controller import MovementMethod
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
            np.array([20, 86, 93]), 
            np.array([85, 255, 199])
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

        self.img_view_pub = self.create_publisher(
            Image,
            '/pontus/path_marker_image',
            10
        )
        self.starting_pose = None
        self.current_pose = None
        self.desired_depth = None
        self.current_yaw = 0.0
        self.img = None
        self.hsv_img = None
        self.marker_x = None
        self.marker_y = None
        self.img_width = None
        self.img_height = None
        self.sent_pose = False

    # <---- Callbacks ---- >
    def odom_callback(self,msg: Odometry) -> None:
        if self.current_pose is None:
            self.desired_depth = msg.pose.pose.position.z
            self.starting_pose = msg.pose.pose
        self.current_pose = msg.pose.pose
        quat = [self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w]
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(quat)

    def camera_callback(self, msg: Image) -> None:
        self.img = self.cv_bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding='bgr8')
        self.img_height, self.img_width, _ = self.img.shape
        # self.get_logger().info(f"img shape: {self.img.shape}")
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

        if cmd_pose_obj:
            self.go_to_pose_client.go_to_pose(cmd_pose_obj)

    # <---- Autonomy ----> 
    def state_machine_debugger(self):
        if self.previous_state != self.state:
            self.get_logger().info(f"State changed from [{self.previous_state}] to [{self.state}]")
            self.previous_state = self.state

    def search(self) -> Optional[PoseObj]:
        
        result = self.detect_path_marker()

        if (result != None):
            contour, marker_angle = result
            self.marker_x, self.marker_y = self.get_contour_center(contour)
            self.state = self.State.Centering
            
            # self.get_logger().info("DETECTED PATH MARKER lksjdflksdjflkdsjflksdfljdslfkj")
            # self.get_logger().info(f"Center of detected path marker contour: ({self.marker_x}, {self.marker_y})")
            # cv.imwrite("raw_downward.jpg", self.img)
            # cv.circle(self.img, (self.marker_x, self.marker_y), 10, (0,0,255), -1)
            # cv.drawContours(self.img, [line_pair[3]], -1, (0, 0, 255), 10)
            # cv.imwrite("contour and center.jpg", self.img)
        return None

    def center(self):
        """
        cmd velocity so that the sub moves to center the path marker in the sub
        """
        if self.marker_x == None or self.marker_y == None:
            result = self.detect_path_marker()
            if (result != None):

                contour, marker_angle = result
                self.marker_x, self.marker_y = self.get_contour_center(contour)
            else:
                self.state = self.State.Searching
                self.get_logger().info(f"Lost path marker, no longer detected")
                return None

        center_x, center_y = (self.img_width // 2, self.img_height // 2)

        # in image coords
        x_diff = center_x - self.marker_x
        y_diff = - (center_y - self.marker_y)

        path_marker_image = self.img.copy()
        cv.circle(path_marker_image, (center_x, center_y), 3, (0, 0, 255), -1)
        cv.circle(path_marker_image, (self.marker_x, self.marker_y), 3, (255, 0, 0), -1)
        img_msg = self.cv_bridge.cv2_to_imgmsg(path_marker_image, encoding="bgr8")
        self.img_view_pub.publish(img_msg)
        
        self.marker_x = self.marker_y = None

        cmd_twist = Twist()
        diff_threshold = 10 # within center of contour is within 10 pixels of the center of the image
        if (abs(x_diff) < diff_threshold and abs(y_diff) < diff_threshold):

            self.state = self.State.Aligning
            # self.state = self.State.Done

            cmd_pose = self.current_pose
            return PoseObj(cmd_pose=cmd_pose)
        
        max_speed = 0.3 # max m / s in each direction
        min_speed = 0.05 # min m / s in each direction

        #image x, y axis are swapped with real world x, y
        cmd_twist.linear.x = (-y_diff / self.img_height) * max_speed + math.copysign(min_speed, -y_diff) 
        cmd_twist.linear.y = (x_diff / self.img_width) * max_speed + math.copysign(min_speed, x_diff)

        self.get_logger().info(f"Centering with X Speed: {cmd_twist.linear.x} ({x_diff}) and Y Speed: {cmd_twist.linear.y} ({y_diff})")
        return PoseObj(cmd_twist=cmd_twist, 
                desired_depth=self.desired_depth,
                movement_method=MovementMethod.VelocityMaintainDepth)

    def align(self):
        """
        """
        if not self.sent_pose:
            result = self.detect_path_marker()
            if (result != None):
                contour, marker_angle = result

                # adjust from cv.HoughLines angle convention to sub yaw convention
                if marker_angle >= math.pi / 2 :
                    self.get_logger().info(f"Marker Angle greater than pi, original marker angle: {math.degrees(marker_angle)}")
                    marker_angle -= math.pi
                marker_angle *= -1
                
                self.marker_x, self.marker_y = self.get_contour_center(contour)
            else:
                self.state = self.State.Searching
                self.get_logger().info(f"Lost path marker, no longer detected")
                return None
  
            # marker_angle = self.get_pca_orientation(contour)
            deg_angle = math.degrees(marker_angle)
            yaw_threshold = 10
            if abs(deg_angle) < yaw_threshold:
                if (deg_angle == 0.0):
                    self.get_logger().info(f"Debug 0 degrees: {marker_angle}")
                    cv.drawContours(self.img, [contour], -1, (0, 0, 255), 10)
                    cv.imwrite("debug_zero_angle.jpg", self.img)
                else:
                    self.get_logger().info(f"(Within Threshold) Marker Angle: {math.degrees(marker_angle)}")
                self.state = self.State.Done
                return None
            self.get_logger().info(f"Current Yaw: {self.current_yaw}")

            desired_yaw = self.current_yaw + marker_angle

            # check to make sure the angle is going the right way
            

        

            # desired_angle = self.current_yaw + 0.1
            self.get_logger().info(f"Marker Angle: {math.degrees(marker_angle)} Desired Yaw: {math.degrees(desired_yaw)}")
            quat = tf_transformations.quaternion_from_euler(0, 0, desired_yaw)
            cmd_pose = Pose()
            cmd_pose.position.z = self.desired_depth
            cmd_pose.orientation.x = quat[0]
            cmd_pose.orientation.y = quat[1]
            cmd_pose.orientation.z = quat[2]
            cmd_pose.orientation.w = quat[3]

            self.marker_x = self.marker_y = None
            self.sent_pose = True

            return PoseObj(cmd_pose=cmd_pose)

        # center_x, center_y = (self.img_width // 2, self.img_height // 2)

        # path_marker_image = self.img.copy()
        # cv.circle(path_marker_image, (center_x, center_y), 3, (0, 0, 255), -1)
        # cv.circle(path_marker_image, (self.marker_x, self.marker_y), 3, (255, 0, 0), -1)
        # img_msg = self.cv_bridge.cv2_to_imgmsg(path_marker_image, encoding="bgr8")
        # self.img_view_pub.publish(img_msg)

        if self.go_to_pose_client.at_pose():
            self.state = self.State.Done
            return None
    
    def done(self):
        # log pose at the end
        self.get_logger().info(f"Starting Pose: ({self.starting_pose.position.x},{self.starting_pose.position.y})")
        self.get_logger().info(f"Ending Pose: ({self.current_pose.position.x},{self.current_pose.position.y})")
        self.complete(True)
    # <---- Vision ---->
    def detect_path_marker(self):

        parallel_lines = []
        if (self.hsv_img is None):
            self.get_logger().error("hsv_img is EMPTY")
            if (self.img is None):
                self.get_logger().error("img is also EMPTY")
            return None

        contours = get_color_threshold_contours(self.hsv_img, self.threshold)
        contour_mask = None
        for contour in contours:

            contour_mask = np.zeros(shape=(self.hsv_img.shape[0], self.hsv_img.shape[1]), dtype=np.uint8)
            cv.drawContours(contour_mask, [contour], -1, 255, 1)
            
            lines = cv.HoughLines(contour_mask, 1, np.pi / 180, threshold=100)
            if lines is not None:
                n = len(lines)
                parallel_threshold = 5
                for i in range(n):
                    for j in range(i + 1, n):
                        angle_diff = math.degrees(abs(lines[i][0][1] - lines[j][0][1]))
                        
                        if angle_diff < parallel_threshold:
                            parallel_lines.append((lines[i][0][1], angle_diff, contour)) # angle of first line, difference of angles between two, contour

                if len(parallel_lines) > 0:
                    break
        
        if len(parallel_lines) == 0:
            self.get_logger().error("Unable to detect path marker!")
            self.img_view_pub.publish(self.cv_bridge.cv2_to_imgmsg(self.img, encoding="bgr8"))
            return None
        
        best = min(parallel_lines, key= lambda x: x[1])

        return (contour, best[0]) # contour center and line angle

    def get_contour_center(self, contour: cv.Mat):
        M = cv.moments(contour)
        if (M["m00"] != 0):
            return int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
        

    def get_pca_orientation(self, contour) -> float:
        sz = len(contour)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts[i,0] = contour[i,0,0]
            data_pts[i,1] = contour[i,0,1]
 
        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv.PCACompute2(data_pts, mean)

        return math.atan2(eigenvectors[0,1], eigenvectors[0,0])