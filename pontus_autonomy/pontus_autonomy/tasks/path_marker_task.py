import rclpy
from enum import Enum
import numpy as np
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image

from pontus_autonomy.tasks.base_task import BaseTask

class State(Enum):
    Searching = 0
    Aligning = 1
    Done = 2

class PathMarkerTask(Node):
    def __init__(self):
        super().__init__("Path_Marker_Task")
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        qos_profile = QoSProfile(
                          reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                          history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                          depth=1
                      )

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/pontus/camera_1/image_raw',
            self.camera_callback,
            qos_profile=qos_profile
        )

        self.state = State.Searching
    
    def camera_callback(self, msg):
        vel_msg = Twist()
        bottom_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        match self.state:
            case State.Searching:
                vel_msg = self.search(bottom_image)
            case State.Aligning:
                vel_msg = self.align(bottom_image)
            case State.Done:
                rclpy.shutdown()
            case _:
                pass
        self.cmd_vel_pub.publish(vel_msg) 

    def search(self, bottom_image):
        desired_vel = Twist()
        # INSERT AUTONOMY CODE HERE

        ##
        return desired_vel
        

    def align(self, bottom_image):
        desired_vel = Twist()
        # INSERT AUTONOMY CODE HERE
        
        ##
        return desired_vel


def main(args=None):
    rclpy.init(args=args)
    node = PathMarkerTask()
    rclpy.spin(node)