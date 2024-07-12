import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from enum import Enum
import numpy as np
import cv2

# ROS Messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Image

# Helpers
from pontus_autonomy.helpers.cv_threshold import CVThreshold

from pontus_autonomy.tasks.base_task import BaseTask

class State(Enum):
    Searching = 1
    Approaching = 2
    Passing_Through = 3


# Standard task to pass through gate
class GateTask(BaseTask):

    def __init__(self):
        super().__init__("Gate_Task")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        qos_profile = QoSProfile(
                          reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                          history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                          depth=1
                      )

        self.subscription = self.create_subscription(
            Image,
            '/pontus/camera_0/image_raw',
            self.camera_callback,
            qos_profile=qos_profile
        )

        self.state = State.Searching

        self.last_seen_gate_time = None
        self.gate_seen_timeout = Duration(seconds=1.0)
        self.drive_through_gate_time = Duration(seconds=3.0)

        # values can be near 0 or 180 so we have to threshold twice then merge
        self.gate_hsv1 = np.uint8([0, 255, 100])
        self.gate_hsv2 = np.uint8([180, 255, 100])

        # around 0
        self.lower1 = self.gate_hsv1 - np.uint8([0, 20, 20,])
        self.upper1 = self.gate_hsv1 + np.uint8([20, 0, 20,])

        # around 180
        self.lower2 = self.gate_hsv2 - np.uint8([20, 20, 20,])
        self.upper2 = self.gate_hsv2 + np.uint8([0, 0, 20,])

        self.cv_threshold = CVThreshold(self.lower1, self.upper1, self.lower2, self.upper2)

    def camera_callback(self, data):
        vel_msg = Twist()

        # Convert ROS Image message to OpenCV image
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        height, width, _ = hsv_frame.shape

        pos = None
        markers = self.cv_threshold.get_markers(hsv_frame)
        if len(markers) != 0:
            self.last_seen_gate_time = self.get_clock().now()
            pos = self.detect_gate(markers)


        match self.state:
            case State.Searching:
                vel_msg = self.search(pos, height, width)
            case State.Approaching:
                vel_msg = self.approach(pos, height, width)
            case State.Passing_Through:
                vel_msg = self.pass_through()
            case _:
                pass

        self.cmd_vel_pub.publish(vel_msg)

        self.debug_string = "Gate Task: " + str(self.state)

        mask3 = cv2.cvtColor(self.cv_threshold.mask, cv2.COLOR_GRAY2BGR)
        self.overlay_image = cv2.bitwise_and(image, mask3)
        self.publish_debug_info()

    def search(self, pos, height, width):
        vel_msg = Twist()

        # TODO: Make this better
        if pos:
            vel_msg.angular.z = 2.0 if pos[0] < width/2 else -2.0

            # The center of the gate is in the middle of the camera
            if abs(width / 2.0 - pos[0]) < width / 5.0:
                self.state = State.Approaching
        else:
            vel_msg.angular.z = 2.0

        return vel_msg

    def approach(self, pos, height, width):
        vel_msg = Twist()

        if (pos):
            # Pass a bit lower so we don't run into the signs
            if pos[1] < height/4.5:
                vel_msg.linear.z = 0.3
            elif pos[1] > height/4.0:
                vel_msg.linear.z = -0.3

            vel_msg.linear.x = 0.8
            # Rotate towards the center
            vel_msg.angular.z = 1.0 if pos[0] < width/2 else -1.0
        elif self.get_clock().now() - self.last_seen_gate_time < self.gate_seen_timeout:
            vel_msg.linear.x = 0.6
        else:
            self.state = State.Passing_Through

        return vel_msg

    def pass_through(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.5

        # TODO: if DVL is good we can use odom travel distance instead of travel time
        if self.get_clock().now() - self.last_seen_gate_time > self.drive_through_gate_time:
            self.complete(True)

        return vel_msg

    def detect_gate(self, markers):
        pos = [0.0, 0.0]

        # TODO: Improve this
        for marker in markers:
            pos[0] += marker[0]
            pos[1] += marker[1]

        pos[0] = pos[0] / len(markers)
        pos[1] = pos[1] / len(markers)

        return pos
