import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from enum import Enum
import numpy as np
import cv2
from cv_bridge import CvBridge

# ROS Messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Image

# Helpers
from pontus_autonomy.helpers.cv_threshold import CVThreshold

from pontus_autonomy.tasks.base_task import BaseTask

class State(Enum):
    Approaching = 1
    DrivingPast = 2
    Looping = 3


# Standard task to loop around vertical marker
class VerticalMarkerTaskOld(BaseTask):

    def __init__(self):
        super().__init__("Vertical_Marker_Task")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        qos_profile = QoSProfile(
                          reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                          history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                          depth=1
                      )

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/pontus/camera_0/image_raw',
            self.camera_callback,
            qos_profile=qos_profile
        )

        self.state = State.Approaching

        self.circle_radius = 0.5 # This is going to end up being very approximate
        self.circle_linear_velocity = 0.4
        self.circle_time = (2 * np.pi * self.circle_radius) / self.circle_linear_velocity # Time to complete full circle
        self.circle_angular_velocity = 2 * np.pi /  self.circle_time

        self.marker_offset = 10.0 # This will almost certainly need tuning depending on camera resolution and pole width

        self.last_seen_marker_time = None
        self.marker_seen_timeout = Duration(seconds=1.0)
        self.drive_past_marker_time = Duration(seconds=3.0)

        self.marker_hsv = np.uint8([90, 128, 0])
        self.marker_lower = self.marker_hsv - np.uint8([90, 128, 0,])
        self.marker_upper= self.marker_hsv + np.uint8([90, 127, 5,])

        self.cv_threshold = CVThreshold(self.marker_lower, self.marker_upper)

        # Gate Values
        # values can be near 0 or 180 so we have to threshold twice then merge
        self.gate_hsv1 = np.uint8([0, 255, 100])
        self.gate_hsv2 = np.uint8([180, 255, 100])

        # around 0
        self.lower1 = self.gate_hsv1 - np.uint8([0, 20, 20,])
        self.upper1 = self.gate_hsv1 + np.uint8([20, 0, 20,])

        # around 180
        self.lower2 = self.gate_hsv2 - np.uint8([20, 20, 20,])
        self.upper2 = self.gate_hsv2 + np.uint8([0, 0, 20,])

        self.gate_threshold = CVThreshold(self.lower1, self.upper1, self.lower2, self.upper2)

    def camera_callback(self, data):
        vel_msg = Twist()

        # Convert ROS Image message to OpenCV image
        hsv_frame = cv2.cvtColor(self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8'), cv2.COLOR_BGR2HSV)

        match self.state:
            case State.Approaching:
                # Scan for Marker
                vel_msg = self.approach(hsv_frame)
            case State.DrivingPast:
                vel_msg = self.drive_past()
            case State.Looping:
                vel_msg = self.loop(hsv_frame)
            case _:
                pass

        self.cmd_vel_pub.publish(vel_msg)

        self.debug_string = "Vertical Marker Task: " + str(self.state)

        self.overlay_image = cv2.cvtColor(self.cv_threshold.mask, cv2.COLOR_GRAY2BGR)
        self.publish_debug_info()


    def approach(self, hsv_frame):
        vel_msg = Twist()

        height, width, _ = hsv_frame.shape
        pos = None
        markers = self.cv_threshold.get_markers(hsv_frame)
        if len(markers) != 0:
            self.last_seen_marker_time = self.get_clock().now()
            pos = self.detect_marker(markers)

        vel_msg.linear.x = 1.0

        if (pos):
            # Keep the marker far off to the side so we don't hit it
            vel_msg.angular.z = 0.4 if pos[0] < width/self.marker_offset else -0.4

        elif self.get_clock().now() - self.last_seen_marker_time > self.marker_seen_timeout:
            self.state = State.DrivingPast

        return vel_msg


    def drive_past(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.5

        # TODO: if DVL is good we can use odom travel distance instead of travel time
        if self.get_clock().now() - self.last_seen_marker_time > self.drive_past_marker_time:
            self.state = State.Looping

        return vel_msg


    def loop(self, hsv_frame):
        # TODO: if DVL is good it might be better to do a proper 180 degree loop
        # around the marker and then just drive straight for a set distance instead of searching
        # for the gate in this task again

        vel_msg = Twist()

        # Scan for Gate
        pos = None
        markers = self.gate_threshold.get_markers(hsv_frame)
        if len(markers) != 0:
            self.complete(True)

        vel_msg.linear.x = self.circle_linear_velocity
        vel_msg.angular.z = 2.0 * self.circle_angular_velocity # Need to compensate for bad controls :(
        return vel_msg


    def detect_marker(self, markers):
        pos = [0.0, 0.0]

        # TODO: Improve this
        for marker in markers:
            pos[0] += marker[0]
            pos[1] += marker[1]

        pos[0] = pos[0] / len(markers)
        pos[1] = pos[1] / len(markers)

        return pos
