import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np

import tf2_ros
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # sets quality of service for nodes
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Image

class PrequalificationNode(Node):

    def __init__(self):
        super().__init__('prequalification_node')

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


        self.state = 0

        # Gate Variables
        # We are driving towards the gate
        self.approaching_gate = False

        # We are passing through the gate, keep driving fowards
        self.gate_timestamp = None

        # values can be near 0 or 180 so we have to threshold twice then merge
        self.gate_hsv1 = np.uint8([0, 255, 100])
        self.gate_hsv2 = np.uint8([180, 255, 100])

        # around 0
        self.lower1 = self.gate_hsv1 - np.uint8([0, 20, 20,])
        self.upper1 = self.gate_hsv1 + np.uint8([20, 0, 20,])

        # around 180
        self.lower2 = self.gate_hsv2 - np.uint8([20, 20, 20,])
        self.upper2 = self.gate_hsv2 + np.uint8([0, 0, 20,])


        # Vertical Marker Variables

        # We are driving past the marker, keep driving fowards
        self.drive_past_pole = False
        self.marker_timestamp = None

        # Circle around the pole
        self.looping_pole = False

        # how far to the side to keep the pole while driving toward it
        self.marker_offset = 10.0 # This will almost certainly need tuning depending on camera resolution and pole width

        self.marker_hsv = np.uint8([30, 255, 255])
        self.marker_lower = self.marker_hsv - np.uint8([10, 20, 20,])
        self.marker_upper= self.marker_hsv + np.uint8([10, 0, 0,])

        self.circle_radius = 0.5 # This is going to end up being very approximate
        self.circle_linear_velocity = 0.4
        self.circle_time = (2 * np.pi * self.circle_radius) / self.circle_linear_velocity # Time to complete full circle
        self.circle_angular_velocity = 2 * np.pi /  self.circle_time

    def camera_callback(self, data):
     
        msg = Twist()

        match self.state:
            # Pass through the gate the first time
            case 0:
                msg = self.gate_task(data)

            # Loop around the marker
            case 1:
                msg = self.vertical_marker_task(data)

            # Pass back through the gate
            case 2:
                msg = self.gate_task(data)

            # Do nothing if task complete
            case _:
                pass

        self.cmd_vel_pub.publish(msg)

    def detect_gate(self, hsv_frame):
        mask1 = cv2.inRange(hsv_frame, self.lower1, self.upper1)
        mask2 = cv2.inRange(hsv_frame, self.lower2, self.upper2)
        mask = mask1 + mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        by_size = sorted(contours, key = lambda c: cv2.contourArea(c))

        markers = []
        for contour in by_size:
            # only consider the 2 largest blobs
            # This can be tweaked if we want to consider the two side bars and the middle marker
            if len(markers) > 1:
                break

            M = cv2.moments(contour)

            # Only use contours with nonzero area
            if M["m00"] != 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                #cv2.circle(canvas, center, 2, (0,0,255), -1)
                markers.append(center)

        return markers

    def detect_marker(self, hsv_frame):
        mask = cv2.inRange(hsv_frame, self.marker_lower, self.marker_upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        by_size = sorted(contours, key = lambda c: cv2.contourArea(c))

        canvas = cv2.cvtColor(hsv_frame.copy(), cv2.COLOR_HSV2BGR)
        marker = None
        for contour in by_size:
            M = cv2.moments(contour)

            # Only use contours with nonzero area
            if M["m00"] != 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                return center

        # Didn't find the pole
        return None


    def gate_task(self, data):

        # Convert ROS Image message to OpenCV image
        hsv_frame = cv2.cvtColor(self.bridge.imgmsg_to_cv2(data), cv2.COLOR_RGB2HSV)
        height, width, _ = hsv_frame.shape

        markers = self.detect_gate(hsv_frame)

        msg = Twist()

        # Drive towards the center of the gate
        if len(markers) == 2:
            #print("Drive Toward Gate")
            self.approaching_gate = True
            self.gate_timestamp = self.get_clock().now()

            target_x = (markers[0][0] + markers[1][0]) / 2
            target_y = (markers[0][1] + markers[1][1]) / 2

            msg.linear.x = 1.0

            # Pass a bit lower so we don't run into the signs
            msg.linear.z = 0.5 if target_y < height/4 else -0.5

            # Rotate towards the center
            msg.angular.z = 1.0 if target_x < width/2 else -1.0

        else:
            # Just keep driving forward for a bit to pass through the gate
            if self.approaching_gate:

                if self.get_clock().now() - self.gate_timestamp > Duration(seconds=3.0):
                    print("Gate Complete")
                    self.state += 1
                    self.approaching_gate = False
                    self.gate_timestamp = None
                    msg.linear.x = 0.0
                else:
                    #print("Passing Through Gate")
                    msg.linear.x = 0.5

            # Look for the gate
            else:
                #print("Looking for Gate")

                # Spin towards gate
                if len(markers) == 1:
                   msg.angular.z = 2.0 if markers[0][0] < width/2 else -2.0
                else:
                    msg.angular.z = 2.0

        return msg


    def vertical_marker_task(self, data):

        # Convert ROS Image message to OpenCV image
        hsv_frame = cv2.cvtColor(self.bridge.imgmsg_to_cv2(data), cv2.COLOR_RGB2HSV)
        height, width, _ = hsv_frame.shape

        gate_markers = self.detect_gate(hsv_frame)

        msg = Twist()

        # If we can see the gate we have completed the vertical_marker_task
        if len(gate_markers):
            print("Marker Complete")
            self.state += 1
            self.looping_pole = False
        else:

            # Passed the pole
            if self.drive_past_pole and self.get_clock().now() - self.marker_timestamp > Duration(seconds=3.0):
                self.marker_timestamp = None
                self.drive_past_pole = False
                self.looping_pole = True

            # Drive past the pole so we don't hit it as we turn
            elif self.drive_past_pole:
                #print("Driving Past Marker")
                msg.linear.x = 0.5

            # Loop around the pole
            elif self.looping_pole:
                #print("Looping Around Marker")
                msg.linear.x = self.circle_linear_velocity
                msg.angular.z = 2.0 * self.circle_angular_velocity # Need to compensate for bad controls :(

            # Drive forward keeping the pole off to our side so we can pass it
            else:
                marker = self.detect_marker(hsv_frame)

                # Hopefully we just passed by the marker
                if marker == None:
                    self.drive_past_pole = True
                    self.marker_timestamp = self.get_clock().now()
                else:
                    #print("Driving to Marker")
                    msg.linear.x = 1.0
                    msg.linear.z = 0.5 if marker[1] < height/2 else -0.5

                    # Keep the marker far off to the side so we don't hit it
                    msg.angular.z = 0.4 if marker[0] < width/self.marker_offset else -0.4
                
        return msg


def main(args=None):
    rclpy.init(args=args)

    prequalification_node = PrequalificationNode()
    rclpy.spin(prequalification_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
