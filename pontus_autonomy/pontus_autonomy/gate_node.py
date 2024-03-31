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

class GateNode(Node):

    def __init__(self):
        super().__init__('gate_controller')

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

        self.task_completed = False

        # We are driving towards the gate
        self.approaching_gate = False

        # We are passing through the gate, keep driving fowards
        self.drive_time = None

        # values can be near 0 or 180 so we have to threshold twice then merge
        self.gate_hsv1 = np.uint8([0, 255, 100])
        self.gate_hsv2 = np.uint8([180, 255, 100])

        # around 0
        self.lower1 = self.gate_hsv1 - np.uint8([0, 20, 20,])
        self.upper1 = self.gate_hsv1 + np.uint8([20, 0, 20,])

        # around 180
        self.lower2 = self.gate_hsv2 - np.uint8([20, 20, 20,])
        self.upper2 = self.gate_hsv2 + np.uint8([0, 0, 20,])

        # positions of the red bars on the gate
        self.markers = []

    def camera_callback(self, data):
     
        # Convert ROS Image message to OpenCV image
        hsv_frame = cv2.cvtColor(self.bridge.imgmsg_to_cv2(data), cv2.COLOR_RGB2HSV)
        height, width, _ = hsv_frame.shape

        mask1 = cv2.inRange(hsv_frame, self.lower1, self.upper1)
        mask2 = cv2.inRange(hsv_frame, self.lower2, self.upper2)
        mask = mask1 + mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        by_size = sorted(contours, key = lambda c: cv2.contourArea(c))

        self.markers = []
        for contour in by_size:
            # only consider the 2 largest blobs
            # This can be tweaked if we want to consider the two side bars and the middle marker
            if len(self.markers) > 1:
                break

            M = cv2.moments(countour)

            # Only use contours with nonzero area
            if M["m00"] != 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                #cv2.circle(canvas, center, 2, (0,0,255), -1)
                self.markers.append(center)


        msg = Twist()

        # Drive towards the center of the gate
        if len(self.markers) == 2:
            print("Drive Mode")
            self.approaching_gate = True
            self.drive_time = self.get_clock().now()

            target_x = (self.markers[0][0] + self.markers[1][0]) / 2
            target_y = (self.markers[0][1] + self.markers[1][1]) / 2

            msg.linear.x = 1.0

            # Pass a bit lower so we don't run into the signs
            msg.linear.z = 0.5 if target_y < height/4 else -0.5

            # Rotate towards the center
            msg.angular.z = 1.0 if target_x < width/2 else -1.0

        else:
            # Just keep driving forward for a bit to pass through the gate
            if self.approaching_gate:

                if self.get_clock().now() - self.drive_time > Duration(seconds=0.5):
                    print("Task Complete")
                    self.task_completed = True
                    msg.linear.x = 0.0
                else:
                    print("Passing Through Gate")
                    msg.linear.x = 0.5

            # Look for the gate
            else:
                print("Looking for Gate")

                # Spin towards gate
                if len(self.markers) == 1:
                   msg.angular.z = 2.0 if self.markers[0][0] < width/2 else -2.0
                else:
                    msg.angular.z = 2.0

        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    gate_node = GateNode()
    rclpy.spin(gate_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
