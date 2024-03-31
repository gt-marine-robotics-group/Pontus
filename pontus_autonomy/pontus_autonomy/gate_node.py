import rclpy
from rclpy.node import Node
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

        # ROS infrastructure
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        ''' # Don't need this right now
        self.odom_sub = self.create_subscription(
          Odometry,
          '/pontus/odometry',
          self.odometry_callback,
          10)
        '''

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

        self.gate_color = cv2.cvtColor(np.uint8([[[100, 0, 0]]]), cv2.COLOR_RGB2HSV)
        self.upper_tolerance = cv2.cvtColor(np.uint8([[[130, 20, 20]]]), cv2.COLOR_RGB2HSV)
        self.lower_tolerance = cv2.cvtColor(np.uint8([[[70, 0, 0]]]), cv2.COLOR_RGB2HSV)


    def camera_callback(self, data):
     
        # Convert ROS Image message to OpenCV image
        hsv_frame = cv2.cvtColor(self.bridge.imgmsg_to_cv2(data), cv2.COLOR_RGB2HSV)

        # This is finding black not red :'(
        mask = cv2.inRange(hsv_frame, self.lower_tolerance, self.upper_tolerance)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        by_size = sorted(contours, key = lambda c: cv2.contourArea(c))

        canvas = cv2.cvtColor(hsv_frame.copy(), cv2.COLOR_HSV2BGR)

        print("blobs: ")
        for i in by_size:
            M = cv2.moments(i)
            # Mostly working, devided by 0 for some reason
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(canvas, center, 2, (0,0,255), -1)

            print(center)

        print("-----------------------")

        cv2.imshow("test", canvas)
        cv2.waitKey(0)
        
    '''
    def odometry_callback(self, msg):

        # Get the current velocities from odometry
        v_linear = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        v_angular = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

        # Compute the error between desired velocity and current velocity
        linear_err = self.cmd_linear - v_linear
        angular_err = self.cmd_angular - v_angular

        # Compute and publish the body accelerations
        msg = Twist()
        msg.linear.x = self.pid_linear[0](linear_err[0], self.get_clock().now() - self.prev_time)
        msg.linear.y = self.pid_linear[1](linear_err[1], self.get_clock().now() - self.prev_time)
        msg.linear.z = self.pid_linear[2](linear_err[2], self.get_clock().now() - self.prev_time)

        msg.angular.x = self.pid_angular[0](angular_err[0], self.get_clock().now() - self.prev_time)
        msg.angular.y = self.pid_angular[1](angular_err[1], self.get_clock().now() - self.prev_time)
        msg.angular.z = self.pid_angular[2](angular_err[2], self.get_clock().now() - self.prev_time)

        self.cmd_accel_pub.publish(msg)

        self.prev_time = self.get_clock().now()
    '''

def main(args=None):
    rclpy.init(args=args)

    gate_node = GateNode()
    rclpy.spin(gate_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
