#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import requests
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan

class SonoptixDriver(Node):
    
    def __init__(self):
        super().__init__('sonoptix_driver')

        # Ros params
        self.declare_parameter('ip_address', "192.168.1.211")
        self.declare_parameter('range', 15)
        self.declare_parameter('gain', 1)
        self.declare_parameter('angle_range', 2.09)
        self.declare_parameter('intensity_threshold', 127)

        self.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.range = self.get_parameter('range').get_parameter_value().integer_value
        self.gain = self.get_parameter('gain').get_parameter_value().integer_value
        self.angle_range = self.get_parameter('angle_range').get_parameter_value().double_value
        self.intensity_threshold = self.get_parameter('intensity_threshold').get_parameter_value().integer_value

        self.get_logger().info(f"{self.ip_address}")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.pub = self.create_publisher(
            Image,
            '/pontus/sonar_0/image_debug',
            qos_profile=qos_profile
        )

        self.laser_scan_publish = self.create_publisher(
            LaserScan,
            '/pontus/sonar_0',
            qos_profile=qos_profile,
        )

        self.cv_bridge = CvBridge()
        # self.timer = self.create_timer(0.1, self.timer_callback)
        
        rtsp_url = f'rtsp://{self.ip_address}:8554/raw'
        api_url = f'http://{self.ip_address}:8000/api/v1'
        
        requests.patch(api_url + '/transponder', json={
            "enable": True,
            "sonar_range": self.range,
        })
        
        # TODO: Make sure that these are correct
        requests.patch(api_url + '/config', json={
            "contrast": 0,
            "gain": self.gain,
            "mirror_image": True,
            "autodetect_orientation": 0
        })

        # Puts the stream into RTSP
        requests.put(api_url + '/streamtype', json={
            "value": 2,
        })

        self.cap = cv2.VideoCapture(rtsp_url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.get_sonoptix()


    def get_sonoptix(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                cv2.imshow("Frame", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                preprocessed_image = self.preprocess_frame(frame)
                # preprocessed_image = frame
                ros_image = self.cv_bridge.cv2_to_imgmsg(preprocessed_image, encoding="bgr8")
                # laser_scan = self.convert_to_laserscan(preprocessed_image)
                # self.laser_scan_publish.publish(laser_scan)
                self.pub.publish(ros_image)
            else:
                self.get_logger().warn("Failed to read frame")


    def convert_to_laserscan(self, image):
        num_rows, num_cols = image.shape
        laser_msg = LaserScan()
        laser_msg.header.frame_id = "sonar_0"
        laser_msg.header.stamp = self.get_clock().now().to_msg()
        laser_msg.angle_min = -self.angle_range / 2
        laser_msg.angle_max = self.angle_range / 2
        laser_msg.angle_increment = self.angle_range / num_cols
        laser_msg.range_min = 0.2
        laser_msg.range_max = self.range
        ranges = []
        for col in range(0, num_cols):
            for row in range(0, num_rows + 1):
                # If we reach the end of the column and still ahve yet to see anything,
                # set the range to the max value
                if row == num_rows:
                    ranges.append(float(self.range)) 
                elif image[row][col] == 255:
                    ranges.append(float(row / num_rows * self.range))
                    break
        laser_msg.ranges = ranges
        return laser_msg


    def preprocess_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        masked = cv2.inRange(gray, 3, 255)
        # masked = cv2.inRange(gray, self.intensity_threshold, 255)
        color = cv2.cvtColor(masked, cv2.COLOR_GRAY2BGR)
        return color


def main(args=None):
    rclpy.init(args=args)
    sonoptix_node = SonoptixDriver()
    rclpy.spin_once(sonoptix_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
