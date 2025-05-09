#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import requests
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from typing import Optional, List
import numpy as np


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
        self.intensity_threshold = self.get_parameter('intensity_threshold')\
            .get_parameter_value().integer_value

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

        self.cv_bridge = CvBridge()
        # self.timer = self.create_timer(0.1, self.timer_callback)

        rtsp_url = f'rtsp://{self.ip_address}:8554/raw'
        api_url = f'http://{self.ip_address}:8000/api/v1'

        requests.patch(api_url + '/transponder', json={
            "enable": True,
            "sonar_range": self.range,
        })

        # Puts the stream into RTSP
        requests.put(api_url + '/streamtype', json={
            "value": 2,
        })
        # TODO: Make sure that these are correct
        requests.patch(api_url + '/config', json={
            "contrast": 0,
            "gain": 20.0,
            "mirror_image": True,
            "autodetect_orientation": 0
        })

        self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.get_sonoptix()

    def get_sonoptix(self) -> None:
        """
        Loop to get sonoptix image.

        Args:
        ----
        None

        Return:
        ------
        None

        """
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                cv2.imshow("Frame", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                print(frame)
                preprocessed_image = self.preprocess_frame(frame)
                # preprocessed_image = frame
                ros_image = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.pub.publish(ros_image)
            else:
                self.get_logger().warn("Failed to read frame")

    def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        Preprocess the image before using.

        Args:
        ----
        frame (np.ndarray): the frame we want to apply the preprocessing

        Return:
        ------
        np.ndarray: the preprocessed iamge

        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # masked = cv2.inRange(gray, 3, 255)
        # masked = cv2.inRange(gray, self.intensity_threshold, 255)
        # color = cv2.cvtColor(masked, cv2.COLOR_GRAY2BGR)
        # return gray


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    sonoptix_node = SonoptixDriver()
    rclpy.spin_once(sonoptix_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
