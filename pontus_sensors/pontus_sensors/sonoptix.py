#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import requests
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class SonoptixDriver(Node):
    
    def __init__(self):
        super().__init__('sonoptix_node')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(Image, '/pontus/sonar_0', qos_profile=qos_profile)

        self.cv_bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        sonar_ip = "192.168.1.211"
        rtsp_url = f'rtsp://{sonar_ip}:8554/raw'
        api_url = f'http://{sonar_ip}:8000/api/v1'
        
        requests.patch(api_url + '/transponder', json={
            "enable": True,
            "sonar_range": 3,
        })
        
        # Puts the stream into RTSP
        requests.put(api_url + '/streamtype', json={
            "value": 2,
        })

        self.cap = cv2.VideoCapture(rtsp_url)


    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            preprocessed_image = self.preprocess_frame(frame)
            ros_image = self.cv_bridge.cv2_to_imgmsg(preprocessed_image, encoding="bgr8")
            self.pub.publish(ros_image)
        else:
            self.get_logger().warn("Failed to read frame")


    def preprocess_frame(self, frame):
        
        return frame


def main(args=None):
    rclpy.init(args=args)
    sonoptix_node = SonoptixDriver()
    rclpy.spin(sonoptix_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
