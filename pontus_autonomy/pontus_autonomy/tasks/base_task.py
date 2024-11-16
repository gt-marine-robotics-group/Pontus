import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.task import Future

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import String

class BaseTask(Node):

    # TODO: make state be an abstract property and set up automated state publishing

    debug_string = ""
    overlay_image = None # OpenCV BGR8 Mat image for providing debug information

    def __init__(self, name):
        super().__init__(name)

        self.task_future = rclpy.task.Future()

        self.debug_string_publisher = self.create_publisher(
            String,
            'pontus/debug_string',
            1
        )
        self.bridge = CvBridge()
        self.overlay_publisher = self.create_publisher(
            Image,
            'pontus/overlay_image',
            1
        )

    def wait_for_task(self):
        return self.task_future

    def complete(self, success):
        self.task_future.set_result(success)

    def publish_debug_info(self, image = None):
        self.publish_overlay(image)
        self.publish_debug_string()

    def publish_debug_string(self):
        if self.debug_string == "":
            # No string to publish
            return

        msg = String()
        msg.data = self.debug_string
        self.debug_string_publisher.publish(msg)

        self.debug_string  = "" # Clear the string so it is ready to be used again

    def publish_overlay(self, image = None):
        if image is None:
            if self.overlay_image is None:
                # No image to publish
                return
            image = self.overlay_image

        if self.count_subscribers('pontus/overlay_image') == 0:
            # No reason to publish the image
            return

        self.overlay_publisher.publish(self.bridge.cv2_to_imgmsg(image,  encoding="bgr8"))
