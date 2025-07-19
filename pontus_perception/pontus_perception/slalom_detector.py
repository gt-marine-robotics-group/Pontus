import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image, CompressedImage
from pontus_msgs.msg import SlalomDetectorResults

import numpy as np
from pontus_perception.color_threshold_detection import (
    ColorThreshold, 
    BoundingBox, 
    get_color_threshold_contours,
    get_bounding_boxes,
    display_bounding_boxes
    )

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Iterator, List, Tuple
import operator


class SlalomSide(Enum):
    """
    Enum determining which side of the slalom task we attempt to stick to
    """
    RIGHT = 0
    LEFT = 1


@dataclass
class SlalomPair:
    """
    Dataclass to manage Slalom pair the AUV must traverse between
    """
    red_slalom: BoundingBox
    white_slalom: BoundingBox

    def midpoint(self) -> Tuple[int, int]:
        """X-coordinate halfway between the *centres* of the two poles."""
        red_center_x   = self.red_slalom.x   + self.red_slalom.w   / 2
        red_center_y   = self.red_slalom.y   + self.red_slalom.h   / 2

        white_center_x   = self.white_slalom.x   + self.white_slalom.w   / 2
        white_center_y   = self.white_slalom.y   + self.white_slalom.h   / 2

        midpoint_x = int((red_center_x + white_center_x) // 2)
        midpoint_y = int((red_center_y + white_center_y) // 2)
        return (midpoint_x, midpoint_y)

    def __iter__(self) -> Iterator[BoundingBox]:
        yield self.red_slalom
        yield self.white_slalom

    def __len__(self) -> int:
        return 2

    def __getitem__(self, idx: int) -> BoundingBox:
        if idx == 0:
            return self.red_slalom
        elif idx == 1:
            return self.white_slalom
        raise IndexError("SlalomPair has only two elements (0, 1)")

# Slalom Color Thresholds

# White Threshold
WHITE_THRESHOLD = ColorThreshold(lower_bound=np.array([0, 0, 200]),
                                 upper_bound=np.array([200, 100, 255]))

# Red Threshold (Need two due to wrapping effect with red in HSV)
RED_THRESHOLDS = [
    ColorThreshold(lower_bound=np.array([0, 120, 80]),
                    upper_bound=np.array([10, 255, 255])),

    ColorThreshold(lower_bound=np.array([170, 120, 80]),
                    upper_bound=np.array([180, 255, 255]))
]

WHITE_SLALOM_BOUDNING_BOX_COLOR = (255, 0, 0)
RED_SLALOM_BOUDNING_BOX_COLOR = (0, 255, 0)

MIDPOINT_DOT_DEBUG_COLOR = (255, 0, 255)
MIDPOINT_DOT_DEBUG_RADIUS = 3



def select_target_slaloms(
        white_slaloms: List[BoundingBox], 
        red_slaloms: List[BoundingBox],
        direction: SlalomSide = SlalomSide.RIGHT
        ) -> Optional[SlalomPair]:

    if not red_slaloms:
        return None

    red_slalom = max(red_slaloms, key=operator.attrgetter('w'))
    red_slalom_area = red_slalom.area

    if not white_slaloms:
        return None

    if direction is SlalomSide.RIGHT:
        white_candidates = [b for b in white_slaloms if b.x > red_slalom.x 
                            + red_slalom.w]
    else:
        white_candidates = [b for b in white_slaloms if b.x < red_slalom.x]

    # Finding the slalom with the closest area to the red slalom
    white_slalom = min(
        white_candidates,
        key=lambda sl: abs(red_slalom_area - sl.area),
        default=None
    )

    if white_slalom is None:
        return None

    return SlalomPair(red_slalom, white_slalom)


def get_slalom_pair(image_bgr: np.ndarray,
                    direction: SlalomSide) -> Optional[SlalomPair]:
    """
    Single call to detect the Slalom pair to navigate through
    """
    image_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

    whites = get_color_threshold_contours(image_hsv, WHITE_THRESHOLD)
    reds = get_color_threshold_contours(image_hsv, RED_THRESHOLDS)

    if not whites and not reds:
        return None

    white_boxes = get_bounding_boxes(whites, WHITE_SLALOM_BOUDNING_BOX_COLOR)
    red_boxes = get_bounding_boxes(reds, RED_SLALOM_BOUDNING_BOX_COLOR)

    return select_target_slaloms(white_boxes, red_boxes, direction)


# ------ Ros Detection Node ------

class SlalomDetector(Node):

    def __init__(self):
        super().__init__('slalom_detector')
        
        self.enabled = False
        self.cv_bridge = CvBridge()
        self.slalom_direction = SlalomSide.RIGHT

        self.image_sub = self.create_subscription(
            Image,
            'input',
            self.image_callback,
            10
        )

        self.slalom_pair_pub = self.create_publisher(
            SlalomDetectorResults,
            '/pontus/slalom_detector/results',
            10
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            '/pontus/slalom_detector/debug',
            10
        )

        self.debug_image_pub_compressed = self.create_publisher(
            CompressedImage,
            '/pontus/slalom_detector/debug/compressed',
            10
        )

        self.set_enabled_srv = self.create_service(
            SetBool,
            '/pontus/slalom_detector/set_enabled',
            self.set_enabled_callback
        )

        self.get_logger().info(f"Slalom Detector Node started")

    def set_enabled_callback(self, 
                                request: SetBool.Request, 
                                response: SetBool.Response) -> SetBool.Response:
        """
        Turn detector on only when doing the Slalom task as to not 
        take up excessive system resources.
        """
        self.enabled = request.data

        response.message = 'enabled' if self.enabled else 'disabled'
        self.get_logger().info(f"Slalom Detector {response.message}")
        return response

    def image_callback(self, image_msg: Image) -> None:
        """
        Take in an image and detect where the slalom pair is

        Arguments:
        ---
        image (Image): image to detect Slaloms in

        Return:
        ---
        None

        """

        if not self.enabled:
            return

        image_bgr = self.cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        pair: SlalomPair = get_slalom_pair(image_bgr, self.slalom_direction)

        detection_results = SlalomDetectorResults()
        detection_results.header = image_msg.header
        detection_results.slalom_detector_is_active = pair is not None

        if pair:
            red, white = list(pair)

            detection_results.red_slalom_found = red is not None
            detection_results.white_slalom_found = white is not None

            if red:
                detection_results.red_slalom_x = red.x
                detection_results.red_slalom_y = red.y
                detection_results.red_slalom_w = red.w
                detection_results.red_slalom_h = red.h

            if white:
                detection_results.white_slalom_x = white.x
                detection_results.white_slalom_y = white.y
                detection_results.white_slalom_w = white.w
                detection_results.white_slalom_h = white.h

            boxed_image = display_bounding_boxes(image_bgr, list(pair))
        else:
            boxed_image = image_bgr

        # ------ Publish Results ------
        self.slalom_pair_pub.publish(detection_results)


        # ------ Publish Debug Images ------

        # Add midpoint Marker:
        if pair:
            boxed_image = cv2.circle(boxed_image, pair.midpoint(),
                    MIDPOINT_DOT_DEBUG_RADIUS, MIDPOINT_DOT_DEBUG_COLOR, -1)

        ros_image = self.cv_bridge.cv2_to_imgmsg(boxed_image, 
                                                 encoding='bgr8')

        ros_image_compressed = self.cv_bridge.cv2_to_compressed_imgmsg(
            boxed_image)

        self.debug_image_pub.publish(ros_image)
        self.debug_image_pub_compressed.publish(ros_image_compressed)

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)

    node = SlalomDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
