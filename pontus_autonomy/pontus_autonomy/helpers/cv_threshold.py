import cv2
import numpy as np

from typing import List, Tuple
from dataclasses import dataclass


@dataclass
class ColorThreshold:
    lower_bound: np.array
    upper_bound: np.array


@dataclass(slots=True)
class BoundingBox:
    x: int
    y: int
    w: int
    h: int

    color: Tuple[int, int, int]
    thickness: int = 2


def get_color_threshold_contours(image: np.ndarray, 
                                 thresholds: ColorThreshold | List[ColorThreshold]) -> List[np.array]:
    """ 
    Function that finds contours of specified color thresholds:

    Arguments:
    ---
    image: np.ndarray   
        - The image itself either from a live feed or still image

    threshold: List[ColorThreshold]
        - Thresholds to mask the image with. The results of individual 
          thresholds will be OR'd together, combining contours into one

    Ouputs:
    ---
    contours: List[np.ndarray]  
        - List of all the contours found in the image within the color threshold
    """

    hsv_image: np.ndarray = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask_partials: List[np.ndarray] = []

    if isinstance(thresholds, ColorThreshold):
        mask = cv2.inRange(hsv_image,
                           thresholds.lower_bound,
                           thresholds.upper_bound)

    else:

        for threshold in thresholds:
            mask_partials.append(cv2.inRange(hsv_image, threshold.lower_bound, 
                                            threshold.upper_bound)) 

        mask = (mask_partials[0] if len(mask_partials) == 1 
                else cv2.bitwise_or(*mask_partials))
    
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contours


def get_bounding_boxes(contours: List[np.ndarray], 
                       color: Tuple[int, int, int] = (255, 0, 0), 
                       line_thickness=2,
                       contour_area_threshold=500) -> List[BoundingBox]:
    """
    Creates the bounding boxes from a given list of contours.

    Argumenst:
    contours:   List of contours
    color:      Color to make the bounding box in RGB 
    contour_area_threshold: reject contours smaller than this value
    """
    bounding_boxes: List[BoundingBox] = []

    for contour in contours:

        if cv2.contourArea(contour) > contour_area_threshold:
            x, y, w, h = cv2.boundingRect(contour)
            bounding_boxes.append(BoundingBox(x, y, w, h, color, line_thickness))

    return bounding_boxes


def display_bounding_boxes(image: np.ndarray, bounding_boxes: List[BoundingBox]) -> np.ndarray:
    """
    Displaying given bounding boxes onto an image.
    """
    bounding_box_image = image.copy()
    
    for box in bounding_boxes:
        cv2.rectangle(bounding_box_image, (box.x, box.y), (box.x + box.w, box.y + box.h),
                      box.color, box.thickness)

    return bounding_box_image




# import cv2
# import numpy as np


# class CVThreshold:
#     def __init__(self, lower1: int, upper1: int, lower2: int = None, upper2: int = None):
#         self.lower1 = lower1
#         self.upper1 = upper1

#         self.lower2 = lower2
#         self.upper2 = upper2

#         self.masked = None

#     def mask_image(self, image: np.ndarray) -> np.ndarray:
#         """
#         Apply a mask on an image.

#         Args:
#         ----
#         image (np.ndarray): the image we want to mask

#         Return:
#         ------
#         np.ndarray: the masked image

#         """
#         self.mask = cv2.inRange(image, self.lower1, self.upper1)
#         if self.lower2 is not None and self.upper2 is not None:
#             self.mask += cv2.inRange(image, self.lower2, self.upper2)

#         return self.mask

#     def get_markers(self, image: np.ndarray) -> list[tuple[int, int]]:
#         """
#         Return a list of contours based on the bounds.

#         Args:
#         ----
#         image (np.ndarray): the image we want to get the markers from

#         Return:
#         ------
#         list[tuple[int, int]]: list of tuples representing the markers

#         """
#         self.mask_image(image)

#         contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#         by_size = sorted(contours, key=lambda c: cv2.contourArea(c))

#         markers = []
#         for contour in by_size:
#             M = cv2.moments(contour)

#             # Only use contours with nonzero area
#             if M['m00'] != 0:
#                 center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
#                 markers.append(center)

#         return markers
