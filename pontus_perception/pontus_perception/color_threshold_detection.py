import cv2
import numpy as np
import functools

from typing import List, Tuple
from dataclasses import dataclass


@dataclass
class ColorThreshold:
    """
    HSV Color value bounds
    """
    lower_bound: np.array
    upper_bound: np.array


@dataclass(slots=True)
class BoundingBox:
    x: int
    y: int
    w: int
    h: int

    color_bgr: Tuple[int, int, int]
    thickness: int = 2


def get_color_threshold_contours(
        hsv_image: np.ndarray, 
        thresholds: ColorThreshold | List[ColorThreshold]
        ) -> List[np.array]:
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

    if isinstance(thresholds, ColorThreshold):
        mask = cv2.inRange(hsv_image,
                           thresholds.lower_bound,
                           thresholds.upper_bound)

    else:
        mask_partials = (cv2.inRange(hsv_image, th.lower_bound, th.upper_bound)
                         for th in thresholds)

        mask = functools.reduce(cv2.bitwise_or, mask_partials)   

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                   cv2.CHAIN_APPROX_SIMPLE)

    return contours

def get_bounding_boxes(contours: List[np.ndarray], 
                       color_bgr = (255, 0, 0), 
                       line_thickness=2,
                       contour_area_threshold=150) -> List[BoundingBox]:
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
            bounding_boxes.append(BoundingBox(x, y, w, h,
                                              color_bgr, line_thickness))

    return bounding_boxes


def display_bounding_boxes(image: np.ndarray, 
                           bounding_boxes: List[BoundingBox]) -> np.ndarray:
    """
    Displaying given bounding boxes onto an image.
    """
    bounding_box_image = image.copy()
    
    for box in bounding_boxes:
        cv2.rectangle(bounding_box_image, (box.x, box.y), 
                      (box.x + box.w, box.y + box.h),
                      box.color_bgr, box.thickness)

    return bounding_box_image
