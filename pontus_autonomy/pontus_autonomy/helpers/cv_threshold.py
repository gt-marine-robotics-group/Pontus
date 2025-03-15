import cv2
import numpy as np


class CVThreshold:
    def __init__(self, lower1: int, upper1: int, lower2: int = None, upper2: int = None):
        self.lower1 = lower1
        self.upper1 = upper1

        self.lower2 = lower2
        self.upper2 = upper2

        self.masked = None

    def mask_image(self, image: np.ndarray) -> np.ndarray:
        """
        Apply a mask on an image.

        Args:
        ----
            image (np.ndarray): the image we want to mask

        Return:
        ------
            np.ndarray: the masked image

        """
        self.mask = cv2.inRange(image, self.lower1, self.upper1)
        if self.lower2 is not None and self.upper2 is not None:
            self.mask += cv2.inRange(image, self.lower2, self.upper2)

        return self.mask

    def get_markers(self, image: np.ndarray) -> list[tuple[int, int]]:
        """
        Return a list of contours based on the bounds.

        Args:
        ----
            image (np.ndarray): the image we want to get the markers from

        Return:
        ------
            list[tuple[int, int]]: list of tuples representing the markers

        """
        self.mask_image(image)

        contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        by_size = sorted(contours, key=lambda c: cv2.contourArea(c))

        markers = []
        for contour in by_size:
            M = cv2.moments(contour)

            # Only use contours with nonzero area
            if M["m00"] != 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                markers.append(center)

        return markers
