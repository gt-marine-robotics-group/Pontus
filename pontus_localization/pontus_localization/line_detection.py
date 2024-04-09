import cv2
import numpy as np
from .pf_constants import *


class LineDetection:
    @staticmethod
    def get_lines(left_image, depth):
        # Detect edges for both images
        edges = cv2.Canny(left_image, 50, 150, apertureSize=3)

        # Get lines
        lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=100)
        
        line_pairs = []
        lines = lines[:, 0]

        max_distance_horizontal = np.tan(np.radians(FOV_H/2)) * depth
        max_distance_vertical = np.tan(np.radians(FOV_V/2)) * depth
        for line in lines:
            rho, theta = line
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            percentage_horizontal = x0 / left_image.size[1]
            percentage_vertical = y0 / left_image.size[0]

            true_distance_horizontal = percentage_horizontal * max_distance_horizontal
            true_distance_vertical = percentage_vertical * max_distance_vertical
            
            line_pairs.append([(true_distance_horizontal, true_distance_vertical, theta)])
        return line_pairs
        cv2.imshow("original left", left_image)
        cv2.waitKey(1)