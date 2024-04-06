import cv2
import numpy as np

class LineDetection:
    @staticmethod
    def detect_lines(left_image, right_image):
        cv2.imshow("original left", left_image)
        cv2.imshow("original right", right_image)
        cv2.waitKey(1)