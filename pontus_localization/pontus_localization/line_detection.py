import cv2
import numpy as np
from .pf_constants import *


class LineDetection:
    @staticmethod
    def get_lines(image, depth):
        # Detect edges for both images
        edges = cv2.Canny(image, 50, 150, apertureSize=3)

        line_pairs = np.array([])
        
        # Get lines
        lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=80)
        if lines is not None:
        
            line_pairs = []
            lines = lines[:, 0]
            test_lines = []
            # Calculate the maximum distance in the horizontal and vertical direction in meters
            # print("Depth:", depth)
            max_distance_horizontal = np.tan(np.radians(FOV_H/2)) * depth * 2
            # print("Distance horizontal:", max_distance_horizontal)
            max_distance_vertical = np.tan(np.radians(FOV_V/2)) * depth * 2
            for line in lines:
                # Convert the line to cartesian
                rho, theta = line
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                
                # Create line of y=mx+b form
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                
                # Calculate the projected point
                # This point is perpendicular to the line and goes through the center of the image
                v = np.array([x2 - x1, y2 - y1])
                p = np.array([image.shape[1]/2, image.shape[0]/2])
                p0 = np.array([x1, y1])
                projected_point = p0 + ((p - p0).dot(v) / np.linalg.norm(v)**2) * v
                
                # Convert to real world coordinates and center to the origin of the camera
                percentage_horizontal = (projected_point[0] - image.shape[1]/2) / image.shape[1]
                percentage_vertical = (image.shape[0]/2 - projected_point[1]) / image.shape[0]

                true_distance_horizontal = percentage_horizontal * max_distance_horizontal
                true_distance_vertical = percentage_vertical * max_distance_vertical
                
                # See if the potential point is really similar to any other points
                # If it is, disregard the line
                true_theta = (3 * np.pi / 2 - theta) % np.pi
                new_line = np.array([true_distance_horizontal, true_distance_vertical, true_theta])
                test_line = np.array([projected_point[0], projected_point[1], true_theta])
                if len(test_lines) > 0:
                    diff = np.square(test_lines - test_line)
                    if len(diff.shape) == 1:
                        total = np.sum(diff)
                    else:
                        total = np.sum(diff, axis = 1)
                    if np.min(total) < CUTOFF_THRESHOLD:
                        continue
                    line_pairs = np.vstack([line_pairs, new_line])
                    test_lines = np.vstack([test_lines, test_line])
                else:
                    line_pairs = new_line
                    test_lines = test_line
                cv2.circle(image, (int(projected_point[0]), int(projected_point[1])), 5, (0, 255, 0), -1)
                cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                # break
            cv2.imshow("edges", edges)
        cv2.circle(image, (int(image.shape[1]/2), int(image.shape[0]/2)), 5, (0, 255, 0), -1)
        cv2.imshow("original left", image)
        cv2.waitKey(1)

        return line_pairs