import cv2
import numpy as np
from .pf_constants import *


class LineDetection:
    @staticmethod
    def get_lines(image, depth):
        # Detect edges for both images
        edges = cv2.Canny(image, 50, 150, apertureSize=3)
        image_copy = image.copy()
        line_pairs = np.array([])
        
        # Get lines
        # lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=80)
        lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=70)
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
                
                # Calculate the gradient perpendicular to the line
                # This will tell us which part of the line we are looking at
                gradient_theta = (theta + np.pi/2) % np.pi
                
                # Need to see if its either -gradient_theta or gradient_theta
                # Sample area
                true_theta = (3 * np.pi / 2 - theta) % np.pi
                total_left = []
                total_right = []
                max_image_size = max(image.shape)
                sample_area = int(max_image_size / 7)
                print(sample_area)
                
                # Sample one side
                for sample in range(1, 20):
                    sample_x = np.linspace((projected_point[0] + sample * np.sin(true_theta)) - max_image_size * np.sin(-theta), 
                                        (projected_point[0] + sample * np.sin(true_theta)) + max_image_size * np.sin(-theta), sample_area)
                    sample_y = np.linspace((projected_point[1] + sample * np.cos(true_theta)) - max_image_size * np.cos(-theta), 
                                        (projected_point[1] + sample * np.cos(true_theta)) + max_image_size * np.cos(-theta), sample_area)
                    
                    for sample_x, sample_y in zip(sample_x, sample_y):
                        if int(sample_y) < 0 or int(sample_y) >= image.shape[0] or int(sample_x) < 0 or int(sample_x) >= image.shape[1]:
                            continue
                        total_left.append(image_copy[int(sample_y), int(sample_x)])
                        cv2.circle(image, (int(sample_x), int(sample_y)), 2, (255, 0, 0), -1)   
                        # pass
                        
                # Sample other side
                for sample in range(1,20):
                    sample_x = np.linspace((projected_point[0] - sample * np.sin(true_theta)) - max_image_size * np.sin(-theta), 
                                        (projected_point[0] - sample * np.sin(true_theta)) + max_image_size * np.sin(-theta), sample_area)
                    sample_y = np.linspace((projected_point[1] - sample * np.cos(true_theta)) - max_image_size * np.cos(-theta), 
                                        (projected_point[1] - sample * np.cos(true_theta)) + max_image_size * np.cos(-theta), sample_area)
                    
                    for sample_x, sample_y in zip(sample_x, sample_y):
                        if int(sample_y) < 0 or int(sample_y) >= image.shape[0] or int(sample_x) < 0 or int(sample_x) >= image.shape[1]:
                            continue
                        total_right.append(image_copy[int(sample_y), int(sample_x)])
                        cv2.circle(image, (int(sample_x), int(sample_y)), 2, (127, 0, 0), -1)
                        # pass
                total_left_mean = np.mean(total_left)
                total_right_mean = np.mean(total_right)
                # print("Total left mean:", total_left_mean, "Total right mean:", total_right_mean)
                
                if total_left_mean > total_right_mean:
                    sampled_point = ((projected_point[0] + 5 * np.sin(true_theta)), (projected_point[1] + 5 * np.cos(true_theta)))
                    gradient_theta = np.arctan2(-(sampled_point[1] - projected_point[1]), (sampled_point[0] - projected_point[0]))
                else:
                    sampled_point = ((projected_point[0] - 5 * np.sin(true_theta)), (projected_point[1] - 5 * np.cos(true_theta)))
                    gradient_theta = np.arctan2(-(sampled_point[1] - projected_point[1]), (sampled_point[0] - projected_point[0]))
                # print("Gradient_theta", gradient_theta, "True theta:", true_theta)
                
                # Convert to real world coordinates and center to the origin of the camera
                percentage_horizontal = (projected_point[0] - image.shape[1]/2) / image.shape[1]
                percentage_vertical = (image.shape[0]/2 - projected_point[1]) / image.shape[0]

                true_distance_horizontal = percentage_horizontal * max_distance_horizontal
                true_distance_vertical = percentage_vertical * max_distance_vertical
                
                # See if the potential point is really similar to any other points
                # If it is, disregard the line
                true_theta = (3 * np.pi / 2 - theta) % np.pi
                new_line = np.array([true_distance_horizontal, true_distance_vertical, true_theta, gradient_theta])
                test_line = np.array([projected_point[0], projected_point[1], true_theta, gradient_theta])
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