from scipy import stats
import numpy as np

class StereoGateDetction:
    def __init__(self):
        pass
    
    @staticmethod
    def detect_gate(point_cloud):
        new_point_cloud = StereoGateDetction.remove_floor(point_cloud)
        return new_point_cloud

    @staticmethod
    def remove_floor(point_cloud):
        # Since point cloud is in optical frame, y (optical frame) values correlate to the z (body frame)
        y_values = point_cloud[:, 1]
        unique, counts = np.unique(y_values, return_counts=True)
        top_5_y = unique[np.argsort(counts)[-5:]]
        floor_height = np.min(top_5_y)
        tolerance = 0.3
        filtered_points = point_cloud[floor_height - point_cloud[:, 1] > tolerance]
        return filtered_points

