import open3d as o3d
import numpy as np

np.set_printoptions(suppress=True)

class StereoGateDetectionParams:
    def __init__(self, 
                 pool_depth, 
                 gate_size, 
                 gate_size_tolerance, 
                 remove_statistical_outlier_nb_neighbors,
                 remove_statistical_outlier_std_ratio,
                 dbscan_eps,
                 dbscan_min_points,
                 remove_floor_tolerance,
                 side_pole_height_min,
                 side_pole_height_max
                 ):
        self.pool_depth = pool_depth
        self.gate_size = gate_size
        self.gate_size_tolerance = gate_size_tolerance
        self.remove_statistical_outlier_nb_neighbors = remove_statistical_outlier_nb_neighbors
        self.remove_statistical_outlier_std_ratio = remove_statistical_outlier_std_ratio
        self.dbscan_eps = dbscan_eps
        self.dbscan_min_points = dbscan_min_points 
        self.remove_floor_tolerance = remove_floor_tolerance
        self.side_pole_height_min = side_pole_height_min
        self.side_pole_height_max = side_pole_height_max

class StereoGateDetection:
    def __init__(self):
        pass
    
    @staticmethod
    def detect_gate(point_cloud, current_depth, params: StereoGateDetectionParams):
        # Ground filter
        new_point_cloud = StereoGateDetection.remove_floor(point_cloud, current_depth, params.pool_depth, params.remove_floor_tolerance)
        
        # Convert to open3d point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(new_point_cloud)
        
        # Remove outliers
        filtered_pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=params.remove_statistical_outlier_nb_neighbors,
            std_ratio=params.remove_statistical_outlier_std_ratio
        )

        filtered_point_cloud_numpy = np.asarray(filtered_pcd.points)

        # DBSCAN clustering
        labels = np.array(filtered_pcd.cluster_dbscan(eps=params.dbscan_eps, min_points=params.dbscan_min_points, print_progress=False))
        
        # Get clusters
        clusters = []
        for label in np.unique(labels):
            if label == -1:
                continue
            cluster = filtered_point_cloud_numpy[labels == label]
            clusters.append(cluster)
        
        # Sort the centroids of the clusters by their distance
        centroids = []
        distances = []
        for cluster in clusters:
            centroid = np.mean(cluster, axis=0)
            centroids.append(centroid)
            distance = np.linalg.norm(centroid)
            distances.append(distance)
        distances = np.array(distances)
        centroids = np.array(centroids)
        sorted_indices = np.argsort(distances)
        
        # Process each cluster to see which is similar to a gate
        valid_clusters = []
        for cluster_indx in sorted_indices:
            cluster_array = np.array(clusters[cluster_indx])
            pole_detection_array = StereoGateDetection.extract_side_poles(cluster_array, params.side_pole_height_min, params.side_pole_height_max)
            if len(pole_detection_array) == 0:
                continue

            for detection in pole_detection_array:
                valid_clusters.append(detection)
        
        for first in range(len(valid_clusters) - 1):
            for second in range(first + 1, len(valid_clusters)):
                first_cluster_centroid = valid_clusters[first]
                second_cluster_centroid = valid_clusters[second]
                if StereoGateDetection.gate_like(first_cluster_centroid, second_cluster_centroid, params.gate_size, params.gate_size_tolerance, filtered_point_cloud_numpy):
                    left_cluster_centroid = first_cluster_centroid if first_cluster_centroid[1] > second_cluster_centroid[1] else second_cluster_centroid
                    right_cluster_centroid = first_cluster_centroid if first_cluster_centroid[1] < second_cluster_centroid[1] else second_cluster_centroid
                    return left_cluster_centroid, right_cluster_centroid, filtered_point_cloud_numpy

        return None, None, filtered_point_cloud_numpy

    @staticmethod
    def gate_like(first_cluster_centroid, second_cluster_centroid, gate_size, gate_size_tolerance, filtered_point_cloud_numpy):
        # If the two poles are not the expected width apart, its probably not a gate
        print(np.linalg.norm(first_cluster_centroid[:2] - second_cluster_centroid[:2]))
        if abs(np.linalg.norm(first_cluster_centroid[:2] - second_cluster_centroid[:2]) - gate_size) > gate_size_tolerance:
            return False
        
        # Sample in between the poles, there should be little to nothing there
        left_cluster = first_cluster_centroid if first_cluster_centroid[1] > second_cluster_centroid[1] else second_cluster_centroid
        right_cluster = first_cluster_centroid if first_cluster_centroid[1] < second_cluster_centroid[1] else second_cluster_centroid
        # Angle of the gate
        angle = np.arctan2(left_cluster[0] - right_cluster[0], left_cluster[1] - right_cluster[1])
        print(angle)

        # Sample points in between the two poles
        in_between_points = filtered_point_cloud_numpy[
            (filtered_point_cloud_numpy[:, 1] < left_cluster[1] - 0.15) & 
            (filtered_point_cloud_numpy[:, 1] > right_cluster[1] + 0.15) &
            (filtered_point_cloud_numpy[:, 2] < left_cluster[2] + 0.15) &
            (filtered_point_cloud_numpy[:, 2] > left_cluster[2] - 0.15) &
            (filtered_point_cloud_numpy[:, 0] > min(left_cluster[0], right_cluster[0]) - 0.1) &
            (filtered_point_cloud_numpy[:, 0] < max(left_cluster[0], right_cluster[0]) + 0.1)
        ]
        print("In between points: ", len(in_between_points))
        return True


    @staticmethod
    def remove_floor(point_cloud, current_depth, pool_depth, tolerance = 0.4):
        # Since point cloud is in optical frame, y (optical frame) values correlate to the z (body frame)
        filtered_points = point_cloud[current_depth - pool_depth + point_cloud[:, 2] > tolerance]
        return filtered_points
    

    @staticmethod
    def extract_side_poles(cluster, side_pole_height_min, side_pole_height_max):
        ret_array = []
        if len(cluster) == 0:
            return ret_array
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cluster)
        min_z = np.min(cluster[:, 2])
        max_z = np.max(cluster[:, 2])
        height = max_z - min_z
        # If not pole shape, return
        if height < side_pole_height_min or height > side_pole_height_max:
            return ret_array
        
        # Remove top beam to isolate poles
        filtered_points = cluster[cluster[:, 2] < (max_z + min_z) / 2.0]
        # Rerun clustering to get true pole centroids
        pcd.points = o3d.utility.Vector3dVector(filtered_points)
        labels = np.array(pcd.cluster_dbscan(eps=0.2, min_points=10, print_progress=False))

        # If only one cluster, return it
        unique_labels, counts = np.unique(labels[labels != -1], return_counts=True)
        if len(unique_labels) < 1:
            return ret_array
        sorted_indices = np.argsort(counts)[::-1]
        largest_clusters = unique_labels[sorted_indices[:2]]
        cluster1_points = filtered_points[labels == largest_clusters[0]]
        ret_array.append(np.mean(cluster1_points, axis=0))
        if len(unique_labels) < 2:
            return ret_array
        
        # Else, return the two largest clusters
        cluster2_points = filtered_points[labels == largest_clusters[1]]

        # If the second cluster is roughly the same size as the first, return both
        # In this case, our cluster may contain the entire gate
        if len(cluster2_points) > 0.5 * len(cluster1_points):
            ret_array.append(np.mean(cluster2_points, axis=0))
        
        return ret_array
        
        

