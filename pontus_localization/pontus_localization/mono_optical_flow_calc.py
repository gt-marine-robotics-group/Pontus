import numpy as np
import cv2
from scipy import spatial

class MonoOpticalFlowCalculations():
    def __init__(self):
        # self.n_features = 0
        # self.lk_params = dict(winSize  = (21,21), criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        # self.focal_length = 372
        # self.principal_point = (160, 120)
        # self.fast_detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
        # self.curr_pos = np.zeros(shape=(3,3))
        # self.curr_rotation_matrix = np.zeros(shape=(3,3))
        # self.MIN_FEATURES = 2000
        # self.odometry_calculator = cv2.rgbd.Odometry_create()
        pass
            

    def calculate(self, image_count, old_frame, new_frame, old_time, curr_time):
        # if self.n_features <= self.MIN_FEATURES:
        #     p0 = self.fast_detector.detect(old_frame)
        #     self.old_points = np.array([x.pt for x in p0], dtype=np.float32).reshape(-1, 1, 2)

        # self.new_points, status, error = cv2.calcOpticalFlowPyrLK(old_frame, new_frame, self.old_points, None, **self.lk_params)

        # self.good_new = self.new_points[status==1]
        # self.good_old = self.old_points[status==1]

        # E, _ = cv2.findEssentialMat(self.good_new, self.good_old, self.focal_length, self.principal_point, cv2.RANSAC, 0.999, 1.0, None)

        # if (image_count < 2):
        #     _, self.curr_rotation_matrix, self.pos, _ = cv2.recoverPose(E, self.good_old, self.good_new, self.curr_rotation_matrix, self.curr_pos, self.focal_length, self.principal_point, None)
        # else:
        #     _, rot_mat, pos, _ = cv2.recoverPose(E, self.good_old, self.good_new, self.curr_rotation_matrix.copy(), self.curr_pos.copy(), self.focal_length, self.principal_point, None)
        #     # NEED TO FIGURE OUT HOW TO GET ABSOLUTE SCALE

        depth_frame = np.ones(shape=(240, 320), dtype = np.uint8) * 3
        mask = np.ones(shape=(240,320), dtype = np.uint8)
        old_frame_odom = cv2.rgbd.OdometryFrame.create(old_frame, depth_frame.astype(np.float32), mask)
        new_frame_odom = cv2.rgbd.OdometryFrame.create(new_frame, depth_frame.astype(np.float32), mask)

        odometry = cv2.rgbd.Odometry.create("RgbdOdometry")
        odometry.setCameraMatrix(np.array([[277, 0, 160],[0, 277, 120],[0, 0, 1]], dtype=np.float32))
        _, rotation_translation_matrix = odometry.compute2(old_frame_odom, new_frame_odom)
        print(rotation_translation_matrix)
        translation_vector = rotation_translation_matrix[0:3, 3]
        rotation_matrix = rotation_translation_matrix[0:3, 0:3]
        time_difference = (curr_time - old_time).nanoseconds * 1e-9
        print(time_difference)
        linear_velocity = translation_vector / time_difference
        rotation_converter = spatial.transform.Rotation.from_matrix(rotation_matrix)
        rpy = rotation_converter.as_euler("zxy", degrees = False)
        angular_velocity = np.asarray(rpy) / time_difference
        return linear_velocity, angular_velocity, translation_vector
