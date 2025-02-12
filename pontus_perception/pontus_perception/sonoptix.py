import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN


class Sonoptix(Node):
    def __init__(self):
        super().__init__("sonoptix")
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sonop_sub = self.create_subscription(
            Image,
            "/pontus/sonar_0/image_debug",
            self.sonoptix_callback,
            qos_profile
        )

        self.sonop_repub = self.create_publisher(
            Image,
            "/sonop_image",
            10
        )
        self.sonop_repub_other = self.create_publisher(
            Image,
            "/sonop_image_other",
            10
        )
        self.laser_scan_publish = self.create_publisher(
            LaserScan,
            '/pontus/sonar_0',
            qos_profile=qos_profile,
        )

        self.bridge = CvBridge()
        self.previous_image_exponential = None
        self.previous_image = None
        self.angle_range = 2.0944
        self.range = 15.0
        self.feature_params = dict( maxCorners = 100000,
                       qualityLevel = 0.3,
                       minDistance = 11,
                       blockSize = 5 )
        
        self.lk_params = dict( winSize  = (15, 15),
                  maxLevel = 3,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.color = np.random.randint(0, 255, (100, 3))
        self.mask = None
        self.corners = None
        self.traj_frame = None
        self.prev_keypoints = None
        self.feature_lifetimes = {}
        self.LIFETIME_THRESHOLD = 10

    def convert_to_laserscan(self, image):
        num_rows, num_cols = image.shape
        laser_msg = LaserScan()
        laser_msg.header.frame_id = "sonar_0"
        laser_msg.header.stamp = self.get_clock().now().to_msg()
        laser_msg.angle_min = -self.angle_range / 2
        laser_msg.angle_max = self.angle_range / 2
        laser_msg.angle_increment = self.angle_range / num_cols
        laser_msg.range_min = 0.2
        laser_msg.range_max = self.range
        ranges = []
        for col in range(0, num_cols):
            for row in range(0, num_rows + 1):
                # If we reach the end of the column and still ahve yet to see anything,
                # set the range to the max value
                if row == num_rows:
                    ranges.append(float(self.range)) 
                elif image[row][col] == 255:
                    ranges.append(float(row / num_rows * self.range))
                    break
        laser_msg.ranges = ranges
        return laser_msg

    def exponential_moving_average(self, cut_image, alpha):
        float_image = cut_image.astype(np.float32)
        if self.previous_image_exponential is None:
            self.previous_image_exponential = float_image
        filtered_image = alpha * float_image + (1 - alpha) * self.previous_image_exponential
        filtered_image = np.clip(filtered_image, 0, 255).astype(np.uint8)
        self.previous_image_exponential = filtered_image.astype(np.float32)
        return filtered_image


    # def capture_optical_flow(self, masked_image):
    #     feature_params = dict(
    #         maxCorners=200,
    #         qualityLevel=0.3,
    #         minDistance=5,
    #         blockSize=7
    #     )
    #     return_image = cv2.cvtColor(masked_image, cv2.COLOR_GRAY2BGR)
    #     if self.traj_frame is None:
    #         self.traj_frame = np.zeros_like(return_image)
    #     if self.corners is None:
    #         self.corners = cv2.goodFeaturesToTrack(self.previous_image, mask=None, **feature_params)
    #     if self.corners is not None and len(self.corners) > 0:
    #         p1, st, err = cv2.calcOpticalFlowPyrLK(self.previous_image, masked_image, self.corners, None, **self.lk_params)

    #         if p1 is not None:
    #             good_new = p1[st ==1]
    #             good_old = self.corners[st == 1]
    #             for new, old in zip(good_new, good_old):
    #                 a, b = new.ravel()
    #                 c, d = old.ravel()
    #                 if (a - c) * (a - c) + (b - d) * (b - d) > 5:
    #                     continue
    #                 cv2.line(self.traj_frame, (int(a), int(b)), (int(c), int(d)), (0, 255,0), 2)
    #                 cv2.circle(return_image, (int(a), int(b)), 5, (0, 0, 255), -1)
    #             self.corners = good_new.reshape(-1, 1, 2)
    #     new_features = cv2.goodFeaturesToTrack(masked_image, mask=None, **feature_params)
    #     if new_features is not None:
    #         if self.corners is not None:
    #             self.corners = np.vstack((self.corners, new_features))
    #         else:
    #             self.corners = new_features
    #     return self.traj_frame

    # def capture_optical_flow(self, masked_image):
    #     feature_params = dict(
    #         maxCorners=200,
    #         qualityLevel=0.3,
    #         minDistance=5,
    #         blockSize=7
    #     )
    #     return_image = cv2.cvtColor(masked_image, cv2.COLOR_GRAY2BGR)
    #     if self.traj_frame is None:
    #         self.traj_frame = np.zeros_like(return_image)
    #     if len(self.feature_lifetimes) > 0:
    #         p0 = np.array(list(self.feature_lifetimes.keys()), dtype=np.float32).reshape(-1, 1, 2)
    #         p1, st, err = cv2.calcOpticalFlowPyrLK(self.previous_image, masked_image, p0, None, **self.lk_params)

    #         if p1 is not None:
    #             good_new = p1[st ==1]
    #             good_old = p0[st == 1]

    #             updated_lifetimes = {}

    #             for new, old in zip(good_new, good_old):
    #                 a, b = new.ravel()
    #                 c, d = old.ravel()

    #                 distance = np.linalg.norm(np.array([a, b]) - np.array([c, d]))
    #                 if distance > 10:
    #                     continue
    #                 key = (int(a), int(b))
    #                 updated_lifetimes[key] = self.feature_lifetimes.get((int(c), int(d)), 0) + 1

    #                 if updated_lifetimes[key] > self.LIFETIME_THRESHOLD:
    #                     cv2.line(self.traj_frame, (int(a), int(b)), (int(c), int(d)), (0, 255,0), 2)
    #                     cv2.circle(return_image, (int(a), int(b)), 5, (0, 0, 255), -1)
    #             self.feature_lifetimes = updated_lifetimes

    #     new_features = cv2.goodFeaturesToTrack(masked_image, mask=None, **feature_params)
    #     if new_features is not None:
    #         for f in new_features:
    #             x, y = f.ravel()
    #             key = (int(x), int(y))
    #             if key not in self.feature_lifetimes:
    #                 self.feature_lifetimes[key] = 1
    #     return self.traj_frame


    def cluster_points(self, frame):
        points = np.column_stack(np.where(frame > 0))
        clustering = DBSCAN(eps=5, min_samples=40).fit(points)
        clusters = []
        for label in set(clustering.labels_):
            if label == -1:
                continue
            clusters.append(points[clustering.labels_ == label])
        
        return clusters

    def track_motion(self, clusters, frame):
        if self.previous_image is None or self.prev_keypoints is None:
            self.previous_image = frame.copy()
            self.prev_keypoints = np.float32([c[:, ::-1].mean(axis=0) for c in clusters]) if clusters else None
            return []
        # self.prev_keypoints = np.float32([c[:, ::-1].mean(axis=0) for c in clusters]) if clusters else None
        if clusters:  
            new_keypoints_temp = np.float32([c[:, ::-1].mean(axis=0) for c in clusters])
            self.prev_keypoints = np.vstack([self.prev_keypoints, new_keypoints_temp])

        # Calculate optical flow
        new_keypoints, status, _ = cv2.calcOpticalFlowPyrLK(
            self.previous_image,
            frame,
            self.prev_keypoints,
            None
        )

        # Create motion vector array
        motion_vectors = []
        if new_keypoints is not None:
            updated_lifetimes = {}
            for i, (new, old) in enumerate(zip(new_keypoints, self.prev_keypoints)):
                if status[i] == 1:
                    motion_vectors.append((old, new))
        
                a, b = new.ravel()
                c, d = old.ravel()

                distance = np.linalg.norm(np.array([a, b]) - np.array([c, d]))
                if distance > 10:
                    continue
                # Keep track of vector consistency
                key = (int(a), int(b))
                updated_lifetimes[key] = self.feature_lifetimes.get((int(c), int(d)), 0) + 1

                # if updated_lifetimes[key] > self.LIFETIME_THRESHOLD:
                    # cv2.arrowedLine(self.traj_frame, tuple(old.astype(int)), tuple(new.astype(int)), (0, 255, 0), 2)
            self.feature_lifetimes = updated_lifetimes

        self.previous_image = frame.copy()
        self.prev_keypoints = new_keypoints
        return motion_vectors

    def preprocess_image(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        denoised = cv2.fastNlMeansDenoising(gray, None, h=18, templateWindowSize=7, searchWindowSize=7)
        blurred = cv2.GaussianBlur(denoised, (3, 3), 0)
        return blurred

    def sonoptix_callback(self, msg: Image):
        # Preprocessing
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        image = image[20:, :, :]
        preprocessed_image = self.preprocess_image(image) 

        if self.traj_frame is None:
            self.traj_frame = np.zeros_like(image)
        

        # masked = cv2.inRange(gray, 2, 255)
        self.traj_frame = np.zeros_like(image)
        cut_image = preprocessed_image * 50
        returned_image = self.exponential_moving_average(cut_image, 0.1)
        masked = cv2.inRange(returned_image, 20, 255)
        # masked = cut_image
        # clusters = self.cluster_points(masked)
        # motion_vectors = self.track_motion(clusters, masked)

        # ### DBSCAN DEBUGGING
        # # self.traj_frame = np.zeros_like(image)
        # keypoints = np.float32([c[:, ::-1].mean(axis=0) for c in clusters]) if clusters else None
        # # print("Keypoints:", keypoints)
        # if keypoints is not None:
        #     for key_point in keypoints:
        #         cv2.circle(self.traj_frame, (int(key_point[0]), int(key_point[1])), 3, (0, 0, 255), 2)
        # ###
        # # for (old, new) in motion_vectors:
        #     # print(old, new)
        #     # cv2.arrowedLine(self.traj_frame, tuple(old.astype(int)), tuple(new.astype(int)), (0,255,0), 2)
        #     # cv2.arrowedLine(self.traj_frame, (int(old[1]), int(old[0])), (int(new[1]), int(new[0])), (0,255,0), 2)

        # self.previous_image = masked

        # normalized_image = cv2.normalize(filtered_image, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        ros_image_other = self.bridge.cv2_to_imgmsg(masked, "mono8")
        self.sonop_repub_other.publish(ros_image_other)
        # ros_image = self.bridge.cv2_to_imgmsg(self.traj_frame, "bgr8")
        # self.sonop_repub.publish(ros_image)
        laser_scan = self.convert_to_laserscan(masked)
        self.laser_scan_publish.publish(laser_scan)


def main(args=None):
    rclpy.init(args=args)
    node = Sonoptix()
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()
