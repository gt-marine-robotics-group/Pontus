import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from enum import Enum
import numpy as np
import cv2
from sklearn.cluster import DBSCAN

# ROS Messages
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

import tf_transformations
# Helpers
from pontus_autonomy.helpers.cv_threshold import CVThreshold

from pontus_autonomy.tasks.base_task import BaseTask

class State(Enum):
    Submerging = 0
    Searching = 1
    Aligning = 2
    Passing_Through = 3
    PassedThrough = 4

class SearchingState(Enum):
    Searching = 0
    Verifying = 1

class VerifyingState(Enum):
    VerifyingFirst = 0
    VerifyingSecond = 1

# Standard task to pass through gate
class GateTask(BaseTask):

    def __init__(self):
        super().__init__("Gate_Task")

        # self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.cmd_pos_pub = self.create_publisher(Pose, '/cmd_pos', 1)

        qos_profile = QoSProfile(
                          reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                          history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                          depth=1
                      )

        self.sonoptix_subscription = self.create_subscription(
            LaserScan,
            '/pontus/sonar_0',
            self.sonoptix_callback,
            qos_profile=qos_profile
        )

        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odometry_callback,
            qos_profile=qos_profile
        )

        self.pc_subscription = self.create_subscription(
            PointCloud2,
            '/pontus/sonar_0_transformed_pc',
            self.sonoptix_pc_callback,
            qos_profile=qos_profile
        )

        self.timer = self.create_timer(0.1, self.state_machine_callback)

        self.state = State.Submerging
        self.searching_state = None
        self.verifying_state = None

        self.last_seen_gate_time = None
        self.gate_seen_timeout = Duration(seconds=1.0)
        self.drive_through_gate_time = Duration(seconds=3.0)
        self.points = None
        self.current_odometry: Odometry = None
        self.position_command_sent = False
        self.desired_pose = None
        self.previous_searching_state = None
        self.previous_verifying_state = None
        self.previous_state = None
        self.pointcloud = None
        self.potential_gates = None
        self.current_gate = 0
        self.selected_gate = []

        # TODO: Make a separate set of values for real life and sim

        ### Hyperparameters ###
        # This represents the min distance between two points to be
        # considered in the same cluster
        self.eps = 0.4
        # This is the min amount of points needed to form a cluster
        self.min_points = 1
        # The variance represents the variance in the euclidean distances
        # between points in the same cluster. It is used to determine if
        # points are very tighly coupled together (a real cluser) or if
        # points are very spread out (most likely the sonar hitting a wall 
        # or floor)
        self.variance_threshold = 1
        # Distance at which we are satisfied that we have arrived
        self.distance_threshold = 0.3
        # Angle threshold at which we are satisfied that we have arrived
        self.angle_threshold = 0.05
        # The width of the gate
        self.gate_width = 3
        # The allowed error in measurements relating to the gate width
        self.gate_width_epsilon = 0.4
        # This is the allowed threshold to verify the gate is a gate
        self.verifying_threshold = 0.2


    #### Callbacks

    def odometry_callback(self, msg):
        self.current_odometry = msg

    def sonoptix_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        self.points = np.vstack((x,y)).T

    def sonoptix_pc_callback(self, msg: PointCloud2):
        points = point_cloud2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        point_array = np.array([(point[0], point[1], point[2]) for point in points])
        self.pointcloud = point_array


    #### Helpers
    # Temporary function
    # TODO: Create logic to automatically handle this
    def go_to_pose(self, desired_pose: Pose, verify_only_yaw = False):
        if self.current_odometry is None:
            return False
        at_pose = False
        distance_diff, angle_diff = self.pose_difference(self.current_odometry.pose.pose, desired_pose)
        # self.get_logger().info(f"{distance_diff} {angle_diff}")
        if (distance_diff < self.distance_threshold or verify_only_yaw ) and angle_diff < self.angle_threshold:
            at_pose = True
        
        if self.position_command_sent:
            if at_pose:
                self.position_command_sent = False
            return at_pose
        self.cmd_pos_pub.publish(desired_pose)
        self.position_command_sent = True

    def pose_difference(self, pose1: Pose, pose2: Pose):
        x1, y1, z1 = pose1.position.x, pose1.position.y, pose1.position.z
        x2, y2, z2 = pose2.position.x, pose2.position.y, pose2.position.z
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        q1 = [pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]
        q2 = [pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w]
        
        dot_product = sum(q1[i] * q2[i] for i in range(4))
        angle = 2 * np.arccos(abs(dot_product))
        
        return distance, angle

    def cluster_points(self, points):
        # TODO: Ensure that these hyperparamters are well tuned
        dbscan = DBSCAN(eps=self.eps, min_samples=self.min_points)
        labels = dbscan.fit_predict(points)
        return labels

    def percent_ground_points(self, filter_depth, points) -> bool:
        return np.sum(points[..., 2] < filter_depth)/points.size

    def get_clusters(self, points):
        if points is None:
            return
        clusters = self.cluster_points(points)
        unique_labels = np.unique(clusters)
        potential_centroids = []
        for label in unique_labels:
            # Points with this label do not fall into any cluster and most likely noise
            if label == -1:
                continue

            cluster_points = points[clusters == label]
            potential_centroid = np.mean(cluster_points, axis=0)
            # centroid_variance = np.var(cluster_points)
            squared_diff = (cluster_points - potential_centroid) ** 2
            centroid_variance = np.sum(np.mean(squared_diff, axis=0))
            # self.get_logger().info(f"Label: {label} centroid: {potential_centroid} var: {centroid_variance} percent ground: {self.percent_ground_points(-1.8, cluster_points)}")
            # Ground filter
            if self.percent_ground_points(-1.8, cluster_points) > 0.2:
                continue

            # TODO: Ensure that this variance threshold is correct
            if centroid_variance > self.variance_threshold:
                continue
            potential_centroids.append(potential_centroid)
        self.get_logger().info(f"{potential_centroids}") 
        return np.array(potential_centroids)

    def calculate_euclidean_distance(self, point_a, point_b):
        return np.sqrt(np.sum((point_a - point_b) ** 2))

    # This function will take every pair of clusters and see if it represents a gate
    # by checking the distance between every two points and seeing if there is a set
    # of two clusters such that their distance is self.gate_width apart
    def get_potential_gates(self, clusters):
        potential_gates = []
        # We need at least two points to see if they are a potential cluster
        if len(clusters) < 1:
            return potential_gates
        
        for i in range(len(clusters) - 1):
            for j in range(i + 1, len(clusters)):
                euclidean_distance = self.calculate_euclidean_distance(clusters[i], clusters[j])
                if self.gate_width - self.gate_width_epsilon / 2 <= euclidean_distance \
                    and euclidean_distance <= self.gate_width + self.gate_width_epsilon / 2:

                    potential_gates.append((clusters[i], clusters[j]))
        return potential_gates

    def state_debugger(self):
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state
        if self.previous_searching_state != self.searching_state:
            self.get_logger().info(f"Now at: {self.searching_state.name}")
            self.previous_searching_state = self.searching_state
        if self.previous_verifying_state != self.verifying_state:
            self.get_logger().info(f"Now at: {self.verifying_state.name}")
            self.previous_verifying_state = self.verifying_state

    #### Autonomy
    # Run the gate state machine
    def state_machine_callback(self):
        self.state_debugger()
        match self.state:
            case State.Submerging:
                self.submerge()
            case State.Searching:
                self.search()
            case State.Aligning:
                self.align()
            case State.Passing_Through:
                self.pass_through()
            case _:
                pass
    
    def search(self):
        self.state_debugger()
        match self.searching_state:
            case SearchingState.Searching:
                self.forward()
            case SearchingState.Verifying:
                self.verify_potential_gates()
    
    def verify_potential_gates(self):
        self.state_debugger()
        match self.verifying_state:
            case VerifyingState.VerifyingFirst:
                self.verify_side(first = True)
            case VerifyingState.VerifyingSecond:
                self.verify_side(first = False)

    # This function is used to first submerge 
    def submerge(self):
        desired_pose = Pose()
        desired_pose.position.z = -1.0
        arrived = self.go_to_pose(desired_pose)
        if arrived:
            self.state = State.Searching
            self.searching_state = SearchingState.Searching

    # Keep going forward until we see the gate
    def forward(self):
        if self.pointcloud is None:
            return
        potential_clusters = self.get_clusters(self.pointcloud)
        if self.desired_pose is None:
            self.desired_pose = self.current_odometry.pose.pose
            self.desired_pose.position.z = -1.0
            self.desired_pose.position.x += 0.25
        potential_gates = self.get_potential_gates(potential_clusters)
        arrived = self.go_to_pose(self.desired_pose)
        
        if len(potential_gates) == 0 and arrived:
            self.desired_pose.position.x += 0.25
        elif len(potential_gates) > 0:
            self.get_logger().info(f"{potential_gates}")
            self.desired_pose = self.current_odometry.pose.pose
            arrived = self.go_to_pose(self.desired_pose)
            self.searching_state = SearchingState.Verifying
            self.verifying_state = VerifyingState.VerifyingFirst
        self.potential_gates = potential_gates

    def verify_side(self, first):
        # Test the gate to make sure its correct
        candidate_point = self.potential_gates[self.current_gate][0 if first else 1]
        if not self.position_command_sent:
            # Determine yaw to current point
            current_position = [self.current_odometry.pose.pose.position.x, self.current_odometry.pose.pose.position.y, self.current_odometry.pose.pose.position.z]
            # Yaw to face the gate pole
            new_yaw = np.arctan2(candidate_point[1] - current_position[1], candidate_point[0] - current_position[0])
            new_quaternion = tf_transformations.quaternion_from_euler(0, 0, new_yaw)
            self.desired_pose.orientation.x = new_quaternion[0]
            self.desired_pose.orientation.y = new_quaternion[1]
            self.desired_pose.orientation.z = new_quaternion[2]
            self.desired_pose.orientation.w = new_quaternion[3]
        arrived = self.go_to_pose(self.desired_pose, verify_only_yaw = True)
        if not arrived:
            return
        # Verify that we see a cluster in front of us
        potential_clusters = self.get_clusters(self.pointcloud)
        self.get_logger().info(f"{potential_clusters}")
        for potential_cluster in potential_clusters:
            diff = np.sqrt((potential_cluster[0] - candidate_point[0]) ** 2 + (potential_cluster[1] - candidate_point[1]) ** 2)
            if diff < self.verifying_threshold and first:
                self.verifying_state = VerifyingState.VerifyingSecond
                return
            if diff < self.verifying_threshold and not first:
                self.state = State.Aligning
                self.selected_gate = self.potential_gates[self.current_gate]
                return
        # If no gates are verified, and this is the late potential gate, retry searching for a gate
        # TODO: Fix this logic
        if self.current_gate == len(self.potential_gates) - 1:
            self.searching_state = SearchingState.Searching
        else:
            self.current_gate +=1

    # TODO: Improve this. Currently this assume that the gate will be right in front of us
    def align(self):
        # Determine the middle point of the gate
        middle_point_y = (self.selected_gate[0][1] + self.selected_gate[1][1])/2
        middle_point_y += 1/4 * self.gate_width
        self.desired_pose.position.y = middle_point_y
        self.desired_pose.orientation.x = 0.0
        self.desired_pose.orientation.y = 0.0
        self.desired_pose.orientation.z = 0.0
        self.desired_pose.orientation.w = 1.0
        arrived = self.go_to_pose(self.desired_pose)
        if arrived:
            self.state = State.Passing_Through

    def pass_through(self):
        if not self.position_command_sent:
            self.desired_pose.position.x += 5
        arrived = self.go_to_pose(self.desired_pose)
        if arrived:
            self.state = State.PassedThrough
