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

# Helpers
from pontus_autonomy.helpers.cv_threshold import CVThreshold

from pontus_autonomy.tasks.base_task import BaseTask

class State(Enum):
    Submerging = 0
    Searching = 1
    Approaching = 2
    Passing_Through = 3

class SearchingState(Enum):
    Searching = 0
    Verifying = 1

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

        self.last_seen_gate_time = None
        self.gate_seen_timeout = Duration(seconds=1.0)
        self.drive_through_gate_time = Duration(seconds=3.0)
        self.points = None
        self.current_odometry: Odometry = None
        self.position_command_sent = False
        self.desired_forward_pose = None
        self.previous_searching_state = None
        self.previous_state = None
        self.pointcloud = None

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
        self.distance_threshold = 0.1
        # Angle threshold at which we are satisfied that we have arrived
        self.angle_threshold = 0.05
        # The width of the gate
        self.gate_width = 3
        # The allowed error in measurements relating to the gate width
        self.gate_width_epsilon = 0.4


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
    def go_to_pose(self, desired_pose: Pose):
        if self.current_odometry is None:
            return False
        at_pose = False
        distance_diff, angle_diff = self.pose_difference(self.current_odometry.pose.pose, desired_pose)
        if distance_diff < self.distance_threshold and angle_diff < self.angle_threshold:
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

    def get_clusers(self, points):
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
        for i in range(len(clusters) - 1):
            for j in range(i + 1, len(clusters)):
                euclidean_distance = self.calculate_euclidean_distance(clusters[i], clusters[j])
                if self.gate_width - self.gate_width_epsilon / 2 <= euclidean_distance \
                    and euclidean_distance <= self.gate_width + self.gate_width_epsilon / 2:

                    potential_gates.append((clusters[i], clusters[j]))
        return potential_gates


    def state_debugger(self):
        # self.get_logger().info(f"Goal pose: {self.goal_pose}")
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state
        if self.previous_searching_state != self.searching_state:
            self.get_logger().info(f"Now at: {self.searching_state.name}")
            self.previous_searching_state = self.searching_state

    #### Autonomy
    # Run the gate state machine
    def state_machine_callback(self):
        self.state_debugger()
        match self.state:
            case State.Submerging:
                self.submerge()
            case State.Searching:
                self.search()
            # case State.Approaching:
            #     vel_msg = self.approach(pos, height, width)
            # case State.Passing_Through:
            #     vel_msg = self.pass_through()
            case _:
                pass
    
    # This function is used to first submerge 
    def submerge(self):
        desired_pose = Pose()
        desired_pose.position.z = -1.0
        arrived = self.go_to_pose(desired_pose)
        if arrived:
            self.state = State.Searching
            self.searching_state = SearchingState.Searching

    def search(self):
        self.state_debugger()
        match self.searching_state:
            case SearchingState.Searching:
                self.forward()
            case SearchingState.Verifying:
                pass
                # self.potential_clusters = self.verify

        # return vel_msg

    # Keep going forward until we see the gate
    def forward(self):
        # Flow of logic:
        # Take our current pose, go forward 0.5 meters
        # While going forward, check for clusters, and if we see a cluster stop
        if self.pointcloud is None:
            return
        potential_clusters = self.get_clusers(self.pointcloud)
        if self.desired_forward_pose is None:
            self.desired_forward_pose = self.current_odometry.pose.pose
            self.desired_forward_pose.position.x += 0.25
        potential_gates = self.get_potential_gates(potential_clusters)
        arrived = self.go_to_pose(self.desired_forward_pose)
        
        if len(potential_gates) == 0 and arrived:
            self.desired_forward_pose.position.x += 0.25
        elif len(potential_gates) > 0:
            self.get_logger().info(f"{potential_gates}")
            self.desired_forward_pose = self.current_odometry.pose.pose
            self.searching_state = SearchingState.Verifying

    def approach(self, pos, height, width):
        vel_msg = Twist()

        if (pos):
            # Pass a bit lower so we don't run into the signs
            if pos[1] < height/4.5:
                vel_msg.linear.z = 0.3
            elif pos[1] > height/4.0:
                vel_msg.linear.z = -0.3

            vel_msg.linear.x = 0.8
            # Rotate towards the center
            vel_msg.angular.z = 1.0 if pos[0] < width/2 else -1.0
        elif self.get_clock().now() - self.last_seen_gate_time < self.gate_seen_timeout:
            vel_msg.linear.x = 0.6
        else:
            self.state = State.Passing_Through

        return vel_msg

    def pass_through(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.5

        # TODO: if DVL is good we can use odom travel distance instead of travel time
        if self.get_clock().now() - self.last_seen_gate_time > self.drive_through_gate_time:
            self.complete(True)

        return vel_msg

    def detect_gate(self, markers):
        pos = [0.0, 0.0]

        # TODO: Improve this
        for marker in markers:
            pos[0] += marker[0]
            pos[1] += marker[1]

        pos[0] = pos[0] / len(markers)
        pos[1] = pos[1] / len(markers)

        return pos
