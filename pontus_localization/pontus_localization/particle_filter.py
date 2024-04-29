import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu
from .line_detection import LineDetection
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Vector3
from builtin_interfaces.msg import Duration

from sensor_msgs.msg import PointCloud2, PointField

import numpy as np
from .pf_constants import *

# ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id world
        
class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__("odometry_node")
        self.get_logger().info("Particle filter node started")
        # Subscribers
        
        # Camera subscription
        self.camera_sub = self.create_subscription(
            Image, 
            "/pontus/camera_1", 
            self.camera_callback, 
            10
        )
        
        # Imu subscription
        # self.imu_sub = self.create_subscription(
        #     Imu, 
        #     "/pontus/imu_0",
        #     self.imu_callback,
        #     10
        # )
        
        self.depth_sub = self.create_subscription(
            Odometry,
            "/pontus/depth_0",
            self.depth_callback,
            10
        )

        # self.odom_pub = self.create_publisher(
        #     Odometry,
        #     "/pf_odometry",
        #     10,
        # )

        # Publisher for publishing the nodes so that they can be displayed
        self.particle_pub_arrow = self.create_publisher(
            MarkerArray,
            "/particle_arrow_topic",
            10
        )
        
        self.particle_pub = self.create_publisher(
            PointCloud2,
            "/particle_topic",
            10
        )
        self.true_pub = self.create_publisher(
            Marker,
            "/sub_true",
            10
        )

        # Subscriber to odometry to get the true position of the robot
        self.odom_sub = self.create_subscription(
            Odometry,
            "/pontus/odometry",
            self.odom_callback,
            10
        )

        self.bridge = CvBridge()

        # This will be used to calculate the time between each imu callback
        self.velocity = np.array([0.0, 0.0, 0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, POOL_DEPTH, 0.0])
        self.depth = POOL_DEPTH + 0.19
        self.linear_acceleration_current = np.zeros((4,))
        self.previous_angular_velocity = 0
        self.previous_time = None
        self.camera_previous_time = None
        
        # Particles
        self.particles = np.zeros((400, 4))
        self.init_particles()
        self.particles = np.tile([0, 0, POOL_DEPTH + 0.19, -3 * np.pi / 4 ], (1, 1))
        # self.particles = np.array([[0, 0, POOL_DEPTH + 0.19, 0],
        #                            [0.1,0.4, POOL_DEPTH + 0.19, 0],
        #                            [0.6,-0.1, POOL_DEPTH + 0.19, 0.2]])
        if DISPLAY_PARTICLES:
            self.particles_display()
        self.start_up = 0

    ######################
    # Particle related functions
    ######################
    # This function will uniformly initialize the particles in the grid
    # This is used to detect where the robot starts
    def init_particles(self):
        total_grid_width = GRID_LINE_LENGTH + GRID_LINE_THICKNESS
        num_rows = int(np.sqrt(START_UP_PARTICLES))
        counter = 0
        for row in range(num_rows):
            for col in range(num_rows):
                for angle in range(NUM_START_UP_ANGLES):
                    x = row * total_grid_width / num_rows - total_grid_width / 2
                    y = col * total_grid_width / num_rows - total_grid_width / 2
                    # Need to sample betwee 0 and pi/2 for the angle
                    self.particles[counter] = np.array([x, y, POOL_DEPTH, angle / NUM_START_UP_ANGLES * np.pi / 2])
                    counter+=1  
    
    # This calculates the average position and yaw of the particles
    # This is what our odometry would be
    def calculate_avg_position_yaw(self):
        avg_position = np.mean(self.particles, axis=0)
        return avg_position

    # This updates the particles with the new position and yaw
    def update_particles(self, change_in_position, use_guassian_noise = True): 
        # Update the particles with the change in position
        # Add guassian noise to simulate uncertainty / error in imu readings
        if use_guassian_noise:
            self.particles[:, :2] = self.particles[:, :2] + change_in_position[:, :2] + self.guassian_noise(TRANS_MU, TRANS_SIGMA, change_in_position[:, :2].shape)
            self.particles[:, 3] = self.particles[:, 3] + change_in_position[:, 3] + self.guassian_noise(ROTATE_MU, ROTATE_SIGMA, change_in_position[:, 3].shape)
        # Bound the particles by their x,y position and angle
        # Ensures that adding guassian noise does not cause particles to wander too far
        self.particles[:,2] = self.depth    

    # This function is used to publish particles to the particles topic
    # so it can be displayed in Rviz
    def particles_display(self):
        
        # Display particle spheres
        msg = PointCloud2()
        
        particle_positions = self.particles[:, :3]
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(self.particles)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * len(particle_positions)
        msg.is_dense = True
        msg.data = np.array(particle_positions, dtype=np.float32).tobytes()
        
        self.particle_pub.publish(msg)

        
        # Display yaw
        markers = MarkerArray()
        marker_id = 0
        for particle in self.particles:
            marker = Marker()
            yaw = particle[3]
            marker.header.frame_id = 'map'  # Set the frame ID
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'particle_markers'
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = particle[0]
            marker.pose.position.y = particle[1]
            marker.pose.position.z = particle[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = np.sin(yaw / 2.0)
            marker.pose.orientation.w = np.cos(yaw / 2.0)
            marker.scale = Vector3(x=0.2, y=0.02, z=0.02) 
            marker.color = ColorRGBA(r=0.0, g=0.67, b=1.0, a=1.0)  
            marker.lifetime = Duration(sec=0)
            markers.markers.append(marker)
            marker_id += 1

        # Display assumed position
        marker = Marker()
        yaw = self.position[3]
        marker.header.frame_id = 'map'  # Set the frame ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'particle_markers'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = np.sin(yaw/ 2.0)
        marker.pose.orientation.w = np.cos(yaw / 2.0)
        marker.scale = Vector3(x=0.4, y=0.15, z=0.15) 
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  
        marker.lifetime = Duration(sec=0)
        markers.markers.append(marker)
        marker_id += 1

        self.particle_pub_arrow.publish(markers)
        # self.get_logger().info('Published particle markers')

    # This function ensures that particle yaw are in the range [-pi, pi)
    def yaw_norm(self, yaw):
        yaw = np.where(yaw >= np.pi, yaw - 2 * np.pi, yaw)
        yaw = np.where(yaw < -np.pi, yaw + 2 * np.pi, yaw)
        return yaw

    # This function reutrns guassian noise 
    def guassian_noise(self, mu, sigma, shape):
        return np.random.normal(mu, sigma, shape)

    # This function will rotate a numpy array of points in the robot frame to the global frame
    def rotate_points(self, points, yaws):
        # Extract change in x and change in y
        if len(points.shape) == 1:
            points = np.expand_dims(points, axis=0)
        dx_dy = points[:, :2]
        rotated_dx_dy = np.zeros_like(dx_dy)
        # Rotate x and y points
        rotated_dx_dy[:, 0] = dx_dy[:, 0] * np.cos(yaws) - dx_dy[:, 1] * np.sin(yaws)
        rotated_dx_dy[:, 1] = dx_dy[:, 0] * np.sin(yaws) + dx_dy[:, 1] * np.cos(yaws)
        # Add depth and rotation back
        rotated_points = np.concatenate((rotated_dx_dy, points[:, 2:]), axis=1)
        return rotated_points

    # This function will return a point rotated by theta
    def rotate_point(self, x, y, theta):
        new_x = x * np.cos(theta) - y * np.sin(theta)
        new_y = x * np.sin(theta) + y * np.cos(theta)
        return new_x, new_y
    
    # This function will bound the particles by the x and y position and yaw
    def bound_particles(self):
        # If the distance of the particle between the assumed position is greater than the CUTOFF_DISTANCE,
        # then the particle is set to the max_distance_particle 
        squared_distances = np.sum(np.square(self.particles[:, :2] - self.position[:2]), axis=1)
        indices = np.where(squared_distances > CUTOFF_DISTANCE ** 2)
        unit_vectors = ((self.particles[indices, :2] - self.position[:2])) / np.linalg.norm(self.particles[indices, :2] - self.position[:2], axis=1)[:, np.newaxis]

        # Adjust particle positions for the selected indices
        self.particles[indices, :2] = self.position[:2] + CUTOFF_DISTANCE * unit_vectors
        
        # Bound yaw
        diffs = self.particles[:, 3] - self.position[3]
        diffs = (diffs + np.pi) % (2 * np.pi) - np.pi
        bounded_angles = np.where(diffs > CUTOFF_ANGLE, self.position[3] + CUTOFF_ANGLE, np.where(diffs < -CUTOFF_ANGLE, self.position[3] - CUTOFF_ANGLE, self.particles[:, 3]))
        self.particles[:, 3] = self.yaw_norm(bounded_angles)

    # This function will get the likelihood of the paticle given the camera_markers
    def get_particle_likelihood(self, camera_markers, particle):
        # Pair the camera markers with the particle
        particle_markers = self.get_markers_from_particle(particle)
        # self.get_logger().info("Particle Markers:\n" + str(particle_markers))
        likelihood = self.get_likelihood(particle_markers, camera_markers, particle)
        # self.get_logger(). info(str(likelihood))
        return likelihood
        
    # Given a particle, return the expected lines it sees
    def get_markers_from_particle(self, particle):
         # This represents how much in centimeteres the robot can see in the x and y direction
        horizontal_FOV_magnitude = particle[2] * np.tan(np.radians(FOV_H / 2))
        vertical_FOV_magnitude = particle[2] * np.tan(np.radians(FOV_V / 2))
        
        # Getting corners in global frame
        corner_top_left = np.abs(self.rotate_point(-vertical_FOV_magnitude, horizontal_FOV_magnitude, particle[3]))
        corner_top_right = np.abs(self.rotate_point(vertical_FOV_magnitude, horizontal_FOV_magnitude, particle[3]))
        corner_bottom_left = np.abs(self.rotate_point(-vertical_FOV_magnitude, -horizontal_FOV_magnitude, particle[3]))
        corner_bottom_right = np.abs(self.rotate_point(vertical_FOV_magnitude, -horizontal_FOV_magnitude, particle[3]))

        # Convert the maximum x and y values to the global frame in real life coordiantes
        true_x_max = np.max([corner_top_left[0], corner_top_right[0], corner_bottom_left[0], corner_bottom_right[0]]) - PADDING
        true_y_max = np.max([corner_top_left[1], corner_top_right[1], corner_bottom_left[1], corner_bottom_right[1]]) - PADDING

        
        # Assuming that the values above are correct
        position_x = particle[0]
        position_y = particle[1]
        
        end_positions = [(position_x + true_x_max, 0), 
                         (position_x - true_x_max, 0),
                         (0, position_y + true_y_max), 
                         (0, position_y - true_y_max)]
        
        markers = []
        
        grid_length = GRID_LINE_LENGTH + GRID_LINE_THICKNESS
        
        # Get all rows that are seen
        # This will compose of all the lanes that span horizontally that appear in the camera
        # The rows_seen array will contain all the y coordinates of the lines
        rows_seen = []
        # This represents the row coordinate of the last row. 
        current_row = np.floor((end_positions[3][1] - (GRID_LINE_LENGTH / 2 + GRID_LINE_THICKNESS)) / grid_length) * grid_length - (GRID_LINE_LENGTH / 2 + GRID_LINE_THICKNESS) 
        while current_row <= end_positions[2][1]:
            # Check outer line
            if end_positions[3][1] <= current_row <= end_positions[2][1]:
                if GRADIENT_POINTS_INTO_LINE:
                    gradient_angle = np.pi / 2
                else:
                    gradient_angle = -np.pi / 2
                rows_seen.append((current_row, gradient_angle))
                
            # Check inner line
            if end_positions[3][1] <= current_row + GRID_LINE_THICKNESS <= end_positions[2][1]:
                if GRADIENT_POINTS_INTO_LINE:
                    gradient_angle = -np.pi / 2
                else:
                    gradient_angle = np.pi / 2
                rows_seen.append((current_row + GRID_LINE_THICKNESS, gradient_angle))
            
            current_row += grid_length 
            # self.get_logger().info(str(current_row))
        # self.get_logger().info("Rows:" + str(rows_seen))
        # Do the same for columns
        columns_seen = []
        current_column = np.floor((end_positions[1][0] - (GRID_LINE_LENGTH / 2 + GRID_LINE_THICKNESS)) / grid_length) * grid_length - (GRID_LINE_LENGTH / 2 + GRID_LINE_THICKNESS) 
        while current_column <= end_positions[0][0]:
            # Check outer line
            if end_positions[1][0] <= current_column <= end_positions[0][0]:
                if GRADIENT_POINTS_INTO_LINE:
                    gradient_angle = 0
                else:
                    gradient_angle = np.pi
                columns_seen.append((current_column, gradient_angle))
                
            # Check inner line
            if end_positions[1][0] <= current_column + GRID_LINE_THICKNESS <= end_positions[0][0]:
                if GRADIENT_POINTS_INTO_LINE:
                    gradient_angle = np.pi
                else:
                    gradient_angle = 0
                columns_seen.append((current_column + GRID_LINE_THICKNESS, gradient_angle))
            
            current_column += grid_length 
        # self.get_logger().info("Cols:" + str(columns_seen))
        # Add the positions of the rows and columns to the markers
        # The hough transform returns the closest distance to the line. 
        # This means that it will always return the perpendicular distance to the line
        # Since we convert our measurements from the robot to the global frame,
        # We can assume that the distance to the line is the perpendicular distance 
        for row, gradient_angle in rows_seen:
            markers.append(np.array([particle[0], row, 0, gradient_angle])) 
        for column, gradient_angle in columns_seen:
            markers.append(np.array([column, particle[1], np.pi / 2, gradient_angle]))   

        return markers

    # This function will get the pairs of markers that are closest to each other
    # using nearest neighbor
    def get_likelihood(self, particle_markers, camera_markers, particle):
        l = 1
        # If no markers are detected, return 0 because we are super uncertain about our location
        if len(particle_markers) == 0 or len(camera_markers) == 0:
            # self.get_logger("No markers detected")
            return 0
        
        # Iterate through each particle marker and find the closest camera marker
        for particle_marker in particle_markers:
            max_likelihood = 0
            best_camera_marker = None
            for camera_marker in camera_markers:
                # Rejection criteria
                # if np.abs(particle_marker[2] - camera_marker[2]) > np.pi / 2 or np.abs(np.linalg.norm(particle_marker[:2] - camera_marker[:2])) > 1:
                #     # self.get_logger().info("Rejection criteria met")
                #     continue
                # self.get_logger().info("Passed rejection criteria")
                gradient_angle_distance = self.angle_absolute_difference(camera_marker[3], particle_marker[3])
                line_angle_distance = np.min([abs(camera_marker[2] - particle_marker[2]), abs(np.pi - abs(camera_marker[2] - particle_marker[2]))])
                if gradient_angle_distance > np.pi / 4 or line_angle_distance > np.pi / 4:
                    continue
                current_likelihood = self.calculate_marker_pair_likelihood(particle_marker, camera_marker)
                # print("Particle marker:", particle_marker, "Camera marker:", camera_marker, "Likelihood:", current_likelihood)  
                if current_likelihood > max_likelihood:
                    max_likelihood = current_likelihood
                    best_camera_marker = camera_marker

            if max_likelihood != 0:
                # self.get_logger().info("Max likelihood: " + str(max_likelihood) + " for particle: " + str(particle_marker) + " and camera: " + str(best_camera_marker))
                l = l * max_likelihood
            else:
                # This means that the marker is not matched with anything
                # Penalize not seeing a marker
                # Bound the distance, so markers far away do not penalize
                # print("Distance:", np.linalg.norm(particle_marker[:2] - particle[:2]), "Particle_marker", particle_marker)
                if np.linalg.norm(particle_marker[:2] - particle[:2]) < (GRID_LINE_LENGTH + GRID_LINE_THICKNESS) / 2:
                    l = l * 0.75
        return l
    
    # This function will return the distance between two markers
    def calculate_marker_pair_likelihood(self, marker1, marker2):
        grid_distance = np.sum(np.square(marker1[:2] - marker2[:2]))
        # Used to account for wrapping
        angle_distance = np.min([abs(marker1[2] - marker2[2]), abs(np.pi - abs(marker1[2] - marker2[2]))])
        return np.exp(- grid_distance**2 / (2 * MARKER_DISTANCE_SIGMA**2) - angle_distance**2 / (2 * MARKER_ANGLE_SIGMA**2))
        
    # Resample particles based on prob distribution 
    def resample_particles(self, particle_likelihoods):
        # Normalize the likelihoods
        particle_likelihoods = particle_likelihoods / np.sum(particle_likelihoods)
        # Resample the particles
        indices = np.random.choice(a = [idx for idx in range(len(self.particles))], size = len(self.particles), p = particle_likelihoods, replace = True)    
        self.particles = self.particles[indices]
        
    ######################
    # Callback functions
    ######################
    # Odometry call back
    # This is only used for displaying the true position of the sub in Rviz
    def odom_callback(self, msg):
        if not DISPLAY_PARTICLES:
            return
        # Display true position
        marker = Marker()
        marker.header.frame_id = 'map'  # Set the frame ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'particle_markers'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = msg.pose.pose.position.x
        marker.pose.position.y = msg.pose.pose.position.y
        marker.pose.position.z = POOL_DEPTH + msg.pose.pose.position.z
        marker.pose.orientation.x = msg.pose.pose.orientation.x
        marker.pose.orientation.y = msg.pose.pose.orientation.y
        marker.pose.orientation.z = msg.pose.pose.orientation.z
        marker.pose.orientation.w = msg.pose.pose.orientation.w
        marker.scale = Vector3(x=0.4, y=0.15, z=0.15) 
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  
        marker.lifetime = Duration(sec=0)
        self.true_pub.publish(marker)
        # self.get_logger().info("Published true position marker")

    # Imu callback
    # This callback function calculates the translation and rotation of the sub based on the imu. 
    # This function will also update the particles based on the imu data.
    def imu_callback(self, msg):
        # Get first previous time
        if self.previous_time is None:
            self.previous_time = self.get_clock().now()
            return
        # Get current time
        current_time = self.get_clock().now()    
        # Calculate the change in time
        dt = (current_time - self.previous_time).nanoseconds / 1e9
        # self.get_logger().info("dt: " + str(dt))
        self.previous_time = current_time

        # Get imu data and interpolate the data to calculate change in position
        self.linear_acceleration_current = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z - 9.7999999])
        change_in_position = self.interpolate_imu_data(self.linear_acceleration_current, self.acceleration, msg.angular_velocity.z, self.previous_angular_velocity, dt, 0.01)
        self.acceleration = self.linear_acceleration_current
        self.previous_angular_velocity = msg.angular_velocity.z
        
        # Update each particles position with the imu update
        self.update_particles(change_in_position) 
        self.bound_particles()
        self.position = self.calculate_avg_position_yaw()
        if DISPLAY_PARTICLES:
            self.particles_display()
        
        return

        self.get_logger().info("Change in velocity: " + str(self.linear_acceleration_current))
        self.get_logger().info("Change in velocity: " + str(change_in_velocity))
    
    # Depth callback
    # This callback function will update the depth of the sub
    def depth_callback(self, msg):
        # self.depth = -msg.pose.pose.position.z + POOL_DEPTH
        self.depth = POOL_DEPTH + msg.pose.pose.position.z
        self.update_particles(np.array([0, 0, self.depth, 0]), False)
        return
    
    # Camera callback
    def camera_callback(self, msg):
        self.motion_update(msg)

    ######################
    # Data processing functions
    ######################
    # This function will take in the current IMU data, and previous IMU data and interpolate the data over a timestep
    # This should return change in position
    # This function will also handle converting each particles frame into the global frame
    def interpolate_imu_data(self, current_imu_data, previous_imu_data, current_angular_velocity, previous_angular_velocity, dt, desired_time_step):
        change_in_acceleration = (current_imu_data - previous_imu_data) / dt
        change_in_angular_velocity = (current_angular_velocity - previous_angular_velocity) / dt
        change_in_position = np.zeros((len(self.particles), 4))
        current_step = 0
        while current_step < dt:
            # Calculate the time step
            time_step = desired_time_step
            if current_step + desired_time_step > dt:
                time_step = dt - current_step
            # Get current acceleration based on time step with respect to the robot frame
            current_acceleration = previous_imu_data + change_in_acceleration * current_step
            # Change the current acceleration to the global frame
            # Calculate change in velocity based on time_step
            change_in_velocity = current_acceleration / 4 * time_step
            change_in_velocity = np.append(change_in_velocity, 0)
            # Update new velocity
            self.velocity = self.velocity + change_in_velocity
            # Calculate current angular velocity
            angular_velocity = previous_angular_velocity + change_in_angular_velocity * current_step
            self.velocity[3] = angular_velocity / 2
            # Calculate change in position
            change_in_position_particle = np.tile((self.velocity * time_step), (len(self.particles),1))
            # Rotate the change in position to the global frame
            change_in_position_global = self.rotate_points(change_in_position_particle, self.particles[:,3])
            change_in_position = change_in_position + change_in_position_global
            current_step = current_step + time_step
        return change_in_position

    # This function will get the observation from the camera
    # This will return the an array of lines with their distance and angle
    def get_observation_from_camera(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'mono8')
        lines = LineDetection.get_lines(cv_image, self.depth - CAMERA_DEPTH_OFFSET)
        # self.get_logger().info(str(lines))
        return lines
    
    def angle_absolute_difference(self, angle1, angle2):
        # Compute the absolute difference between the angles
        diff = abs(angle1 - angle2)
        
        # Ensure the result is between 0 and 2π
        diff = diff % (2 * np.pi)
        
        # Adjust to the smallest angle (either diff or 2π - diff)
        return min(diff, 2 * np.pi - diff)

    ######################]
    # Particle filter functions
    ######################
    # This function will update the particles based on the observation
    def motion_update(self, image_msg):
        # Start up iteration to center particles at robot
        # if self.start_up < START_UP_ITERATIONS:
        #     self.start_up += 1
        # # When we reach the start up iterations, we will set all particles to the robots current location
        # elif self.start_up == START_UP_ITERATIONS:
        #     self.start_up = START_UP_ITERATIONS + 1
        #     # Reset all particles to the origin
        #     origin_particle = np.array([0, 0, self.depth, 0])
        #     self.particles = np.tile(origin_particle, (NUM_PARTICLES, 1))

        #     # Set the offsets
        #     self.x_offset = self.position[0]
        #     self.y_offset = self.position[1]
        #     self.yaw_offset = self.position[3]
        #     self.camera_previous_time = self.get_clock().now()
        #     return

        # If we are past the start up iterations, we will update the particles based on the observation
        # Get the observation from the camera
        camera_markers_r = self.get_observation_from_camera(image_msg)
        likelihoods = []
        # return 
        # Calculate the likelihood of each particle given the camera
        if len(camera_markers_r) > 0:
            for particle in self.particles:
                # Convert the camera markers to the global frame with respect to the particle
                # self.get_logger().info("Camera markers raw: \n" + str(camera_markers_r))
                camera_markers_g = self.rotate_points(camera_markers_r, particle[3] - np.pi / 2)
                camera_markers_g[:, 2] = (particle[3] + camera_markers_g[:, 2] - np.pi / 2) % np.pi
                camera_markers_g[:, 3] = self.yaw_norm((particle[3] + camera_markers_g[:, 3] - np.pi / 2) % (2 * np.pi))
                # Only add the particles current position to the camera markers if its angle corresponds with it
                
                camera_markers_g[:, 0] += particle[0]
                camera_markers_g[:, 1] += particle[1]
                # print(camera_markers_g)
                likelihood = self.get_particle_likelihood(camera_markers_g, particle)
                print(likelihood)
                likelihoods.append(likelihood)
                
                # self.get_logger().info("Camera markers: \n" + str(camera_markers_g))
            # self.get_logger().info(str(likelihoods))
            self.resample_particles(np.array(likelihoods))
        # Calculate change in position based on particle filter
        previous_position = self.position
        self.position = self.calculate_avg_position_yaw()
        if self.camera_previous_time is not None:
            dt = (self.get_clock().now() - self.camera_previous_time).nanoseconds / 1e9
            self.camera_previous_time = self.get_clock().now()
            change_in_position = self.position - previous_position
            velocity_pf = change_in_position / dt
            # Average Imu and particle filter velocity
            self.velocity = velocity_pf*0.2 + self.velocity * 0.8
            self.get_logger().info(str(self.velocity))
        if DISPLAY_PARTICLES:
            self.particles_display()
            
def main(args = None):
    rclpy.init(args=args)   
    node = ParticleFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()