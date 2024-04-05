import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from .mono_optical_flow_calc import MonoOpticalFlowCalculations
from sensor_msgs.msg import Imu

# Where does this dependency go lol
from message_filters import ApproximateTimeSynchronizer

import numpy as np
from pf_constants import *

class Particle:
    def __init__(self, x = 0, y = 0, z = 0, yaw = 0):
        self.position = np.zeros((3,))
        self.position[0] = x
        self.position[1] = y
        self.position[2] = z
        self.yaw = yaw

class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__("odometry_node_pf")


        self.camera_sub_left = self.create_subscription(
            Image,
            "/pontus/camera_1", # this will get remapped in launch file
        )

        self.camera_sub_right = self.create_subscription(
            Image,
            "/pontus/camera_2", # this will get remapped in launch file
        )

        self.imu_sub = self.create_subscription(
            Imu,
            "/pontus/imu_0",
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            "/pf_odometry",
        )

        # Synchronize the subscriptions
        # This means that the cameras and imu will be synced
        # This is important for odometry
        
        self.sync_sub = ApproximateTimeSynchronizer([self.camera_sub_left, self.camera_sub_right, self.imu_sub], 10, 0.1)
        self.sync_sub.registerCallback(self.motion_update)

        # Used to integrate the IMU data
        # Not sure if this should be set to None first
        # This may cause error
        # The reason for this is that we need to know the previous time for the first iteration
        self.previous_time = self.get_clock().now()
        self.dt = 0
        
        # Assume only rotation in yaw direction
        # self.rotation = np.zeros((1,3))
        self.yaw = 0
        self.position = np.array([0.0, 0.0, 0.0])
        self.position_offset = np.array([0.0, 0.0, 0.0])
        self.yaw_offset = 0

        self.velocity = np.array([0.0, 0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0, 0.0])
        
        self.start_up = 0

        # init particles for start up
        self.particles = []
        total_grid_width = GRID_LINE_LENGTH + GRID_LINE_THICKNESS
        num_rows = np.sqrt(START_UP_ITERATIONS)
        for row in range(num_rows):
            for col in range(num_rows):
                for angle in NUM_START_UP_ANGLES:
                    x = row * total_grid_width / num_rows + total_grid_width / 20
                    y = col * total_grid_width / num_rows + total_grid_width / 20
                    # Need to sample betwee 0 and pi/2 for the angle
                    self.particles.append(Particle(x, y, DEPTH, np.pi / 2 * angle / NUM_START_UP_ANGLES))

    def get_odom_from_imu(self, msg):
        # Get current time
        current_time = self.get_clock().now()    
        # Calculate the change in time
        self.dt = current_time - self.previous_time
        
        # Calculate the change in velocity
        linear_acceleration_current = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        # Use average acceleration to gestimate the velocity
        change_in_velocity = 1/2 * (self.acceleration + linear_acceleration_current) * self.dt
        
        # Update current velocity
        current_velocity = self.velocity + change_in_velocity
        # d = vt
        change_in_position = current_velocity * self.dt
        
        change_in_yaw = msg.orientation.z - self.previous_yaw
        
        # Update previous variables
        self.previous_time = current_time
        self.previous_yaw = msg.orientation.z
        return change_in_position[0], change_in_position[1], change_in_position[2], change_in_yaw
    
    # Samples from Gaussian distribution to simulate noise in odometry
    def guass_noise_translational(self):
        sample = np.random.normal(TRANS_MU, TRANS_SIGMA)
        return sample
    def guass_noise_rotational(self):
        sample = np.random.normal(ROTATE_MU, ROTATE_SIGMA)
        return max(min(sample, ROTATE_SAMPLE_MAX), -ROTATE_SAMPLE_MAX)

    # Prevents particles from being sampled too far from the assumed position
    def cuttoff_distance(self, particle):
        # If the distance of the particle between the assumed position is greater than the CUTOFF_DISTANCE,
        # then the particle is set to the max_distance_particle 
        if np.sum(np.square(particle.position - self.position)) > CUTOFF_DISTANCE:
            # Get unit vector from assumed position to particle
            unit_vector = np.lingalg.norm(particle.position - self.position)
            # Adjust location with magnitude CUTOFF_DISTANCE
            particle.position = self.position + CUTOFF_DISTANCE * unit_vector
        return particle
            
    # Ensure that the given angle is within the range (-pi, pi]
    def rotation_range(self, particle):
        if particle.yaw > np.pi:
            particle.yaw -= 2 * np.pi
        elif particle.yaw <= -np.pi:
            particle.yaw += 2 * np.pi
        return particle
    
    # Updates the position of each particle with IMU odometry     
    def update_particles(self, dx, dy, dz, dr):
        # Iterate through each particle
        for particle in self.particles:
            # Add noise to the particle to simulate the error/uncertainty in the odometry
            particle.x = particle.x + dx + self.guass_noise_translational()
            particle.y = particle.y + dy + self.guass_noise_translational()
            particle.z = particle.z + dz + self.guass_noise_translational()
            particle.yaw = particle.yaw + dr + self.guass_noise_rotational()
            # Prevents particles from wandering too far 
            # This must be done because the grid map is a very repetitive pattern
            # Particles sampled too far from the assumed position have a chance to match the pattern
            particle = self.cuttoff_distance(particle)
            
            # Ensures that the yaw is within the range (-pi, pi]
            particle = self.rotation_range(particle)

    # Resampling based on the likelihood of each particle
    def resample_particles(self, particle_likelihoods):
        # Normalize the likelihoods
        particle_likelihoods = particle_likelihoods / np.sum(particle_likelihoods)
        # Resample the particles
        self.particles = np.random.choice(a = self.particles, size = NUM_PARTICLES, p = particle_likelihoods)

    # Calculates the average position of the particles to get the estimated position of the robot
    def calculate_avg_position_yaw(self):
        avg_position = np.zeros((3,))
        avg_yaw = 0
        for particle in self.particles:
            avg_position += particle.position
            avg_yaw += particle.yaw
        avg_position = avg_position / NUM_PARTICLES
        avg_yaw = avg_yaw / NUM_PARTICLES
        return avg_position, avg_yaw

    # Get the markers from the particle
    # Idea:
    # Take the maximal distance from the center of the particle in all four directions
    # This can be calculated using horizontal and vertical FOV
    def get_markers_from_particle(self, particle):
        horizontal_FOV_magnitude = particle.depth * np.tan(np.radians(HORIZONTAL_FOV / 2))
        vertical_FOV_magnitude = particle.depth * np.tan(np.radians(VERTICAL_FOV / 2))
        
        # This probably needs to be fixed 
        horizontal_vector = horizontal_FOV_magnitude * np.array([abs(np.cos(particle.yaw)), abs(np.sin(particle.yaw))])
        vertical_vector = vertical_FOV_magnitude * np.array([abs(np.sin(particle.yaw)), abs(np.cos(particle.yaw))])

        true_x_max = 0
        true_y_max = 0

        # Assuming that the values above are correct
        position_x = particle.position[0]
        position_y = particle.positoin[1]

        end_position_x = position_x + true_x_max
        end_position_y = position_y + true_y_max

        

    
    # Given the markers from the particle and the caemra markers
    # Generate pairs of markers that are close to each other 
    def generate_marker_pairs_and_calculate_likelihood(self, particle_markers, camera_markers):
        l = 1
        # Keep iterating until one of the lists run out of markers
        # Nothing left to match
        while len(particle_markers) > 0 and len(camera_markers) > 0:
            max_prob = -np.inf
            max_pair = None
            # Iterate through each possible pair
            for particle_marker in particle_markers:
                for camera_marker in camera_markers:
                    # Calculate the distance between the two markers
                    prob = self.calculate_likelihood(particle_marker, camera_marker)
                    # If the distance is less than the minimum distance, update the minimum distance and pair
                    if prob > max_prob:
                        max_prob = prob
                        max_pair = (particle_marker, camera_marker)
            # Remove the pair from the list of markers
            particle_markers.remove(max_pair[0])
            camera_markers.remove(max_pair[1])
            
            l = l * max_prob
        return l
    
    # Calculates the probabily of two given markers being the same
    def calculate_likelihood(self, particle_marker, camera_marker):
        grid_distance = np.sum(np.square(particle_marker[:3] - camera_marker[:3]))
        angle_distance = self.rotation_range(np.abs(particle_marker[3] - camera_marker[3]))
        return np.exp(-grid_distance**2 / (2 * MARKER_DISTANCE_SIGMA**2) - angle_distance**2 / (2 * MARKER_ANGLE_SIGMA**2))
    
    # Calculates the likelihood of a particle given the current camera image
    def get_particle_likelihood(self, particle, camera_markers):
        particle_makers = self.get_markers_from_particle(particle)
        l = self.generate_marker_pairs_and_calculate_likelihood(particle_makers, camera_markers)
        return l
    
    def get_observation_from_camera(self, left_image, right_image):
        pass
    
    # Rotates point
    def rotate_point(self, x, y, theta):
        new_x = x * np.cos(theta) - y * np.sin(theta)
        new_y = x * np.sin(theta) + y * np.cos(theta)
        return new_x, new_y
    
    # Converts the camera observations into the global frame and into cartesian coordinates
    
    # IDEA #1
    
    # def convert_to_global_frame(self, camera_markers_r):
    #     camera_markers_g = []
    #     for marker in camera_markers_r:
    #         # Extra angle and stiacne
    #         angle = marker[0]
    #         distance = marker[1]
    #         # Convert to cartesian coordinates
    #         x = distance * np.cos(angle)
    #         y = distance * np.sin(angle)
    #         # Rotate the point to the global frame
    #         x_g, y_g = self.rotate_point(x, y, self.yaw)
    #         # Translate the point to the global frame
    #         x_g += self.position[0]
    #         y_g += self.position[1]
    #         camera_markers_g.append((x_g, y_g, self.position[2]))
    #     return camera_markers_g
    
    # IDEA #2
    
    def convert_to_global_frame(self, camera_markers_r, particle):
        camera_markers_g = []
        for marker in camera_markers_r:
            x = marker[0]
            y = marker[1]
            z = marker[2]
            theta = marker[3]
            # I think this is right?
            theta = theta + particle.yaw
            # Rotate the point to the global frame
            x_g, y_g = self.rotate_point(x, y, particle.yaw)
            # Translate the point to the global frame
            x_g = x_g + particle.position[0]
            y_g = y_g + particle.position[1]
            z_g = z
            camera_markers_g.append(np.array([x_g, y_g, z_g, theta]))
        return camera_markers_g

    # Run the particle filter every time we get an input of imu + camera data
    # All angles will be in radians
    # All global frame coordinates will reflect real world units, therefore, meters
    def motion_update(self, left_image_msg, right_image_msg, imu_msg):
        # Startup protocol
        # When the robot starts, we need to know where the robot currently is within the map to set its origin
        # Steps
        # Basically run the particle filter
        # Assume that the map has already been set up
        # Sample 100 particle uniformly within a grid
        # Calculate the likelihood of each particle given the current camera image
        # Resample the particles based on the likelihoods

        # Keep on running until the start_up iterations are done
        if self.start_up < START_UP_ITERATIONS:
            self.start_up += 1

        # When we have met the start up iterations, we can start running the particle filter.
        # Here we need to set the offsets for the position.
        # That is, the original position of the robot should be 0,0.
        # However, the origin of the map will be considered the center of the pre configured map
        # (Center of a grid)
        # Therefore the robot's position may not necessarly be the middle of the map
        # We need an offset here to account for this
        # This means that when we return the position of the robot, we return
        # self.position - self.offset
        elif self.start_up == START_UP_ITERATIONS:
            self.start_up = START_UP_ITERATIONS + 1
            # Reset all particles to the origin
            new_particles = []
            for particle in NUM_PARTICLES:
                new_particles.append(Particle(self.position[0], self.position[1], DEPTH, self.yaw))
            self.particles = new_particles

            # Set the offsets
            self.x_offset = self.position[0]
            self.y_offset = self.position[1]
            self.yaw_offset = self.yaw

        # Get odometry from IMU
        dx_r, dy_r, dz_g, dr_g = self.get_odom_from_imu(imu_msg)
        
        # Rotate the odometry to the global frame
        # Not sure if we should update yaw here before rotating the point
        dx_g, dy_g = self.rotate_point(dx_r, dy_r, self.yaw + dr_g)
        
        # Update the particles with imu odometry
        self.update_particles(dx_g, dy_g, dz_g, dr_g)
        
        # IDEA #1
        # ----------
        # Get angle and intecept from images
        # current_observations will return
        # [ ( angle -> angle of the line, 
        #     distance -> distance to the line
        #   )
        #   ...
        # ]
        # This assumes that the distance is the true distance to the line
        # This true distance would be a 2D polar coordinates.
        # We would rely on IMU and depth sensor to get the current depth
        
        # IDEA #2
        # ----------
        # Get angle and intercept from images
        # Calculate cartesian coordinates
        # Scale points to match real units using focal lengths of x and y
        # Do this for both cameras
        # When you have x_real_0 and y_real_0, and x_real_1 and y_real_1, 
        # You can calculate the disparity between the two points
        # This disparity will give you the distance to the point
        # Disparity = x_real_0 - x_real_1
        # distance = f * (B) / disparity
        # To get z, we can do
        # dist_2d = sqrt(x^2 + y^2)
        # z = sqrt(distance^2 - dist_2d^2)
        # current observations would return
        # [ ( x -> x coordinate in real world in robot frame,
        #     y -> y coordinate in real world in robot frame,
        #     z -> z coordinate in real world in robot frame,
        #     theta -> angle of the line in robot frame
        #   )
        #   ...
        # ]
        camera_markers_r = self.get_observation_from_camera(left_image_msg, right_image_msg)
        
        # Convert the camera observations into the global frame and into cartesian coordinates
        # camera_markers_g would return
        # [ np.array([ x -> x coordinate in global frame,
                #      y -> y coordinate in global frame,
                #      z -> z coordinate in global frame,
                #      theta -> angle of the line in global frame,
                #    ]),
        #   ...
        # ]
        
        # Calculate prob of each particle given the current camera image
        # P(Particle | Camera Image)
        particle_likelihoods = []
        for particle in self.particles:
            camera_markers_g = self.convert_to_global_frame(camera_markers_r, particle)
            likelihood = self.get_particle_likelihood(particle, camera_markers_g)
            particle_likelihoods.append(likelihood)
        
        # Resample particles based on likelihoods
        self.resample_particles(particle_likelihoods)

        # Calculate the average position of the particles
        avg_position, avg_yaw = self.calculate_avg_position_yaw()
        
        # Update velocity and yaw?
        self.velocity = (avg_position - self.position) / self.dt
        self.previous_yaw = avg_yaw
        
        # Update the assumed position of the robot
        self.position = avg_position
        self.yaw = avg_yaw
        
        # Publish the odometry
        # These values are centered to be the starting point of the sub
        if self.start_up >= self.START_UP_ITERATIONS:
            odom_msg = Odometry()
            odom_msg.pose.pose.position.x = avg_position[0] - self.x_offset
            odom_msg.pose.pose.position.y = avg_position[1] - self.y_offset
            odom_msg.pose.pose.position.z = avg_position[2] 
            odom_msg.pose.pose.orientation.z = avg_yaw - self.yaw_offset
            self.odom_pub.publish(odom_msg)

def main(args = None):
    rclpy.init(args=args)
    node = ParticleFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()