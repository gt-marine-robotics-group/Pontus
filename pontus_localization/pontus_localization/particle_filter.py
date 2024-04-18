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
        
        # # Imu subscription
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
        self.depth = POOL_DEPTH
        self.linear_acceleration_current = np.zeros((4,))
        self.previous_angular_velocity = 0
        self.previous_time = None

        # Particles
        self.particles = np.zeros((400, 4))
        self.init_particles()
        if DISPLAY_PARTICLES:
            self.particles_display()
        
        
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
        avg_pos = self.calculate_avg_position_yaw()
        marker = Marker()
        yaw = avg_pos[3]
        marker.header.frame_id = 'map'  # Set the frame ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'particle_markers'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = avg_pos[0]
        marker.pose.position.y = avg_pos[1]
        marker.pose.position.z = avg_pos[2]
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

    ######################
    # Particle related functions
    ######################
    # This function will uniformly initialize the particles in the grid
    # This is used to detect where the robot starts
    def init_particles(self):
        total_grid_width = GRID_LINE_LENGTH + GRID_LINE_THICKNESS
        num_rows = int(np.sqrt(START_UP_ITERATIONS))
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
    def update_particles(self, change_in_position): 
        self.particles += change_in_position
        self.particles[:,2] = self.depth    
            
    # This function ensures that particle yaw are in the range [-pi, pi)
    def yaw_norm(self, yaw):
        while yaw >= np.pi:
            yaw -= 2 * np.pi
        while yaw < -np.pi:
            yaw += 2 * np.pi
        return yaw

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
        marker.pose.position.z = msg.pose.pose.position.z
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
        self.get_logger().info("dt: " + str(dt))
        self.previous_time = current_time

        if DISPLAY_PARTICLES:
            self.particles_display()

        # Get imu data and interpolate the data to calculate change in position
        self.linear_acceleration_current = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z - 9.7999999])
        change_in_position = self.interpolate_imu_data(self.linear_acceleration_current, self.acceleration, msg.angular_velocity.z, self.previous_angular_velocity, dt, 0.05)
        self.acceleration = self.linear_acceleration_current
        self.previous_angular_velocity = msg.angular_velocity.z
        
        # Update each particles position with the imu update
        self.update_particles(change_in_position) 
        return

        self.get_logger().info("Change in velocity: " + str(self.linear_acceleration_current))
        self.get_logger().info("Change in velocity: " + str(change_in_velocity))
    
    # Depth callback
    # This callback function will update the depth of the sub
    def depth_callback(self, msg):
        self.depth = msg.pose.pose.position.z + POOL_DEPTH
        return
    
    # Camera callback
    def camera_callback(self, msg):
        stuff = self.get_observation_from_camera(msg)

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
            change_in_velocity = current_acceleration / 2 * time_step
            change_in_velocity = np.append(change_in_velocity, 0)
            # Update new velocity
            self.velocity = self.velocity + change_in_velocity
            # Calculate current angular velocity
            angular_velocity = previous_angular_velocity + change_in_angular_velocity * current_step
            self.velocity[3] = angular_velocity / 2
            # Calculate change in position
            change_in_position_particle = np.tile((self.velocity * time_step), (400,1))
            # Rotate the change in position to the global frame
            change_in_position_global = self.rotate_points(change_in_position_particle, self.particles[:,3])
            change_in_position = change_in_position + change_in_position_global
            current_step = current_step + time_step
        return change_in_position

    # This function will rotate a point in the robot frame to the global frame
    def rotate_points(self, points, yaws):
        # Extract change in x and change in y
        dx_dy = points[:, :2]
        rotated_dx_dy = np.zeros_like(dx_dy)
        # Rotate x and y points
        rotated_dx_dy[:, 0] = dx_dy[:, 0] * np.cos(yaws) - dx_dy[:, 1] * np.sin(yaws)
        rotated_dx_dy[:, 1] = dx_dy[:, 0] * np.sin(yaws) + dx_dy[:, 1] * np.cos(yaws)
        # Add depth and rotation back
        rotated_points = np.concatenate((rotated_dx_dy, points[:, 2:]), axis=1)
        return rotated_points

    # This function will get the observation from the camera
    # This will return the an array of lines with their distance and angle
    def get_observation_from_camera(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'mono8')
        lines = LineDetection.get_lines(cv_image, self.depth)
        self.get_logger().info(str(lines))
        return lines

def main(args = None):
    rclpy.init(args=args)
    node = ParticleFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()