import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu
from .line_detection import LineDetection
from pontus_msgs.msg import ParticleMsg, ParticleList
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Vector3
from builtin_interfaces.msg import Duration

from sensor_msgs.msg import PointCloud2, PointField

import numpy as np
from .pf_constants import *

# ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id world

class Particle:
    def __init__(self, x = 0, y = 0, z = 0, yaw = 0):
        self.position = np.zeros((3,))
        self.position[0] = x
        self.position[1] = y
        self.position[2] = z
        self.yaw = yaw
        
class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__("odometry_node")
        self.get_logger().info("Particle filter node started")
        # Subscribers
        
        # Camera subscription
        # self.camera_sub = self.create_subscription(
        #     Image, 
        #     "/pontus/camera_1", 
        #     self.camera_callback, 
        #     10
        # )
        
        # # Imu subscription
        self.imu_sub = self.create_subscription(
            Imu, 
            "/pontus/imu_0",
            self.imu_callback,
            10
        )
        
        # self.depth_sub = self.create_subscription(
        #     Odometry,
        #     "/pontus/depth_0",
        #     self.depth_callback,
        #     10
        # )

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

        # This will be used to calculate the time between each imu callback
        self.previous_time = self.get_clock().now()
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, POOL_DEPTH])
        self.previous_yaw = 0
        self.imu_batch_size = 0    
        self.linear_acceleration_current = np.zeros((3,))
        
        # Particles
        self.particles = []
        self.init_particles()
        self.particles_display()
        
    # This function is used to publish particles to the particles topic
    # so it can be displayed in Rviz
    def particles_display(self):
        
        # Display particle spheres
        msg = PointCloud2()
        
        particle_positions = [(p.position[0], p.position[1], p.position[2]) for p in self.particles]
        
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
        msg.row_step = msg.point_step * len(self.particles)
        msg.is_dense = True
        msg.data = np.array(particle_positions, dtype=np.float32).tobytes()
        self.particle_pub.publish(msg)
        
        
        # Display yaw
        markers = MarkerArray()
        marker_id = 0
        for particle in self.particles:
            marker = Marker()
            new_yaw = particle.yaw
            marker.header.frame_id = 'map'  # Set the frame ID
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'particle_markers'
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = particle.position[0]
            marker.pose.position.y = particle.position[1]
            marker.pose.position.z = particle.position[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = np.sin(new_yaw/ 2.0)
            marker.pose.orientation.w = np.cos(new_yaw / 2.0)
            marker.scale = Vector3(x=0.2, y=0.02, z=0.02) 
            marker.color = ColorRGBA(r=0.0, g=0.67, b=1.0, a=1.0)  
            marker.lifetime = Duration(sec=0)
            markers.markers.append(marker)
            marker_id += 1

        # Display assumed position
        avg_pos, avg_yaw = self.calculate_avg_position_yaw()
        marker = Marker()
        new_yaw = avg_yaw 
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
        marker.pose.orientation.z = np.sin(new_yaw/ 2.0)
        marker.pose.orientation.w = np.cos(new_yaw / 2.0)
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
        for row in range(num_rows):
            for col in range(num_rows):
                for angle in range(NUM_START_UP_ANGLES):
                    x = row * total_grid_width / num_rows - total_grid_width / 2
                    y = col * total_grid_width / num_rows - total_grid_width / 2
                    # Need to sample betwee 0 and pi/2 for the angle
                    self.particles.append(Particle(x, y, POOL_DEPTH, np.pi / 2 * angle / NUM_START_UP_ANGLES))
    
    # This calculates the average position and yaw of the particles
    # This is what our odometry would be
    def calculate_avg_position_yaw(self):
        avg_position = np.zeros((3,))
        avg_yaw = 0
        for particle in self.particles:
            avg_position += particle.position
            avg_yaw += particle.yaw
        avg_position = avg_position / len(self.particles)
        avg_yaw = avg_yaw / len(self.particles)
        return avg_position, avg_yaw

    # This updates the particles with the new position and yaw
    def update_particles(self, change_in_position, change_in_yaw):
        for particle in self.particles:
            # Update position
            particle.position += change_in_position
            # Update yaw
            particle.yaw = self.yaw_norm(particle.yaw + change_in_yaw)
        self.particles_display()
            
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
        # Get current time
        current_time = self.get_clock().now()    
        # Calculate the change in time
        dt = (current_time - self.previous_time).nanoseconds / 1e9
        self.get_logger().info("dt: " + str(dt))
        
        self.linear_acceleration_current = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z - 9.7999999])

        # Calculate the change in velocity
        # Use average acceleration to gestimate the velocity
        change_in_velocity = self.linear_acceleration_current * dt
        
        # d = vt
        change_in_position = self.velocity  * dt

        # Update current velocity
        self.velocity = self.velocity + change_in_velocity
        
        change_in_yaw = msg.orientation.z - self.previous_yaw
        
        # Update previous variables
        self.previous_time = current_time
        self.previous_yaw = msg.orientation.z
        
        # self.get_logger().info("Change in position: " + str(change_in_position))
        # self.get_logger().info("Change in yaw: " + str(change_in_yaw))
        self.get_logger().info("Change in velocity: " + str(self.linear_acceleration_current))
        self.get_logger().info("Change in velocity: " + str(change_in_velocity))
        # self.get_logger().info("Current velocity: " + str(self.velocity))
        self.linear_acceleration_current = np.zeros((3,))
        self.update_particles(change_in_position, change_in_yaw) 
        
        
def main(args = None):
    rclpy.init(args=args)
    node = ParticleFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()