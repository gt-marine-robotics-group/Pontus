import rclpy
from rclpy.node import Node
import numpy as np
import yaml
import math

import tf2_ros


from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class ThrusterController(Node):

    class Thruster:
        def __init__(self, Node, thruster_id, max_thrust,  pos, quaternion):
            self.pub = Node.create_publisher(Float64, 'pontus/thruster_' + str(thruster_id) + '/cmd_thrust', 10)

            self.max_thrust = max_thrust

            q = quaternion
            axis = np.array([0, 0, 1]) # Our thrusters rotate the propeller around the z axis

            # Build the rows of the rotation matrix from the quaternion
            r_row_0 = np.array([
                2 * (q[3] * q[3] + q[0] * q[0]) - 1,
                2 * (q[0] * q[1] - q[3] * q[2]),
                2 * (q[0] * q[2] + q[3] * q[1])
            ])

            r_row_1 = np.array([
                2 * (q[0] * q[1] + q[3] * q[2]),
                2 * (q[3] * q[3] + q[1] * q[1]) - 1,
                2 * (q[1] * q[2] - q[3] * q[0])
            ])

            r_row_2 = np.array([
                2 * (q[0] * q[2] - q[3] * q[1]),
                2 * (q[1] * q[2] + q[3] * q[0]),
                2 * (q[3] * q[3] + q[2] * q[2]) - 1
            ])

            rotation_matrix = np.row_stack([r_row_0, r_row_1, r_row_2])

            # Get the column of the rotation_matrix corresponding to the axis our propeller generates thrust on
            thrust_effect = rotation_matrix.dot(axis.transpose())

            torque_effect = np.cross(pos, thrust_effect)

            # The vector that converts the thruster generated force/torque into body force/torque
            self.jacobian_column = np.hstack((
                thrust_effect, torque_effect)).transpose()


        def set_thrust(self, value):
            msg = Float64()

            # Clamp values to max thrust
            if abs(value) > self.max_thrust:
              value = math.copysign(self.max_thrust, value)

            msg.data = value
            self.pub.publish(msg)


    def __init__(self):
        super().__init__('thruster_controller')

        #TODO: make these parameters or something
        self.vehicle_name = 'pontus'
        self.max_thrust = 52.0

        #TODO: These could be pulled from the robot description topic

        mass = 18.65
        # Moments of inertia of the vehicle (these also need to be properly calculated)
        ixx = 55
        ixy = 0
        ixz = 0
        iyy = 0.585
        iyz = 0
        izz = 55

        self.inertial_tensor = np.array([
            [ixx, ixy, ixz],
            [ixy, iyy, iyz],
            [ixz, iyz, izz]
        ])
        # Full inertial matrix of the vehicle
        self.inertial_matrix = np.vstack([
            np.hstack([mass * np.identity(3), np.zeros([3,3])]),
            np.hstack([np.zeros([3,3]), self.inertial_tensor])
        ])


        # Class variables
        self.thrusters = []
        self.jacobian_matrix = None
        self.inverse_jacobian = None


        # ROS infrastructure

        # Body acceleration command
        self.cmd_accel_sub = self.create_subscription(
          Twist,
          '/cmd_accel',
          self.cmd_accel_callback,
          10)

        # TF listener to get the positions of the thrusters
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Attempt to generate thrusters from TF once a second
        # TODO: Should this keep monitoring the TF for updates or just stop once it sees the initial TF?
        self.tf_timer = self.create_timer(1.0, self.generate_thrusters)

    def cmd_accel_callback(self, msg):
        if len(self.thrusters) == 0:
            return # TF isn't available yet

        # Build the vector of desired accelerations [x y z roll pitch yaw]
        linear = np.array((msg.linear.x, msg.linear.y, msg.linear.z))
        angular = np.array((msg.angular.x, msg.angular.y, msg.angular.z))
        accel = np.hstack((linear, angular)).transpose()

        # Calculate forces and torques required to generate acceleration
        force_torque = self.inertial_matrix.dot(accel)

        # Calculate individual thruster force to generate desired total force
        thruster_forces = self.inverse_jacobian.dot(force_torque)

        # Handle matching sets of thrusters (vertical and horizontal) independently.
        # Otherwise trying to command full forward velocity could cause our 
        # vertical thrusters to decrease their output making us sink
        for i in range(2):

            # Calculate the highest commanded output thrust so we can scale
            # all of the thrusters down if we exceed max thrust.
            # This helps pevent issues with multiple thrusters being saturated
            # leading to the vehicle moving in unintended directions
            max_commanded = np.max(thruster_forces)
            scale = (max_commanded / self.max_thrust) if max_commanded > self.max_thrust else 1.0

            # Publish thruster commands
            for index in range(4*i, (4*i) + 4):
                self.thrusters[index].set_thrust(thruster_forces[index] / scale)


    def generate_thrusters(self):
        tf_string = self.tf_buffer.all_frames_as_yaml()
        number_thrusters = tf_string.count("thruster_")

        # Failed to find thrusters
        if number_thrusters == 0:
          return

        print("detected ", number_thrusters, " thrusters")

        base_frame = 'base_link'
        self.jacobian_matrix = np.zeros((6, number_thrusters))

        for i in range(0, number_thrusters):
            transform = self.tf_buffer.lookup_transform(base_frame, 'thruster_' + str(i), rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))

            pos = np.array([
                               transform.transform.translation.x,
                               transform.transform.translation.y,
                               transform.transform.translation.z])

            quaternion = np.array([
                                      transform.transform.rotation.x,
                                      transform.transform.rotation.y,
                                      transform.transform.rotation.z,
                                      transform.transform.rotation.w])

            self.thrusters.append(self.Thruster(self, i, self.max_thrust, pos, quaternion))
            self.jacobian_matrix[:, i] = self.thrusters[i].jacobian_column

        # Eliminate small values
        self.jacobian_matrix[np.abs(self.jacobian_matrix) < 1e-3] = 0.0

        # Calculate the inverse Jacobian.
        self.inverse_jacobian = np.linalg.pinv(self.jacobian_matrix)

        # Stop trying to generate thrusters
        self.tf_timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    thruster_Node = ThrusterController()
    rclpy.spin(thruster_Node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
