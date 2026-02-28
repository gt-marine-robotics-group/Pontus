import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation
from typing import Optional, List, Dict
from pontus_controller.PID import PID

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from pontus_msgs.msg import CommandMode

from pontus_description.vehicle_params_helper import VehicleParams

class VelocityNode(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        self.cmd_linear = np.zeros(3)
        self.cmd_angular = np.zeros(3)

        self.prev_time = self.get_clock().now()

        param_list = (
            ('default_command_mode', CommandMode.ESTOP),
            ('x_kp', 10.0),
            ('x_ki', 0.0),
            ('x_kd', 0.0),
            ('y_kp', 10.0),
            ('y_ki', 0.0),
            ('y_kd', 0.0),
            ('z_kp', 10.0),
            ('z_ki', 0.0),
            ('z_kd', 0.0),
            ('r_kp', 1.0),
            ('r_ki', 0.0),
            ('r_kd', 0.0),
            ('p_kp', 0.01),
            ('p_ki', 0.0),
            ('p_kd', 0.0),
            ('yaw_kp', 0.2),
            ('yaw_ki', 0.0),
            ('yaw_kd', 0.05),
            ('x_C', 0.82),
            ('y_C', 1.2),
            ('z_C', 1.2),
            ('r_C', 0.0),
            ('p_C', 0.0),
            ('yaw_C', 4.0),
            ('linear_drag_gain', 1.0),
            ('angular_drag_gain', 1.0),
            ('direct_mode_linear_gain', 12.0),
            ('direct_mode_angular_gain', 0.35)
        )

        self.add_on_set_parameters_callback(self.param_callback)
        self.declare_parameters(namespace="", parameters=param_list)

        self.vehicle_params = VehicleParams(self)

        # TODO: Tune these
        self.pid_linear = [
          PID(self.x_kp, self.x_ki, self.x_kd),  # X (Surge)
          PID(self.y_kp, self.y_ki, self.y_kd, windup_max=10),  # Y (Sway)
          PID(self.z_kp, self.z_ki, self.z_kd)  # Z (Heave)
        ]

        self.pid_angular = [
          PID(self.r_kp, self.r_ki, self.r_kd),  # Roll
          PID(self.p_kp, self.p_ki, self.p_kd),  # Pitch
          PID(self.yaw_kp, self.yaw_ki, self.yaw_kd, windup_max=1)  # Yaw
        ]

        self.command_mode = self.default_command_mode

        # ROS infrastructure
        self.command_mode_sub = self.create_subscription(
            CommandMode,
            '/command_mode',
            self.command_mode_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
          Twist,
          '/cmd_vel_fused',
          self.cmd_vel_callback,
          10)

        self.odom_sub = self.create_subscription(
          Odometry,
          '/pontus/odometry',
          self.odometry_callback,
          10)

        self.cmd_accel_pub = self.create_publisher(Twist, '/cmd_accel', 10)

    def command_mode_callback(self, msg: CommandMode) -> None:
        """
        Set the specific command mode

        Args:
        ----
        msg (CommandMode): Which method of selecting/combining joystick and autonomy commands to use:
            See the CommandMode Message

        Return:
        ------
        None

        """

        if self.command_mode != msg.command_mode:
            self.command_mode = msg.command_mode

            # Send zero command to stop thrusters on first frame of estop
            if self.command_mode == CommandMode.ESTOP:
                self.cmd_accel_pub.publish(Twist())

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Save command velocity to then use in odometry callback.
        Also handles direct control by immediately publishing command

        Args:
        ----
        msg (Twist): the desired twist

        Return:
        ------
        None

        """
        self.cmd_linear = np.array([
           msg.linear.x,
           msg.linear.y,
           msg.linear.z,
        ])

        self.cmd_angular = np.array([
           msg.angular.x,
           msg.angular.y,
           msg.angular.z,
        ])

        if self.command_mode == CommandMode.DIRECT_CONTROL:
          direct_command = Twist()
          direct_command.linear.x = msg.linear.x * self.direct_mode_linear_gain
          direct_command.linear.y = msg.linear.y * self.direct_mode_linear_gain
          direct_command.linear.z = msg.linear.z * self.direct_mode_linear_gain
          direct_command.angular.x = msg.angular.x * self.direct_mode_angular_gain
          direct_command.angular.y = msg.angular.y * self.direct_mode_angular_gain
          direct_command.angular.z = msg.angular.z * self.direct_mode_angular_gain

          self.cmd_accel_pub.publish(direct_command)

    def odometry_callback(self, msg: Odometry) -> None:
        """
        Use odometry as feedback to command a desired acceleration to reach the commanded velocity.

        Args:
        ----
        msg (Odometry): our current odometry

        Return:
        ------
        None

        """

        # Direct control is handled in the cmd_vel callback, ESTOP is handled by command_mode_callback
        if self.command_mode == CommandMode.DIRECT_CONTROL or self.command_mode == CommandMode.ESTOP:
            return

        # Get the current velocities from odometry
        v_linear = np.array([msg.twist.twist.linear.x,
                             msg.twist.twist.linear.y,
                             msg.twist.twist.linear.z])
        v_angular = np.array([msg.twist.twist.angular.x,
                              msg.twist.twist.angular.y,
                              msg.twist.twist.angular.z])

        linear_ff, angular_ff = self.calculate_feedforward(msg)

        # Compute the error between desired velocity and current velocity
        linear_err = self.cmd_linear - v_linear
        angular_err = self.cmd_angular - v_angular

        # Compute and publish the body accelerations
        msg = Twist()
        dt = self.get_clock().now() - self.prev_time

        msg.linear.x = linear_ff[0] + self.pid_linear[0](linear_err[0], dt)
        msg.linear.y = linear_ff[1] + self.pid_linear[1](linear_err[1], dt)
        msg.linear.z = linear_ff[2] + self.pid_linear[2](linear_err[2], dt)

        msg.angular.x = angular_ff[0] + self.pid_angular[0](angular_err[0], dt)
        msg.angular.y = angular_ff[1] + self.pid_angular[1](angular_err[1], dt)
        msg.angular.z = angular_ff[2] + self.pid_angular[2](angular_err[2], dt)

        self.prev_time = self.get_clock().now()

        self.cmd_accel_pub.publish(msg)

    def calculate_feedforward(self, odom: Odometry) -> tuple[np.ndarray, np.ndarray]:
        """
        Calculates the feedforward term for each degree of freedom

        Args:
        ----
        msg (Twist): our current velocities

        Return:
        ------
        np.ndarray: the linear feedforward [x, y, z]
        np.ndarray: the angular feedfoward [r, p, y]

        """
        # Sub Coefficients
        cross_sectional_area = np.array([
           np.pi * (self.vehicle_params.radius) ** 2,
           2.0 * self.vehicle_params.length * self.vehicle_params.radius,
           2.0 * self.vehicle_params.length * self.vehicle_params.radius
        ])
        sub_volume = self.vehicle_params.volume

        # Drag Coefficients
        # Values taken from: https://phys.libretexts.org/Bookshelves/Classical_Mechanics/Classical_Mechanics_(Dourmashkin)/08%3A_Applications_of_Newtons_Second_Law/8.06%3A_Drag_Forces_in_Fluids  # noqa: E501
        C = np.array([
           self.x_C,
           self.y_C,
           self.z_C,
           self.r_C,
           self.p_C,
           self.yaw_C
        ]) 

        # Calculate buoyancy force
        # F = pVg
        f_buoyancy = self.vehicle_params.fluid_density * sub_volume * self.vehicle_params.gravity
        # F = mg
        f_gravity = self.vehicle_params.mass * self.vehicle_params.gravity
        f_net = f_buoyancy - f_gravity
        world_acceleration_buoyancy = np.array([0.0, 0.0, f_net / self.vehicle_params.mass])
        body_rotation = Rotation.from_quat(np.array([
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ]))

        body_acceleration_buoyancy = body_rotation.inv().apply(world_acceleration_buoyancy)

        # TODO: The feed forward terms might also want to take into account Added Mass

        # Linear Drag: F = 1/2 CpAv^2
        # Approximated as a cylinder
        linear_f_drag = 0.5 * C[0:3] * self.vehicle_params.fluid_density * cross_sectional_area \
          * (np.sign(self.cmd_linear) * self.cmd_linear ** 2)

        # Rotational Drag: TODO: Estimate drag coefficients
        # TODO: Not sure this equation is correct, might be better modeled
        # as an integral of the linear velocities of the tube along its length
        angular_f_drag = np.copysign(C[3:] * self.cmd_angular ** 2, self.cmd_angular)

        # Compute Feed Forward Terms
        linear_ff = -body_acceleration_buoyancy + (self.linear_drag_gain * linear_f_drag / self.vehicle_params.mass)
        angular_ff = (self.angular_drag_gain * angular_f_drag / self.vehicle_params.mass)

        return linear_ff, angular_ff

    def param_callback(self, params: Dict[str, any]) -> SetParametersResult:
        """
        Handles ROS2 param changes to update the nodes member variables as well
        as the internal PID parameters

        Args:
        ----
        params (dict): A dictionary of parameter values to set on the class

        Return:
        ------
        SetParametersResult

        """
        for param in params:
            setattr(self, param.name, param.value)

            if hasattr(self, "pid_linear") and hasattr(self, "pid_angular"):
                split = param.name.split("_")
                if len(split) <= 1: break
                dof = split[0]
                gain = split[1]

                pid_obj = None
                match dof:
                    case "x":
                        pid_obj = self.pid_linear[0]
                    case "y":
                        pid_obj = self.pid_linear[1]
                    case "z":
                        pid_obj = self.pid_linear[2]
                    case "r":
                        pid_obj = self.pid_angular[0]
                    case "p":
                        pid_obj = self.pid_angular[1]
                    case "yaw":
                        pid_obj = self.pid_angular[2]

                if (pid_obj is not None):
                    setattr(pid_obj, gain, param.value)

        return SetParametersResult(successful=True)

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    velocity_Node = VelocityNode()
    rclpy.spin(velocity_Node)
    rclpy.shutdown()
