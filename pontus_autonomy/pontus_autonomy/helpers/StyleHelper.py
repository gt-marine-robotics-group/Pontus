import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.task import Future
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from scipy.spatial.transform import Rotation


from pontus_msgs.msg import CommandMode
from pontus_msgs.action import GoToPose
from pontus_msgs.srv import GetPathToObject
from pontus_controller.position_controller import PositionControllerState
from pontus_autonomy.base_run import BaseTask

class StyleHelper:
    def __init__(self, node: BaseTask, axis: int = 2, num_times: int = 2):
        self.node = node
        self.axis = axis # 0,1,2 = roll, pitch, yaw
        self.num_times = num_times

        self.odom_sub = node.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odometry_callback,
            10
        )

        self.command_pub = node.create_publisher(CommandMode, "/command_mode", 10)
        self.cmd_pose_pub = node.create_publisher(Pose, "/cmd_pos", 10)
        self.cmd_vel_pub = node.create_publisher(Twist, "cmd_vel", 10)

        self.odom: Odometry | None = None
        self.hold_point: Pose = None # Only uses x,y,z and yaw

        self.times_styled: int = 0
        self.enabled = False
        self.completed = False
        
        self.actively_styling_ = False
        self.rotation_counter_: int = 0
        self.style_vel_ = np.array([ # TODO: Tune these
            2.0,
            2.0,
            2.0
        ])


    def odometry_callback(self, msg):
        self.odom = msg

        if not self.enabled:
            return

        if self.hold_point is None:
            self.set_hold_point()

        hold_yaw = Rotation.from_quat([
                self.hold_point.orientation.x,
                self.hold_point.orientation.y,
                self.hold_point.orientation.z,
                self.hold_point.orientation.w,
            ]).as_euler("xyz")[0]

        # Roll/Pitch commands must return to level
        hold_state = np.array([
            self.hold_point.position.x,
            self.hold_point.position.y,
            self.hold_point.position.z,
            0.0,
            0.0,
            hold_yaw
        ])

        lin_state = np.array([
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y,
            self.odom.pose.pose.position.z,
            self.odom.twist.twist.linear.x,
            self.odom.twist.twist.linear.y,
            self.odom.twist.twist.linear.z
        ])

        angular_state = Rotation.from_quat([
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w,
        ]).as_euler("xyz")

        if self.actively_styling_:
            self.style_()

            angle_target = angular_state[self.axis] + (self.rotation_counter_ + 1) * np.pi
            diff = angle_target - hold_state[3 + self.axis]
            angular_dist = (diff + np.pi) % (2 * np.pi) - np.pi

            if abs(angular_dist) < (np.pi / 8):
                self.rotation_counter_ += 1

            if self.rotation_counter_ >= 2:
                self.times_styled += 1
                self.rotation_counter_ = 0
                self.actively_styling_ = False # Stabilize between each full rotation

        else:
            self.stabilize_()

            lin_dist = np.linalg.norm(lin_state[0:2] - hold_state[0:2])
            depth_dist = abs(lin_state[2] - hold_state[2])

            lin_vel = np.linalg.norm(lin_state[3:5])
            depth_vel = abs(lin_state[5])

            dist_good = lin_dist < 0.3 and depth_dist < 0.3
            vel_good = lin_vel < 0.15 and depth_vel < 0.1

            if dist_good and vel_good:
                # Styling complete, maintain position while waiting for next command
                if self.times_styled >= self.num_times:
                    self.completed = True
                    self.stabilize_()
                else:
                    self.actively_styling_ = True
            else:
                self.stabilize_()

    def style_(self):
        command_msg = CommandMode()
        command_msg.command_mode = CommandMode.VELOCITY_HOLD_POSITION
        self.command_pub.publish(command_msg)

        vel_msg = Twist()
        target_vel = self.style_vel_[self.axis]

        match self.axis:
            case 0:
                vel_msg.angular.x = target_vel
            case 1:
                vel_msg.angular.y = target_vel
            case 2 | _: # 2 or default
                vel_msg.angular.z = target_vel

        self.cmd_vel_pub.publish(vel_msg)

    def stabilize_(self):
        command_msg = CommandMode()
        command_msg.command_mode = CommandMode.POSITION_WITH_STRAFE
        self.command_pub.publish(command_msg)

    def set_hold_point(self, hold_point = None):
        if (hold_point is None):
            self.hold_point = self.odom.pose.pose
        else:
            self.hold_point = hold_point

    def run_style(self):
        self.enabled = True

    def stop_style(self):
        self.enabled = False