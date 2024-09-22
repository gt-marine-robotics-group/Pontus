import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from enum import Enum
import numpy as np

# ROS Messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from pontus_autonomy.tasks.base_task import BaseTask

class State(Enum):
    Surfacing = 1
    HoveringAtSurface = 2


# Task to immediately return to surface for the end of a run.
# TODO: create alternate task to surface at the target for extra points
class SurfaceTask(BaseTask):

    def __init__(self):
        super().__init__("Surface_Task")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        self.state = State.Surfacing

        # This might need to be tweaked if we ever start accounting for the vertical offset on the pressure sensor
        self.surfaced_threshold = -0.4

        # TODO: if the DVL depth feedback somehow isn't good we can swap this over to the /pontus/depth_0 topic
        # self.odom_sub = self.create_subscription(Odometry, '/pontus/depth_0', self.odometry_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/pontus/odometry', self.odometry_callback, 10)

    def odometry_callback(self, data):
        vel_msg = Twist()

        match self.state:
            case State.Surfacing:
                vel_msg.linear.z = 0.8

                # I think we are using -z values for underwater with 0 at the surface, need to check results from odom and depth sensor to be sure
                if (data.pose.pose.position.z >= self.surfaced_threshold):
                    self.state = State.HoveringAtSurface
            case State.HoveringAtSurface:
                # Wait until we have settled to complete
                if (abs(data.twist.twist.linear.z) < 0.1):
                    self.complete(True)
            case _:
                pass

        self.cmd_vel_pub.publish(vel_msg)

        self.debug_string = "Surfacing Task: " + str(self.state)
        self.publish_debug_string()
