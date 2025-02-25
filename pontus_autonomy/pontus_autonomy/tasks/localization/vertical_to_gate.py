import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

from pontus_autonomy.helpers.GoToPoseClient import GoToPoseClient
from pontus_autonomy.tasks.base_task import BaseTask
from pontus_msgs.srv import GateInformation

class VerticalToGate(BaseTask):
    def __init__(self):
        super().__init__("vertical_to_gate")
        
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        self.gate_information_client = self.create_client(
            GateInformation,
            '/pontus/gate_information',
            callback_group=self.service_callback_group
        )

        while not self.gate_information_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting to connect to /pontus/gate_information service")   

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
            10,
        )     

        self.go_to_pose_client = GoToPoseClient(self)

        self.current_pose = Pose()
        self.timer = self.create_timer(
            0.2, self.go
        )
        self.cmd_sent = False


    # Callbacks
    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
    

    # Autonomy?
    def go(self):
        if not self.cmd_sent and self.current_pose:
            request = GateInformation.Request()
            future = self.gate_information_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            cmd_pose = Pose()
            gate_location = None
            if future.result() is not None:
                response = future.result()
                gate_location = response.gate_location 
            else:
                new_point = Point()
                new_point.x = self.current_pose.position.x - 7.0
                new_point.y = self.current_pose.position.y
                new_point.z = self.current_pose.position.z
                gate_location = new_point
                self.get_logger().info("Failed to get location from service, using current position instead")
            cmd_pose.position.x = gate_location.x + 3.0
            cmd_pose.position.y = gate_location.y
            cmd_pose.position.z = gate_location.z
            cmd_pose.orientation.x = -1.0
            self.go_to_pose_client.go_to_pose(cmd_pose)
            self.cmd_sent = True

        elif self.cmd_sent and self.go_to_pose_client.at_pose():
            self.complete(True)
        
        