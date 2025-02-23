import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_msgs.srv import GateInformation

class VerticalToGate(BaseTask):
    def __init__(self):
        super().__init__("vertical_to_gate")

        self.cmd_pose_pub = self.create_publisher(
            Pose,
            '/cmd_pos',
            10
        )
        
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        self.gate_information_client = self.create_client(
            GateInformation,
            '/pontus/gate_information',
            callback_group=self.service_callback_group
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
            10,
        )

        while not self.gate_information_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting to connect to /pontus/gate_information service")        

        self.position_controller_state = self.create_subscription(
            Int32,
            '/pontus/position_controller_state_machine_status',
            self.position_controller_state_callback,
            10
        )

        self.current_pose = Pose()
        self.started = False
        self._done = False

        self.timer = self.create_timer(
            0.2, self.go
        )

        self.cmd_sent = False


    # Callbacks
    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose


    def position_controller_state_callback(self, msg: Int32):
        if not self.started and msg.data > 1:
            self.started = True
        if self.started and msg.data == 1:
            self._done = True
            self.started = False
    

    # Autonomy?
    def go(self):
        if not self.cmd_sent:
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
                new_point.x = self.current_pose.position.x
                new_point.y = self.current_pose.position.y
                new_point.z = self.current_pose.position.z
                gate_location = new_point
                self.get_logger().info("Failed to get location from service, using current position instead")
            cmd_pose.position.x = gate_location.x + 3.0
            cmd_pose.position.y = gate_location.y
            cmd_pose.position.z = gate_location.z
            cmd_pose.orientation.x = -1.0
            self.cmd_pose_pub.publish(cmd_pose)
            self.cmd_sent = True
        elif self.cmd_sent and self._done:
            self.complete(True)
        
        