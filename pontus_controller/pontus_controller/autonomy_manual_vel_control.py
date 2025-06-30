import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from typing import Optional, List


class AutonomyManualVelControl(Node):
    def __init__(self):
        super().__init__('autonomy_manual_vel_control')
        self.autonomy_sub = self.create_subscription(
            Bool,
            '/autonomy_mode',
            self.autonomy_mode_callback,
            10
        )
        self.autonomy_mode_selected = True
        self.autonomy_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_autonomy',
            self.autonomy_vel_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.manual_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_joy',
            self.manual_vel_callback,
            10
        )

    def autonomy_mode_callback(self, msg: Bool) -> None:
        """
        Handle when autonomy mode is turned off and on.

        Args:
        ----
        msg (Bool): whether or not autonomy mode is turned off or on

        Return:
        ------
        None

        """
        if msg.data:
            self.autonomy_mode_selected = True
        else:
            self.autonomy_mode_selected = False

    def autonomy_vel_callback(self, msg: Twist) -> None:
        """
        Publish cmd_vel from autonomy if autonomy mode is selected.

        Args:
        ----
        msg (Twist): command velocity

        Return:
        ------
        None

        """
        if self.autonomy_mode_selected:
            self.cmd_vel_pub.publish(msg)

    def manual_vel_callback(self, msg: Twist) -> None:
        """
        Publish cmd_vel from manual controller if autonomy mode is not selected.

        Args:
        ----
        msg (Twist): command velocity

        Return:
        ------
        None

        """
        if not self.autonomy_mode_selected:
            self.cmd_vel_pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = AutonomyManualVelControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
