import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from pontus_msgs.msg import CommandMode
from enum import Enum

class Button(Enum):
    ESTOP=2
    DIRECT=0
    VELOCITY=8
    POSITION_HOLD=1
    AUTONOMY_POSITION=9

class JoyListener(Node):
    def __init__(self):
        super().__init__('joy_listener_node')
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.estop_pub = self.create_publisher(
            Bool,
            '/estop',
            10
        )

        self.command_mode_pub = self.create_publisher(
            CommandMode,
            '/CommandMode',
            10
        )

    def release_estop(self):
        estop_msg = Bool()
        estop_msg.data = False
        self.estop_pub.publish(estop_msg)
        self.get_logger().info("Publishing estop: FALSE")

    def set_command_mode(self, command_mode):
        cmd_mode_msg = CommandMode()
        cmd_mode_msg.data = command_mode
        self.command_mode_pub.publish(cmd_mode_msg)

    def joy_callback(self, joy_msg):
        buttons = joy_msg.buttons

        if buttons[Button.ESTOP]:
            estop_msg = Bool()
            estop_msg.data = True
            for i in range(0,5):
                self.estop_pub.publish(estop_msg)
                self.get_logger().info("Publishing estop: TRUE")
            
            self.set_command_mode(CommandMode.ESTOP)

        elif buttons[Button.DIRECT]:
            self.release_estop()
            self.set_command_mode(CommandMode.DIRECT_CONTROL)
            self.get_logger().info("Publishing command mode: DIRECT_CONTROL")

        elif buttons[Button.VELOCITY]:
            self.release_estop()
            self.set_command_mode(CommandMode.VELOCITY_CONTROL)
            self.get_logger().info("Publishing command mode: VELOCITY_CONTROL")
        
        elif buttons[Button.POSITION_HOLD]:
            self.release_estop()
            self.set_command_mode(CommandMode.VELOCITY_HOLD_POSITION)
            self.get_logger().info("Publishing command mode: VELOCITY_HOLD_POSITION")

        elif buttons[Button.AUTONOMY_POSITION]:
            self.release_estop()
            self.set_command_mode(CommandMode.POSITION_FACE_TRAVEL)
            self.get_logger().info("Publishing command mode: POSITION_FACE_TRAVEL")


def main(args=None):
    rclpy.init(args=args)
    node = JoyListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()