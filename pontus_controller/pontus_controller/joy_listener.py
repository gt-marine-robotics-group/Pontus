import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from pontus_msgs.msg import CommandMode
from pontus_msgs.message_enums import CommandModeEnum
from enum import Enum

# Button mapping is for a Logitech F310 Controller
class Button(Enum):
    ESTOP = 2              # B (red)
    DIRECT = 0             # X (blue)
    VELOCITY = 3           # Y (Yellow)
    POSITION_HOLD = 1      # A (green)
    AUTONOMY_POSITION = 9  # Start

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

    def set_estop(self, estop_enabled):
        estop_msg = Bool()
        estop_msg.data = estop_enabled
        self.estop_pub.publish(estop_msg)
        self.get_logger().info(f"Publishing estop: {str(estop_enabled).upper()}")

    def set_command_mode(self, command_mode):
        cmd_mode_msg = CommandMode()
        cmd_mode_msg.command_mode = command_mode
        self.command_mode_pub.publish(cmd_mode_msg)
        self.get_logger().info(f"Publishing command mode: {CommandModeEnum(command_mode).name}")

    def joy_callback(self, joy_msg):
        buttons = joy_msg.buttons

        if buttons[Button.ESTOP]:
            self.set_estop(True)
            self.set_command_mode(CommandMode.ESTOP)

        elif buttons[Button.DIRECT]:
            self.set_estop(False)
            self.set_command_mode(CommandMode.DIRECT_CONTROL)

        elif buttons[Button.VELOCITY]:
            self.set_estop(False)
            self.set_command_mode(CommandMode.VELOCITY_CONTROL)
        
        elif buttons[Button.POSITION_HOLD]:
            self.set_estop(False)
            self.set_command_mode(CommandMode.VELOCITY_HOLD_POSITION)

        elif buttons[Button.AUTONOMY_POSITION]:
            self.set_estop(False)
            self.set_command_mode(CommandMode.POSITION_FACE_TRAVEL)


def main(args=None):
    rclpy.init(args=args)
    node = JoyListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()