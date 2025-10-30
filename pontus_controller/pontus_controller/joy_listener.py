import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from pontus_msgs.msg import CommandMode

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

    def joy_callback(self, joy_msg):
        buttons = joy_msg.buttons
        estop_pressed = True if buttons[2] == 1 else False
        if estop_pressed:
            estop_msg = Bool()
            estop_msg.data = True
            for i in range(0,5):
                self.estop_pub.publish(estop_msg)
                self.get_logger().info("Publishing estop: TRUE")
            
            cmd_mode_msg = CommandMode()
            cmd_mode_msg.data = CommandMode.ESTOP
            self.command_mode_pub.publish(cmd_mode_msg)

        manual_pressed = True if buttons[8] == 1 else False
        if not estop_pressed and manual_pressed:
            estop_msg = Bool()
            estop_msg.data = False
            self.estop_pub.publish(estop_msg)
            self.get_logger().info("Publishing estop: FALSE")
            autonomy_msg = Bool()
            autonomy_msg.data = False
            self.autonomy_mode_pub.publish(autonomy_msg)
            self.get_logger().info("Publishing autonomy mode: FALSE")
        
        autonomy_pressed = True if buttons[9] else False
        if not estop_pressed and not manual_pressed and autonomy_pressed:
            estop_msg = Bool()
            estop_msg.data = False
            self.estop_pub.publish(estop_msg)
            self.get_logger().info("Publishing estop: FALSE")
            cmd_mode_msg = CommandMode()
            cmd_mode_msg.data = CommandMode.POSITION_FACE_TRAVEL
            self.command_mode_pub.publish(cmd_mode_msg)
            self.get_logger().info("Publishing command mode: POSITION_FACE_TRAVEL")


def main(args=None):
    rclpy.init(args=args)
    node = JoyListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()