import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

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

        self.autonomy_mode_pub = self.create_publisher(
            Bool,
            '/autonomy_mode',
            10
        )

        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            '/cmd_vel_joy_raw',
            self.cmd_vel_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_joy',
            10
        )

        self.cmd_vel_current = Twist()
    
    def cmd_vel_callback(self, msg):
        self.cmd_vel_current = msg

    def joy_callback(self, joy_msg):
        buttons = joy_msg.buttons
        estop_pressed = True if buttons[2] == 1 else False
        if estop_pressed:
            estop_msg = Bool()
            estop_msg.data = True
            for i in range(0,5):
                self.estop_pub.publish(estop_msg)
                self.get_logger().info("Publishing estop: TRUE")

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
            autonomy_msg = Bool()
            autonomy_msg.data = True
            self.autonomy_mode_pub.publish(autonomy_msg)
            self.get_logger().info("Publishing autonomy mode: TRUE")

        
        strafe_pressed = True if buttons[6] == 1 else False
        if strafe_pressed:
            self.cmd_vel_current.linear.y = self.cmd_vel_current.angular.z
            self.cmd_vel_current.angular.z = 0.0
            
        self.cmd_vel_pub.publish(self.cmd_vel_current)

        


def main(args=None):
    rclpy.init(args=args)
    node = JoyListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()