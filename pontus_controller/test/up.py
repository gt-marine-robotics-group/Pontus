import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_0 = self.create_publisher(Float64, '/model/pontus/joint/prop_joint_0/cmd_thrust', 10)
        self.publisher_1 = self.create_publisher(Float64, '/model/pontus/joint/prop_joint_1/cmd_thrust', 10)
        self.publisher_2 = self.create_publisher(Float64, '/model/pontus/joint/prop_joint_2/cmd_thrust', 10)
        self.publisher_3 = self.create_publisher(Float64, '/model/pontus/joint/prop_joint_3/cmd_thrust', 10)
        
        msg = Float64()
        msg.data = 500.0
        self.publisher_0.publish(msg)
        self.publisher_1.publish(msg)
        self.publisher_2.publish(msg)
        self.publisher_3.publish(msg)
        self.get_logger().info('Publishing')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
