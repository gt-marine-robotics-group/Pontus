import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist 

class OnboardingTopic1Mock(Node):
    def __init__(self):
        super().__init__('onboarding_node_q1_1')
        self.pub1 = self.create_publisher(
            String,
            '/onboarding/StringPub',
            10
        )
        self.pub2 = self.create_publisher(
            Odometry,
            '/onboarding/OdometryPub',
            10
        )
        self.pub3 = self.create_publisher(
            Twist,
            '/onboarding/MysteryPub',
            10
        )

        self.string_timer = self.create_timer(
            0.5,
            self.timer_callback
        )

    def timer_callback(self):
        msg = String()
        msg.data = "mrg"
        self.pub1.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    onboarding_topic_1_mock = OnboardingTopic1Mock()
    rclpy.spin(onboarding_topic_1_mock)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
