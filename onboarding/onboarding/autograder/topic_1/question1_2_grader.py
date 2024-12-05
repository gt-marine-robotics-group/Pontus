import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from ...questions.topic_1 import question1_2
from ...autograder import utils


def verify_q1_2_a(subscription_variale: Subscription):
    utils.verify_answer('String', subscription_variale.msg_type.__qualname__, 'Q1.2.a Check Message Type')
    utils.verify_answer('/onboarding/basic_topic', subscription_variale.topic_name, 'Q1.2.a Check Topic Name')
    utils.verify_answer('topic_callback', subscription_variale.callback.__name__, 'Q1.2.a Check Callback')


def verify_q1_2_b(publisher_variable: Publisher):
    utils.verify_answer('String', publisher_variable.msg_type.__qualname__, 'Q1.2.b Check Message Type')
    utils.verify_answer('/onboarding/new_topic', publisher_variable.topic_name, 'Q1.2.b Check Topic Name')


def verify_q1_2_c(topic_string_message: str):
    utils.verify_answer('Hello World!', topic_string_message, 'Q1.2.c Check Message')


def verify_q1_2_d(published_string: str):
    utils.verify_answer(lambda x: x is not None, published_string, 'Q1.2.d Check Message Published')
    utils.verify_answer('Hello World! ROS', published_string, 'Q1.2.d Check Correct Published String')


class OnboardingTopic1MockNode(Node):
    def __init__(self):
        super().__init__('mock_node')
        self.basic_topic_pub = self.create_publisher(
            String,
            '/onboarding/basic_topic',
            10
        )
        self.new_topic_sub = self.create_subscription(
            String,
            '/onboarding/new_topic',
            self.new_topic_callback,
            10
        )
        self.published_string = None
        self.pub_timer = self.create_timer(1, self.timer_callback)


    def new_topic_callback(self, msg: String):
        self.published_string = msg.data


    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World!'
        self.basic_topic_pub.publish(msg)


class OnboardingTopic1TestNode(Node):
    def __init__(self):
        super().__init__('topic1_test_node')
        self.student_node = question1_2.OnboardingTopic1_2()
        verify_q1_2_a(self.student_node.basic_topic_subscription)
        verify_q1_2_b(self.student_node.new_topic_publisher)
        self.mock_node = OnboardingTopic1MockNode()
        rclpy.spin_once(self.mock_node)
        rclpy.spin_once(self.student_node)
        rclpy.spin_once(self.mock_node)
        verify_q1_2_c(self.student_node.topic_string_message)
        verify_q1_2_d(self.mock_node.published_string)


def main(args=None):
    rclpy.init(args=args)
    OnboardingTopic1TestNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
