import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
import time

GREEN = '\033[32m'
RED = '\033[31m'
RESET = '\033[0m'


def verify_answer(expected_condition, received_value, question_name):
    try:
        if callable(expected_condition):
            condition_met = expected_condition(received_value)
        else:
            condition_met = expected_condition == received_value

        if condition_met:
            print(f"{question_name} {GREEN}PASSED{RESET}")
        else:
            print(f"{question_name} {RED}FAILED{RESET}")
    except Exception as e:
        print(f"{question_name} {RED}ERROR{RESET}")
        print(e)


def verify_q1_1_a(subscription_variale: Subscription):
    verify_answer('String', subscription_variale.msg_type.__qualname__, 'Q1.1.a Check Message Type')
    verify_answer('/onboarding/basic_topic', subscription_variale.topic_name, 'Q1.1.a Check Topic Name')
    verify_answer('topic_callback', subscription_variale.callback.__name__, 'Q1.1.a Check Callback')


def verify_q1_1_b(publisher_variable: Publisher):
    verify_answer('String', publisher_variable.msg_type.__qualname__, 'Q1.1.b Check Message Type')
    verify_answer('/onboarding/new_topic', publisher_variable.topic_name, 'Q1.1.b Check Topic Name')


def verify_q1_1_c(topic_string_message: str):
    verify_answer('Hello World!', topic_string_message, 'Q1.1.c Check Message')


def verify_q1_1_d(published_string: str):
    verify_answer(lambda x: x is not None, published_string, 'Q1.1.d Check Message Published')
    verify_answer('Hello World! ROS', published_string, 'Q1.1.d Check Correct Published String')


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