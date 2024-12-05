import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from ...questions.topic_1 import question1_3
from ...autograder import utils
import time
from rclpy.executors import MultiThreadedExecutor

class Question1_3_Grader(Node):
    def __init__(self):
        super().__init__('question1_3_grader_node')
        self.counter_subscriber = self.create_subscription(
            Int32, 
            '/onboarding/counter250',
            self.counter_callback,
            10
        )
        self.expected = 0
        self.not_failed = True
        self.passed = False
        self.last_message_time = time.time()
        self.timeout_timer = self.create_timer(0.5, self.timeout_callback)


    def counter_callback(self, msg: Int32):
        self.last_message_time = time.time()
        if msg.data > 250 or msg.data < 0:
            self.not_failed = False
        elif msg.data != self.expected:
            self.not_failed = False
        elif msg.data == self.expected and msg.data == 250:
            self.passed = True
        else:
            self.expected += 1

    
    def timeout_callback(self):
        if time.time() - self.last_message_time > 1:
            utils.verify_answer(True, self.not_failed and self.passed, 'Q1.3 Counter Node')
            self.get_logger().info("Test finished. Press CTRL + c to exit")
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    student_node = question1_3.OnboardingTopic1_3()
    executor = MultiThreadedExecutor()
    grader_node = Question1_3_Grader()
    executor.add_node(grader_node)
    executor.add_node(student_node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()