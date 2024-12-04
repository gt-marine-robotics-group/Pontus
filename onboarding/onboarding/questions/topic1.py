import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ..autograder import topic1_grader


class OnboardingTopic1(Node):
    def __init__(self):
        super().__init__('onboarding_node')

        ## TODO: Q1.1.a Creating a subscriber to a topic
        # Please check the software trainings or https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html 
        # if you are stuck.
        # Please fill in the blanks for the following line to create a subscriber. This subscriber should:
        #  - Subscribe to the topic /onboarding/basic_topic
        #  - Have its callback be self.topic_callback
        #  - Take in a message type String
        #  - Qos profile of 10
        # Fill in the function here. HINT: There should be four parameters that you fill
        self.basic_topic_subscription = self.create_subscription(
            ### STUDENT CODE HERE

            ### END STUDENT CODE
        )
        topic1_grader.verify_q1_1_a(self.basic_topic_subscription)
        # TODO: Q1.1.b Creating Publisher
        # Please create a publisher that has the following attributes:
        #  - Publishes a message type String
        #  - Publishes to a topic /onboarding/new_topic
        #  - Qos profile of 10
        self.new_topic_publisher = self.create_publisher(
            ### STUDENT CODE HERE

            ### END STUDENT CODE
        )
        topic1_grader.verify_q1_1_b(self.new_topic_publisher)
        self.onboardiang_topic1_mock_node = topic1_grader.OnboardingTopic1MockNode()
        rclpy.spin_once(self.onboardiang_topic1_mock_node)


    def topic_callback(self, msg: String):
        ## TODO: Q1.1.c Message Data
        # Please see what the String message type consists of: https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html
        # For this question, access the string in the variable msg and store it into the following variable
        topic_string_message = None
        ### STUDENT CODE HERE

        ### END STUDENT CODE
        topic1_grader.verify_q1_1_c(topic_string_message)
            
        # TODO: Q1.1.d Publishing Data
        # Take the value from topic_string_message and append the string " ROS" so the new string would
        # look something like "Hello World! ROS"
        # Then use the variable below to publish this new string using the publisher from Q1.1.b
        new_message = String()
        # STUDENT CODE HERE

        # END STUDENT CODE
        rclpy.spin_once(self.onboardiang_topic1_mock_node)
        topic1_grader.verify_q1_1_d(self.onboardiang_topic1_mock_node.published_string)


def main(args=None):
    rclpy.init(args=args)
    onboarding_topic_1_node = OnboardingTopic1()
    rclpy.spin_once(onboarding_topic_1_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()