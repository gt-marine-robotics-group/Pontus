from rclpy.node import Node
from std_msgs.msg import String


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
        
        # TODO: Q1.1.b Creating Publisher
        # Please create a publisher that has the following attributes:
        #  - Publishes a message type String
        #  - Publishes to a topic /onboarding/new_topic
        #  - Qos profile of 10
        self.new_topic_publisher = self.create_publisher(
            ### STUDENT CODE HERE

            ### END STUDENT CODE
        )
        self.topic_string_message = None


    def topic_callback(self, msg: String):
        ## TODO: Q1.1.c Message Data
        # Please see what the String message type consists of: https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html
        # For this question, access the string in the variable msg and store it into the following variable
        self.topic_string_message = None
        ### STUDENT CODE HERE

        ### END STUDENT CODE

        # TODO: Q1.1.d Publishing Data
        # Take the value from topic_string_message and append the string " ROS" so the new string would
        # look something like "Hello World! ROS"
        # Then use the variable below to publish this new string using the publisher from Q1.1.b
        new_message = String()
        # STUDENT CODE HERE

        # END STUDENT CODE
