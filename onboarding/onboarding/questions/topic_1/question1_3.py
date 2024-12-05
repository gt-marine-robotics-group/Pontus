from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32

# TODO: Q1.3
class OnboardingTopic1_3(Node):
    def __init__(self):
        super().__init__('onboarding_node_q1_3')
        # STUDENT CODE HERE

        # END STUDENT CODE
        
        # This calls the function timer_callback every 0.01 second
        self.timer = self.create_timer(0.01, self.timer_callback)
    

    def timer_callback(self):
        # STUDENT CODE HERE

        # END STUDENT CODE
        pass