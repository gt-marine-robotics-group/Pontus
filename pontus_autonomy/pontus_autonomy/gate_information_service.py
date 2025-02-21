import rclpy
from rclpy.node import Node

from pontus_msgs.srv import GateInformation

class GateInformationService(Node):
    def __init__(self):
        super().__init__('gate_information_service')
        self.create_service(
            GateInformation,
            '/pontus/gate_information',
            self.handle_gate_information
        )

        self.entered_left_side = None
        self.gate_location = None

    def handle_gate_information(self, request, response):
        if request.set.data:
            self.entered_left_side = request.entered_left_side
            self.gate_location = request.gate_location
        else:
            response.entered_left_side = self.entered_left_side
            response.gate_location = self.gate_location
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GateInformationService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()