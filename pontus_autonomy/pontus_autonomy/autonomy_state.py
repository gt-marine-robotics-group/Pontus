# ------ Libraries ------
import rclpy
from rclpy.node import Node

from pontus_msgs.msg import AutonomyState
from pontus_msgs.srv import SetEnum


class AutonomyStateManager(Node):

    def __init__(self):

        super().__init__('autonomy_state_manager')

        self.state = AutonomyState()

        self.state_publisher = self.create_publisher(
            AutonomyState,
            '/pontus/autonomy/state',
            10
        )

        self.gate_side_info_service = self.create_service(
            SetEnum,
            '/pontus/autonomy/set_gate_side_info',
            self.set_gate_side_info_callback
        )

        self.gate_picture_info_service = self.create_service(
            SetEnum,
            '/pontus/autonomy/set_gate_picture_info',
            self.set_gate_picture_info_callback
        )

        self.slalom_side_info_service = self.create_service(
            SetEnum,
            '/pontus/autonomy/set_slalom_side_info',
            self.set_slalom_side_info_callback
        )

        self.create_timer(1.0, self.publish_autonomy_state)

    def set_gate_side_info_callback(self, request: SetEnum.Request,
                                    response: SetEnum.Response) -> SetEnum.Response:
        self.state.gate_side_info = request.data
        response.success = True
        return response

    def set_gate_picture_info_callback(self, request: SetEnum.Request,
                                       response: SetEnum.Response) -> SetEnum.Response:
        self.state.gate_picture_info = request.data
        response.success = True
        return response

    def set_slalom_side_info_callback(self, request: SetEnum.Request,
                                      response: SetEnum.Response) -> SetEnum.Response:
        self.state.slalom_side_info = request.data
        response.success = True
        return response

    def publish_autonomy_state(self) -> None:
        self.state.header.stamp = self.get_clock().now().to_msg()
        self.state.header.frame_id = "map"

        self.state_publisher.publish(self.state)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
