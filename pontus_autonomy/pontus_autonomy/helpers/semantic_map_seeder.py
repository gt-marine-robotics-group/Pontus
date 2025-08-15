import rclpy
from rclpy.node import Node

from pontus_msgs.srv import AddSemanticObject
from pontus_autonomy.helpers.run_info import semantic_map


class SemanticMapSeeder(Node):
    def __init__(self):
        super().__init__('semantic_map_seeder')
        self.cli = self.create_client(AddSemanticObject, '/pontus/add_semantic_object')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /pontus/add_semantic_object ...')

    def seed(self):
        req = AddSemanticObject.Request()

        # Build one batch request
        for obj in semantic_map:
            req.ids.append(int(obj.label.value))           # enum -> int32
            req.positions.append(obj.get_pose_stamped())   # PoseStamped in 'map'

        self.get_logger().info(f'Sending {len(req.ids)} semantic objects...')
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Service response: added={future.result().added}")
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")


def main():
    rclpy.init()
    node = SemanticMapSeeder()
    node.seed()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
