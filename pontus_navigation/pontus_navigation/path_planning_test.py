import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from pontus_msgs.srv import GetPathToObject

class path_planner_tester_client(Node):
    def __init__(self) -> None:
        super().__init__('path_planner_client')
        self.cli = self.create_client(GetPathToObject, '/pontus/path_planning_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        
        self.req = GetPathToObject.Request()

    def send_request(self, pose: PoseStamped):
        self.req.goal = pose
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = path_planner_tester_client()
    test_pose = PoseStamped()
    test_pose.pose.position.x = 15.5
    test_pose.pose.position.y = 4.0
    test_pose.pose.position.z = 0.0
    response = minimal_client.send_request(test_pose)
    minimal_client.get_logger().info(
        'Result of path: for %2f, %2f, %2f ' %
        (test_pose.pose.position.x, test_pose.pose.position.y, test_pose.pose.position.z))
    
    minimal_client.get_logger().info(str([x.pose.position for x in response.path_to_object.poses]))
    rclpy.spin(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()