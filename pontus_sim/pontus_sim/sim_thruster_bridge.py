#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32MultiArray
from typing import Optional, List

class SimThrusterBridge(Node):
    def __init__(self):
        super().__init__('sim_thruster_bridge')

        self.create_subscription(Float32MultiArray, '/thrust_cmds',
                                    self.thrust_cmd_callback, 10)

        self.max_thruster_force = 52 # in N

        self.thruster_publishers = []
        for i in range(8):
            thruster_pub = self.create_publisher(Float64,
                                                    f'/pontus/thruster_{i}/cmd_thrust',
                                                    10)
            self.thruster_publishers.append(thruster_pub)

    def thrust_cmd_callback(self, msg: Float32MultiArray) -> None:
        for i in range(8):
            thrust_msg = Float64()
            thrust_msg.data = msg.data[i] * self.max_thruster_force

            self.thruster_publishers[i].publish(thrust_msg)

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)

    node = SimThrusterBridge()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()