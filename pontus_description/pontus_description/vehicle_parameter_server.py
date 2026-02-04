#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from typing import Optional, List

class VehicleParameterServer(Node):
    def __init__(self):
        super().__init__(
            'vehicle_parameter_server',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = VehicleParameterServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()