"""Centralized topic/frame config loader for gt-bbx nodes.

Usage:
    from gtbbx_bringup.topic_config import TopicConfig

    class MyNode(Node):
        def __init__(self):
            super().__init__('my_node')
            self.topics = TopicConfig(self, [
                'map_frame', 'lidar_input_topic',
            ])
            self.create_subscription(PointCloud2, self.topics.lidar_input_topic, ...)
"""

from __future__ import annotations

from typing import List

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor


class TopicConfig:
    """Loads required topic/frame names from ROS2 parameters.

    Declares each key as a parameter on the node, validates that all
    are set (i.e. provided via topics.yaml), and exposes them as
    string attributes.  Crashes on startup with a clear error if any
    are missing.

    Attributes are accessed by name:
        config.map_frame          -> "bb04/map"
        config.lidar_input_topic  -> "livox/points"
        config.cameras            -> ["port_oakd", "stbd_oakd"]

    String parameters are returned as str, list parameters (like
    cameras) are returned as list[str].
    """

    def __init__(self, node: Node, required: List[str]) -> None:
        for key in required:
            node.declare_parameter(
                key, descriptor=ParameterDescriptor(dynamic_typing=True)
            )

        missing = [
            key for key in required
            if node.get_parameter(key).type_ == rclpy.Parameter.Type.NOT_SET
        ]
        if missing:
            raise RuntimeError(
                f"[{node.get_name()}] Missing required topics/frames: {missing}. "
                f"Load the topics.yaml config file via the launch file."
            )

        for key in required:
            value = node.get_parameter(key).value
            if isinstance(value, list):
                setattr(self, key, [str(v) for v in value])
            else:
                setattr(self, key, str(value))