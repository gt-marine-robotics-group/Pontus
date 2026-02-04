# ------- Imports ------
from __future__ import annotations

import copy
import os
from typing import Optional, List, Dict, Any

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
import yaml

# ------- Loading configuration helper -------


def load_camera_info_from_yaml(yaml_path: str) -> CameraInfo:
    """
    Load a CameraInfo message from a standard ROS camera calibration YAML file.
    """
    with open(yaml_path, "r") as f:
        data: Dict[str, Any] = yaml.safe_load(f)

    msg = CameraInfo()
    msg.width = int(data.get("image_width", data.get("width", 0)))
    msg.height = int(data.get("image_height", data.get("height", 0)))
    msg.distortion_model = str(data.get("distortion_model", "plumb_bob"))

    # K, D, R, P
    cam_mat = data.get("camera_matrix", {})
    msg.k = list(cam_mat.get("data", data.get("k", [])))

    dist = data.get("distortion_coefficients", {})
    msg.d = list(dist.get("data", data.get("d", [])))

    rect = data.get("rectification_matrix", {})
    msg.r = list(rect.get("data", data.get("r", [])))

    proj = data.get("projection_matrix", {})
    msg.p = list(proj.get("data", data.get("p", [])))

    # Optional extras
    msg.binning_x = int(data.get("binning_x", 0))
    msg.binning_y = int(data.get("binning_y", 0))
    roi = data.get("roi", {})
    msg.roi.x_offset = int(roi.get("x_offset", 0))
    msg.roi.y_offset = int(roi.get("y_offset", 0))
    msg.roi.height = int(roi.get("height", 0))
    msg.roi.width = int(roi.get("width", 0))
    msg.roi.do_rectify = bool(roi.get("do_rectify", False))

    return msg


# ------ Camera Info Publisher Node -------
class CameraInfoPublisher(Node):
    """
    Generic CameraInfo publisher that reads calibration from a YAML file.
    """

    def __init__(self) -> None:
        super().__init__("camera_info_publisher")

        # Parameters
        self.declare_parameters(
            "",
            [
                # If relative, it will be resolved as <package_share>/config/<calibration_file>
                ("calibration_file", "front_left_camera_calibration.yaml"),
                ("calibration_package", "pontus_sensors"),
                ("camera_info_topic", "/pontus/camera_front/camera_info"),
                # TODO: figure out the correct frame_id
                ("frame_id", "narrow_stereo/left"),
                ("publish_rate_hz", 5.0),
            ],
        )

        calibration_file = self.get_parameter(
            "calibration_file").get_parameter_value().string_value
        calibration_package = self.get_parameter(
            "calibration_package").get_parameter_value().string_value
        camera_info_topic = self.get_parameter(
            "camera_info_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter(
            "frame_id").get_parameter_value().string_value
        publish_rate_hz = float(self.get_parameter(
            "publish_rate_hz").get_parameter_value().double_value)

        if os.path.isabs(calibration_file):
            yaml_path = calibration_file
        else:
            pkg_share = get_package_share_directory(calibration_package)
            yaml_path = os.path.join(pkg_share, "config", calibration_file)

        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"Calibration YAML not found: {yaml_path}")

        self.camera_info_template: CameraInfo = load_camera_info_from_yaml(
            yaml_path)

        self.publisher = self.create_publisher(
            CameraInfo,
            camera_info_topic,
            10
        )

        self.get_logger().info(
            f"Loaded calibration from '{yaml_path}', publishing on '{camera_info_topic}'"
        )

        # Timer
        # Period either infinantly long or the given rate
        period = 1.0 / max(1e-6, publish_rate_hz)
        self.timer = self.create_timer(period, self.publish_camera_info)

    def publish_camera_info(self) -> None:
        msg: CameraInfo = copy.deepcopy(self.camera_info_template)

        msg.header = Header()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = CameraInfoPublisher()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
