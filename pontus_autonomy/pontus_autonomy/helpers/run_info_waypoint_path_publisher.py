#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from pontus_autonomy.helpers import run_info


class WaypointPathPublisher(Node):
    def __init__(self):
        super().__init__("run_info_waypoint_visualizer")

        # Minimal params
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("markers_topic", "/run_waypoints_markers")
        self.declare_parameter("path_topic", "/run_waypoints_path")
        self.declare_parameter("line_width", 0.08)
        self.declare_parameter("point_diameter", 0.20)

        frame_id = self.get_parameter(
            "frame_id").get_parameter_value().string_value
        markers_topic = self.get_parameter(
            "markers_topic").get_parameter_value().string_value
        path_topic = self.get_parameter(
            "path_topic").get_parameter_value().string_value
        line_width = self.get_parameter(
            "line_width").get_parameter_value().double_value
        point_diameter = self.get_parameter(
            "point_diameter").get_parameter_value().double_value

        # Transient Local so late subscribers (e.g., Foxglove) still see it
        qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        markers_pub = self.create_publisher(MarkerArray, markers_topic, qos)
        path_pub = self.create_publisher(Path, path_topic, qos)

        # Build 3D points from PoseObj.cmd_pose + desired_depth
        wps = list(run_info.waypoints_list)

        def point_from_wp(wp):
            x = float(wp.cmd_pose.position.x)
            y = float(wp.cmd_pose.position.y)
            # Prefer desired_depth if provided; depth down -> negative z
            if getattr(wp, "desired_depth", None) is not None:
                z = -float(wp.desired_depth)
            else:
                # fallback to pose.z if present (default 0)
                z = float(getattr(wp.cmd_pose.position, "z", 0.0))
            return Point(x=x, y=y, z=z)

        pts = [point_from_wp(wp) for wp in wps]

        # ----- Markers (3D) -----
        ma = MarkerArray()

        line = Marker()
        line.header.frame_id = frame_id
        line.ns = "run_waypoints"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = line_width
        line.color.r = 0.7
        line.color.g = 0.2
        line.color.b = 0.9
        line.color.a = 0.95
        line.points = pts
        ma.markers.append(line)

        for i, p in enumerate(pts, start=1):
            s = Marker()
            s.header.frame_id = frame_id
            s.ns = "run_waypoints_points"
            s.id = i
            s.type = Marker.SPHERE
            s.action = Marker.ADD
            s.scale.x = s.scale.y = s.scale.z = point_diameter
            s.color.r = 0.5
            s.color.g = 0.75
            s.color.b = 1.00
            s.color.a = 0.95
            s.pose.position = p
            ma.markers.append(s)

        # Publish markers once
        markers_pub.publish(ma)

        # ----- nav_msgs/Path (3D) -----
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = now

        poses = []
        for p in pts:
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.header.stamp = now
            ps.pose.position.x = p.x
            ps.pose.position.y = p.y
            ps.pose.position.z = p.z
            ps.pose.orientation = Quaternion(
                x=0.0, y=0.0, z=0.0, w=1.0)  # no yaw needed
            poses.append(ps)
        path.poses = poses

        # Publish path once
        path_pub.publish(path)

        self.get_logger().info(
            f"Published {len(pts)} 3D waypoints: markers->{markers_topic}, path->{path_topic} (frame={frame_id})."
        )


def main():
    rclpy.init()
    node = WaypointPathPublisher()
    rclpy.spin(node)  # keep alive so Transient Local serves late subscribers
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
