"""
Semantic Map Manager (drop-in replacement)

- Maintains a SemanticMap message grouped by object type
- Fuses duplicate detections with an online mean of positions
- Publishes a MarkerArray visualization
- Exposes a service to add semantic objects
"""

# ------ Libraries ------
from dataclasses import dataclass, field
from typing import Iterator, Optional
import math

import rclpy
from rclpy.node import Node

import tf2_ros

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseWithCovariance, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

from pontus_msgs.msg import SemanticObject, SemanticMap
from pontus_msgs.srv import AddSemanticObject
from pontus_tools.transform_helpers import convert_to_map_frame


# ------- Semantic Map Wrapper ------
@dataclass
class SemanticMapDC:
    """Lightweight manager around the SemanticMap ROS message"""

    semantic_map: SemanticMap = field(default_factory=SemanticMap)
    objects: dict[int, list[SemanticObject]] = field(init=False)

    def __post_init__(self):
        self.objects = {
            SemanticObject.GATE_IMAGE_SHARK: self.semantic_map.gate_image_shark,
            SemanticObject.GATE_IMAGE_FISH:  self.semantic_map.gate_image_fish,
            SemanticObject.GATE_LEFT:        self.semantic_map.gate_left,
            SemanticObject.GATE_RIGHT:       self.semantic_map.gate_right,
            SemanticObject.SLALOM_RED:       self.semantic_map.slalom_red,
            SemanticObject.SLALOM_WHITE:     self.semantic_map.slalom_white,
            SemanticObject.VERTICAL_MARKER:  self.semantic_map.vertical_marker,
            SemanticObject.BIN:              self.semantic_map.bin,
            SemanticObject.OCTAGON:          self.semantic_map.octagon,
            SemanticObject.TARGET:           self.semantic_map.target,
        }

    @staticmethod
    def check_semantic_object_duplicant(obj1: SemanticObject, obj2: SemanticObject) -> bool:
        """Return True if obj1 and obj2 are same type and within duplicate tolerance."""
        if not isinstance(obj1, SemanticObject) or not isinstance(obj2, SemanticObject):
            return False

        if obj1.object_type != obj2.object_type:
            return False

        p1 = obj1.pose.pose.position
        p2 = obj2.pose.pose.position
        dist = math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y)
                         ** 2 + (p1.z - p2.z) ** 2)

        duplicant_tolerance_m = min(
            obj1.duplicant_tolerance_m, obj2.duplicant_tolerance_m)
        return dist <= duplicant_tolerance_m

    def add(self, obj: SemanticObject) -> None:
        """Add or fuse a semantic object into the map."""
        obj_list = self.objects.get(int(obj.object_type))
        if obj_list is None:
            return

        for existing in obj_list:
            if self.check_semantic_object_duplicant(obj, existing):
                # Fuse via online mean for position only.
                existing.num_detections += 1
                n = existing.num_detections

                p_old = existing.pose.pose.position
                p_new = obj.pose.pose.position

                # Online mean: mean += (new - mean) / n
                p_old.x += (p_new.x - p_old.x) / n
                p_old.y += (p_new.y - p_old.y) / n
                p_old.z += (p_new.z - p_old.z) / n

                # TODO: confidence update policy
                # existing.confidence = max(existing.confidence, obj.confidence)

                # Timestamps: keep latest
                existing.header.stamp = obj.header.stamp
                existing.last_updated = obj.last_updated
                return

        obj_list.append(obj)

    def create_message(self) -> SemanticMap:
        """Return the SemanticMap ROS message"""
        return self.semantic_map

    def __iter__(self) -> Iterator[SemanticObject]:
        """Iterate over all semantic objects in the map"""
        for obj_list in self.objects.values():
            for obj in obj_list:
                yield obj


# ------- Node ------
class SemanticMapManager(Node):

    def __init__(self):

        super().__init__('semantic_map_manager')

        # Map Services
        self.add_semantic_object_srv = self.create_service(
            AddSemanticObject,
            '/pontus/add_semantic_object',
            self.add_semantic_object_callback,
        )

        # Publishers
        self.semantic_map_pub = self.create_publisher(
            SemanticMap,
            '/pontus/semantic_map',
            10
        )

        self.semantic_map_visual_pub = self.create_publisher(
            MarkerArray,
            '/pontus/semantic_map_visual',
            10
        )

        map_visual_period = 1.0
        self.map_visual_timer = self.create_timer(
            map_visual_period, self.publish_semantic_map_visual)

        # TODO: put semantic_map publisher on a timer

        self.semantic_map = SemanticMapDC()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def add_semantic_object_callback(self,
                                     request: AddSemanticObject.Request,
                                     response: AddSemanticObject.Response) -> AddSemanticObject.Response:
        """
        Add objects by transforming detections into 'map' and fusing into the semantic map.
        Expects request.ids and request.positions
        """

        now = self.get_clock().now().to_msg()

        for class_id, detection_pose in zip(request.ids, request.positions):
            # Convert to 'map' frame Pose (returns None if TF unavailable)
            map_pose: Optional[Pose] = convert_to_map_frame(
                detection_pose, self.tf_buffer, target_frame="map")

            if map_pose is None:
                self.get_logger().warn('Unable to find map transform, skipping add')
                continue

            # Build SemanticObject message
            obj = SemanticObject()
            obj.header = Header()
            obj.header.stamp = now
            obj.header.frame_id = "map"

            obj.object_type = int(class_id)

            obj.pose = PoseWithCovariance()
            obj.pose.pose.position = Point(
                x=map_pose.position.x,
                y=map_pose.position.y,
                z=map_pose.position.z,
            )
            obj.pose.pose.orientation = Quaternion(
                x=map_pose.orientation.x,
                y=map_pose.orientation.y,
                z=map_pose.orientation.z,
                w=map_pose.orientation.w,
            )

            obj.num_detections = 1
            obj.confidence = 1.0

            obj.last_updated = now
            obj.duplicant_tolerance_m = 0.1

            self.semantic_map.add(obj)

        self.publish_semantic_map()

        response.added = True
        return response

    def publish_semantic_map(self) -> None:
        """Publish the SemanticMap message"""
        msg = self.semantic_map.create_message()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        self.semantic_map_pub.publish(msg)

    def publish_semantic_map_visual(self) -> None:
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        marker_id = 0

        for obj in self.semantic_map:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now
            marker.ns = '/pontus'

            marker.id = marker_id
            marker_id += 1

            marker.pose = obj.pose.pose

            self._set_marker_shape(obj, marker)

            marker.action = Marker.ADD

            marker_array.markers.append(marker)

        self.semantic_map_visual_pub.publish(marker_array)

    def _set_marker_shape(self, obj: SemanticObject, marker: Marker) -> None:
        GATE_DIAMETER_VISUAL = 0.15
        SLALOM_DIAMETER_VISUAL = 0.12

        match int(obj.object_type):
            case SemanticObject.GATE_LEFT:
                marker.type = Marker.CYLINDER
                marker.scale.x = GATE_DIAMETER_VISUAL
                marker.scale.y = GATE_DIAMETER_VISUAL
                marker.scale.z = 1.20  # height
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                # marker.type = Marker.MESH_RESOURCE
                # marker.mesh_resource = 'package://pontus_mapping/visual_meshes/LeftGate.obj'
                # marker.scale.x = 1.0
                # marker.scale.y = 1.0
                # marker.scale.z = 1.0
                # marker.mesh_use_embedded_materials = True

            case SemanticObject.GATE_RIGHT:
                marker.type = Marker.CYLINDER
                marker.scale.x = GATE_DIAMETER_VISUAL
                marker.scale.y = GATE_DIAMETER_VISUAL
                marker.scale.z = 1.20  # height
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                # marker.type = Marker.MESH_RESOURCE
                # marker.mesh_resource = 'package://pontus_mapping/visual_meshes/RightGate.obj'
                # marker.scale.x = 1.0
                # marker.scale.y = 1.0
                # marker.scale.z = 1.0
                # marker.mesh_use_embedded_materials = True

            case SemanticObject.VERTICAL_MARKER:
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = 'package://pontus_mapping/visual_meshes/VerticalMarker.obj'
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.mesh_use_embedded_materials = True

            case SemanticObject.GATE_IMAGE_SHARK:
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = 'package://pontus_mapping/visual_meshes/shark_marker.obj'
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0

                marker.mesh_use_embedded_materials = True

            case SemanticObject.GATE_IMAGE_FISH:
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = 'package://pontus_mapping/visual_meshes/fish_marker.obj'
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0

                marker.mesh_use_embedded_materials = True

            case SemanticObject.SLALOM_RED:
                marker.type = Marker.CYLINDER
                marker.scale.x = SLALOM_DIAMETER_VISUAL
                marker.scale.y = SLALOM_DIAMETER_VISUAL
                marker.scale.z = 0.90  # height
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

            case SemanticObject.SLALOM_WHITE:
                marker.type = Marker.CYLINDER
                marker.scale.x = SLALOM_DIAMETER_VISUAL
                marker.scale.y = SLALOM_DIAMETER_VISUAL
                marker.scale.z = 0.90
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0

            case SemanticObject.OCTAGON:
                # (use a cube for now; meshes can be flaky in Foxglove)
                marker.type = Marker.CUBE
                marker.scale.x = 2.70
                marker.scale.y = 2.70
                marker.scale.z = 0.0254
                marker.color.r = 0.9
                marker.color.g = 0.7
                marker.color.b = 0.1
                marker.color.a = 1.0

            case SemanticObject.BIN:
                marker.type = Marker.CUBE
                marker.scale.x = 0.40
                marker.scale.y = 0.40
                marker.scale.z = 0.40
                marker.color.r = 0.2
                marker.color.g = 0.6
                marker.color.b = 0.9
                marker.color.a = 1.0

            case SemanticObject.TARGET:
                marker.type = Marker.CUBE
                marker.scale.x = 0.60
                marker.scale.y = 0.60
                marker.scale.z = 1.10
                marker.color.r = 0.8
                marker.color.g = 0.2
                marker.color.b = 0.1
                marker.color.a = 1.0

            case _:
                self.get_logger().info('Found marker with unknown object type, skipping')
                marker.action = Marker.DELETE


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
