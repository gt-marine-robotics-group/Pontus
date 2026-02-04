"""
Semantic Map Manager

- Maintains a SemanticMap message grouped by object type
- Fuses duplicate detections with an online mean of positions
- Publishes a MarkerArray visualization
- Exposes a service to add semantic objects
"""

# ------ Libraries ------
from dataclasses import dataclass, field
from typing import Iterator, Optional
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
import tf2_geometry_msgs

from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose, PoseWithCovariance, Point, Quaternion, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from pontus_msgs.msg import SemanticObject, SemanticMap, SemanticMetaGate
from pontus_msgs.srv import AddSemanticObject, AddMetaGate
from pontus_tools.transform_helpers import convert_to_map_frame


# ------- Semantic Map Wrapper ------
@dataclass
class SemanticMapDC:
    """Lightweight manager around the SemanticMap ROS message"""

    semantic_map: SemanticMap = field(default_factory=SemanticMap)
    objects: dict[int, list[SemanticObject]] = field(init=False)

    # ------ Meta Objects ------
    meta_gate: Optional[SemanticMetaGate] = None

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

        if self.meta_gate is not None:
            self.semantic_map.meta_gate = self.meta_gate

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

    # ------ Add Meta Objects ------
    def add_meta_gate(self, left_gate: SemanticObject, right_gate: SemanticObject) -> None:
        gate = SemanticMetaGate()

        gate.header = Header()
        gate.header.stamp = left_gate.header.stamp
        gate.header.frame_id = left_gate.header.frame_id

        gate.left_gate = left_gate
        gate.right_gate = right_gate

        self.meta_gate = gate
        self.semantic_map.meta_gate = self.meta_gate

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

        # ------ Parameters ------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gate_width', 3.048),          # RoboSub handbook
                ('gate_width_tolerance', 0.3),  # tolerance for pairing
            ]
        )

        self.gate_width_m = float(self.get_parameter('gate_width').value)
        self.gate_width_tolerance_m = float(
            self.get_parameter('gate_width_tolerance').value)

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

        self.semantic_map_debug_pub = self.create_publisher(
            String,
            '/pontus/semantic_map_debug',
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

        self._update_meta_gate()

        self.publish_semantic_map()

        response.added = True
        return response

    def add_meta_gate_callback(self,
                               request: AddMetaGate.Request,
                               response: AddMetaGate.Response) -> AddMetaGate.Response:
        """
        Add a meta_object that represents the collection of gate_sides and eventually
        gate_pictures we group together to collectively call a "gate"
        """

        self.semantic_map.add_meta_gate(request.left_gate, request.right_gate)

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

        self.publish_semantic_map()
        self.semantic_map_visual_pub.publish(marker_array)
        self.publish_semantic_map_debug()

    def publish_semantic_map_debug(self) -> None:
        """Publish a debug string listing how many objects are in each SemanticMap list."""
        sm = self.semantic_map.semantic_map

        debug_msg = String()
        debug_msg.data = (
            "SemanticMap counts | "
            f"gate_image_shark={len(sm.gate_image_shark)}, "
            f"gate_image_fish={len(sm.gate_image_fish)}, "
            f"gate_left={len(sm.gate_left)}, "
            f"gate_right={len(sm.gate_right)}, "
            f"slalom_red={len(sm.slalom_red)}, "
            f"slalom_white={len(sm.slalom_white)}, "
            f"vertical_marker={len(sm.vertical_marker)}, "
            f"bin={len(sm.bin)}, "
            f"octagon={len(sm.octagon)}, "
            f"target={len(sm.target)}, "
            f"meta_gate_set={sm.meta_gate.header.frame_id != ''}"
        )

        self.semantic_map_debug_pub.publish(debug_msg)

    def _update_meta_gate(self) -> None:
        """
        Look at gate side detections and, if a valid pair is found,
        update the SemanticMetaGate in the semantic map.

        Criteria:
        - Use gate_left list
        - Require distance between poles to be ~ gate_width_m within gate_width_tolerance_m.
        - Use body_frame transform to decide which is left vs right.
        """

        # Lock Check: First meta_gate we find we keep set
        if self.semantic_map.semantic_map.meta_gate.header.frame_id != "":
            return

        gate_list = self.semantic_map.semantic_map.gate_left

        if len(gate_list) < 2:
            return

        candidate_pair: list[SemanticObject] | None = None

        # Find a pair whose distance matches the expected gate width
        for i in range(len(gate_list) - 1):
            for j in range(i + 1, len(gate_list)):
                g1 = self._pose_to_vec2(gate_list[i].pose.pose)
                g2 = self._pose_to_vec2(gate_list[j].pose.pose)

                gate_width_est = float(np.linalg.norm(g1 - g2))

                if abs(gate_width_est - self.gate_width_m) <= self.gate_width_tolerance_m:
                    candidate_pair = [gate_list[i], gate_list[j]]

        if candidate_pair is None:
            return

        # Decide which is left/right relative to the robot (body_frame)
        body_frame_poses: list[Pose] = []
        for side in candidate_pair:
            body_frame_side_pose = self._transform_sem_obj_to_body(side)
            body_frame_poses.append(body_frame_side_pose)

        # Higher y in body_frame is "left"
        if body_frame_poses[0].position.y > body_frame_poses[1].position.y:
            left_gate = candidate_pair[0]
            right_gate = candidate_pair[1]
        else:
            left_gate = candidate_pair[1]
            right_gate = candidate_pair[0]

        # Actually store it in the DC + message
        self.semantic_map.add_meta_gate(left_gate, right_gate)
        self.get_logger().info(
            f"Meta gate updated: left_gate at ({left_gate.pose.pose.position.x:.2f}, "
            f"{left_gate.pose.pose.position.y:.2f}), "
            f"right_gate at ({right_gate.pose.pose.position.x:.2f}, "
            f"{right_gate.pose.pose.position.y:.2f})"
        )

        

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
                # marker.type = Marker.MESH_RESOURCE
                # marker.mesh_resource = 'package://pontus_mapping/visual_meshes/VerticalMarker.obj'
                # marker.scale.x = 1.0
                # marker.scale.y = 1.0
                # marker.scale.z = 1.0
                # marker.mesh_use_embedded_materials = True
                marker.type = Marker.CYLINDER
                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = 3.0  # height
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

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

    def _pose_to_vec2(self, pose: Pose) -> np.ndarray:
        """Convert a Pose into 2D np.array [x, y] for planar distance calculations."""
        return np.array([pose.position.x, pose.position.y], dtype=float)

    def _transform_sem_obj_to_body(self, obj: SemanticObject) -> Pose:
        """
        Transform a SemanticObject's pose to the robot's body_frame.

        This mimics the logic in PrequalGateTask so meta_gate.left/right
        are defined in the same way (left/right in the body frame).
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = obj.header
        pose_stamped.pose = obj.pose.pose

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='body_frame',
                source_frame=pose_stamped.header.frame_id,
                time=Time(
                    seconds=pose_stamped.header.stamp.sec,
                    nanoseconds=pose_stamped.header.stamp.nanosec,
                )
            )

            pose_transformed_stamped = tf2_geometry_msgs.do_transform_pose(
                pose_stamped,
                transform
            )

            return pose_transformed_stamped.pose

        except Exception as e:
            self.get_logger().warn(
                f"Failed to transform semantic object to body_frame "
                f"(current frame: {obj.header.frame_id})"
            )
            self.get_logger().warn(f"exception: {e}")
            return pose_stamped.pose


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
