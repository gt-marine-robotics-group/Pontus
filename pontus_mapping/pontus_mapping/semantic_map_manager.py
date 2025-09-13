from enum import Enum
import numpy as np
from pandas import DataFrame, Series
from typing import List, Optional

from geometry_msgs.msg import Point, Polygon, Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import tf_transformations
from visualization_msgs.msg import Marker, MarkerArray

# from pontus_mapping.helpers import get_fov_polygon, polygon_contained
from pontus_mapping.helpers import polygon_contained
from pontus_msgs.srv import (AddSemanticObject,
                             GateSideInformation,
                             GetGateLocation,
                             GetVerticalMarkerLocation)


"""
TODO List:
handle_get_gate_detection()
remove_duplicates()
"""


# These will correlate to YOLO
class SemanticObject(Enum):
    GateFish = 0
    GateShark = 1
    LeftGate = 2
    RightGate = 3
    SlalomRed = 4
    SlalomWhite = 5
    VerticalMarker = 6
    Octagon = 7


# class SemanticObject(Enum):
#     LeftGate = 0
#     RightGate = 1
#     VerticalMarker = 2


# class SemanticObject(Enum):
#     GateLeft = 0
#     GateRight = 1
#     Red_Slalom = 2
#     White_Slalom = 3
#     Octagon = 4


class SemanticMapManager(Node):
    def __init__(self):
        super().__init__('semantic_map_manager')

        # Map Services
        self.add_service = self.create_service(
            AddSemanticObject,
            '/pontus/add_semantic_object',
            self.handle_add_semantic_object,
        )
        self.gate_side_information_service = self.create_service(
            GateSideInformation,
            '/pontus/gate_side_information',
            self.handle_gate_side_information
        )

        self.semantic_map_manager_pub = self.create_publisher(
            MarkerArray,
            '/pontus/semantic_map_visual',
            10
        )

        # Detection services:
        self.gate_service = self.create_service(
            GetGateLocation,
            '/pontus/get_gate_detection',
            self.handle_get_gate_detection
        )
        self.vertical_marker_service = self.create_service(
            GetVerticalMarkerLocation,
            '/pontus/get_vertical_marker_detection',
            self.handle_get_vertical_marker_location
        )

        # Since we're having issues with detections while moving, we turn off the semantic map
        # when we move. This will also be useful for calculating fov confidences
        self.odom_sub = self.create_subscription(
            Odometry,
            '/pontus/odometry',
            self.odom_callback,
            10,
        )

        self.semantic_map = DataFrame(columns=['type',
                                               'x_loc', 'y_loc', 'z_loc', 'yaw_orien',
                                               'num_detected', 'num_expected', 'confidence',
                                               'last_updated'])
        self.object_seen_order = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.current_odom = Odometry()
        self.iteration_updated = 0
        self.entered_left_side = None

    # Callbacks
    def handle_gate_side_information(self,
                                     request: GateSideInformation.Request,
                                     response: GateSideInformation.Response) -> None:
        """
        Handle add semantic object to the map.

        Args:
        ----
        request (GateSideInformation.Request): request containing which side we entered
        response (GateSideInformation.Response): response for service

        Return:
        ------
        None

        """
        if request.set:
            self.entered_left_side = request.entered_left_side
        else:
            response.entered_left_side = self.entered_left_side
            response.fish_on_left_side = self.determine_fish_left()
        return response

    def handle_get_gate_detection(self,
                                  request: GetGateLocation.Request,
                                  response: GetGateLocation.Response) -> None:
        """
        Handle get gate detection request.

        Handler to handle getting the gate detection. If either gate is not found,
        it will set the respective left_valid/right_valid True/False. This will resolve
        ties by taking the detection that we have seen the most.

        TODO: Handle the case where there are multiple gate_sides at the same location

        Args:
        ----
        request (GetGateLocation.Request): request field in the service request
        response (GetGateLocation.Response): response field in the service request

        Return:
        ------
        None

        """
        response.left_location = Point()
        response.right_location = Point()
        response.left_valid = False
        response.right_valid = False
        left_gate_location = None
        # Default to at least 30 detections (seeing the object for 2 seconds)
        left_gate_confidence = 30
        right_gate_location = None
        right_gate_confidence = 30

        for _, row in self.semantic_map.iterrows():
            # Publish Gate detection
            # TODO: Implement actual confidence, for now, we'll just take the detection
            # that we see the most
            if row['type'] == SemanticObject.LeftGate and \
                    row['num_detected'] > left_gate_confidence:
                left_gate_confidence = row['num_detected']
                left_gate_location = row
            if row['type'] == SemanticObject.RightGate and \
                    row['num_detected'] > right_gate_confidence:
                right_gate_confidence = row['num_detected']
                right_gate_location = row

        # Convert to service message
        if left_gate_location is not None:
            response.left_location.x = left_gate_location['x_loc']
            response.left_location.y = left_gate_location['y_loc']
            response.left_location.z = left_gate_location['z_loc']
            response.left_valid = True
        if right_gate_location is not None:
            response.right_location.x = right_gate_location['x_loc']
            response.right_location.y = right_gate_location['y_loc']
            response.right_location.z = right_gate_location['z_loc']
            response.right_valid = True
        # If no gate found, say no gate found and return
        return response

    def handle_get_vertical_marker_location(self,
                                            request: GetVerticalMarkerLocation.Request,
                                            response: GetVerticalMarkerLocation.Response
                                            ) -> None:
        """
        Handle get vertical marker request.

        If we have seen the vertical marker, and is tracked in the semantic map, this
        will return the detection, else return None. This will resolve ties by taking
        the detection that we have seen the most.

        Args:
        ----
        request (GetGateLocation.Request): request field in the service request
        response (GetGateLocation.Response): response field in the service request

        Return:
        ------
        None

        """
        response.location = Point()
        response.found = False
        vertical_marker_confidence = 30
        vertical_marker = None
        for _, row in self.semantic_map.iterrows():
            if row['type'] == SemanticObject.VerticalMarker and \
                    row['num_detected'] > vertical_marker_confidence:
                vertical_marker = row
                vertical_marker_confidence = row['num_detected']
        if vertical_marker is not None:
            response.location.x = vertical_marker['x_loc']
            response.location.y = vertical_marker['y_loc']
            response.location.z = vertical_marker['z_loc']
            response.found = True
        return response

    def handle_add_semantic_object(self,
                                   request: AddSemanticObject.Request,
                                   response: AddSemanticObject.Response) -> None:
        """
        Handle add semantic object to the map.

        Args:
        ----
        request (AddSemanticObject.Request): request containing semantic object to be added
        response (AddSemanticObject.Response): response for service

        Return:
        ------
        None

        """
        if self.moving(self.current_odom.twist.twist):
            response.added = False
            self.get_logger().info('Moving, skipping add to semantic map')
            return response
        if self.not_submerged(self.current_odom.pose.pose):
            response.added = False
            self.get_logger().info('Not submerged, skipping add to semantic map')
            return response
        # Calculate current FOV polygon
        # This will be used to make sure faulty detections are not included
        # This will also be used to adjust confidences
        # fov_polygon = get_fov_polygon(self.current_odom)

        # TODO:
        # See if there is a way to see if the self.tf_buffer is valid
        for class_id, detection_pose in zip(request.ids, request.positions):
            map_position = self.convert_to_map_frame(
                detection_pose, self.tf_buffer)
            if not map_position:
                self.get_logger().warn('Unable to find map transform, skipping add')
                return response
            # Test to see if this is within our FOV
            # if not polygon_contained(fov_polygon,
            #                          (map_position.position.x, map_position.position.y)):
            #     self.get_logger().info(
            #         f'Detection {class_id} {detection_pose} fell outside of fov, ignoring'
            #     )
            #     continue
            current_object = SemanticObject(class_id)
            quat = [map_position.orientation.x,
                    map_position.orientation.y,
                    map_position.orientation.z,
                    map_position.orientation.w]
            _, _, yaw = tf_transformations.euler_from_quaternion(quat)
            new_entry = {'type': current_object,
                         'x_loc': map_position.position.x,
                         'y_loc': map_position.position.y,
                         'z_loc': map_position.position.z,
                         'yaw_orien': yaw,
                         'num_detected': 1,
                         'num_expected': 1.0,
                         'confidence': 1.0,
                         'last_updated': 0}
            self.semantic_map.loc[len(self.semantic_map)] = new_entry
            self.remove_duplicates(
                current_object, map_position, 1.5, yaw, self.iteration_updated)
        # self.update_confidences(fov_polygon, self.iteration_updated)
        self.publish_semantic_map()
        response.added = True
        self.iteration_updated += 1
        return response

    def odom_callback(self, msg: Odometry) -> None:
        """
        Handle odom callback.

        Args:
        ----
        msg (Odometry): odometry message

        Return:
        ------
        None

        """
        self.current_odom = msg

    # Helpers
    def convert_to_map_frame(self, position: PoseStamped, tf_buffer: tf2_ros.buffer) -> Pose:
        """
        Convert a point in relation to the body frame to the map frame.

        Args:
        ----
        position (PoseStamped): the point to transform
        tf_buffer (tf2_ros.buffer): tf buffer to get transform

        Return:
        ------
        Pose: the pose in the map frame

        """
        try:
            transform = tf_buffer.lookup_transform(
                'map',
                position.header.frame_id,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().info(
                f'Exception {e}. Failed to get map transfrom. Skipping')
            return None

        pose_map_frame = do_transform_pose(position.pose, transform)
        return pose_map_frame

    def remove_duplicates(self,
                          current_object: SemanticObject,
                          current_object_pose: Pose,
                          min_distance: float,
                          newest_yaw: float,
                          iteration_number: int) -> None:
        """
        Remove all duplicate detections within a min distance.

        TODO: Handle removing incorrect gate locations

        Args:
        ----
        current_object (SemanticObject): the object we want to check duplicates for
        current_object_pose (Pose): the pose of the current object
        min_distance (float): the minimum distance to another marker
        iteration_number (int): the iteration number used to keep track when we last updated a
                                detection
        newest_yaw (float): the yaw orientation of the object

        Return:
        ------
        None

        """
        distances = ((np.array(self.semantic_map['x_loc']) - current_object_pose.position.x)**2
                     + (np.array(self.semantic_map['y_loc']) - current_object_pose.position.y)**2
                     + (np.array(self.semantic_map['z_loc']) - current_object_pose.position.z)**2
                     )**0.5
        matches = self.semantic_map[(distances < min_distance) &
                                    (self.semantic_map['type'] == current_object)]
        # If the only match is itself, then there are no duplicates.
        if len(matches) <= 1:
            return

        total_weight = matches['confidence'].sum()
        weighted_x = (matches['x_loc'] *
                      matches['confidence']).sum() / total_weight
        weighted_y = (matches['y_loc'] *
                      matches['confidence']).sum() / total_weight
        weighted_z = (matches['z_loc'] *
                      matches['confidence']).sum() / total_weight

        total_num_expected = matches['num_expected'].sum()
        total_num_detected = matches['num_detected'].sum()

        confidence = total_num_detected / total_num_expected

        # Confidence is handled later
        new_entry = {
            'type': current_object,
            'x_loc': weighted_x,
            'y_loc': weighted_y,
            'z_loc': weighted_z,
            'yaw_orien': newest_yaw,
            'num_detected': total_num_detected,
            'num_expected': total_num_expected,
            'confidence': confidence,
            'last_updated': iteration_number
        }
        # Remove all duplicates
        self.semantic_map.drop(matches.index, inplace=True)
        self.semantic_map.reset_index(drop=True, inplace=True)
        # Add combined new entry
        self.semantic_map.loc[len(self.semantic_map)] = new_entry

    def moving(self, current_twist: Twist) -> bool:
        """
        Return true if the sub is moving based on our current_velocity.

        Args:
        ----
        current_twist (Twist): the current velocity where the first row is linear,
                               and the second row is angular

        Return:
        ------
        bool: true if the sub is moving

        """
        current_linear_velocity = np.array([current_twist.linear.x,
                                            current_twist.linear.y,
                                            current_twist.linear.z])

        current_angular_velocity = np.array([current_twist.angular.x,
                                             current_twist.angular.y,
                                             current_twist.angular.z])

        return (np.linalg.norm(current_linear_velocity) > 0.1
                or np.linalg.norm(current_angular_velocity) > 0.05)

    def not_submerged(self, current_pose: Pose) -> bool:
        """
        Return true if the sub is above a certain depth else false.

        Args:
        ----
        current_pose (Pose): the current pose

        Return:
        ------
        bool: true if the sub is moving

        """
        # return current_pose.position.z > -1
        return False

    def update_confidences(self, fov_polygon: Polygon, update_iteration: int) -> None:
        """
        Update confidences in semantic map.

        Iterates through the list of objects in the map. If it is within our fov and we did not
        detect it, decrease its confidence

        Args:
        ----
        fov_polygon (Polygon): the polygon representing our FOV
        update_iteration (int): the iteration number to keep track of what has already
                                been updated

        Return:
        ------
        None

        """
        for _, row in self.semantic_map.iterrows():
            if row['last_updated'] == update_iteration or \
                    not polygon_contained(fov_polygon, (row['x_loc'], row['y_loc'])):
                continue
            # Else, we know that we have a detection that we didn't just see and is within our fov
            # because of this, we should decrease its confidence
            row['num_expected'] += 1
            row['confidence'] = row['num_detected'] / row['num_expected']
            row['last_updated'] = update_iteration
        return

    def marker_hash(self, row: Series) -> int:
        """
        Convert a given marker row to an int value to represent in the semantic map.

        Args:
        ----
        row (Series): the pandas series representing the object

        Return:
        ------
        int: the hash valued

        """
        return row.name

    def set_marker_shape(self, row: Series, marker: Marker) -> None:
        """
        Take in a series from the semantic map and create the visual for it.

        Args:
        ----
        row (Series): the pd series containing the semantic object
        marker (Marker): the marker message to be edited (passed by reference)

        Return:
        ------
        None

        """
        obj = row['type']

        # Common pose for all markers
        marker.pose = Pose()
        marker.pose.position.x = float(row['x_loc'])
        marker.pose.position.y = float(row['y_loc'])
        marker.pose.position.z = float(row['z_loc'])

        GATE_DIAMETER = 0.15
        SLALOM_DIAMETER = 0.12

        match obj:
            case SemanticObject.LeftGate:
                marker.type = Marker.CYLINDER
                marker.scale.x = GATE_DIAMETER
                marker.scale.y = GATE_DIAMETER
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

            case SemanticObject.RightGate:
                marker.type = Marker.CYLINDER
                marker.scale.x = GATE_DIAMETER
                marker.scale.y = GATE_DIAMETER
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

            case SemanticObject.VerticalMarker:
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = 'package://pontus_mapping/visual_meshes/VerticalMarker.obj'
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.mesh_use_embedded_materials = True

            case SemanticObject.GateShark:
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = 'package://pontus_mapping/visual_meshes/shark_marker.obj'
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                # orientation as you had it...
                roll = np.pi/2
                pitch = 0.0
                yaw = -np.pi/2 + row['yaw_orien']
                q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
                marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = q
                marker.mesh_use_embedded_materials = True

            case SemanticObject.GateFish:
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = 'package://pontus_mapping/visual_meshes/fish_marker.obj'
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                roll = np.pi/2
                pitch = 0.0
                yaw = -np.pi/2 + row['yaw_orien']
                q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
                marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = q
                marker.mesh_use_embedded_materials = True

            case SemanticObject.SlalomRed:
                marker.type = Marker.CYLINDER
                marker.scale.x = SLALOM_DIAMETER
                marker.scale.y = SLALOM_DIAMETER
                marker.scale.z = 0.90  # height
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

            case SemanticObject.SlalomWhite:
                marker.type = Marker.CYLINDER
                marker.scale.x = SLALOM_DIAMETER
                marker.scale.y = SLALOM_DIAMETER
                marker.scale.z = 0.90
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0

            case SemanticObject.Octagon:
                # (use a cube for now; meshes can be flaky in Foxglove)
                marker.type = Marker.CUBE
                marker.scale.x = 2.70
                marker.scale.y = 2.70
                marker.scale.z = 0.0254
                marker.color.r = 0.9
                marker.color.g = 0.7
                marker.color.b = 0.1
                marker.color.a = 1.0

            case _:
                self.get_logger().info('Found marker with unknown object type, skipping')
                marker.action = Marker.DELETE

    def determine_fish_left(self) -> bool:
        """
        Return if the fish sign is on the left side of the gate.

        This will look through the semantic map and try to find it.
        Cases:
            1. Both found -> compare with respect to each other
            2. One found but not the other -> compare to side gates
            3. Neither found -> return True

        Args:
        ----
        None

        Return:
        ------
        bool: if the fish sign is on the left side of the gate

        """
        gate_fish = None
        gate_shark = None
        left_gate = None
        right_gate = None
        for _, row in self.semantic_map.iterrows():
            if row['type'] == SemanticObject.GateFish:
                gate_fish = row
            if row['type'] == SemanticObject.GateShark:
                gate_shark = row
            if row['type'] == SemanticObject.LeftGate:
                left_gate = row
            if row['type'] == SemanticObject.RightGate:
                right_gate = row

        # Case 1
        if gate_fish is not None and gate_shark is not None:
            return gate_fish['y_loc'] > gate_shark['y_loc']
        # Case 2
        if gate_fish is None and gate_shark is not None:
            # Determine which side the shark is closer to
            diff_to_left = np.array([gate_shark['x_loc'] - left_gate['x_loc'],
                                     gate_shark['y_loc'] - left_gate['y_loc']])
            distance_to_left = np.linalg.norm(diff_to_left)
            diff_to_right = np.array([gate_shark['x_loc'] - right_gate['x_loc'],
                                      gate_shark['y_loc'] - right_gate['y_loc']])
            distance_to_right = np.linalg.norm(diff_to_right)
            # If the shark sign is further away from the left, then the fish is probably
            # on the right hand side
            return distance_to_right < distance_to_left
        if gate_fish is not None and gate_shark is None:
            # Determine which side the fish is closer to
            diff_to_left = np.array([gate_fish['x_loc'] - left_gate['x_loc'],
                                     gate_fish['y_loc'] - left_gate['y_loc']])
            distance_to_left = np.linalg.norm(diff_to_left)
            diff_to_right = np.array([gate_fish['x_loc'] - right_gate['x_loc'],
                                      gate_fish['y_loc'] - right_gate['y_loc']])
            distance_to_right = np.linalg.norm(diff_to_right)
            # If the fish sign is closer to the left side, then the fish sign is on the left
            return distance_to_right > distance_to_left
        # Case 3
        return True

    # Debug
    def publish_semantic_map(self) -> None:
        """
        Publish the semantic map to the topic /pontus/semantic_map_visual.

        Args:
        ----
        None

        Return:
        ------
        None

        """
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        # Iterate through semantic map dataframe and convert to display
        for _, row in self.semantic_map.iterrows():
            # if row['num_detected'] < 30:
            #     continue
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = '/pontus'
            marker.id = self.marker_hash(row)
            self.set_marker_shape(row, marker)
            marker.action = Marker.ADD
            marker_array.markers.append(marker)
        self.get_logger().info(f'\n {self.semantic_map}')
        self.semantic_map_manager_pub.publish(marker_array)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SemanticMapManager()
    rclpy.spin(node)
