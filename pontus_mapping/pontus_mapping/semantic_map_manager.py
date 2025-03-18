import pandas as pd
from pandas import Series
from enum import Enum
import numpy as np
from typing import Optional, List

import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import rclpy
from rclpy.node import Node
from pontus_msgs.srv import AddSemanticObject
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from pontus_msgs.srv import GetGateLocation
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from pontus_msgs.srv import GetVerticalMarkerLocation
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon

from pontus_mapping.helpers import get_fov_polygon, polygon_contained


class SemanticObject(Enum):
    LeftGate = 0
    RightGate = 10
    VerticalMarker = 1


class SemanticMapManager(Node):
    def __init__(self):
        super().__init__('semantic_map_manager')

        self.add_service = self.create_service(
            AddSemanticObject,
            '/pontus/add_semantic_object',
            self.handle_add_semantic_object,
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

        self.semantic_map = pd.DataFrame(columns=['type',
                                                  'x_loc',
                                                  'y_loc',
                                                  'z_loc',
                                                  'num_detected',
                                                  'num_expected',
                                                  'confidence',
                                                  'last_updated'])
        self.object_seen_order = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.current_odom = Odometry()
        self.iteration_updated = 0

    # Callbacks
    def handle_get_gate_detection(self,
                                  request: GetGateLocation.Request,
                                  response: GetGateLocation.Response) -> None:
        """
        Handle get gate detection request.

        Handler to handle getting the gate detection. If either gate is not found,
        it will set the respective left_valid/right_valid True/False. This will resolve
        ties by taking the detection that we have seen the most.

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
            self.get_logger().info(f'Exception {e}. Failed to get map transfrom. Skipping')
            return None

        pose_map_frame = do_transform_pose(position.pose, transform)
        return pose_map_frame

    def remove_duplicates(self,
                          current_object: SemanticObject,
                          current_object_pose: Pose,
                          min_distance: float,
                          iteration_number: int) -> None:
        """
        Remove all duplicate detections within a min distance.

        Args:
        ----
        current_object (SemanticObject): the object we want to check duplicates for
        current_object_pose (Pose): the pose of the current object
        min_distance (float): the minimum distance to another marker
        iteration_number (int): the iteration number used to keep track when we last updated a
                                detection

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
        weighted_x = (matches['x_loc'] * matches['confidence']).sum() / total_weight
        weighted_y = (matches['y_loc'] * matches['confidence']).sum() / total_weight
        weighted_z = (matches['z_loc'] * matches['confidence']).sum() / total_weight

        total_num_expected = matches['num_expected'].sum()
        total_num_detected = matches['num_detected'].sum()

        confidence = total_num_detected / total_num_expected

        # Confidence is handled later
        new_entry = [
            current_object,
            weighted_x,
            weighted_y,
            weighted_z,
            total_num_detected,
            total_num_expected,
            confidence,
            iteration_number]
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

        return (np.linalg.norm(current_linear_velocity) > 0.15
                or np.linalg.norm(current_angular_velocity) > 0.1)

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
        # Calculate current FOV polygon
        # This will be used to make sure faulty detections are not included
        # This will also be used to adjust confidences
        fov_polygon = get_fov_polygon(self.current_odom)

        # TODO:
        # See if there is a way to see if the self.tf_buffer is valid
        for class_id, detection_pose in zip(request.ids, request.positions):
            map_position = self.convert_to_map_frame(detection_pose, self.tf_buffer)
            if not map_position:
                self.get_logger().warn('Unable to find map transform, skipping add')
                return response
            # Test to see if this is within our FOV
            if not polygon_contained(fov_polygon,
                                     (map_position.position.x, map_position.position.y)):
                self.get_logger().info(
                    f'Detection {class_id} {detection_pose} fell outside of fov, ignoring'
                )
                continue
            current_object = SemanticObject(class_id)
            new_entry = {'type': current_object,
                         'x_loc': map_position.position.x,
                         'y_loc': map_position.position.y,
                         'z_loc': map_position.position.z,
                         'num_detected': 1,
                         'num_expected': 1.0,
                         'confidence': 1.0,
                         'last_updated': 0}
            self.semantic_map.loc[len(self.semantic_map)] = new_entry
            self.remove_duplicates(current_object, map_position, 1.5, self.iteration_updated)
        self.update_confidences(fov_polygon)
        self.publish_semantic_map()
        response.added = True
        self.iteration_updated += 1
        return response

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
        match row['type']:
            case SemanticObject.LeftGate:
                # 3D models of the gate
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = 'package://pontus_mapping/visual_meshes/LeftGate.obj'
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.pose = Pose()
                marker.pose.position.x = row['x_loc']
                marker.pose.position.y = row['y_loc']
                marker.pose.position.z = row['z_loc']
                marker.mesh_use_embedded_materials = True

            case SemanticObject.RightGate:
                # 3D models of the gate
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = 'package://pontus_mapping/visual_meshes/RightGate.obj'
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.pose = Pose()
                marker.pose.position.x = row['x_loc']
                marker.pose.position.y = row['y_loc']
                marker.pose.position.z = row['z_loc']
                marker.mesh_use_embedded_materials = True
            case SemanticObject.VerticalMarker:
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = 'package://pontus_mapping/visual_meshes/VerticalMarker.obj'
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.pose = Pose()
                marker.pose.position.x = row['x_loc']
                marker.pose.position.y = row['y_loc']
                marker.pose.position.z = row['z_loc']
                marker.mesh_use_embedded_materials = True
            case _:
                self.get_logger().info('Found marker with unknown object type, skipping')

        return

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
            if row['num_detected'] < 30:
                continue
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
