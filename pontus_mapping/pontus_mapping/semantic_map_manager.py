import pandas as pd
from pandas import Series
from enum import Enum
import numpy as np

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

        # TODO: Remove this
        # For now since we're having issues with detections while moving, we turn off the semantic map
        # when we move
        self.velocity_sub = self.create_subscription(
            Odometry, 
            '/pontus/odometry',
            self.velocity_callback,
            10,
        )

        self.semantic_map = pd.DataFrame(columns=['type', 'x_loc', 'y_loc', 'z_loc', 'num_detected', 'num_expected', 'confidence'])
        self.object_seen_order = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.current_velocity = np.zeros((2, 3))


    # Callbacks
    def handle_get_gate_detection(self, request: GetGateLocation.Request, response: GetGateLocation.Response):
        """
        Handler to handle getting the gate detection. This will only return the gate sides that it has seen before.
        If either gate is not found, it will set the respective left_valid/right_valid True/False.
        This will resolve ties by taking the detection that we have seen the most.
        """
        response.left_location = Point()
        response.right_location = Point()
        response.left_valid = False
        response.right_valid = False
        left_gate_location = None
        # Default to at least 30 detections (correlates to at least seeing the object for 2 seconds)
        left_gate_confidence = 30
        right_gate_location = None
        right_gate_confidence = 30

        for _, row in self.semantic_map.iterrows():
            # Publish Gate detection
            # TODO: Implement actual confidence, for now, we'll just take the detection that we see the most
            if row['type'] == SemanticObject.LeftGate and row['num_detected'] > left_gate_confidence:
                left_gate_confidence = row['num_detected']
                left_gate_location = row
            if row['type'] == SemanticObject.RightGate and row['num_detected'] > right_gate_confidence:
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
    
    def handle_get_vertical_marker_location(self, request: GetVerticalMarkerLocation.Request, response: GetVerticalMarkerLocation.Response):
        response.location = Point()
        response.found = False
        vertical_marker_confidence = 30
        vertical_marker = None
        for _, row in self.semantic_map.iterrows():
            if row['type'] == SemanticObject.VerticalMarker and row['num_detected'] > vertical_marker_confidence:
                vertical_marker = row
                vertical_marker_confidence = row['num_detected']
        if vertical_marker is not None:
            response.location.x = vertical_marker['x_loc']
            response.location.y = vertical_marker['y_loc']
            response.location.z = vertical_marker['z_loc']
            response.found = True
        return response


    def velocity_callback(self, msg: Odometry):
        self.current_velocity[0] = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        self.current_velocity[1] = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])


    def convert_to_map_frame(self, position: PoseStamped, tf_buffer: tf2_ros.buffer) -> Pose:
        """
        Converts a point in relation to the body frame to the map frame

        Parameters:
        position (PoseStamped) : the point to transform
        tf_buffer (tf2_ros.buffer) : tf buffer to get transform

        Returns:
        Pose : the pose in the map frame
        """
        for i in range(0,5):
            try:
                transform = tf_buffer.lookup_transform(
                    "map",
                    position.header.frame_id,
                    rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().info(f"Attempt: {i} Failed to get map transfrom")

        pose_map_frame = do_transform_pose(position.pose, transform)
        return pose_map_frame


    def remove_duplicates(self, current_object: SemanticObject, current_object_pose: Pose, min_distance: float) -> None:
        """
        Removes all duplicate detections within a min distance

        Parameters:
        current_object (SemanticObject) : the object we want to check duplicates for
        current_object_pose (Pose) : 
        min_distance (float) : the minimum distance to another marker

        Returns:
        None
        """
        distances = ((np.array(self.semantic_map['x_loc']) - current_object_pose.position.x)**2 \
            + (np.array(self.semantic_map['y_loc']) - current_object_pose.position.y)**2 \
            + (np.array(self.semantic_map['z_loc']) - current_object_pose.position.z)**2)**0.5
        matches = self.semantic_map[(distances < min_distance) & (self.semantic_map['type'] == current_object)]
        if len(matches) <= 1:
            return
        
        total_weight = matches['confidence'].sum()
        weighted_x = (matches['x_loc'] * matches['confidence']).sum() / total_weight
        weighted_y = (matches['y_loc'] * matches['confidence']).sum() / total_weight
        weighted_z = (matches['z_loc'] * matches['confidence']).sum() / total_weight

        new_entry = [current_object, weighted_x, weighted_y, weighted_z, matches['num_detected'].sum(), matches['num_expected'].sum(), matches['num_detected'].sum() / matches['num_expected'].sum()]
        # new_entry = [current_object, current_object_pose.position.x, current_object_pose.position.y, current_object_pose.position.z, matches['num_detected'].sum(), matches['num_expected'].sum(), matches['num_detected'].sum() / matches['num_expected'].sum()]
        self.semantic_map.drop(matches.index, inplace=True)
        self.semantic_map.reset_index(drop=True, inplace=True)
        self.semantic_map.loc[len(self.semantic_map)] = new_entry


    def moving(self, current_velocity: np.ndarray) -> bool:
        """
        Returns true if the sub is moving based on our current_velocity
        
        Parameters:
        current_velocity (np.ndarray) : the current velocity where the first row is linear, and the second row is angular

        Returns:
        bool : true if the sub is moving
        """
        return np.linalg.norm(current_velocity[0]) > 0.2 or np.linalg.norm(current_velocity[1]) > 0.1

    
    def handle_add_semantic_object(self, request: AddSemanticObject.Request, response: AddSemanticObject.Response):
        """
        Add semantic object to the map.
        """
        if self.moving(self.current_velocity):
            response.added = False
            self.get_logger().info("Moving, skipping add to semantic map")
            return response
        map_position = self.convert_to_map_frame(request.position, self.tf_buffer)
        current_object = SemanticObject(request.id)
        new_entry = [current_object, map_position.position.x, map_position.position.y, map_position.position.z, 1, 1, 1]
        self.semantic_map.loc[len(self.semantic_map)] = new_entry
        self.remove_duplicates(current_object, map_position, 1.5)
        self.publish_semantic_map()
        response.added = True
        return response


    def marker_hash(self, row: Series) -> int:
        """
        Converts a given marker row to an int value to represent in the semantic map

        Parameters:
        row (Series): the pandas series representing the object

        Returns:
        int : the hash valued
        """
        return row.name


    def set_marker_shape(self, row: Series, marker: Marker) -> None:
        """
        Takes in a series from the semantic map and creates the visual for it.
        
        Parameters:
        row (Series) : the pd series containing the semantic object
        marker (Marker) : the marker message to be edited (passed by reference)

        Returns:
        None
        """
        
        match row['type']:
            case SemanticObject.LeftGate:
                # 3D models of the gate
                marker.type=Marker.MESH_RESOURCE
                marker.mesh_resource = "package://pontus_mapping/visual_meshes/LeftGate.obj"
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
                marker.type=Marker.MESH_RESOURCE
                marker.mesh_resource = "package://pontus_mapping/visual_meshes/RightGate.obj"
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.pose = Pose()
                marker.pose.position.x = row['x_loc']
                marker.pose.position.y = row['y_loc']
                marker.pose.position.z = row['z_loc']
                marker.mesh_use_embedded_materials = True
            case SemanticObject.VerticalMarker:
                marker.type=Marker.MESH_RESOURCE
                marker.mesh_resource = "package://pontus_mapping/visual_meshes/VerticalMarker.obj"
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.pose = Pose()
                marker.pose.position.x = row['x_loc']
                marker.pose.position.y = row['y_loc']
                marker.pose.position.z = row['z_loc']
                marker.mesh_use_embedded_materials = True
            case _:
                self.get_logger().info("Found marker with unknown object type, skipping")
        
        return


    def publish_semantic_map(self) -> None:
        """
        This publishes the semantic map to the topic /pontus/semantic_map_visual.
        It should take from the self.semantic_map object

        Parameters:
        None

        Returns: 
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
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = '/pontus'
            marker.id = self.marker_hash(row)
            self.set_marker_shape(row, marker)
            marker.action = Marker.ADD
            marker_array.markers.append(marker)
        self.get_logger().info(f"\n {self.semantic_map}")
        self.semantic_map_manager_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapManager()
    rclpy.spin(node)
