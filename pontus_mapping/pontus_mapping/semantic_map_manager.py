import pandas as pd
from pandas import Series
from enum import Enum

import rclpy
from rclpy.node import Node
from pontus_msgs.srv import AddSemanticObject
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose


class SemanticObject(Enum):
    LeftGate = 0
    RightGate = 1


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

        self.semantic_map = pd.DataFrame(columns=['type', 'unique_id', 'pose', 'num_detected', 'num_expected'])
        self.object_seen_order = 0

    
    def handle_add_semantic_object(self, request: AddSemanticObject.Request, response: AddSemanticObject.Response):
        """
        Add semantic object to the map.
        """
        self.object_seen_order += 1
        self.semantic_map.loc[len(self.semantic_map)] = [SemanticObject(request.id), self.object_seen_order, request.position, 1.0, 1.0]
        self.publish_semantic_map()
        return response


    def marker_hash(self, row: Series) -> int:
        """
        Converts a given marker row to an int value to represent in the semantic map

        Parameters:
        row (Series): the pandas series representing the object

        Returns:
        int : the hash valued
        """
        return row['type'].value * 10000 + row['unique_id']


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
                marker.pose = row['pose']
                marker.mesh_use_embedded_materials = True

            case SemanticObject.RightGate:
                # 3D models of the gate
                marker.type=Marker.MESH_RESOURCE
                marker.mesh_resource = "package://pontus_mapping/visual_meshes/RightGate.obj"
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.pose = Pose()
                marker.pose = row['pose']
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
        # Iterate through semantic map dataframe and convert to display
        for _, row in self.semantic_map.iterrows():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = '/pontus'
            marker.id = self.marker_hash(row)
            self.set_marker_shape(row, marker)
            marker.action = Marker.ADD
            marker_array.markers.append(marker)
        
        self.semantic_map_manager_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapManager()
    rclpy.spin(node)
