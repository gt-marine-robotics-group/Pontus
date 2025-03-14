import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d


class PointCloudDownsampling(Node):
    def __init__(self):  # noqa D107
        super().__init__('point_cloud_downsampling')
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/points2',
            self.point_cloud_callback,
            10
        )

        self.point_cloud_pub = self.create_publisher(
            PointCloud2,
            '/pontus/camera_2/depth',
            10
        )
        self.voxel_size = 0.05

    def point_cloud_callback(self, msg: PointCloud2) -> None:
        """
        Downsample pointcloud and republish to topic.

        Args:
        ----
            msg (PointCloud2): pointcloud message from topic
        Return:
        ------
            None

        """
        pc_data = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        if pc_data.shape[0] == 0:
            self.get_logger().warn("Received empty point cloud!")
            return
        converted = pc_data.view(np.float32).reshape(len(pc_data), -1)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(converted)
        downsampled_pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        downsampled_points = np.asarray(downsampled_pcd.points)
        # Rotation matrix from optical to body frame
        R = np.array([[0, 0, 1],
                      [-1, 0, 0],
                      [0, -1, 0]])
        points_body = np.dot(downsampled_points, R.T)
        # Convert back to PointCloud2 and publish
        msg.header.frame_id = 'camera_2'
        downsampled_msg = pc2.create_cloud_xyz32(msg.header, points_body)
        self.point_cloud_pub.publish(downsampled_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDownsampling()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
