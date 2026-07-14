"""In-process ROS integration test for the complete V2 fusion data flow."""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from vision_msgs.msg import Detection2DArray

from .fusion_test_helpers import make_camera_info
from .fusion_test_helpers import make_cloud
from .fusion_test_helpers import make_detection
from .fusion_test_helpers import make_detection_array
from .fusion_test_helpers import make_optical_to_map_transform


pytestmark = pytest.mark.integration

SemanticRequest = tuple[
    list[int],
    list[tuple[float, float, float]],
]


def wait_for(
    predicate,
    *,
    timeout_s: float = 5.0,
    period_s: float = 0.01,
) -> None:
    """Wait until a ROS test condition becomes true."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(period_s)
    raise AssertionError("Timed out waiting for ROS test condition")


def cloud_xyz(message: PointCloud2) -> np.ndarray:
    """Convert a PointCloud2 message into an N-by-3 float array."""
    points = pc2.read_points_numpy(
        message,
        field_names=("x", "y", "z"),
        skip_nans=True,
    )
    array = np.asarray(points, dtype=float)
    if array.size == 0:
        return np.empty((0, 3), dtype=float)
    return array.reshape((-1, 3))


def test_async_two_camera_fusion_end_to_end(
    fusion_module,
    topics_yaml_path,
    topics_config,
):
    """Exercise asynchronous fusion using the production TopicConfig path."""
    camera_names = [str(name) for name in topics_config["cameras"]]
    assert len(camera_names) == 2, (
        "This integration geometry expects exactly two configured cameras; "
        f"topics.yaml contains {camera_names}."
    )
    camera_a, camera_b = camera_names

    rclpy.init(
        args=[
            "--ros-args",
            "--params-file",
            str(topics_yaml_path),
        ]
    )

    executor = MultiThreadedExecutor(num_threads=4)
    service_node = Node("fusion_test_semantic_service")
    io_node = Node("fusion_test_io")
    semantic_requests: list[SemanticRequest] = []
    unlabeled_messages: list[PointCloud2] = []

    fusion_node = None
    spin_thread = None

    def semantic_callback(request, response):
        positions = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            for pose in request.positions
        ]
        semantic_requests.append((list(request.ids), positions))
        return response

    semantic_service = service_node.create_service(
        fusion_module.AddSemanticObject,
        str(topics_config["semantic_object_service"]),
        semantic_callback,
    )

    executor.add_node(service_node)
    executor.add_node(io_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # ImageCoordinator waits for the semantic service in its constructor.
        # The real service name comes from the same topics.yaml that the node
        # passes through TopicConfig.
        fusion_node = fusion_module.ImageCoordinator()
        executor.add_node(fusion_node)

        # Verify that the production TopicConfig loaded the exact requested
        # fields from the real wildcard parameter file.
        for key in (
            "cameras",
            "camera_frame_template",
            "camera_info_topic_template",
            "detections_topic_template",
            "semantic_object_service",
            "pointcloud_cluster_topic",
        ):
            expected = topics_config[key]
            if isinstance(expected, list):
                expected = [str(value) for value in expected]
            else:
                expected = str(expected)
            assert getattr(fusion_node.topics, key) == expected

        camera_frames = {
            camera_name: fusion_node.topics.camera_frame_template.replace(
                "{camera}",
                camera_name,
            )
            for camera_name in camera_names
        }
        camera_info_topics = {
            camera_name: fusion_node.topics.camera_info_topic_template.replace(
                "{camera}",
                camera_name,
            )
            for camera_name in camera_names
        }
        detection_topics = {
            camera_name: fusion_node.topics.detections_topic_template.replace(
                "{camera}",
                camera_name,
            )
            for camera_name in camera_names
        }

        sonar_publisher = io_node.create_publisher(
            PointCloud2,
            fusion_node.topics.pointcloud_cluster_topic,
            10,
        )
        camera_info_publishers = {
            camera_name: io_node.create_publisher(
                CameraInfo,
                camera_info_topics[camera_name],
                10,
            )
            for camera_name in camera_names
        }
        detection_publishers = {
            camera_name: io_node.create_publisher(
                Detection2DArray,
                detection_topics[camera_name],
                10,
            )
            for camera_name in camera_names
        }
        unlabeled_subscription = io_node.create_subscription(
            PointCloud2,
            "/pontus/unlabeled_candidate_tracks",
            unlabeled_messages.append,
            10,
        )

        # Retain the ROS entities for the duration of the test.
        assert semantic_service is not None
        assert unlabeled_subscription is not None

        wait_for(lambda: sonar_publisher.get_subscription_count() >= 1)
        wait_for(
            lambda: all(
                publisher.get_subscription_count() >= 1
                for publisher in camera_info_publishers.values()
            )
        )
        wait_for(
            lambda: all(
                publisher.get_subscription_count() >= 1
                for publisher in detection_publishers.values()
            )
        )
        wait_for(
            lambda: fusion_node.unlabeled_tracks_publisher
            .get_subscription_count() >= 1
        )

        # Camera A is at the map origin. Camera B is one metre map-left.
        # The helper maps optical right/down/forward to map forward/left/up.
        fusion_node.tf_buffer.set_transform_static(
            make_optical_to_map_transform(
                camera_frames[camera_a],
                origin_xyz=(0.0, 0.0, 0.0),
            ),
            "fusion-test",
        )
        fusion_node.tf_buffer.set_transform_static(
            make_optical_to_map_transform(
                camera_frames[camera_b],
                origin_xyz=(0.0, 1.0, 0.0),
            ),
            "fusion-test",
        )

        # Different focal lengths prove that each configured camera retains a
        # separate PinholeCameraModel.
        camera_info_publishers[camera_a].publish(
            make_camera_info(camera_frames[camera_a], fx=400.0)
        )
        camera_info_publishers[camera_b].publish(
            make_camera_info(camera_frames[camera_b], fx=800.0)
        )
        wait_for(
            lambda: all(
                context.calibration_ready
                for context in fusion_node.camera_contexts.values()
            )
        )
        assert (
            fusion_node.camera_contexts[camera_a].model
            is not fusion_node.camera_contexts[camera_b].model
        )
        assert fusion_node.camera_contexts[camera_a].model.fx() == pytest.approx(
            400.0
        )
        assert fusion_node.camera_contexts[camera_b].model.fx() == pytest.approx(
            800.0
        )

        # The first sonar scan creates one tentative track.
        first_cloud = make_cloud([(4.0, 0.0, 100.0)], stamp_s=1.0)
        sonar_publisher.publish(first_cloud)
        wait_for(
            lambda: len(fusion_node.track_manager.active_tracks()) == 1
            and fusion_node.track_manager.active_tracks()[0].sonar_hit_count == 1
        )

        # Re-publishing an identical stamp must not produce another hit.
        sonar_publisher.publish(first_cloud)
        time.sleep(0.15)
        assert fusion_node.track_manager.active_tracks()[0].sonar_hit_count == 1

        # Large z changes must update the same planar track. Waiting after each
        # message also prevents test scheduling from changing message order.
        sonar_updates = (
            (2.0, -100.0, 2),
            (3.0, 50.0, 3),
            (4.0, 0.0, 4),
        )
        for stamp_s, z_value, expected_hits in sonar_updates:
            sonar_publisher.publish(
                make_cloud([(4.0, 0.0, z_value)], stamp_s=stamp_s)
            )
            wait_for(
                lambda count=expected_hits: (
                    len(fusion_node.track_manager.active_tracks()) == 1
                    and fusion_node.track_manager.active_tracks()[0]
                    .sonar_hit_count == count
                )
            )

        wait_for(
            lambda: fusion_node.track_manager.active_tracks()[0].state
            is fusion_module.TrackState.CONFIRMED_UNLABELED
        )
        wait_for(
            lambda: any(
                cloud_xyz(message).shape[0] == 1
                for message in unlabeled_messages
            )
        )

        track = fusion_node.track_manager.active_tracks()[0]
        np.testing.assert_allclose(track.position_xy, [4.0, 0.0], atol=1e-6)

        # Camera A sees the target at its principal point and contributes
        # evidence immediately without waiting for Camera B.
        camera_a_detection = make_detection(
            class_name="red_slalom",
            confidence=0.95,
            center_u=320.0,
            frame_id=camera_frames[camera_a],
            stamp_s=4.05,
        )
        detection_publishers[camera_a].publish(
            make_detection_array(
                [camera_a_detection],
                frame_id=camera_frames[camera_a],
                stamp_s=4.05,
            )
        )
        wait_for(
            lambda: track.label_evidence.get(
                fusion_module.SemanticObject.SLALOM_RED,
                0.0,
            ) > 0.0
        )
        first_camera_evidence = track.label_evidence[
            fusion_module.SemanticObject.SLALOM_RED
        ]
        assert semantic_requests == []

        # Camera B is one metre map-left. With fx=800, u=520 observes the
        # target at (4, 0), because optical x/z is 1/4.
        camera_b_detection = make_detection(
            class_name="red_slalom",
            confidence=0.95,
            center_u=520.0,
            frame_id=camera_frames[camera_b],
            stamp_s=4.10,
        )
        detection_publishers[camera_b].publish(
            make_detection_array(
                [camera_b_detection],
                frame_id=camera_frames[camera_b],
                stamp_s=4.10,
            )
        )
        wait_for(
            lambda: track.label_evidence.get(
                fusion_module.SemanticObject.SLALOM_RED,
                0.0,
            ) > first_camera_evidence
        )

        # A later asynchronous Camera-A observation crosses the promotion
        # threshold without any hard camera/camera/sonar synchronization.
        camera_a_repeat = make_detection(
            class_name="red_slalom",
            confidence=0.95,
            center_u=320.0,
            frame_id=camera_frames[camera_a],
            stamp_s=4.20,
        )
        detection_publishers[camera_a].publish(
            make_detection_array(
                [camera_a_repeat],
                frame_id=camera_frames[camera_a],
                stamp_s=4.20,
            )
        )

        wait_for(lambda: track.state is fusion_module.TrackState.LABELED)
        wait_for(lambda: len(semantic_requests) >= 1)
        assert semantic_requests[-1][0] == [
            fusion_module.SemanticObject.SLALOM_RED
        ]
        np.testing.assert_allclose(
            semantic_requests[-1][1][0][:2],
            [4.0, 0.0],
            atol=1e-6,
        )

        # Promotion republishes the complete unlabeled snapshot with no points.
        wait_for(
            lambda: bool(unlabeled_messages)
            and cloud_xyz(unlabeled_messages[-1]).shape[0] == 0
        )
    finally:
        executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)

        if fusion_node is not None:
            executor.remove_node(fusion_node)
            fusion_node.destroy_node()
        executor.remove_node(io_node)
        executor.remove_node(service_node)
        io_node.destroy_node()
        service_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
