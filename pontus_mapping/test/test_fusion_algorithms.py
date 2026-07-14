"""Fast unit tests for V2 tracking, association, stereo, and promotion."""

from __future__ import annotations

from math import radians
from types import SimpleNamespace

import numpy as np
import pytest

from .fusion_test_helpers import make_class_config
from .fusion_test_helpers import make_observation
from .fusion_test_helpers import make_track
from .fusion_test_helpers import stamp_namespace


def test_hungarian_finds_global_optimum_instead_of_greedy(fusion_module):
    costs = np.array([[1.0, 2.0], [1.1, 100.0]], dtype=float)
    valid = np.ones_like(costs, dtype=bool)

    assignments = fusion_module.solve_gated_assignment(
        costs,
        valid,
        unmatched_cost=200.0,
    )

    assert set(assignments) == {(0, 1), (1, 0)}


def test_hungarian_respects_invalid_pairs_and_unmatched_rows(fusion_module):
    costs = np.array([[0.1, 0.2], [0.1, 0.2]], dtype=float)
    valid = np.array([[False, True], [False, False]], dtype=bool)

    assignments = fusion_module.solve_gated_assignment(
        costs,
        valid,
        unmatched_cost=1.0,
    )

    assert assignments == [(0, 1)]


def test_stamp_deduplicator_is_bounded(fusion_module):
    deduplicator = fusion_module.StampDeduplicator(capacity=2)
    first = stamp_namespace(1.0)
    second = stamp_namespace(2.0)
    third = stamp_namespace(3.0)

    assert deduplicator.seen(first) is False
    assert deduplicator.seen(first) is True
    assert deduplicator.seen(second) is False
    assert deduplicator.seen(third) is False
    # The first stamp has fallen out of the bounded history.
    assert deduplicator.seen(first) is False


def test_track_manager_matches_in_xy_and_ignores_large_z_changes(fusion_module):
    settings = fusion_module.FusionSettings(
        sonar_track_gate_m=0.6,
        confirmation_hits=2,
    )
    manager = fusion_module.TrackManager(settings)

    manager.process_sonar_centroids(
        np.array([[0.0, 0.0, 100.0], [1.0, 0.0, -100.0]]),
        stamp_s=1.0,
        stamp_msg=stamp_namespace(1.0),
    )
    manager.process_sonar_centroids(
        # Reverse measurement order and invert z to exercise global assignment.
        np.array([[0.9, 0.0, 500.0], [0.1, 0.0, -500.0]]),
        stamp_s=2.0,
        stamp_msg=stamp_namespace(2.0),
    )

    tracks = sorted(manager.active_tracks(), key=lambda track: track.track_id)
    assert len(tracks) == 2
    assert all(track.sonar_hit_count == 2 for track in tracks)
    assert all(
        track.state is fusion_module.TrackState.CONFIRMED_UNLABELED
        for track in tracks
    )
    np.testing.assert_allclose(tracks[0].position_xy, [0.05, 0.0], atol=1e-8)
    np.testing.assert_allclose(tracks[1].position_xy, [0.95, 0.0], atol=1e-8)


def test_track_manager_retires_tentative_track_after_configured_misses(fusion_module):
    settings = fusion_module.FusionSettings(
        tentative_max_misses=1,
        confirmed_max_misses=10,
        confirmation_hits=4,
    )
    manager = fusion_module.TrackManager(settings)
    manager.process_sonar_centroids(
        np.array([[2.0, 1.0, 0.0]]),
        stamp_s=1.0,
        stamp_msg=stamp_namespace(1.0),
    )

    manager.process_sonar_centroids(
        np.empty((0, 3)),
        stamp_s=2.0,
        stamp_msg=stamp_namespace(2.0),
    )
    assert len(manager.active_tracks()) == 1

    manager.process_sonar_centroids(
        np.empty((0, 3)),
        stamp_s=3.0,
        stamp_msg=stamp_namespace(3.0),
    )
    assert manager.active_tracks() == []


def test_camera_association_accepts_inside_sector_and_rejects_outside(fusion_module):
    settings = fusion_module.FusionSettings(camera_track_max_age_s=5.0)
    associator = fusion_module.CameraAssociator(settings)
    config = make_class_config(
        fusion_module,
        bearing_margin_deg=2.0,
        max_track_range_m=10.0,
    )

    inside_track = make_track(fusion_module, 1, 5.0, 0.20)
    outside_track = make_track(fusion_module, 2, 5.0, 1.50)
    observation = make_observation(
        fusion_module,
        camera_name="camera_1",
        origin_xy=(0.0, 0.0),
        target_xy=(5.0, 0.0),
        half_width_deg=1.0,
    )

    assignments = associator.assign(
        [observation],
        [inside_track, outside_track],
        {1: config},
        config,
    )

    assert len(assignments) == 1
    assert assignments[0][1].track_id == inside_track.track_id
    assert observation.normalized_bearing_residual is not None
    assert observation.normalized_bearing_residual < 1.0


def test_camera_association_is_independent_of_track_and_ray_z(fusion_module):
    settings = fusion_module.FusionSettings()
    associator = fusion_module.CameraAssociator(settings)
    config = make_class_config(fusion_module)
    track = make_track(fusion_module, 1, 4.0, 0.0, z=10_000.0)

    flat_observation = make_observation(
        fusion_module,
        camera_name="camera_1",
        origin_xy=(0.0, 0.0),
        target_xy=(4.0, 0.0),
        direction_z=0.0,
    )
    pitched_observation = make_observation(
        fusion_module,
        camera_name="camera_1",
        origin_xy=(0.0, 0.0),
        target_xy=(4.0, 0.0),
        direction_z=0.7,
    )

    flat_result = associator.assign(
        [flat_observation], [track], {1: config}, config
    )
    pitched_result = associator.assign(
        [pitched_observation], [track], {1: config}, config
    )

    assert len(flat_result) == 1
    assert len(pitched_result) == 1
    assert pitched_observation.association_cost == pytest.approx(
        flat_observation.association_cost
    )


def test_camera_association_rejects_stale_track(fusion_module):
    settings = fusion_module.FusionSettings(camera_track_max_age_s=1.0)
    associator = fusion_module.CameraAssociator(settings)
    config = make_class_config(fusion_module)
    stale_track = make_track(fusion_module, 1, 4.0, 0.0, stamp_s=1.0)
    observation = make_observation(
        fusion_module,
        camera_name="camera_1",
        origin_xy=(0.0, 0.0),
        target_xy=(4.0, 0.0),
        stamp_s=3.0,
    )

    assert associator.assign(
        [observation], [stale_track], {1: config}, config
    ) == []


def test_stereo_triangulation_recovers_planar_intersection(fusion_module):
    settings = fusion_module.FusionSettings(
        stereo_max_range_m=20.0,
        stereo_condition_number_max=1.0e4,
    )
    triangulator = fusion_module.StereoTriangulator(settings)
    config = make_class_config(
        fusion_module,
        stereo_max_error_m=0.5,
        stereo_min_parallax_deg=3.0,
    )
    track = make_track(fusion_module, 1, 4.0, 0.0)
    first = make_observation(
        fusion_module,
        camera_name="camera_1",
        origin_xy=(0.0, -0.5),
        target_xy=(4.0, 0.0),
    )
    second = make_observation(
        fusion_module,
        camera_name="camera_2",
        origin_xy=(0.0, 0.5),
        target_xy=(4.0, 0.0),
    )

    result = triangulator.triangulate(first, second, track, config)

    assert result is not None
    np.testing.assert_allclose(result.point_xy, [4.0, 0.0], atol=1e-7)
    assert result.sonar_error_m < 1e-7
    assert result.parallax_deg > config.stereo_min_parallax_deg
    assert 0.0 < result.quality <= 1.0


def test_stereo_rejects_weak_parallax(fusion_module):
    settings = fusion_module.FusionSettings(stereo_max_range_m=200.0)
    triangulator = fusion_module.StereoTriangulator(settings)
    config = make_class_config(
        fusion_module,
        stereo_max_error_m=1.0,
        stereo_min_parallax_deg=3.0,
    )
    track = make_track(fusion_module, 1, 100.0, 0.0)
    first = make_observation(
        fusion_module,
        camera_name="camera_1",
        origin_xy=(0.0, -0.01),
        target_xy=(100.0, 0.0),
    )
    second = make_observation(
        fusion_module,
        camera_name="camera_2",
        origin_xy=(0.0, 0.01),
        target_xy=(100.0, 0.0),
    )

    assert triangulator.triangulate(first, second, track, config) is None


def test_fusion_accumulates_evidence_and_promotes_confirmed_track(fusion_module):
    settings = fusion_module.FusionSettings(enable_stereo=False)
    manager = fusion_module.TrackManager(settings)
    track = make_track(fusion_module, 1, 4.0, 0.0, stamp_s=10.0)
    manager.tracks[track.track_id] = track
    config = make_class_config(
        fusion_module,
        label_evidence_threshold=1.5,
        promotion_confidence=0.8,
        promotion_margin=1.0,
    )
    coordinator = fusion_module.FusionCoordinator(
        settings=settings,
        track_manager=manager,
        camera_associator=fusion_module.CameraAssociator(settings),
        stereo_triangulator=fusion_module.StereoTriangulator(settings),
        class_configs={1: config},
        default_class_config=config,
        camera_names=("camera_1", "camera_2"),
    )

    first = make_observation(
        fusion_module,
        camera_name="camera_1",
        origin_xy=(0.0, 0.0),
        target_xy=(4.0, 0.0),
        stamp_s=10.05,
    )
    second = make_observation(
        fusion_module,
        camera_name="camera_1",
        origin_xy=(0.0, 0.0),
        target_xy=(4.0, 0.0),
        stamp_s=10.10,
    )

    _, first_promotions = coordinator.process_camera_observations([first])
    _, second_promotions = coordinator.process_camera_observations([second])

    assert first_promotions == []
    assert second_promotions == [track]
    assert track.state is fusion_module.TrackState.LABELED
    assert track.promoted_label == 1


def test_camera_contexts_can_hold_different_models(fusion_module):
    class FakeModel:
        def __init__(self, focal_length: float):
            self.focal_length = focal_length

        @staticmethod
        def rectifyPoint(pixel):
            return pixel

        def projectPixelTo3dRay(self, pixel):
            # Test-only convention: local x is forward and local y is image-right.
            return np.array(
                [1.0, float(pixel[0]) / self.focal_length, 0.0],
                dtype=float,
            )

    identity_transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )

    class BoxCenter:
        x = 0.0
        y = 0.0

    detection = SimpleNamespace(
        bbox=SimpleNamespace(
            center=BoxCenter(),
            size_x=20.0,
            size_y=40.0,
        )
    )
    first = fusion_module.CameraContext("camera_1", "camera_1_frame")
    second = fusion_module.CameraContext("camera_2", "camera_2_frame")
    first.model = FakeModel(100.0)
    second.model = FakeModel(400.0)

    first_observation = first.detection_to_bearing(
        detection,
        class_id=1,
        confidence=1.0,
        stamp=stamp_namespace(1.0),
        transform=identity_transform,
    )
    second_observation = second.detection_to_bearing(
        detection,
        class_id=1,
        confidence=1.0,
        stamp=stamp_namespace(1.0),
        transform=identity_transform,
    )

    assert first.model is not second.model
    assert first_observation is not None
    assert second_observation is not None
    assert first_observation.half_width_rad > second_observation.half_width_rad
