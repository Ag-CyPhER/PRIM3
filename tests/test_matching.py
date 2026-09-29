import numpy as np
import open3d as o3d
import pytest

from pimm.matching import compute_overlap, compute_chamfer_l2, match_by_overlap, match_by_chamfer
from pimm.management import cap_points_with_fps
from pimm.preprocessing import PointCloudPreprocessor
from pimm.tracker import PodTracker
from pimm.io import to_labeled_pointcloud, to_centroid_pointcloud


def _make_pcd(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=np.float64))
    return pcd


def test_compute_overlap_identical_clouds_is_one():
    pcd = _make_pcd(np.random.default_rng(0).normal(size=(50, 3)))
    assert compute_overlap(pcd, pcd, dist_threshold=1e-6) == 1.0


def test_compute_overlap_far_apart_clouds_is_zero():
    pcd1 = _make_pcd([[0, 0, 0]] * 10)
    pcd2 = _make_pcd([[100, 100, 100]] * 10)
    assert compute_overlap(pcd1, pcd2, dist_threshold=0.01) == 0.0


def test_compute_overlap_empty_cloud_is_zero():
    pcd1 = _make_pcd([[0, 0, 0]])
    pcd2 = o3d.geometry.PointCloud()
    assert compute_overlap(pcd1, pcd2) == 0.0


def test_chamfer_identical_clouds_is_zero():
    pcd = _make_pcd([[0, 0, 0], [1, 1, 1]])
    assert compute_chamfer_l2(pcd, pcd) == 0.0


def test_match_by_overlap_matches_close_clouds():
    a = [_make_pcd([[0, 0, 0], [0.01, 0, 0]])]
    b = [_make_pcd([[0, 0, 0], [0.01, 0, 0]])]
    matches, unmatched_a, unmatched_b, _ = match_by_overlap(a, b, dist_threshold=0.05, score_threshold=0.5)
    assert matches == [(0, 0)]
    assert unmatched_a == []
    assert unmatched_b == []


def test_match_by_overlap_no_match_when_far_apart():
    a = [_make_pcd([[0, 0, 0]])]
    b = [_make_pcd([[10, 10, 10]])]
    matches, unmatched_a, unmatched_b, _ = match_by_overlap(a, b, dist_threshold=0.05, score_threshold=0.5,
                                                             centroid_search_radius=1.0)
    assert matches == []
    assert unmatched_a == [0]
    assert unmatched_b == [0]


def test_match_by_chamfer_matches_close_clouds():
    a = [_make_pcd([[0, 0, 0], [0.01, 0, 0]])]
    b = [_make_pcd([[0.001, 0, 0], [0.011, 0, 0]])]
    matches, unmatched_a, unmatched_b, _ = match_by_chamfer(a, b, chamfer_threshold=0.05)
    assert matches == [(0, 0)]
    assert unmatched_a == []
    assert unmatched_b == []


def test_cap_points_with_fps_reduces_to_exact_max():
    pcd = _make_pcd(np.random.default_rng(0).normal(size=(500, 3)))
    capped = cap_points_with_fps(pcd, max_points=100)
    assert len(capped.points) == 100


def test_cap_points_with_fps_leaves_small_cloud_unchanged():
    pcd = _make_pcd([[0, 0, 0], [1, 1, 1]])
    capped = cap_points_with_fps(pcd, max_points=100)
    assert len(capped.points) == 2


def test_cap_points_with_fps_none_max_disables_cap():
    pcd = _make_pcd(np.random.default_rng(0).normal(size=(500, 3)))
    capped = cap_points_with_fps(pcd, max_points=None)
    assert len(capped.points) == 500


def test_preprocessor_voxel_downsample_reduces_or_keeps_point_count():
    pcd = _make_pcd(np.random.default_rng(0).normal(scale=0.001, size=(200, 3)))
    pre = PointCloudPreprocessor(voxel_size=0.1)
    down = pre.voxel_downsample(pcd)
    assert len(down.points) <= len(pcd.points)


def test_preprocessor_clean_runs_without_error_on_small_cloud():
    pre = PointCloudPreprocessor(dbscan_min_points=2)
    pcd = _make_pcd([[0, 0, 0], [0.001, 0, 0], [0, 0.001, 0], [0.001, 0.001, 0]])
    cleaned = pre.clean(pcd)
    assert cleaned is not None


def _blob(center, n_points=100, spread=0.005, seed=0):
    rng = np.random.default_rng(seed)
    return _make_pcd(center + rng.normal(scale=spread, size=(n_points, 3)))


def test_podtracker_overlap_tracks_drifting_object_as_one_track():
    frames = [
        [_blob([0, 0, 0], seed=0)],
        [_blob([0.001, 0, 0], seed=1)],
        [_blob([0.002, 0, 0], seed=2)],
    ]
    tracker = PodTracker(batch_size=-1, matching_method="overlap",
                          overlap_dist_thresh=0.02, overlap_score_thresh=0.3)
    result = tracker.run(frames)
    assert len(result) == 1


def test_podtracker_chamfer_can_be_used_as_main_matching_method():
    frames = [
        [_blob([0, 0, 0], seed=0)],
        [_blob([0.001, 0, 0], seed=1)],
        [_blob([0.002, 0, 0], seed=2)],
    ]
    tracker = PodTracker(batch_size=-1, matching_method="chamfer", chamfer_threshold=0.05)
    result = tracker.run(frames)
    assert len(result) == 1


def test_podtracker_separate_objects_stay_separate_tracks():
    frames = [
        [_blob([0, 0, 0], seed=0), _blob([5, 5, 5], seed=1)],
        [_blob([0.001, 0, 0], seed=2), _blob([5.001, 5, 5], seed=3)],
    ]
    tracker = PodTracker(batch_size=-1, matching_method="overlap",
                          overlap_dist_thresh=0.02, overlap_score_thresh=0.3)
    result = tracker.run(frames)
    assert len(result) == 2


def test_to_labeled_pointcloud_merges_and_labels_correctly():
    pod_a = _make_pcd([[0, 0, 0], [1, 0, 0]])
    pod_a.paint_uniform_color([1, 0, 0])
    pod_b = _make_pcd([[5, 5, 5]])
    pod_b.paint_uniform_color([0, 1, 0])

    labeled = to_labeled_pointcloud([pod_a, pod_b])

    assert isinstance(labeled, o3d.t.geometry.PointCloud)
    assert labeled.point.positions.shape[0] == 3
    labels = labeled.point["labels"].numpy().flatten()
    assert sorted(labels.tolist()) == [1, 1, 2]
    colors = labeled.point["colors"].numpy()
    assert colors.shape == (3, 3)


def test_to_labeled_pointcloud_skips_empty_instances():
    pod_a = _make_pcd([[0, 0, 0]])
    pod_empty = o3d.geometry.PointCloud()
    labeled = to_labeled_pointcloud([pod_a, pod_empty])
    assert labeled.point.positions.shape[0] == 1


def test_to_labeled_pointcloud_raises_if_all_empty():
    with pytest.raises(ValueError):
        to_labeled_pointcloud([o3d.geometry.PointCloud(), o3d.geometry.PointCloud()])


def test_to_centroid_pointcloud_one_point_per_instance():
    pod_a = _make_pcd([[0, 0, 0], [2, 0, 0]])   # centroid: [1, 0, 0]
    pod_b = _make_pcd([[5, 5, 5]])              # centroid: [5, 5, 5]

    centroids = to_centroid_pointcloud([pod_a, pod_b])

    assert isinstance(centroids, o3d.geometry.PointCloud)
    pts = np.asarray(centroids.points)
    assert pts.shape == (2, 3)
    np.testing.assert_allclose(pts[0], [1, 0, 0])
    np.testing.assert_allclose(pts[1], [5, 5, 5])


def test_to_centroid_pointcloud_skips_empty_instances():
    pod_a = _make_pcd([[0, 0, 0]])
    pod_empty = o3d.geometry.PointCloud()
    centroids = to_centroid_pointcloud([pod_a, pod_empty])
    assert len(centroids.points) == 1


def test_to_centroid_pointcloud_raises_if_all_empty():
    with pytest.raises(ValueError):
        to_centroid_pointcloud([o3d.geometry.PointCloud(), o3d.geometry.PointCloud()])
