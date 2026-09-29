"""
Pairwise similarity and matching primitives for 3D point-cloud detections across frames.

Two independent similarity metrics:

- **Overlap score** (`compute_overlap`): the average fraction of each cloud's points
  that land within `dist_threshold` of the other cloud. Cheap to compute, and works
  well once both clouds were sampled at a broadly similar density (see
  `preprocessing.PointCloudPreprocessor.voxel_downsample` if your upstream detections
  vary a lot in point count).
- **Chamfer-L2 distance** (`compute_chamfer_l2`): mean nearest-neighbor distance in
  both directions. A continuous distance rather than a bounded fraction -- a genuine
  alternative to overlap, not a fallback for it.

Matching two sets of detections (`match_by_overlap`, `match_by_chamfer`) first narrows
candidate pairs with a k-d tree over centroids (avoiding an expensive full N-by-M
comparison), then finds the optimal one-to-one assignment with the Hungarian algorithm
(`scipy.optimize.linear_sum_assignment`). Both share the same (pcds_a, pcds_b) ->
(matches, unmatched_a, unmatched_b, cost_matrix) interface, so either can be used as
the main matching strategy in `tracker.PodTracker`.
"""

import logging

import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import linear_sum_assignment


def compute_overlap(pcd1, pcd2, dist_threshold=0.01, method="symmetric"):
    """
    Overlap score between two point clouds: the fraction of points in each that are
    "close" (within dist_threshold) to the other cloud.

    method="symmetric": average of both directions' overlap fraction.
    method="asymmetric": the larger of the two directions (useful when one cloud is
    expected to be a subset of the other, e.g. a partial view vs. an accumulated track).

    Returns a score in [0, 1], 0 meaning no overlap and 1 meaning perfect overlap.
    """
    if not pcd1.has_points() or not pcd2.has_points():
        return 0.0

    dist1_to_2 = np.asarray(pcd1.compute_point_cloud_distance(pcd2))
    dist2_to_1 = np.asarray(pcd2.compute_point_cloud_distance(pcd1))

    overlap_1_to_2 = np.count_nonzero(dist1_to_2 <= dist_threshold) / max(1, dist1_to_2.size)
    overlap_2_to_1 = np.count_nonzero(dist2_to_1 <= dist_threshold) / max(1, dist2_to_1.size)

    if method == "symmetric":
        return (overlap_1_to_2 + overlap_2_to_1) / 2.0
    elif method == "asymmetric":
        return max(overlap_1_to_2, overlap_2_to_1)
    raise ValueError(f"method must be 'symmetric' or 'asymmetric', got {method!r}")


def compute_chamfer_l2(pcd1, pcd2):
    """
    Chamfer-L2 distance: sum of the mean nearest-neighbor distance in both directions.
    (Not the squared Chamfer distance used in some other papers -- this is the mean of
    raw Euclidean distances, in each direction, summed.)
    """
    dists_1_to_2 = np.asarray(pcd1.compute_point_cloud_distance(pcd2))
    dists_2_to_1 = np.asarray(pcd2.compute_point_cloud_distance(pcd1))
    return np.mean(dists_1_to_2) + np.mean(dists_2_to_1)


def match_by_overlap(pcds_a, pcds_b,
                      dist_threshold=0.01,
                      score_threshold=0.5,
                      centroid_search_radius=0.5,
                      k_neighbors=3,
                      overlap_method="symmetric"):
    """
    Matches two lists of point clouds (e.g. existing tracks vs. a new frame's
    detections) using overlap score, restricted to centroid-nearby candidate pairs and
    resolved via the Hungarian algorithm for a globally optimal one-to-one assignment.

    Returns: matches (list of (i, j) index pairs), unmatched_a (list of indices into
    pcds_a), unmatched_b (list of indices into pcds_b), score_matrix.
    """
    num_a, num_b = len(pcds_a), len(pcds_b)

    if num_a == 0:
        return [], [], list(range(num_b)), np.zeros((0, num_b))
    if num_b == 0:
        return [], list(range(num_a)), [], np.zeros((num_a, 0))

    try:
        centroids_a = np.array([pcd.get_center() for pcd in pcds_a])
        tree_a = cKDTree(centroids_a)
    except Exception as e:
        logging.error(f"Error building k-d tree for pcds_a: {e}", exc_info=True)
        return [], list(range(num_a)), list(range(num_b)), np.zeros((num_a, num_b))

    try:
        centroids_b = np.array([pcd.get_center() for pcd in pcds_b])
    except Exception as e:
        logging.error(f"Error getting centroids for pcds_b: {e}", exc_info=True)
        return [], list(range(num_a)), list(range(num_b)), np.zeros((num_a, num_b))

    cost_matrix = np.ones((num_a, num_b))
    score_matrix = np.zeros((num_a, num_b))

    distances, indices = tree_a.query(centroids_b, k=k_neighbors, distance_upper_bound=centroid_search_radius)
    indices = np.atleast_2d(indices)

    for j, neighbors in enumerate(indices):
        pcd_b = pcds_b[j]
        for i in neighbors:
            if i == num_a:  # scipy sets index to num_a when no neighbor is found within radius
                continue
            score = compute_overlap(pcds_a[i], pcd_b, dist_threshold=dist_threshold, method=overlap_method)
            score_matrix[i, j] = score
            cost_matrix[i, j] = 1.0 - score

    row_ind, col_ind = linear_sum_assignment(cost_matrix)

    matches = []
    unmatched_a = set(range(num_a))
    unmatched_b = set(range(num_b))

    for i, j in zip(row_ind, col_ind):
        if score_matrix[i, j] > score_threshold:
            matches.append((i, j))
            unmatched_a.discard(i)
            unmatched_b.discard(j)

    return matches, list(unmatched_a), list(unmatched_b), score_matrix


def match_by_chamfer(pcds_a, pcds_b, chamfer_threshold=0.05, centroid_search_radius=0.25):
    """
    Matches two lists of point clouds using Chamfer-L2 distance instead of overlap
    score, otherwise identical in structure to `match_by_overlap` (centroid-restricted
    candidates, Hungarian assignment). Lower cost is better here (it's a distance, not
    a score).
    """
    num_a, num_b = len(pcds_a), len(pcds_b)

    if num_a == 0:
        return [], [], list(range(num_b)), np.zeros((0, num_b))
    if num_b == 0:
        return [], list(range(num_a)), [], np.zeros((num_a, 0))

    try:
        centroids_a = np.array([pcd.get_center() for pcd in pcds_a])
        tree_a = cKDTree(centroids_a)
    except Exception as e:
        logging.error(f"Error building k-d tree for pcds_a: {e}", exc_info=True)
        return [], list(range(num_a)), list(range(num_b)), np.full((num_a, num_b), np.inf)

    try:
        centroids_b = np.array([pcd.get_center() for pcd in pcds_b])
    except Exception as e:
        logging.error(f"Error getting centroids for pcds_b: {e}", exc_info=True)
        return [], list(range(num_a)), list(range(num_b)), np.full((num_a, num_b), np.inf)

    cost_matrix = np.full((num_a, num_b), np.inf)
    all_candidate_indices = tree_a.query_ball_point(centroids_b, r=centroid_search_radius)

    for j, indices_i in enumerate(all_candidate_indices):
        pcd_b = pcds_b[j]
        for i in indices_i:
            cost_matrix[i, j] = compute_chamfer_l2(pcds_a[i], pcd_b)

    finite_mask = np.isfinite(cost_matrix)
    if not np.any(finite_mask):
        return [], list(range(num_a)), list(range(num_b)), cost_matrix

    large_cost = max(1e6, np.max(cost_matrix[finite_mask]) * 10.0)
    cost_for_assignment = np.where(finite_mask, cost_matrix, large_cost)

    row_ind, col_ind = linear_sum_assignment(cost_for_assignment)

    matches = []
    unmatched_a = set(range(num_a))
    unmatched_b = set(range(num_b))

    for i, j in zip(row_ind, col_ind):
        if cost_matrix[i, j] < chamfer_threshold:
            matches.append((i, j))
            unmatched_a.discard(i)
            unmatched_b.discard(j)

    return matches, list(unmatched_a), list(unmatched_b), cost_matrix
