"""
Conversion between PIMM's native output (a plain list of `open3d.geometry.PointCloud`,
one per tracked instance) and two community-facing summary forms:

- a single labeled `open3d.t.geometry.PointCloud` -- all instances merged into one
  point cloud, with a per-point integer `labels` field identifying which instance each
  point belongs to (1-indexed) plus each point's color.
- a centroid `open3d.geometry.PointCloud` -- one point per instance, at its centroid.
  A regular (non-tensor) point cloud is the right fit here: unlike the labeled cloud,
  there's no per-point ambiguity to resolve (each point already *is* one instance), so
  there's nothing a `labels` field would add.

`tracker.PodTracker.run()` itself always returns the plain list form -- this module is
an optional convenience for callers who want either summary form instead (e.g. for a
single labeled .ply file, or a quick centroid/count overview), not a change to what
tracking itself produces.
"""

import numpy as np
import open3d as o3d


def to_labeled_pointcloud(pods):
    """
    Merges a list of `open3d.geometry.PointCloud` instances (as returned by
    `PodTracker.run()`) into a single `open3d.t.geometry.PointCloud` with per-point
    `labels` (int32, 1-indexed by instance) and `colors` (uint8) fields.

    Instances with zero points are skipped. Raises ValueError if none of the instances
    have points at all.
    """
    all_points, all_colors, all_labels = [], [], []

    for instance_id, pod in enumerate(pods, start=1):
        pts = np.asarray(pod.points)
        if len(pts) == 0:
            continue
        cols = np.asarray(pod.colors)
        if len(cols) != len(pts):
            # no colors set on this instance -- fall back to a mid-gray so the output
            # still has a well-formed colors field
            cols = np.full((len(pts), 3), 0.5)

        all_points.append(pts)
        all_colors.append((cols * 255).astype(np.uint8))
        all_labels.append(np.full((len(pts), 1), instance_id, dtype=np.int32))

    if not all_points:
        raise ValueError("None of the given pods have any points.")

    pcd_t = o3d.t.geometry.PointCloud(o3d.core.Tensor(np.vstack(all_points), o3d.core.float32))
    pcd_t.point["colors"] = o3d.core.Tensor(np.vstack(all_colors), o3d.core.uint8)
    pcd_t.point["labels"] = o3d.core.Tensor(np.vstack(all_labels), o3d.core.int32)
    return pcd_t


def save_labeled_pointcloud(pods, path: str):
    """Converts via `to_labeled_pointcloud` and writes the result to `path`."""
    pcd_t = to_labeled_pointcloud(pods)
    o3d.t.io.write_point_cloud(path, pcd_t)
    return pcd_t


def to_centroid_pointcloud(pods):
    """
    Reduces a list of `open3d.geometry.PointCloud` instances (as returned by
    `PodTracker.run()`) to a single `open3d.geometry.PointCloud` with one point per
    instance, at that instance's centroid -- a quick overview / count of everything
    tracked, independent of how many points each instance actually has.

    Instances with zero points are skipped. Raises ValueError if none of the instances
    have points at all.
    """
    centroids = [np.asarray(pod.points).mean(axis=0) for pod in pods if len(pod.points) > 0]
    if not centroids:
        raise ValueError("None of the given pods have any points.")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(centroids))
    return pcd


def save_centroid_pointcloud(pods, path: str):
    """Converts via `to_centroid_pointcloud` and writes the result to `path`."""
    pcd = to_centroid_pointcloud(pods)
    o3d.io.write_point_cloud(path, pcd)
    return pcd
