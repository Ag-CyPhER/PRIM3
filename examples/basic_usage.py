"""
Minimal, synthetic, runnable example: a sequence of 3 frames, each with a couple of
point-cloud "detections" (small point clusters), showing two objects seen repeatedly,
each drifting slightly between frames (simulating small camera-to-camera positional
noise). Demonstrates the whole PIMM flow: preprocessing each detection, then tracking
across the sequence.

Run with: python examples/basic_usage.py
"""

import numpy as np
import open3d as o3d

from pimm import PointCloudPreprocessor, PodTracker


def make_blob(center, n_points=150, spread=0.01, seed=0):
    rng = np.random.default_rng(seed)
    pts = center + rng.normal(scale=spread, size=(n_points, 3))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd


def main():
    frames = [
        [make_blob([0.0, 0.0, 1.0], seed=0), make_blob([0.5, 0.2, 1.2], seed=1)],
        [make_blob([0.002, -0.001, 1.001], seed=2), make_blob([0.503, 0.198, 1.199], seed=3)],
        [make_blob([0.001, 0.001, 1.002], seed=5), make_blob([0.498, 0.201, 1.201], seed=6)],
    ]

    preprocessor = PointCloudPreprocessor(dbscan_min_points=10)  # small min_points since these are toy blobs
    cleaned_frames = [[preprocessor.clean(pcd) for pcd in frame] for frame in frames]

    tracker = PodTracker(
        batch_size=-1,                  # one batch, all frames
        matching_method="overlap",      # or "chamfer"
        overlap_dist_thresh=0.05,
        overlap_score_thresh=0.3,
        max_points_per_track=400,       # FPS-capped once a track exceeds this many points
    )

    final_pods = tracker.run(cleaned_frames)

    print(f"Tracked {len(final_pods)} distinct objects across {len(frames)} frames.")
    for i, pcd in enumerate(final_pods):
        print(f"  object {i}: {len(pcd.points)} accumulated points, centroid={pcd.get_center()}")


if __name__ == "__main__":
    main()
