"""
PodTracker: frame-by-frame 3D instance tracking.

Public entry point: `PodTracker(...).run(all_results_data)`, where `all_results_data`
is a list of frames, each frame a list of already-segmented
`open3d.geometry.PointCloud` detections. PIMM never asks how those detections were
produced -- backprojected from a depth map, selected directly from a point cloud,
whatever your upstream pipeline does is out of scope here by design.

The frame-to-frame matcher is selectable via `matching_method`:
  - "overlap" (default): `matching.match_by_overlap` -- symmetric point-overlap score,
    two parameters (distance, score).
  - "chamfer": `matching.match_by_chamfer` -- Chamfer-L2 distance, one parameter
    (chamfer distance). A genuine alternative main-tracking strategy -- it has the
    same (list_a, list_b) -> (matches, unmatched_a, unmatched_b, cost_matrix)
    interface as "overlap" and can be swapped in directly.
"""

import math

import numpy as np

from .matching import match_by_overlap, match_by_chamfer
from .management import cap_points_with_fps

_MATCHERS = {
    "overlap": lambda a, b, p: match_by_overlap(
        a, b,
        dist_threshold=p["overlap_dist_thresh"],
        score_threshold=p["overlap_score_thresh"],
        centroid_search_radius=p["centroid_search_radius"],
        overlap_method=p["overlap_method"],
    ),
    "chamfer": lambda a, b, p: match_by_chamfer(
        a, b,
        chamfer_threshold=p["chamfer_threshold"],
        centroid_search_radius=p["centroid_search_radius"],
    ),
}


class _SequentialTracker:
    """Low-level frame-by-frame tracking loop, used internally by PodTracker."""

    def __init__(self, matching_method="overlap"):
        if matching_method not in _MATCHERS:
            raise ValueError(f"matching_method must be one of {list(_MATCHERS)}, got {matching_method!r}")
        self.matching_method = matching_method

    def _match(self, pcds_a, pcds_b, params):
        return _MATCHERS[self.matching_method](pcds_a, pcds_b, params)

    def track_pods_across_frames(self, all_frames_pcds, max_points_per_track=5500, **match_params):
        import itertools
        np.random.seed(42)  # reproducible track colors across runs
        color_cycle = itertools.cycle([tuple(c) for c in np.random.rand(50, 3)])

        global_tracks = {}  # track_id -> (merged_pcd, color)
        next_track_id = 0
        all_frame_results = []

        for frame_index, current_frame_pcds in enumerate(all_frames_pcds):
            current_frame_tracks = {}  # frame_pcd_index -> track_id

            if frame_index == 0 or not global_tracks:
                for i, pcd in enumerate(current_frame_pcds):
                    new_id = next_track_id
                    new_color = next(color_cycle)
                    pcd.paint_uniform_color(new_color)
                    global_tracks[new_id] = (pcd, new_color)
                    current_frame_tracks[i] = new_id
                    next_track_id += 1

                all_frame_results.append(current_frame_tracks)
                continue

            global_id_list = list(global_tracks.keys())
            global_pcd_list = [track_data[0] for track_data in global_tracks.values()]

            matches, lost_track_indices, new_pod_indices, _ = self._match(
                global_pcd_list, current_frame_pcds, match_params)

            new_global_tracks_state = {}

            for global_idx, frame_idx in matches:
                track_id = global_id_list[global_idx]
                old_pcd, track_color = global_tracks[track_id]
                new_pcd = current_frame_pcds[frame_idx]
                new_pcd.paint_uniform_color(track_color)

                merged_pcd = old_pcd + new_pcd
                merged_pcd = cap_points_with_fps(merged_pcd, max_points_per_track)
                merged_pcd.paint_uniform_color(track_color)

                new_global_tracks_state[track_id] = (merged_pcd, track_color)
                current_frame_tracks[frame_idx] = track_id

            for global_idx in lost_track_indices:
                track_id = global_id_list[global_idx]
                new_global_tracks_state[track_id] = global_tracks[track_id]

            for frame_idx in new_pod_indices:
                new_pcd = current_frame_pcds[frame_idx]
                new_id = next_track_id
                new_color = next(color_cycle)
                next_track_id += 1
                new_pcd.paint_uniform_color(new_color)
                new_global_tracks_state[new_id] = (new_pcd, new_color)
                current_frame_tracks[frame_idx] = new_id

            global_tracks = new_global_tracks_state
            all_frame_results.append(current_frame_tracks)

        return all_frame_results, global_tracks

    def batch_track(self, results, max_points_per_track=5500, **match_params):
        _, final_global_state = self.track_pods_across_frames(
            results, max_points_per_track=max_points_per_track, **match_params)
        return [data[0] for data in final_global_state.values()]


class PodTracker:
    """
    Configure once, then call `.run(all_results_data)`.

    Args:
        batch_size: process frames in batches of this size (<= 0 means "one batch,
            all frames"). Useful for very long sequences where holding every frame's
            detections in memory at once is undesirable.
        matching_method: "overlap" (default) or "chamfer" -- see module docstring.
        overlap_method: "symmetric" or "asymmetric" (see matching.compute_overlap).
            Only used when matching_method="overlap".
        overlap_dist_thresh: distance (world units) for two points to count as
            "overlapping" during overlap-based matching.
        overlap_score_thresh: minimum overlap score for two detections to be
            considered the same object, when using overlap-based matching.
        chamfer_threshold: maximum Chamfer-L2 distance for two detections to be
            considered the same object, when using chamfer-based matching.
        centroid_search_radius: max centroid distance for two detections to even be
            considered as a candidate pair, before the chosen metric is computed.
        max_points_per_track: a merged track is downsampled via farthest point
            sampling if it exceeds this many points (see management.cap_points_with_fps).
            None disables the cap.
    """

    def __init__(self,
                 batch_size=20,
                 matching_method="overlap",
                 overlap_method="symmetric",
                 overlap_dist_thresh=0.15,
                 overlap_score_thresh=0.5,
                 chamfer_threshold=0.05,
                 centroid_search_radius=0.5,
                 max_points_per_track=5500):
        self._tracker = _SequentialTracker(matching_method=matching_method)

        self.batch_size = batch_size
        self.overlap_method = overlap_method
        self.overlap_dist_thresh = overlap_dist_thresh
        self.overlap_score_thresh = overlap_score_thresh
        self.chamfer_threshold = chamfer_threshold
        self.centroid_search_radius = centroid_search_radius
        self.max_points_per_track = max_points_per_track

    def run(self, all_results_data):
        """
        all_results_data: list of frames, each a list of o3d.geometry.PointCloud
        detections for that frame. Returns the final list of tracked/merged point
        clouds, one per distinct object instance found across the whole sequence.
        """
        match_params = dict(
            overlap_dist_thresh=self.overlap_dist_thresh,
            overlap_score_thresh=self.overlap_score_thresh,
            chamfer_threshold=self.chamfer_threshold,
            centroid_search_radius=self.centroid_search_radius,
            overlap_method=self.overlap_method,
        )

        total_frames = len(all_results_data)
        effective_batch_size = self.batch_size if self.batch_size > 0 else total_frames
        num_batches = math.ceil(total_frames / effective_batch_size)

        previous_batch_pods = []

        for i in range(num_batches):
            start_frame = i * effective_batch_size
            end_frame = min((i + 1) * effective_batch_size, total_frames)
            current_batch_frames = all_results_data[start_frame:end_frame]

            frames_to_track = (current_batch_frames if not previous_batch_pods
                                else [previous_batch_pods] + current_batch_frames)

            previous_batch_pods = self._tracker.batch_track(
                frames_to_track,
                max_points_per_track=self.max_points_per_track,
                **match_params,
            )

        return previous_batch_pods
