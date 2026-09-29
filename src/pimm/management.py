"""
Track management: keeping a merged track's point count bounded as it accumulates
observations across many frames.

As a track gets matched and merged frame after frame, its point cloud keeps growing
(old points + new points every match). Left unchecked, a long-lived track can grow to
many times the size of a single detection, which slows down every subsequent distance
computation involving it and doesn't add real information past a point. Farthest Point
Sampling (FPS) keeps a track's size bounded while preserving its overall shape far
better than random or uniform subsampling would -- it greedily keeps the point set that
stays maximally spread out, which is exactly what you want when the goal is "keep the
track's silhouette recognizable," not "keep a random subset of it."
"""


def cap_points_with_fps(pcd, max_points: int):
    """
    If `pcd` has more than `max_points` points, downsamples it to exactly `max_points`
    via farthest point sampling. Returns `pcd` unchanged if it's already at or under
    the limit, or if `max_points` is None (no cap).
    """
    if max_points is None or not pcd.has_points() or len(pcd.points) <= max_points:
        return pcd
    return pcd.farthest_point_down_sample(max_points)
