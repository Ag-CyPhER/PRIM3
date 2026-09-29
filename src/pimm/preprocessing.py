"""
Per-detection point cloud cleanup, applied before matching/tracking.

Three independent, composable steps:

- **Voxel downsampling** (optional, off by default): normalizes point density. Useful
  when the same PIMM pipeline needs to accept detections from upstream methods that
  produce very different point counts per object (tens of points vs. thousands) --
  downsampling to a common voxel size before matching makes distance-based thresholds
  (e.g. the overlap distance used in matching.py) behave consistently regardless of
  the source method.
- **Statistical outlier removal**: drops points whose local neighborhood density is
  anomalously low -- isolated stray points that don't belong to the real surface.
- **DBSCAN clustering, keeping only the largest cluster**: a single detection's raw
  point cloud can span multiple physically disconnected surfaces (e.g. background
  geometry visible through a gap in the foreground object) -- this keeps only the
  largest coherent cluster and discards the rest.
"""

import numpy as np
import open3d as o3d


class PointCloudPreprocessor:

    def __init__(self,
                 voxel_size: float = None,
                 so_std_ratio: float = 0.15,
                 so_nb_neighbors: int = 50,
                 dbscan_eps: float = 0.02,
                 dbscan_min_points: int = 25):
        self.voxel_size = voxel_size
        self.so_std_ratio = so_std_ratio
        self.so_nb_neighbors = so_nb_neighbors
        self.dbscan_eps = dbscan_eps
        self.dbscan_min_points = dbscan_min_points

    def voxel_downsample(self, pcd, voxel_size: float = None):
        voxel_size = self.voxel_size if voxel_size is None else voxel_size
        if voxel_size is None or not pcd.has_points():
            return pcd
        return pcd.voxel_down_sample(voxel_size)

    def remove_statistical_outliers(self, pcd):
        if not pcd.has_points():
            return pcd
        clean_pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=self.so_nb_neighbors, std_ratio=self.so_std_ratio)
        return clean_pcd

    def keep_largest_cluster(self, pcd):
        if not pcd.has_points():
            return pcd
        labels = np.array(pcd.cluster_dbscan(eps=self.dbscan_eps, min_points=self.dbscan_min_points))
        unique_labels, counts = np.unique(labels[labels != -1], return_counts=True)
        if len(counts) == 0:
            return o3d.geometry.PointCloud()
        largest_label = unique_labels[np.argmax(counts)]
        return pcd.select_by_index(np.where(labels == largest_label)[0])

    def clean(self, pcd, perform_voxel_downsample: bool = False, perform_dbscan: bool = True):
        """
        Full pipeline: optional voxel downsample -> statistical outlier removal ->
        optional DBSCAN largest-cluster extraction.
        """
        if perform_voxel_downsample:
            pcd = self.voxel_downsample(pcd)

        pcd = self.remove_statistical_outliers(pcd)
        if not pcd.has_points():
            return pcd

        if perform_dbscan:
            return self.keep_largest_cluster(pcd)
        return pcd
