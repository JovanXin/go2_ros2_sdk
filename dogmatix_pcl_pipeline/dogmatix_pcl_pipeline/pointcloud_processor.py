"""
Point cloud processor — wall, floor, ceiling detection.

Subscribes to PointCloud2, classifies geometry via RANSAC plane segmentation.
"""

import numpy as np
import open3d as o3d
from typing import Optional, Dict, List, Tuple
import logging

logger = logging.getLogger(__name__)


class PlaneResult:
    """Detected plane with normal, points, and classification."""

    def __init__(self, normal: np.ndarray, points: np.ndarray, plane_type: str):
        self.normal = normal
        self.points = points
        self.plane_type = plane_type  # 'floor', 'ceiling', 'wall'
        self.centroid = np.mean(points, axis=0) if len(points) > 0 else np.zeros(3)

    @property
    def height(self) -> float:
        """Mean Z value of the plane."""
        return float(self.centroid[2]) if len(self.points) > 0 else 0.0

    @property
    def area(self) -> float:
        """Approximate area from bounding box."""
        if len(self.points) < 3:
            return 0.0
        bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
            o3d.utility.Vector3dVector(self.points)
        )
        extent = bbox.get_extent()
        # For vertical planes (walls), area = height * width
        # For horizontal planes (floor/ceiling), area = length * width
        if abs(self.normal[2]) > 0.8:  # horizontal
            return float(extent[0] * extent[1])
        else:  # vertical (wall)
            return float(extent[0] * extent[1])  # width * height in XY plane


class PointCloudProcessor:
    """Process point clouds: segmentation, classification, filtering."""

    def __init__(self, config: Optional[Dict] = None):
        cfg = config or {}

        # RANSAC parameters
        self.ransac_distance_threshold = cfg.get('ransac_distance_threshold', 0.02)
        self.ransac_n_points = cfg.get('ransac_n_points', 3)
        self.ransac_iterations = cfg.get('ransac_iterations', 1000)

        # Classification thresholds (radians from vertical/horizontal)
        self.wall_angle_threshold = cfg.get('wall_angle_threshold', 0.3)  # ~17 degrees
        self.floor_ceiling_threshold = cfg.get('floor_ceiling_threshold', 0.3)

        # Filtering
        self.voxel_size = cfg.get('voxel_size', 0.01)  # 1cm downsampling
        self.min_points_per_plane = cfg.get('min_points_per_plane', 100)
        self.z_floor_range = cfg.get('z_floor_range', [-0.1, 0.3])  # expected floor Z
        self.z_ceiling_range = cfg.get('z_ceiling_range', [2.5, 3.5])  # expected ceiling Z

    def preprocess(self, points: np.ndarray) -> o3d.geometry.PointCloud:
        """Downsample and clean point cloud."""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])

        # Voxel downsample
        if self.voxel_size > 0:
            pcd = pcd.voxel_down_sample(self.voxel_size)

        # Remove statistical outliers
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        return pcd

    def segment_planes(self, pcd: o3d.geometry.PointCloud) -> List[PlaneResult]:
        """Extract planes via RANSAC iterative segmentation."""
        planes = []
        remaining = pcd

        for _ in range(10):  # max 10 planes
            if len(remaining.points) < self.min_points_per_plane:
                break

            plane_model, inlier_indices = remaining.segment_plane(
                distance_threshold=self.ransac_distance_threshold,
                ransac_n=self.ransac_n_points,
                num_iterations=self.ransac_iterations
            )

            if len(inlier_indices) < self.min_points_per_plane:
                break

            # Extract plane info
            normal = np.array(plane_model[:3])
            normal = normal / np.linalg.norm(normal)
            inlier_points = np.asarray(remaining.points)[inlier_indices]

            # Classify plane
            plane_type = self._classify_plane(normal, inlier_points)
            planes.append(PlaneResult(normal, inlier_points, plane_type))

            # Remove inliers from remaining cloud
            remaining = remaining.select_by_index(inlier_indices, invert=True)

        return planes

    def _classify_plane(self, normal: np.ndarray, points: np.ndarray) -> str:
        """Classify plane as floor, ceiling, or wall based on normal and height."""
        # Normal Z component determines if horizontal or vertical
        z_component = abs(normal[2])
        mean_z = np.mean(points[:, 2])

        if z_component > (1.0 - self.floor_ceiling_threshold):
            # Horizontal plane
            if mean_z < np.mean(self.z_floor_range) + 0.5:
                return 'floor'
            elif mean_z > np.mean(self.z_ceiling_range) - 0.5:
                return 'ceiling'
            else:
                return 'floor'  # could be a platform/shelf
        elif z_component < self.wall_angle_threshold:
            # Vertical plane = wall
            return 'wall'
        else:
            # Tilted — could be ramp, roof, etc.
            return 'wall'  # classify as wall for now

    def extract_wall_lines(self, walls: List[PlaneResult]) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Extract2D wall line segments from wall planes.

        Returns list of (start_xy, end_xy) tuples in world coordinates.
        """
        wall_lines = []

        for wall in walls:
            if wall.plane_type != 'wall' or len(wall.points) < 10:
                continue

            # Project wall points to XY plane
            xy_points = wall.points[:, :2]

            # Fit line to XY projection using PCA
            centroid = np.mean(xy_points, axis=0)
            centered = xy_points - centroid
            _, _, vh = np.linalg.svd(centered, full_matrices=False)

            # Principal direction
            direction = vh[0]

            # Project points onto principal axis
            projections = centered @ direction
            min_proj = np.min(projections)
            max_proj = np.max(projections)

            # Wall endpoints in XY
            start_2d = centroid + direction * min_proj
            end_2d = centroid + direction * max_proj

            # Keep Z from original points
            z_mean = np.mean(wall.points[:, 2])
            start = np.array([start_2d[0], start_2d[1], z_mean])
            end = np.array([end_2d[0], end_2d[1], z_mean])

            wall_lines.append((start, end))

        return wall_lines

    def process_pointcloud(self, points: np.ndarray) -> Dict:
        """Full pipeline: preprocess → segment → classify → extract.

        Args:
            points: Nx3 or Nx4 numpy array (x, y, z[, intensity])

        Returns:
            Dict with keys: planes, wall_lines, floor_points, ceiling_points, wall_points
        """
        if len(points) < 100:
            logger.warning(f"Too few points: {len(points)}")
            return {'planes': [], 'wall_lines': [], 'floor_points': np.empty((0, 3)),
                    'ceiling_points': np.empty((0, 3)), 'wall_points': np.empty((0, 3))}

        pcd = self.preprocess(points)
        planes = self.segment_planes(pcd)

        floor_points = np.vstack([p.points for p in planes if p.plane_type == 'floor']) if any(
            p.plane_type == 'floor' for p in planes) else np.empty((0, 3))
        ceiling_points = np.vstack([p.points for p in planes if p.plane_type == 'ceiling']) if any(
            p.plane_type == 'ceiling' for p in planes) else np.empty((0, 3))
        wall_points = np.vstack([p.points for p in planes if p.plane_type == 'wall']) if any(
            p.plane_type == 'wall' for p in planes) else np.empty((0, 3))

        wall_lines = self.extract_wall_lines(planes)

        return {
            'planes': planes,
            'wall_lines': wall_lines,
            'floor_points': floor_points,
            'ceiling_points': ceiling_points,
            'wall_points': wall_points,
        }
