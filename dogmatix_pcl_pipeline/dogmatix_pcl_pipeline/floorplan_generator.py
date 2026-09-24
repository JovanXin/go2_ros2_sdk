"""
Floorplan generator — converts wall lines to2D floor plan.

Takes wall line segments from pointcloud_processor and generates
a2D floor plan with dimensions. Exports to SVG/DXF.
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
import logging
import json

logger = logging.getLogger(__name__)


class Wall:
    """A wall segment with start/end points and thickness."""

    def __init__(self, start: np.ndarray, end: np.ndarray, thickness: float = 0.15):
        self.start = start
        self.end = end
        self.thickness = thickness

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.end - self.start))

    @property
    def angle(self) -> float:
        """Angle in radians from positive X axis."""
        diff = self.end - self.start
        return float(np.arctan2(diff[1], diff[0]))

    @property
    def midpoint(self) -> np.ndarray:
        return (self.start + self.end) / 2

    def to_svg_line(self, scale: float = 100.0, offset: np.ndarray = None) -> str:
        """Generate SVG line element."""
        if offset is None:
            offset = np.zeros(2)
        s = (self.start[:2] + offset) * scale
        e = (self.end[:2] + offset) * scale
        return f'<line x1="{s[0]:.1f}" y1="{s[1]:.1f}" x2="{e[0]:.1f}" y2="{e[1]:.1f}" stroke="black" stroke-width="{self.thickness * scale:.1f}"/>'


class FloorplanGenerator:
    """Generate2D floorplan from wall segments."""

    def __init__(self, config: Optional[Dict] = None):
        cfg = config or {}
        self.wall_thickness = cfg.get('wall_thickness', 0.15)  # meters
        self.merge_distance = cfg.get('merge_distance', 0.3)  # merge walls within 30cm
        self.min_wall_length = cfg.get('min_wall_length', 0.5)  # ignore walls < 50cm

    def merge_walls(self, wall_lines: List[Tuple[np.ndarray, np.ndarray]]) -> List[Wall]:
        """Merge collinear wall segments that are close together."""
        if not wall_lines:
            return []

        # Convert to Wall objects
        walls = [Wall(s, e, self.wall_thickness) for s, e in wall_lines]

        # Filter short walls
        walls = [w for w in walls if w.length >= self.min_wall_length]

        # Simple merge: group by similar angle and proximity
        merged = []
        used = set()

        for i, w1 in enumerate(walls):
            if i in used:
                continue

            group = [w1]
            for j, w2 in enumerate(walls):
                if j <= i or j in used:
                    continue

                # Check if collinear (similar angle)
                angle_diff = abs(w1.angle - w2.angle)
                if angle_diff > np.pi:
                    angle_diff = 2 * np.pi - angle_diff

                if angle_diff > 0.2:  # ~11 degrees
                    continue

                # Check proximity (midpoints close or endpoints close)
                dist = min(
                    np.linalg.norm(w1.midpoint - w2.midpoint),
                    np.linalg.norm(w1.start - w2.start),
                    np.linalg.norm(w1.end - w2.end),
                )

                if dist < self.merge_distance:
                    group.append(w2)
                    used.add(j)

            # Merge group into single wall
            all_points = np.vstack([(w.start, w.end) for w in group])
            merged_wall = self._fit_wall_to_points(all_points, w1.angle)
            merged.append(merged_wall)

        return merged

    def _fit_wall_to_points(self, points: np.ndarray, angle_hint: float) -> Wall:
        """Fit a single wall line to a set of endpoints."""
        # Project onto the wall direction
        direction = np.array([np.cos(angle_hint), np.sin(angle_hint)])
        xy = points[:, :2]
        projections = xy @ direction

        min_proj = np.min(projections)
        max_proj = np.max(projections)

        # Use the centroid perpendicular to the wall direction
        centroid = np.mean(xy, axis=0)
        perp = np.array([-direction[1], direction[0]])
        perp_offset = np.mean((xy - centroid) @ perp)

        start_2d = centroid + direction * min_proj + perp * perp_offset
        end_2d = centroid + direction * max_proj + perp * perp_offset

        # Keep Z from original points
        z_mean = np.mean(points[:, 2])
        start_3d = np.array([start_2d[0], start_2d[1], z_mean])
        end_3d = np.array([end_2d[0], end_2d[1], z_mean])

        return Wall(start_3d, end_3d, self.wall_thickness)

    def compute_dimensions(self, walls: List[Wall]) -> List[Dict]:
        """Compute wall dimensions for annotation."""
        dims = []
        for i, wall in enumerate(walls):
            dims.append({
                'wall_id': i,
                'length': wall.length,
                'angle_deg': np.degrees(wall.angle),
                'midpoint': wall.midpoint.tolist(),
            })
        return dims

    def generate_svg(self, walls: List[Wall], output_path: str,
                     scale: float = 100.0, padding: float = 1.0) -> str:
        """Generate SVG floorplan from wall segments."""
        if not walls:
            logger.warning("No walls to render")
            return ""

        # Compute bounds
        all_points = np.vstack([(w.start[:2], w.end[:2]) for w in walls])
        min_xy = np.min(all_points, axis=0) - padding
        max_xy = np.max(all_points, axis=0) + padding
        extent = max_xy - min_xy

        # SVG dimensions
        svg_w = extent[0] * scale + 2 * padding * scale
        svg_h = extent[1] * scale + 2 * padding * scale

        # Offset so everything is positive
        offset = -min_xy + padding

        lines = [w.to_svg_line(scale, offset) for w in walls]

        svg = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{svg_w:.0f}" height="{svg_h:.0f}" viewBox="0 0 {svg_w:.0f} {svg_h:.0f}">
  <rect width="100%" height="100%" fill="white"/>
  <g transform="translate(0,{svg_h:.0f}) scale(1,-1)">
    {chr(10).join(lines)}
  </g>
</svg>'''

        with open(output_path, 'w') as f:
            f.write(svg)

        logger.info(f"SVG floorplan saved to {output_path}")
        return svg

    def generate_json(self, walls: List[Wall], output_path: str) -> Dict:
        """Export floorplan as JSON with wall coordinates and dimensions."""
        data = {
            'walls': [],
            'total_wall_length': sum(w.length for w in walls),
            'wall_count': len(walls),
        }

        for i, wall in enumerate(walls):
            data['walls'].append({
                'id': i,
                'start': wall.start.tolist(),
                'end': wall.end.tolist(),
                'length': wall.length,
                'thickness': wall.thickness,
                'angle_deg': np.degrees(wall.angle),
            })

        with open(output_path, 'w') as f:
            json.dump(data, f, indent=2)

        logger.info(f"JSON floorplan saved to {output_path}")
        return data

    def process_wall_lines(self, wall_lines: List[Tuple[np.ndarray, np.ndarray]],
                           output_dir: str = '/tmp/dogmatix') -> Dict:
        """Full pipeline: merge → dimension → export.

        Args:
            wall_lines: list of (start_xy, end_xy) from pointcloud_processor
            output_dir: directory for output files

        Returns:
            Dict with walls, dimensions, and file paths
        """
        import os
        os.makedirs(output_dir, exist_ok=True)

        walls = self.merge_walls(wall_lines)
        dimensions = self.compute_dimensions(walls)

        svg_path = os.path.join(output_dir, 'floorplan.svg')
        json_path = os.path.join(output_dir, 'floorplan.json')

        self.generate_svg(walls, svg_path)
        self.generate_json(walls, json_path)

        return {
            'walls': walls,
            'dimensions': dimensions,
            'svg_path': svg_path,
            'json_path': json_path,
        }
