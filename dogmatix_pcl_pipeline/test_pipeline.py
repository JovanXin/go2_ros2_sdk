#!/usr/bin/env python3
"""Test the point cloud processor with synthetic room data."""

import numpy as np
from dogmatix_pcl_pipeline.pointcloud_processor import PointCloudProcessor
from dogmatix_pcl_pipeline.floorplan_generator import FloorplanGenerator


def generate_test_room():
    """Generate a simple6x4x2.8m room."""
    points = []
    density = 500

    # Floor
    n = int(6 * 4 * density)
    x, y = np.random.uniform(0, 6, n), np.random.uniform(0, 4, n)
    points.append(np.column_stack([x, y, np.random.normal(0, 0.005, n)]))

    # Ceiling
    n = int(6 * 4 * density * 0.5)
    x, y = np.random.uniform(0, 6, n), np.random.uniform(0, 4, n)
    points.append(np.column_stack([x, y, np.random.normal(2.8, 0.005, n)]))

    # Walls (4 walls)
    for wall_x in [0, 6]:
        n = int(4 * 2.8 * density)
        x = np.random.normal(wall_x, 0.005, n)
        y = np.random.uniform(0, 4, n)
        z = np.random.uniform(0, 2.8, n)
        points.append(np.column_stack([x, y, z]))

    for wall_y in [0, 4]:
        n = int(6 * 2.8 * density)
        x = np.random.uniform(0, 6, n)
        y = np.random.normal(wall_y, 0.005, n)
        z = np.random.uniform(0, 2.8, n)
        points.append(np.column_stack([x, y, z]))

    return np.vstack(points)


def test_processor():
    """Test point cloud processing pipeline."""
    print("Generating test room...")
    points = generate_test_room()
    print(f"Generated {len(points)} points")

    processor = PointCloudProcessor({
        'voxel_size': 0.02,
        'ransac_distance_threshold': 0.03,
    })

    print("Processing point cloud...")
    result = processor.process_pointcloud(points)

    print(f"\nResults:")
    print(f"  Planes detected: {len(result['planes'])}")
    print(f"  Wall lines: {len(result['wall_lines'])}")
    print(f"  Floor points: {len(result['floor_points'])}")
    print(f"  Ceiling points: {len(result['ceiling_points'])}")
    print(f"  Wall points: {len(result['wall_points'])}")

    for i, plane in enumerate(result['planes']):
        print(f"\n  Plane {i}: type={plane.plane_type}, points={len(plane.points)}, "
              f"height={plane.height:.2f}m, normal={plane.normal}")

    if result['wall_lines']:
        print(f"\nWall lines:")
        for i, (start, end) in enumerate(result['wall_lines']):
            length = np.linalg.norm(end - start)
            print(f"  Wall {i}: ({start[0]:.1f},{start[1]:.1f}) -> ({end[0]:.1f},{end[1]:.1f}), length={length:.2f}m")

    return result


def test_floorplan():
    """Test floorplan generation."""
    result = test_processor()

    if not result['wall_lines']:
        print("No wall lines to generate floorplan from")
        return

    print("\n\nGenerating floorplan...")
    gen = FloorplanGenerator()
    fp = gen.process_wall_lines(result['wall_lines'], '/tmp/dogmatix_test')

    print(f"Walls: {len(fp['walls'])}")
    for i, wall in enumerate(fp['walls']):
        print(f"  Wall {i}: length={wall.length:.2f}m, angle={np.degrees(wall.angle):.1f}°")

    print(f"\nSVG: {fp['svg_path']}")
    print(f"JSON: {fp['json_path']}")


if __name__ == '__main__':
    test_floorplan()
