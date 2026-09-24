#!/usr/bin/env python3
"""
Generate synthetic indoor point cloud for testing the pipeline.

Creates a simple room with walls, floor, ceiling, and some furniture.
Publishes as PointCloud2 on /pointcloud topic.

Usage:
  ros2 run dogmatix_pcl_pipeline generate_test_cloud
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import time


def generate_room(length=6.0, width=4.0, height=2.8, density=500):
    """Generate points for a simple rectangular room.

    Args:
        length: room length in meters (X)
        width: room width in meters (Y)
        height: room height in meters (Z)
        density: points per square meter

    Returns:
        Nx3 numpy array of points
    """
    points = []

    # Floor (Z=0)
    n_floor = int(length * width * density)
    x = np.random.uniform(0, length, n_floor)
    y = np.random.uniform(0, width, n_floor)
    z = np.random.normal(0, 0.005, n_floor)  # slight noise
    points.append(np.column_stack([x, y, z]))

    # Ceiling (Z=height)
    n_ceil = int(length * width * density * 0.5)
    x = np.random.uniform(0, length, n_ceil)
    y = np.random.uniform(0, width, n_ceil)
    z = np.random.normal(height, 0.005, n_ceil)
    points.append(np.column_stack([x, y, z]))

    # Wall 1 (X=0)
    n_wall = int(width * height * density)
    x = np.random.normal(0, 0.005, n_wall)
    y = np.random.uniform(0, width, n_wall)
    z = np.random.uniform(0, height, n_wall)
    points.append(np.column_stack([x, y, z]))

    # Wall 2 (X=length)
    x = np.random.normal(length, 0.005, n_wall)
    y = np.random.uniform(0, width, n_wall)
    z = np.random.uniform(0, height, n_wall)
    points.append(np.column_stack([x, y, z]))

    # Wall 3 (Y=0)
    n_wall2 = int(length * height * density)
    x = np.random.uniform(0, length, n_wall2)
    y = np.random.normal(0, 0.005, n_wall2)
    z = np.random.uniform(0, height, n_wall2)
    points.append(np.column_stack([x, y, z]))

    # Wall 4 (Y=width)
    x = np.random.uniform(0, length, n_wall2)
    y = np.random.normal(width, 0.005, n_wall2)
    z = np.random.uniform(0, height, n_wall2)
    points.append(np.column_stack([x, y, z]))

    # Add some noise (furniture-like)
    n_noise = int(length * width * 100)
    x = np.random.uniform(0, length, n_noise)
    y = np.random.uniform(0, width, n_noise)
    z = np.random.uniform(0, height, n_noise)
    points.append(np.column_stack([x, y, z]))

    return np.vstack(points)


class TestCloudPublisher(Node):
    """Publish synthetic room point cloud for testing."""

    def __init__(self):
        super().__init__('test_cloud_publisher')
        self.pub = self.create_publisher(PointCloud2, '/pointcloud', 10)
        self.timer = self.create_timer(2.0, self.publish_cloud)
        self.get_logger().info('Test cloud publisher started. Publishing on /pointcloud')

    def publish_cloud(self):
        points = generate_room()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud = point_cloud2.create_cloud(header, fields, points.tolist())
        self.pub.publish(cloud)
        self.get_logger().info(f'Published {len(points)} points')


def main(args=None):
    rclpy.init(args=args)
    node = TestCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
