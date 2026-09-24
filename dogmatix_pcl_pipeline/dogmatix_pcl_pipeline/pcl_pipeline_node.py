"""
ROS2 pipeline node — subscribes to PointCloud2, runs processing, publishes results.

Subscribes:
  - /pointcloud (sensor_msgs/PointCloud2) — raw LIDAR data

Publishes:
  - /dogmatix/wall_points (sensor_msgs/PointCloud2) — classified wall points
  - /dogmatix/floor_points (sensor_msgs/PointCloud2) — classified floor points
  - /dogmatix/ceiling_points (sensor_msgs/PointCloud2) — classified ceiling points
  - /dogmatix/wall_markers (visualization_msgs/MarkerArray) — wall line markers
  - /dogmatix/floorplan (std_msgs/String) — JSON floorplan

Services:
  - /dogmatix/generate_floorplan (std_srvs/Trigger) — trigger floorplan generation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Header, ColorRGBA
from geometry_msgs.msg import Point
import std_srvs.srv

from .pointcloud_processor import PointCloudProcessor
from .floorplan_generator import FloorplanGenerator

import logging
import os

logger = logging.getLogger(__name__)


class PCLPipelineNode(Node):
    """ROS2 node for point cloud processing pipeline."""

    def __init__(self):
        super().__init__('dogmatix_pcl_pipeline')

        # Declare parameters
        self.declare_parameter('input_topic', '/pointcloud')
        self.declare_parameter('output_dir', '/tmp/dogmatix')
        self.declare_parameter('voxel_size', 0.01)
        self.declare_parameter('ransac_distance_threshold', 0.02)
        self.declare_parameter('wall_thickness', 0.15)
        self.declare_parameter('merge_distance', 0.3)
        self.declare_parameter('min_wall_length', 0.5)
        self.declare_parameter('process_on_receive', False)  # process each cloud or wait for service call

        # Get parameters
        input_topic = self.get_parameter('input_topic').value
        self.output_dir = self.get_parameter('output_dir').value
        voxel_size = self.get_parameter('voxel_size').value
        ransac_dist = self.get_parameter('ransac_distance_threshold').value
        self.process_on_receive = self.get_parameter('process_on_receive').value

        # Init processors
        self.pcl_processor = PointCloudProcessor({
            'voxel_size': voxel_size,
            'ransac_distance_threshold': ransac_dist,
        })
        self.floorplan_gen = FloorplanGenerator({
            'wall_thickness': self.get_parameter('wall_thickness').value,
            'merge_distance': self.get_parameter('merge_distance').value,
            'min_wall_length': self.get_parameter('min_wall_length').value,
        })

        # Accumulated wall lines (from multiple clouds)
        self.all_wall_lines = []
        self.latest_cloud = None

        # QoS for LIDAR data
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        # Subscribers
        self.cloud_sub = self.create_subscription(
            PointCloud2, input_topic, self._cloud_callback, qos
        )

        # Publishers
        self.wall_pub = self.create_publisher(PointCloud2, '/dogmatix/wall_points', 10)
        self.floor_pub = self.create_publisher(PointCloud2, '/dogmatix/floor_points', 10)
        self.ceiling_pub = self.create_publisher(PointCloud2, '/dogmatix/ceiling_points', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dogmatix/wall_markers', 10)
        self.floorplan_pub = self.create_publisher(String, '/dogmatix/floorplan', 10)

        # Services
        self.process_srv = self.create_service(
            std_srvs.srv.Trigger, '/dogmatix/generate_floorplan', self._generate_floorplan_cb
        )
        self.clear_srv = self.create_service(
            std_srvs.srv.Trigger, '/dogmatix/clear_accumulated', self._clear_accumulated_cb
        )

        self.get_logger().info(f'Dogmatix PCL Pipeline started. Subscribing to {input_topic}')

    def _cloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud."""
        # Convert PointCloud2 to numpy
        points = []
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nan=True):
            points.append([p[0], p[1], p[2]])

        if not points:
            return

        points = np.array(points, dtype=np.float32)
        self.latest_cloud = points
        self.get_logger().info(f'Received cloud: {len(points)} points')

        if self.process_on_receive:
            result = self.pcl_processor.process_pointcloud(points)
            self._publish_results(result, msg.header)
            self.all_wall_lines.extend(result['wall_lines'])

    def _publish_results(self, result: dict, header: Header):
        """Publish classified point clouds and markers."""
        # Publish wall points
        if len(result['wall_points']) > 0:
            self._publish_cloud(result['wall_points'], header, self.wall_pub, 'wall')

        # Publish floor points
        if len(result['floor_points']) > 0:
            self._publish_cloud(result['floor_points'], header, self.floor_pub, 'floor')

        # Publish ceiling points
        if len(result['ceiling_points']) > 0:
            self._publish_cloud(result['ceiling_points'], header, self.ceiling_pub, 'ceiling')

        # Publish wall line markers
        if result['wall_lines']:
            self._publish_wall_markers(result['wall_lines'], header)

    def _publish_cloud(self, points: np.ndarray, header: Header, publisher, label: str):
        """Publish numpy array as PointCloud2."""
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.header.frame_id = 'map'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(cloud_msg.header, fields, points[:, :3].tolist())
        publisher.publish(cloud_msg)

    def _publish_wall_markers(self, wall_lines: list, header: Header):
        """Publish wall line segments as visualization markers."""
        markers = MarkerArray()

        for i, (start, end) in enumerate(wall_lines):
            marker = Marker()
            marker.header = header
            marker.header.frame_id = 'map'
            marker.ns = 'dogmatix_walls'
            marker.id = i
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.05  # line width 5cm
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

            p_start = Point(x=float(start[0]), y=float(start[1]), z=0.0)
            p_end = Point(x=float(end[0]), y=float(end[1]), z=0.0)
            marker.points = [p_start, p_end]

            markers.markers.append(marker)

        self.marker_pub.publish(markers)

    def _generate_floorplan_cb(self, request, response):
        """Service callback to generate floorplan from accumulated data."""
        if self.latest_cloud is None:
            response.success = False
            response.message = 'No point cloud data received yet'
            return response

        # Process latest cloud
        result = self.pcl_processor.process_pointcloud(self.latest_cloud)
        self.all_wall_lines.extend(result['wall_lines'])

        if not self.all_wall_lines:
            response.success = False
            response.message = 'No walls detected'
            return response

        # Generate floorplan
        fp_result = self.floorplan_gen.process_wall_lines(
            self.all_wall_lines, self.output_dir
        )

        # Publish JSON floorplan
        import json
        msg = String()
        with open(fp_result['json_path'], 'r') as f:
            msg.data = f.read()
        self.floorplan_pub.publish(msg)

        # Publish wall markers
        self._publish_wall_markers(result['wall_lines'], Header())

        response.success = True
        response.message = (
            f'Floorplan generated: {len(fp_result["walls"])} walls, '
            f'total length {fp_result["dimensions"][0]["length"]:.1f}m. '
            f'Saved to {fp_result["svg_path"]}'
        )
        self.get_logger().info(response.message)
        return response

    def _clear_accumulated_cb(self, request, response):
        """Clear accumulated wall data."""
        self.all_wall_lines.clear()
        response.success = True
        response.message = 'Accumulated wall data cleared'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PCLPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
