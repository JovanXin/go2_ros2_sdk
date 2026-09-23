#include "lidar_processor_cpp/pointcloud_frame_filter.hpp"

#include <cmath>
#include <memory>

#include "pcl/common/point_tests.h"
#include "pcl/filters/voxel_grid.h"

namespace lidar_processor_cpp
{

pcl::PointCloud<pcl::PointXYZ> filterPointCloud(
  const pcl::PointCloud<pcl::PointXYZ> & input,
  const FrameFilterConfig & config)
{
  auto range_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  range_filtered->points.reserve(input.size());

  for (const auto & point : input) {
    if (!pcl::isFinite(point)) {
      continue;
    }

    const double range = std::hypot(point.x, point.y);
    if (range < config.min_range || range > config.max_range) {
      continue;
    }

    if (point.z < config.min_height || point.z > config.max_height) {
      continue;
    }

    range_filtered->push_back(point);
  }

  if (config.voxel_size == 0.0 || range_filtered->empty()) {
    return *range_filtered;
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(range_filtered);
  const auto leaf_size = static_cast<float>(config.voxel_size);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

  pcl::PointCloud<pcl::PointXYZ> output;
  voxel_filter.filter(output);
  return output;
}

}  // namespace lidar_processor_cpp
