#ifndef LIDAR_PROCESSOR_CPP__POINTCLOUD_FRAME_FILTER_HPP_
#define LIDAR_PROCESSOR_CPP__POINTCLOUD_FRAME_FILTER_HPP_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace lidar_processor_cpp
{

struct FrameFilterConfig
{
  double min_range;
  double max_range;
  double min_height;
  double max_height;
  double voxel_size;
};

pcl::PointCloud<pcl::PointXYZ> filterPointCloud(
  const pcl::PointCloud<pcl::PointXYZ> & input,
  const FrameFilterConfig & config);

}  // namespace lidar_processor_cpp

#endif  // LIDAR_PROCESSOR_CPP__POINTCLOUD_FRAME_FILTER_HPP_
