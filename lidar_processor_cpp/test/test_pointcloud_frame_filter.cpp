#include <limits>

#include "gtest/gtest.h"
#include "lidar_processor_cpp/pointcloud_frame_filter.hpp"

namespace
{

using lidar_processor_cpp::FrameFilterConfig;
using lidar_processor_cpp::filterPointCloud;

TEST(PointCloudFrameFilter, FiltersOnlyTheProvidedFrame)
{
  const FrameFilterConfig config{0.3, 20.0, -1.0, 3.0, 0.0};

  pcl::PointCloud<pcl::PointXYZ> first;
  first.emplace_back(1.0F, 0.0F, 0.0F);
  first.emplace_back(0.1F, 0.0F, 0.0F);
  first.emplace_back(21.0F, 0.0F, 0.0F);
  first.emplace_back(1.0F, 0.0F, -2.0F);
  first.emplace_back(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F);

  const auto first_output = filterPointCloud(first, config);
  ASSERT_EQ(first_output.size(), 1U);
  EXPECT_FLOAT_EQ(first_output.front().x, 1.0F);

  pcl::PointCloud<pcl::PointXYZ> second;
  second.emplace_back(2.0F, 0.0F, 0.0F);

  const auto second_output = filterPointCloud(second, config);
  ASSERT_EQ(second_output.size(), 1U);
  EXPECT_FLOAT_EQ(second_output.front().x, 2.0F);
}

}  // namespace
