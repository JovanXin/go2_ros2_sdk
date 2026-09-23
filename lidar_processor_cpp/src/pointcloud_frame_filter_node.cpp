#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "lidar_processor_cpp/pointcloud_frame_filter.hpp"

namespace lidar_processor_cpp
{

class PointCloudFrameFilterNode : public rclcpp::Node
{
public:
  PointCloudFrameFilterNode()
  : Node("pointcloud_frame_filter")
  {
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
    filter_config_.min_range = this->declare_parameter<double>("min_range", 0.1);
    filter_config_.max_range = this->declare_parameter<double>("max_range", 20.0);
    filter_config_.min_height = this->declare_parameter<double>("min_height", -2.0);
    filter_config_.max_height = this->declare_parameter<double>("max_height", 3.0);
    filter_config_.voxel_size = this->declare_parameter<double>("voxel_size", 0.005);
    validateConfiguration();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_out", rclcpp::SensorDataQoS());
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_in",
      rclcpp::SensorDataQoS(),
      std::bind(&PointCloudFrameFilterNode::pointcloudCallback, this, std::placeholders::_1));
  }

private:
  void validateConfiguration() const
  {
    if (target_frame_.empty()) {
      throw std::invalid_argument("target_frame cannot be empty");
    }
    if (filter_config_.min_range < 0.0 ||
      filter_config_.max_range <= filter_config_.min_range)
    {
      throw std::invalid_argument("max_range must be greater than min_range >= 0");
    }
    if (filter_config_.max_height <= filter_config_.min_height) {
      throw std::invalid_argument("max_height must be greater than min_height");
    }
    if (filter_config_.voxel_size < 0.0) {
      throw std::invalid_argument("voxel_size cannot be negative");
    }
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN(this->get_logger(), "Dropping point cloud with an empty frame_id");
      return;
    }

    sensor_msgs::msg::PointCloud2 transformed = *msg;
    try {
      if (msg->header.frame_id != target_frame_) {
        const auto transform = tf_buffer_->lookupTransform(
          target_frame_,
          msg->header.frame_id,
          rclcpp::Time(msg->header.stamp),
          rclcpp::Duration::from_seconds(0.1));
        tf2::doTransform(*msg, transformed, transform);
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Dropping point cloud because TF is unavailable: %s", error.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(transformed, input);
    const auto output_cloud = filterPointCloud(input, filter_config_);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(output_cloud, output);
    output.header.stamp = msg->header.stamp;
    output.header.frame_id = target_frame_;
    publisher_->publish(output);
  }

  std::string target_frame_;
  FrameFilterConfig filter_config_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace lidar_processor_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_processor_cpp::PointCloudFrameFilterNode>());
  rclcpp::shutdown();
  return 0;
}
