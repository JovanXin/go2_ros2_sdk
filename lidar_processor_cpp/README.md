# LiDAR Processor C++

LiDAR processing nodes for the Go2 robot. Navigation consumes a filtered version of the latest sensor message, while optional map accumulation is kept on a separate path.

## Features

- Stateless filtering of one `PointCloud2` message at a time
- TF transformation into a configured target frame before filtering
- Range, height, finite-value, and voxel-grid filters
- Bounded, thread-safe point storage for optional PLY map saving
- Sensor-data QoS on live point-cloud subscriptions and publishers

## Nodes

### 1. lidar_to_pointcloud_node

Accumulates finite points for optional periodic PLY map saving. This node does not publish its historical map into the navigation pipeline. When `map_save` is `false`, incoming clouds are ignored.

**Topics:**
- **Subscribed:**
  - `/robot0/point_cloud2` (single robot mode)
  - `/robot{i}/point_cloud2` (multi-robot mode)

**Parameters:**
- `robot_ip_lst` (string_array): List of robot IP addresses
- `map_name` (string): Name of the map file to save (default: "3d_map")
- `map_save` (string): Whether to save map periodically (default: "true")
- `save_interval` (double): Map saving interval in seconds (default: 10.0)
- `max_points` (int): Maximum points to keep in memory (default: 1000000)
- `voxel_size` (double): Voxel size for downsampling when saving (default: 0.005)

### 2. pointcloud_frame_filter_node

Transforms and filters only the cloud received in the current callback. It does not retain points between messages.

**Topics:**
- `cloud_in` (subscribed): input cloud, intended to be remapped to the live LiDAR topic
- `cloud_out` (published): filtered current-frame cloud

**Parameters:**
- `target_frame` (string): frame used for filtering and output (default: `base_link`)
- `max_range` (double): Maximum range from robot center in meters (default: 20.0)
- `min_range` (double): Minimum range from robot center in meters (default: 0.1)
- `min_height` (double): Minimum z-coordinate in the target frame (default: -2.0)
- `max_height` (double): Maximum z-coordinate in the target frame (default: 3.0)
- `voxel_size` (double): PCL voxel-grid leaf size in meters (default: 0.005)

## Building

```bash
# From your ROS2 workspace
colcon build --packages-select lidar_processor_cpp
```

## Usage

```bash
ros2 run lidar_processor_cpp pointcloud_frame_filter_node --ros-args \
  -r cloud_in:=/point_cloud2 \
  -r cloud_out:=/pointcloud/current_filtered \
  -p target_frame:=base_link \
  -p max_range:=20.0
```

## Dependencies

- **ROS2 Dependencies:**
  - `rclcpp`
  - `sensor_msgs`
  - `geometry_msgs`
  - `std_msgs`
  - `pcl_ros`
  - `pcl_conversions`
  - `tf2`
  - `tf2_ros`
  - `tf2_sensor_msgs`

- **System Dependencies:**
  - `libpcl-dev`

## License

BSD-3-Clause