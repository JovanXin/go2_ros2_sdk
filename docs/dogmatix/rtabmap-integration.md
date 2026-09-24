# RTAB-Map Integration — Go2 3D Mapping

RTAB-Map gives Go2 a graph-SLAM map with loop closure — beyond slam_toolbox's 2D grid.
Built by reusing upstream rtabmap_ros files (not re-implemented):
- Param conventions from `introlab/rtabmap_ros` humble-devel branch:
  `rtabmap_demos/launch/turtlebot3/turtlebot3_scan.launch.py` (2D) and
  `rtabmap_examples/launch/lidar3d.launch.py` (3D cloud)
- RTAB-Map internal params are STRINGS (e.g. `Reg/Strategy: '1'`) — native types break silently

## Status: files in place, install + robot test pending

Shipped:
- `go2_robot_sdk/config/rtabmap_params.yaml` — string-typed RTAB-Map params
- `go2_robot_sdk/launch/rtabmap_mapping.launch.py` — driver + LIDAR chain + RTAB-Map + RViz/rtabmap_viz + joystick

## Modes

| Mode | Input | When |
|------|-------|------|
| `scan2d:=true` (default) | `/scan` (2D, current-frame cloud collapsed) | today, stock robot |
| `scan2d:=false` | `/pointcloud/current_filtered` (3D cloud) | external LIDAR (Livox) |

```
Go2 driver → lidar_to_pointcloud → /point_cloud2
  → pointcloud_frame_filter → /pointcloud/current_filtered
  → pointcloud_to_laserscan → /scan
  → RTAB-Map (subscribe_scan or subscribe_scan_cloud)
      ├── /map                  2D occupancy grid (nav2/frontier compatible)
      ├── /rtabmap/mapData      graph data (rtabmap_viz / RViz)
      └── ~cloud_map            3D accumulated cloud → dogmatix_pcl_pipeline
```

## Install (needs sudo — run on robot PC)

```bash
sudo apt update
sudo apt install ros-humble-rtabmap-ros   # metapackage: rtabmap_slam, rtabmap_viz, rtabmap_odom, ...
```

Rebuild workspace to install the new launch/config:

```bash
cd <ros2_ws>
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_robot_sdk
source install/setup.bash
```

## Run

```bash
export ROBOT_IP=<go2-ip>
export ROBOT_TOKEN=<token-or-empty>
ros2 launch go2_robot_sdk rtabmap_mapping.launch.py          # 2D scan mode
ros2 launch go2_robot_sdk rtabmap_mapping.launch.py scan2d:=false rviz:=false   # 3D cloud mode, headless
```

Drive the dog around. Watch:
- `rtabmap_viz` (auto-launched): 3D cloud + loop closure graph
- RViz: `/map` fills in
- Confirm topics: `ros2 topic list | grep rtabmap`

Fresh mapping session deletes the old RTAB-Map DB (`-d` arg, upstream convention).

## Feed map into dogmatix pipeline

```bash
ros2 launch dogmatix_pcl_pipeline pipeline.launch.py \
    input_topic:=/rtabmap/cloud_map process_on_receive:=true
ros2 service call /dogmatix/generate_floorplan std_srvs/srv/Trigger
```

Note: current stock LIDAR faces down (~1 m reach). Its "3D" is floor clutter —
real walls/ceiling need the external sensor. See sensor-research.md.

## Config knobs (tune on real data)

| Param | Default | When to change |
|-------|---------|----------------|
| `Grid/RangeMin` | 0.2 | ignore robot itself |
| `Reg/Force3DoF` | true | stay 2D; set false for Livox 3D |
| `Optimizer/GravitySigma` | 0 | no IMU constraints; raise if IMU fused |
| `subscribe_scan_cloud` + `scan_cloud_is_2d` | — | set by `scan2d` arg |

## Verification checklist (robot session)

1. `/map` publishes within 30 s of driving
2. Loop closure: drive a loop, revisit start — map aligns, no double walls
3. `rtabmap_viz` shows graph edges closing on revisit
4. External LIDAR path: walls above 1 m appear in cloud (stock LIDAR cannot)
