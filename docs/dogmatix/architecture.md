# Dogmatix — Go2 ROS2 Architecture

Mermaid diagrams (render on GitHub, VS Code, or https://mermaid.live).

## 1. Robot data flow

```mermaid
flowchart LR
    HW["Unitree Go2<br/>WebRTC Wi-Fi / CycloneDDS Ethernet"]
    DRV["go2_driver_node<br/>(go2_robot_sdk, Python)"]
    CAM["RViz / vision nodes"]
    SLAMN["SLAM layer<br/>(section 2: slam_toolbox or RTAB-Map)"]

    HW -->|"joints / IMU / state"| DRV
    DRV -->|"/point_cloud2"| L2P
    DRV -->|"/camera/image_raw + camera_info"| CAM
    DRV -->|"/odom + TF odom→base_link"| SLAMN

    L2P["lidar_to_pointcloud_node<br/>(lidar_processor_cpp)"] -->|"/point_cloud2 cleaned"| FILT
    FILT["pointcloud_frame_filter_node<br/>crop range 0.3–20 m, height -1–3 m,<br/>transform → base_link"] -->|"/pointcloud/current_filtered"| SLAMN
    FILT -->|"/pointcloud/current_filtered"| P2L
    P2L["pointcloud_to_laserscan_node<br/>collapse 3D cloud → 2D slice"] -->|"/scan"| SLAMN
```

## 2. Two SLAM choices (run ONE launch at a time)

```mermaid
flowchart LR
    subgraph OLD["mapping.launch.py — existing (2D)"]
        SCAN1["/scan"]
        ST["slam_toolbox<br/>occupancy grid SLAM"]
        MAP1["/map — 2D grid"]
        SCAN1 --> ST --> MAP1
    end

    subgraph NEW["rtabmap_mapping.launch.py — new (3D)"]
        IN["/scan (scan2d:=true, default)<br/>OR /pointcloud/current_filtered (scan2d:=false)"]
        RT["rtabmap node<br/>(rtabmap_slam)<br/>graph SLAM + loop closure<br/>params: rtabmap_params.yaml"]
        MAP2["/map — 2D grid (nav2-compatible)"]
        CLOUD["/rtabmap/mapData + cloud map (3D)"]
        VIZ["rtabmap_viz<br/>3D cloud + graph view"]
        IN --> RT --> MAP2
        RT --> CLOUD
        CLOUD --> VIZ
    end
```

## 3. Dogmatix floorplan pipeline

```mermaid
flowchart LR
    subgraph SLAMSRC["SLAM layer (one at a time)"]
        SL1["slam_toolbox /map<br/>(2D only)"]
        SL2["RTAB-Map /rtabmap/cloud_map<br/>(3D, walls/ceiling)"]
    end

    SL1 -.->|"nav2 + frontier_explorer<br/>(explore.launch.py)"| NAV
    SL2 -->|"input_topic:=/rtabmap/cloud_map"| PIPE

    subgraph PIPE["dogmatix_pcl_pipeline<br/>(dogmatix_pcl_pipeline package)"]
        PROC["pointcloud_processor<br/>open3d RANSAC plane segmentation<br/>floor / ceiling / wall classification"]
        GEN["floorplan_generator<br/>merge collinear walls<br/>dimensions"]
        PROC --> GEN
    end

    PIPE -->|"/dogmatix/wall_points etc."| RV2["RViz"]
    PIPE -->|"/dogmatix/floorplan (JSON)"| OUT["/tmp/dogmatix/<br/>floorplan.svg + floorplan.json"]
    NAV["nav2 navigation"] -->|"robot drives room"| SL2

    style SL2 stroke:#d32f2f,stroke-width:2px
    style PIPE stroke:#2e7d32,stroke-width:2px
```

## 4. Full scan session (goal state)

```mermaid
sequenceDiagram
    participant U as User
    participant R as Go2 robot
    participant RT as RTAB-Map
    participant DP as dogmatix pipeline

    U->>R: ros2 launch go2_robot_sdk rtabmap_mapping.launch.py
    R->>RT: point cloud frames (/scan or cloud)
    U->>R: drive dog through rooms (joystick / explore)
    loop covering area
        RT->>RT: SLAM + loop closure
        RT-->>U: /map + 3D cloud grow in rtabmap_viz
    end
    U->>DP: ros2 launch dogmatix_pcl_pipeline pipeline.launch.py input_topic:=/rtabmap/cloud_map
    RT->>DP: registered 3D cloud
    U->>DP: service call /dogmatix/generate_floorplan
    DP-->>U: floorplan.svg + floorplan.json (wall lengths)
```

## Files map

| Layer | Files |
|---|---|
| Driver | `go2_robot_sdk/go2_robot_sdk/presentation/go2_driver_node.py`, `infrastructure/ros2/ros2_publisher.py` |
| Sensor chain (C++) | `lidar_processor_cpp/src/lidar_to_pointcloud_node.cpp`, `pointcloud_frame_filter.cpp` |
| SLAM 2D | `go2_robot_sdk/config/mapper_params_online_async.yaml` (slam_toolbox) |
| SLAM 3D (new) | `go2_robot_sdk/config/rtabmap_params.yaml`, `go2_robot_sdk/launch/rtabmap_mapping.launch.py` |
| Exploration | `frontier_exploration_ros2/` (submodule, C++) — the only active explorer; launched by `go2_robot_sdk/launch/explore.launch.py` |
| Pipeline (new) | `dogmatix_pcl_pipeline/dogmatix_pcl_pipeline/{pointcloud_processor,floorplan_generator,pcl_pipeline_node}.py` |
| Docs | `docs/dogmatix/{project-plan,sensor-research,slam-comparison,rtabmap-integration}.md` |

## Naming notes (frontier)

- `frontier_exploration_ros2/` — upstream C++ explorer, vendored as a git **submodule** (not a fork to edit).
- `frontier_exploration_ros2/plugin/frontier_exploration_ros2_rviz` — RViz control panel that ships *inside the submodule*. Kept because removing it would dirty the submodule; it is not loaded by any config here.
- Standalone Python `frontier_explorer/` and `frontier_explorer_rviz/` packages were earlier duplicates of the submodule stack and have been **removed** — do not re-add.
