# Dogmatix 3D Model Generator — Project Plan

## Current State

The Unitree Go2 robot dog with go2_ros2_sdk already has:
- ✅ LIDAR point cloud streaming (7 Hz, 360°)
- ✅ Camera stream with calibration
- ✅ SLAM mapping (slam_toolbox — 2D occupancy grid)
- ✅ Navigation (nav2 — autonomous path planning)
- ✅ Frontier exploration (autonomous room scanning)
- ✅ Object detection (COCO — YOLO-based)
- ✅ WebRTC and CycloneDDS connectivity

**Limitation:** Built-in LIDAR faces down, ~1m range, low resolution. Cannot detect walls or ceiling. Not sufficient for 3D model generation.

## What We Need

A high-resolution external LIDAR sensor that can:
- See walls at 10-15m range
- Capture ceiling and floor geometry
- Generate dense 3D point clouds for BIM

## Recommended Sensor: Livox Mid-360

| Spec | Value |
|------|-------|
| Price | ~$800-$1,145 |
| Type | 3D non-repetitive scan |
| Range | 40m |
| FOV | 360° horizontal, 59° vertical |
| Points/sec | 200,000 |
| Weight | 180g (mountable on Go2) |
| ROS2 driver | livox_ros_driver2 |

**Why this one:** Best value 3D LIDAR. 360° coverage captures all walls in one scan. Light enough for Go2 mounting. Strong ROS2 community support. Used by many quadruped robot projects.

**Alternatives:**
- Budget: SLAMTEC RPLIDAR S2 (~$200) — 2D only, no ceiling
- Premium: Ouster OS0 (~$3,500) — wider vertical FOV, heavier
- Standalone: Leica BLK2GO (~$35,000) — handheld, highest accuracy, no robot integration

## Proposed Architecture

```
Go2 Robot Dog
    ├── Built-in LIDAR → slam_toolbox → 2D floorplan (existing)
    └── External Livox Mid-360 → PointCloud2
            ↓
        RTAB-Map (3D SLAM + loop closure)
            ↓
        Registered 3D point cloud
            ↓
        Dogmatix PCL Pipeline (new — built)
            ├── Wall detection (RANSAC plane segmentation)
            ├── Floor/ceiling classification
            └── Floorplan generator → SVG/JSON export
```

## Deliverables

1. **2D floorplan** — accurate room outlines with dimensions (SVG, PDF)
2. **3D point cloud** — colorized, registered, BIM-ready
3. **Wall schedule** — wall lengths, positions, angles (JSON/CSV)
4. **BIM export** — IFC or DXF for Revit/ArchiCAD import

## Timeline Estimate

| Phase | Duration | Description |
|-------|----------|-------------|
| 1. Sensor procurement | 1-2 weeks | Order Livox Mid-360, mounting hardware |
| 2. Integration | 1 week | Mount on Go2, ROS2 driver setup, calibrate |
| 3. SLAM setup | 1 week | RTAB-Map configuration, indoor tuning |
| 4. Pipeline testing | 1-2 weeks | Test wall detection, floorplan generation |
| 5. BIM export | 1 week | IFC/DXF export, validation |
| Total | 5-7 weeks | From sensor arrival to deliverable |

## Cost Estimate

| Item | Cost |
|------|------|
| Livox Mid-360 sensor | $800-$1,145 |
| Mounting hardware | $50-$100 |
| Total | $850-$1,245 |

## What's Already Built (No Sensor Needed)

I've built the point cloud processing pipeline (`dogmatix_pcl_pipeline`) that handles:
- RANSAC plane segmentation (wall/floor/ceiling detection)
- Wall line extraction and merging
- Floorplan generation (SVG + JSON output)
- ROS2 integration (subscribes to PointCloud2, publishes classified points and markers)

This can be tested with synthetic data or any existing LIDAR dataset while waiting for the sensor.

## Next Steps

1. **Decision:** Approve sensor purchase (Livox Mid-360)
2. **Meanwhile:** Test pipeline with synthetic data, tune RANSAC parameters
3. **On arrival:** Mount sensor, integrate with go2_ros2_sdk, run first scan
4. **Validation:** Compare generated floorplan against manual measurements
