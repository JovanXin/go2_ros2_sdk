# SLAM Library Comparison — Indoor Floorplan Generation

## Context
Goal: generate accurate floorplans and 3D models from LIDAR data in indoor environments. Using Go2 robot dog with external LIDAR sensor.

---

## Candidates

### 1. slam_toolbox (already in go2_ros2_sdk)
- **Type:** 2D SLAM (laser-based)
- **Output:** 2D occupancy grid map
- **Indoor performance:** Good for floorplans, handles large spaces
- **Pros:** Already integrated. Mature. Serializes maps for later use.
- **Cons:** 2D only — no ceiling/height data. Won't give3D model.
- **Verdict:** Keep for2D floorplan baseline. Supplement with3D solution.

### 2. RTAB-Map (Real-Time Appearance-Based Mapping)
- **Type:** 3D SLAM (visual + LIDAR)
- **Output:** 3D point cloud +2D map + loop closure
- **Indoor performance:** Excellent — handles loop closure well in buildings
- **Pros:** RGB-D and LIDAR fusion. Built-in3D reconstruction. ROS2 native. Handles multi-session mapping.
- **Cons:** CPU-heavy. Needs camera + LIDAR (Go2 has both).
- **Verdict:** Best all-rounder for indoor3D mapping. Strong candidate.

### 3. FAST-LIO2
- **Type:** 3D LIDAR-inertial SLAM
- **Output:** 3D point cloud map
- **Indoor performance:** Very good — fast, handles aggressive motion
- **Pros:** Extremely fast. Low drift. Works with Livox, Velodyne, Ouster. Tightly couples LIDAR + IMU.
- **Cons:** No built-in loop closure (needs external module). No visual data.
- **Verdict:** Best raw LIDAR SLAM accuracy. Pair with loop closure module.

### 4. LOAM / A-LOAM / F-LOAM
- **Type:** 3D LIDAR odometry
- **Output:** 3D point cloud trajectory
- **Indoor performance:** Good for odometry, drift accumulates
- **Pros:** Well-understood. Lightweight. Good baseline.
- **Cons:** Drift accumulates without loop closure. Not a full SLAM system alone.
- **Verdict:** Use as odometry source, not standalone SLAM.

### 5. Google Cartographer (3D mode)
- **Type:** 2D/3D SLAM
- **Output:** 2D/3D occupancy grid
- **Indoor performance:** Good — designed for indoor
- **Pros:** Google-backed. Handles2D and3D. Submaps reduce drift.
- **Cons:** Maintenance mode (no active development). Complex setup. ROS1 legacy (ROS2 port exists but less maintained).
- **Verdict:** Viable but aging. RTAB-Map is more actively maintained.

### 6. ORB-SLAM3 (visual)
- **Type:** Visual SLAM (mono/stereo/RGB-D)
- **Output:** Sparse3D map + camera trajectory
- **Indoor performance:** Good with rich visual features
- **Pros:** Works with camera only (no LIDAR needed). Very accurate trajectory.
- **Cons:** Sparse map (not dense point cloud). Needs good lighting. No native ROS2.
- **Verdict:** Good for visual odometry complement, not primary3D mapping.

---

## Recommendation

**Primary:** RTAB-Map
- Handles both LIDAR and camera data from Go2
- Built-in loop closure (critical for multi-room indoor)
- ROS2 native, active development
- Outputs both2D map and3D point cloud
- Already has ROS2 package: `rtabmap_ros`

**Alternative:** FAST-LIO2 (if using Livox Mid-360)
- Better raw accuracy with3D LIDAR
- Pair with SC-PGO or other loop closure
- Better for aggressive robot motion

**Keep:** slam_toolbox for quick2D floorplans alongside3D pipeline

## Pipeline Architecture

```
External LIDAR (Livox Mid-360)
    ↓ PointCloud2
RTAB-Map (SLAM + loop closure)
    ↓ Registered 3D point cloud
Point Cloud Processing (wall/floor detection)
    ↓ Classified geometry
Floorplan Generator
    ↓ 2D floor plan + dimensions
BIM Export (IFC/DXF)
```
