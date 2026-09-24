# External LIDAR Sensor Research — Dogmatix 3D Model Generator

## Requirements
- Indoor scanning (rooms, corridors, buildings)
- Range: 10-15m minimum (walls, ceiling)
- Output: high-res 3D point cloud for BIM/floorplan generation
- Mountable on Go2 dog OR handheld
- Budget: TBD (discuss with supervisor)

---

## Option 1: Livox Mid-360 (~$800-$1,145) — BEST VALUE

| Spec | Value |
|------|-------|
| Type | 3D non-repetitive scan |
| Range | 40m |
| FOV | 360° horizontal, 59° vertical |
| Points/sec | 200,000 |
| Weight | ~180g |
| Interface | Ethernet |
| ROS2 support | Yes (livox_ros_driver2) |

**Pros:** Best value 3D LIDAR. 360° coverage. Light enough to mount on Go2. Strong ROS2 community. Used by many quadruped projects.
**Cons:** Non-repetitive scan pattern (needs movement for full coverage). No built-in IMU.
**Go2 fit:** Excellent — light, small, good range, well-supported.

---

## Option 2: SLAMTEC RPLIDAR S2/S3 (~$200-$500) — BUDGET 2D

| Spec | Value |
|------|-------|
| Type | 2D mechanical |
| Range | 10-30m (model dependent) |
| FOV | 360° horizontal |
| Points/sec | 32,000-100,000 |
| Weight | ~200g |
| Interface | USB/UART |
| ROS2 support | Yes (rplidar_ros) |

**Pros:** Cheap. Simple integration. Good for2D floorplans.
**Cons:** 2D only — no ceiling/3D data. Need to sweep or mount at angle for3D.
**Go2 fit:** OK for2D mapping, not enough for full3D model.

---

## Option 3: Intel RealSense L515 (~$500-$700, discontinued)

| Spec | Value |
|------|-------|
| Type | Solid-state LIDAR |
| Range | 0.25-9m |
| FOV | 70°×55° |
| Points/sec | 23,000,000 (depth) |
| Weight | ~165g |
| Interface | USB-C |
| ROS2 support | Yes (realsense2_camera) |

**Pros:** Very high point density at short range. Color overlay. Compact.
**Cons:** Discontinued (hard to find). Max range only 9m — may miss far walls. USB power hungry.
**Go2 fit:** Good if you can find one. Short range is limiting for large rooms.

---

## Option 4: Ouster OS0 (~$3,500+)

| Spec | Value |
|------|-------|
| Type | 3D mechanical |
| Range | 0.3-20m |
| FOV | 360°×72° (ultra-wide vertical) |
| Points/sec | 655,360 |
| Weight | ~450g |
| Interface | Ethernet |
| ROS2 support | Yes (ouster-ros) |

**Pros:** Ultra-wide vertical FOV captures ceiling + floor in one scan. High point density. Excellent for indoor.
**Cons:** Expensive. Heavy for Go2 mounting.
**Go2 fit:** Best indoor spec but cost and weight are barriers.

---

## Option 5: Handheld SLAM Scanners (standalone, not mounted)

### Leica BLK2GO (~$30,000-$40,000)
- Range: 0.5-25m, accuracy ±6-15mm relative
- 420K pts/sec, 775g handheld
- Walk-through capture, built-in SLAM
- Output: registered point cloud, no post-processing needed

### NavVis VLX3 (~$50,000+ rental available)
- Range: up to 50m, accuracy ±5mm local
- 1.28M pts/sec, 8.5kg wearable
- 4×20MP panoramic cameras
- Professional-grade BIM output

**Pros:** No mounting/integration work. Immediate results. Highest accuracy.
**Cons:** Expensive. Can't mount on robot. Manual operation.
**Use case:** Standalone scanning sessions, not autonomous.

---

## Recommendation

**For mounting on Go2:** Livox Mid-360 — best value,360° coverage, light, ROS2-ready.
**For standalone scanning:** Leica BLK2GO if budget allows; else rent NavVis VLX3 per project.
**Budget option:** RPLIDAR S2 for2D floorplans only (upgrade path to Mid-360 later).

## ROS2 Integration

All recommended sensors have ROS2 drivers:
- Livox: `livox_ros_driver2` → publishes PointCloud2
- RPLIDAR: `rplidar_ros` → publishes LaserScan
- Ouster: `ouster-ros` → publishes PointCloud2
- RealSense: `realsense2_camera` → publishes depth + color

PointCloud2 output from any of these plugs directly into the existing go2_ros2_sdk pipeline (slam_toolbox, nav2).
