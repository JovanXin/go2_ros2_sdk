# Cleanup pass — findings & decisions (2026-09-09 session)

Audit of what each folder does and removal of duplicates. Everything below is
working-tree state; **nothing was committed/staged** (user handles git).

## Verification (tests run after cleanup — all pass)

- `dogmatix_pcl_pipeline/test_pipeline.py` → EXIT 0; synthetic 6×4×2.8 m room correctly segmented (floor/ceiling/4 walls, wall lengths ≈ 6.02/6.02/3.95/3.93 m), SVG+JSON emitted.
- `colcon build --packages-select lidar_processor_cpp` → clean after aggregator-node deletion.
- `colcon test` → `PointCloudFrameFilter.FiltersOnlyTheProvidedFrame` PASSED (1/1).
- Workspace aggregate (recorded results incl. untouched `frontier_exploration_ros2` suite): 161 tests, 0 errors, 0 failures, 6 skipped (pre-existing).

## Removed this pass (with verification)

| Removed | Why safe | Verification |
|---|---|---|
| `frontier_explorer/` (standalone Python explorer) | Duplicate of C++ `frontier_exploration_ros2/` submodule that `explore.launch.py` actually launches. No launch/manifest/docs reference. Same node name + `/control_exploration` + `/explore_status` as C++ → would collide if run. | `rg` across repo, launch AST parse, manifest deps |
| `frontier_explorer_rviz/` (standalone RViz panel) | Panels the deleted Python node; zero references (rviz configs load it via class name only). | `rg` |
| `lidar_processor/` (Python lidar pkg) | Superseded by `lidar_processor_cpp/` — all 7 launch files use the C++ nodes; not in `go2_robot_sdk/package.xml`. Its `.py` aggregator logic lives in C++ now. | launch package scan, manifest, README |
| `lidar_processor_cpp/src/pointcloud_aggregator_node.{cpp,hpp}` | Never built: not in CMakeLists.txt (confirmed: no binary in `build/`), never launched. Aggregation + filtering already merged into `lidar_to_pointcloud_node`. | CMakeLists read, `build/` ls, `rg` |
| `go2_robot_sdk/package.xml`: `exec_depend frontier_exploration_ros2_rviz` | Submodule's bundled RViz plugin is never loaded by any launch/config here. | launch/config scan |

Build dirs for deleted packages (`build/*`, `install/*`, `log/*`) also removed.

## Edits (in-place)

- `.gitignore`: added `.pytest_cache/`, `.mypy_cache/`, `.ruff_cache/`, `*.py[cod]`,
  `*.egg-info/`, `.eggs/`, `dist/`, `.coverage*`, `coverage.xml`, `htmlcov/`,
  `3d_map.pcd` + `3d_map_*.pcd` (runtime dumps per README §"Save a map"),
  `.pi-smart-router/` (local agent state). Old `go2_robot_sdk/.coverage` /
  `go2_robot_sdk/coverage.xml` lines became redundant → replaced by generic rules.
- `go2_robot_sdk/config/{single,multi}_robot_conf.rviz`: removed
  `frontier_explorer_rviz/FrontierExplorerPanel` panel entries (plugin deleted).
- `README.md`: `lidar_processor/lidar_to_pointcloud` → `lidar_processor_cpp/lidar_to_pointcloud_node` (stale line).
- `docs/dogmatix/architecture.md`: files-map no longer lists deleted python
  package; added "Naming notes (frontier)" section.

## The remaining launch matrix (all intentional, keep)

7 launch files in `go2_robot_sdk/launch/`, all using `lidar_processor_cpp` nodes:

| Launch | Mode | Notes |
|---|---|---|
| `robot.launch.py` | Full system, SLAM | **README-documented entry point.** Multi-robot capable (URDF per conn mode), foxglove, joystick. |
| `navigation.launch.py` | AMCL localization + Nav2 | Requires `map:=`; uses `nav2_params.yaml`. Stateless filter (no map accumulator) by design. |
| `mapping.launch.py` | SLAM-optimized | Flags for map saving; map accumulator included. |
| `explore.launch.py` | frontier exploration | `frontier_exploration_ros2` + Nav2; loads `frontier_params.yaml` (tuned for Go2). |
| `rtabmap_mapping.launch.py` | RTAB-Map 3D SLAM | `scan2d:=false` → uses `/pointcloud/current_filtered`. |
| `webrtc_web.launch.py` | Legacy upstream web/demo stack | Video, TTS, camera streams. |

## OPEN QUESTION (resolved): `robot_cpp.launch.py` — DELETED

History: added upstream in `40fed1e` ("Add C++ launch file…") alongside
`robot.launch.py`, which at the time still launched the **Python** `lidar_processor`.
`robot.launch.py` was *since* updated to the C++ nodes too (uncommitted diff shows
`lidar_processor` → `lidar_processor_cpp` + added `pointcloud_frame_filter_node`).

Final analysis (session close): robot_cpp's two modes are each fully covered —
SLAM mode ≈ `robot.launch.py` (incl. map accumulator); AMCL mode ≈
`navigation.launch.py` (driver + teleop + Nav2, stateless filter by design).
Nothing referenced it. **Deleted at user confirmation.**

Lost-with-file tuning deltas (resurrect into `robot.launch.py` if ever needed):
wider pointcloud_to_laserscan slice (`max_height 2.0`, `min_height -0.2`) and
`(scan→/scan)` remap, vs robot.launch's `max_height 0.1/0.5`.

Also kept at user request despite zero references: `go2_robot_sdk/config/cyclonedds_config.rviz`.

## Verified non-issues (no action)

- `go2_robot_sdk/go2_robot_sdk/`: clean-architecture layers all wired (interfaces
  `IRobotController/IRobotDataPublisher/IRobotDataReceiver` imported by services;
  driver node is the composition root). `main.py` is the real entry point
  (`go2_driver_node` console script → `go2_robot_sdk.main:main`).
- `go2_interfaces/`: 31 msgs, all imports resolve; unused msgs are compatibility
  exports of the merged `unitree_go` interfaces — keep.
- `external_lib/aioice`: needed submodule (patched ICE); hot-loaded in
  `go2_robot_sdk/__init__.py`.
- `coco_detector`, `speech_processor`, `dogmatix_pcl_pipeline`: all referenced
  by launch files / docs; no dupes.
- Line-ending churn (129 files CRLF→LF, incl. URDF + binary `3d_map.ply` re-save)
  is pre-existing user work — untouched. Suggest `*.ply -text` + normalization
  decision for the next commit; not done here.
