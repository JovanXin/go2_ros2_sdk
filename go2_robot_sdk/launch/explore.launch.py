# Autonomous frontier exploration launch file
# Usage: ros2 launch go2_robot_sdk explore.launch.py
#
# Starts: robot driver + LiDAR pipeline + SLAM Toolbox + Nav2 (no AMCL)
#         + frontier_exploration_ros2 (mertgulerx)
#
# The robot will autonomously explore unknown space by detecting frontiers
# (boundaries between free and unknown cells) on the occupancy grid and
# navigating to them via Nav2.
#
# Uses the frontier_exploration_ros2 package (cloned as a submodule).
# Click "Start" in the RViz Frontier Exploration panel to begin.

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)


def generate_launch_description():
    # ── Environment ──────────────────────────────────────────────────────────
    robot_token = os.getenv("ROBOT_TOKEN", "")
    robot_ip = os.getenv("ROBOT_IP", "")
    robot_ip_list = robot_ip.replace(" ", "").split(",") if robot_ip else []
    conn_type = os.getenv("CONN_TYPE", "webrtc")

    conn_mode = (
        "single"
        if len(robot_ip_list) == 1 and conn_type != "cyclonedds"
        else "multi"
    )

    # ── Package paths ────────────────────────────────────────────────────────
    package_dir = get_package_share_directory("go2_robot_sdk")
    urdf_file = "go2.urdf" if conn_mode == "single" else "multi_go2.urdf"
    rviz_config = "single_robot_conf.rviz" if conn_mode == "single" else "multi_robot_conf.rviz"

    frontier_pkg = get_package_share_directory("frontier_exploration_ros2")

    config_paths = {
        "joystick": os.path.join(package_dir, "config", "joystick.yaml"),
        "twist_mux": os.path.join(package_dir, "config", "twist_mux.yaml"),
        "nav2": os.path.join(package_dir, "config", "nav2_params.yaml"),
        "slam": os.path.join(package_dir, "config", "mapper_params_online_async.yaml"),
        "rviz": os.path.join(package_dir, "config", rviz_config),
        "urdf": os.path.join(package_dir, "urdf", urdf_file),
        "frontier": os.path.join(package_dir, "config", "frontier_params.yaml"),
    }

    print(f"🤖 Go2 Autonomous Exploration Mode:")
    print(f"   Robot IPs: {robot_ip_list}")
    print(f"   Connection: {conn_type} ({conn_mode})")

    # ── Launch arguments ─────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    with_rviz = LaunchConfiguration("rviz", default="true")
    with_foxglove = LaunchConfiguration("foxglove", default="true")
    with_joystick = LaunchConfiguration("joystick", default="true")

    launch_args = [
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz2"),
        DeclareLaunchArgument("foxglove", default_value="true", description="Launch Foxglove Bridge"),
        DeclareLaunchArgument("joystick", default_value="true", description="Launch joystick control"),
        DeclareLaunchArgument("frontier_params", default_value=config_paths["frontier"],
                              description="Path to frontier_exploration_ros2 params file"),
        DeclareLaunchArgument("autostart", default_value="false",
                              description="Start exploring immediately instead of waiting for the RViz Start button"),
    ]
    frontier_params = LaunchConfiguration("frontier_params")
    autostart = LaunchConfiguration("autostart")

    # ── URDF ─────────────────────────────────────────────────────────────────
    with open(config_paths["urdf"], "r") as f:
        robot_desc = f.read()

    # ── Core nodes (robot driver + LiDAR pipeline) ───────────────────────────
    core_nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="go2_robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
        ),
        Node(
            package="go2_robot_sdk",
            executable="go2_driver_node",
            name="go2_driver_node",
            output="screen",
            parameters=[{"robot_ip": robot_ip, "token": robot_token, "conn_type": conn_type}],
        ),
        Node(
            package="lidar_processor_cpp",
            executable="pointcloud_frame_filter_node",
            name="pointcloud_frame_filter",
            remappings=[
                ("cloud_in", "/point_cloud2"),
                ("cloud_out", "/pointcloud/current_filtered"),
            ],
            parameters=[
                {
                    "target_frame": "base_link",
                    "max_range": 20.0,
                    "min_range": 0.3,
                    "min_height": -1.0,
                    "max_height": 3.0,
                    "voxel_size": 0.005,
                }
            ],
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="go2_pointcloud_to_laserscan",
            remappings=[
                ("cloud_in", "/pointcloud/current_filtered"),
                ("scan", "/scan"),
            ],
            parameters=[
                {
                    "target_frame": "base_link",
                    "max_height": 3.0,
                    "min_height": -1.0,
                    "angle_min": -3.14159,
                    "angle_max": 3.14159,
                    "angle_increment": 0.00872665,
                    "scan_time": 0.1,
                    "range_min": 0.3,
                    "range_max": 20.0,
                    "use_inf": True,
                    "concurrency_level": 1,
                }
            ],
            output="screen",
        ),
    ]

    # ── Teleop nodes ─────────────────────────────────────────────────────────
    teleop_nodes = [
        Node(
            package="joy",
            executable="joy_node",
            condition=IfCondition(with_joystick),
            parameters=[config_paths["joystick"]],
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="go2_teleop_node",
            condition=IfCondition(with_joystick),
            parameters=[config_paths["twist_mux"]],
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            output="screen",
            condition=IfCondition(with_joystick),
            parameters=[{"use_sim_time": use_sim_time}, config_paths["twist_mux"]],
        ),
    ]

    # ── Visualization ────────────────────────────────────────────────────────
    viz_nodes = [
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(with_rviz),
            name="go2_rviz2",
            output="screen",
            arguments=["-d", config_paths["rviz"]],
            parameters=[{"use_sim_time": False}],
        ),
    ]

    # ── Frontier Explorer node (frontier_exploration_ros2) ───────────────────
    explorer_node = Node(
        package="frontier_exploration_ros2",
        executable="frontier_explorer",
        name="frontier_explorer",
        output="screen",
        parameters=[
            frontier_params,
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            },
        ],
    )

    # ── Included launches ────────────────────────────────────────────────────
    foxglove_launch = os.path.join(
        get_package_share_directory("foxglove_bridge"),
        "launch",
        "foxglove_bridge_launch.xml",
    )

    include_launches = [
        # Foxglove Bridge
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(foxglove_launch),
            condition=IfCondition(with_foxglove),
        ),
        # SLAM Toolbox — provides /map and the map→odom transform
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("slam_toolbox"),
                        "launch",
                        "online_async_launch.py",
                    )
                ]
            ),
            launch_arguments={
                "slam_params_file": config_paths["slam"],
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        # Nav2 navigation stack (controller, planner, costmaps, bt_navigator)
        # WITHOUT AMCL — SLAM Toolbox provides localisation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("nav2_bringup"),
                        "launch",
                        "navigation_launch.py",
                    )
                ]
            ),
            launch_arguments={
                "params_file": config_paths["nav2"],
                "use_sim_time": use_sim_time,
            }.items(),
        ),
    ]

    return LaunchDescription(
        launch_args
        + core_nodes
        + teleop_nodes
        + viz_nodes
        + include_launches
        + [explorer_node]
    )
