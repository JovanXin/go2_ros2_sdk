# RTAB-Map 3D mapping launch for Go2
# Usage: ros2 launch go2_robot_sdk rtabmap_mapping.launch.py
#
# Mirrors upstream rtabmap_ros demo structure (turtlebot3_scan / lidar3d):
#   https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/turtlebot3/turtlebot3_scan.launch.py
#
# Modes:
#   scan2d=true  -> consume /scan           (2D laser, robot odom)  [like slam_toolbox]
#   scan2d=false -> consume /pointcloud/current_filtered (3D cloud)
#
# Outputs: /map (2D grid for nav2), /rtabmap/mapData, cloud map for dogmatix_pcl_pipeline.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization').perform(context)
    localization = localization == 'True' or localization == 'true'
    scan2d = LaunchConfiguration('scan2d').perform(context)
    scan2d = scan2d == 'True' or scan2d == 'true'

    with_rviz = LaunchConfiguration('rviz').perform(context)
    with_rviz = with_rviz == 'True' or with_rviz == 'true'
    with_rtabmap_viz = LaunchConfiguration('rtabmap_viz').perform(context)
    with_rtabmap_viz = with_rtabmap_viz == 'True' or with_rtabmap_viz == 'true'

    # Robot env config
    robot_token = os.getenv('ROBOT_TOKEN', '')
    robot_ip = os.getenv('ROBOT_IP', '')
    robot_ip_list = robot_ip.replace(" ", "").split(",") if robot_ip else []
    conn_type = os.getenv('CONN_TYPE', 'webrtc')
    conn_mode = "single" if len(robot_ip_list) == 1 and conn_type != "cyclonedds" else "multi"
    map_name = os.getenv('MAP_NAME', 'rtabmap_map')

    package_dir = get_package_share_directory('go2_robot_sdk')
    urdf_file = 'go2.urdf' if conn_mode == 'single' else 'multi_go2.urdf'
    rviz_config = 'single_robot_conf.rviz' if conn_mode == 'single' else 'multi_robot_conf.rviz'

    config_paths = {
        'joystick': os.path.join(package_dir, 'config', 'joystick.yaml'),
        'twist_mux': os.path.join(package_dir, 'config', 'twist_mux.yaml'),
        'rtabmap': os.path.join(package_dir, 'config', 'rtabmap_params.yaml'),
        'rviz': os.path.join(package_dir, 'config', rviz_config),
        'urdf': os.path.join(package_dir, 'urdf', urdf_file),
    }

    print(f"🗺️  Go2 RTAB-Map Mapping ({'2D scan' if scan2d else '3D cloud'}):")
    print(f"   Robot IPs: {robot_ip_list}  Connection: {conn_type} ({conn_mode})")
    print(f"   Map name: {map_name}")

    with open(config_paths['urdf'], 'r') as file:
        robot_desc = file.read()

    # RTAB-Map parameters + per-mode subscription flags
    rtabmap_params = [config_paths['rtabmap'], {
        'use_sim_time': use_sim_time,
        'subscribe_scan': scan2d,
        'subscribe_scan_cloud': not scan2d,
        'scan_cloud_is_2d': scan2d,
        'frame_id': 'base_link',
    }]

    # RTAB-Map subscribes odom on 'odom' (driver publishes /odom + TF odom->base_link)
    remappings = [
        ('scan', '/scan' if scan2d else 'scan_not_used'),
        ('scan_cloud', '/pointcloud/current_filtered' if not scan2d else 'scan_cloud_not_used'),
    ]

    rtabmap_args = []
    if localization:
        rtabmap_args = []  # keep database (localize against it)
    else:
        rtabmap_args = ['-d']  # delete previous database, fresh map

    # Robot + sensor chain
    core_nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='go2_robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        ),
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            name='go2_driver_node',
            output='screen',
            parameters=[{'robot_ip': robot_ip, 'token': robot_token, 'conn_type': conn_type}],
        ),
        Node(
            package='lidar_processor_cpp',
            executable='lidar_to_pointcloud_node',
            name='lidar_to_pointcloud',
            remappings=[('robot0/point_cloud2', 'point_cloud2')] if conn_mode == 'single' else [],
            parameters=[{'robot_ip_lst': robot_ip_list, 'map_name': map_name, 'map_save': 'false'}],
        ),
        Node(
            package='lidar_processor_cpp',
            executable='pointcloud_frame_filter_node',
            name='pointcloud_frame_filter',
            remappings=[('cloud_in', '/point_cloud2'), ('cloud_out', '/pointcloud/current_filtered')],
            parameters=[{
                'target_frame': 'base_link',
                'max_range': 20.0, 'min_range': 0.3,
                'min_height': -1.0, 'max_height': 3.0,
                'voxel_size': 0.005,
            }],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='go2_pointcloud_to_laserscan',
            remappings=[('cloud_in', '/pointcloud/current_filtered'), ('scan', '/scan')],
            parameters=[{
                'target_frame': 'base_link',
                'max_height': 3.0, 'min_height': -1.0,
                'angle_min': -3.14159, 'angle_max': 3.14159,
                'angle_increment': 0.00872665, 'scan_time': 0.1,
                'range_min': 0.3, 'range_max': 20.0,
                'use_inf': True, 'concurrency_level': 1,
            }],
        ),
        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=rtabmap_params,
            remappings=remappings,
            arguments=rtabmap_args,
        ),
    ]

    # Teleop
    teleop_nodes = [
        Node(package='joy', executable='joy_node',
             condition=IfCondition(LaunchConfiguration('joystick')),
             parameters=[config_paths['joystick']]),
        Node(package='teleop_twist_joy', executable='teleop_node', name='go2_teleop_node',
             condition=IfCondition(LaunchConfiguration('joystick')),
             parameters=[config_paths['twist_mux']]),
        Node(package='twist_mux', executable='twist_mux', output='screen',
             condition=IfCondition(LaunchConfiguration('joystick')),
             parameters=[{'use_sim_time': use_sim_time}, config_paths['twist_mux']]),
    ]

    # Visualization
    viz_nodes = [
        Node(package='rviz2', executable='rviz2', name='go2_rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', config_paths['rviz']],
             parameters=[{'use_sim_time': False}]),
        # Native RTAB-Map visualizer (loop closure graph, cloud view)
        Node(package='rtabmap_viz', executable='rtabmap_viz', output='screen',
             condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
             parameters=rtabmap_params, remappings=remappings),
    ]

    return core_nodes + teleop_nodes + viz_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localize against existing database instead of mapping'),
        DeclareLaunchArgument('scan2d', default_value='true',
                              description='true=consume /scan (2D, current robot); false=consume 3D point cloud (external LIDAR)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('joystick', default_value='true'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
