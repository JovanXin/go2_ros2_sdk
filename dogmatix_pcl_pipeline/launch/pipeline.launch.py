"""Launch file for Dogmatix PCL Pipeline."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/pointcloud'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/dogmatix'),
        DeclareLaunchArgument('process_on_receive', default_value='false'),

        Node(
            package='dogmatix_pcl_pipeline',
            executable='pcl_pipeline_node',
            name='dogmatix_pcl_pipeline',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_dir': LaunchConfiguration('output_dir'),
                'process_on_receive': LaunchConfiguration('process_on_receive'),
            }],
        ),
    ])
