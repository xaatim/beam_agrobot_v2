import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('robot_navigation')
    cartographer_config = os.path.join(pkg, 'config')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', cartographer_config,
                '-configuration_basename', 'agrobot_cartographer.lua'
            ],
            parameters=[{'use_sim_time': True}],
            remappings=[('odom', '/diff_cont/odom')]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True, 'resolution': 0.05}]
        ),
    ])
