import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    my_nav_dir = get_package_share_directory('robot_navigation')

    # Define your exact map path
    map_file = os.path.join(my_nav_dir, 'maps', 'agrobot_field.yaml')

    # GroupAction applies the remap to EVERYTHING launched inside it
    nav2_group = GroupAction(
        actions=[
            # This wires the Nav2 brain directly to your Gazebo wheels
            SetRemap(src='/cmd_vel', dst='/diff_cont/cmd_vel_unstamped'),
            
            # Launch Nav2 (without the buggy params file)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': 'true',
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    return LaunchDescription([nav2_group])