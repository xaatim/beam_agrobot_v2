import os
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'robot_description'
    urdf_file = 'agrobot_v2.urdf.xacro'
    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file)
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        )
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': True
        }]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'Beam-AgroBot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen',
                )
            ]
        )
    )

    controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_cont'],
                    output='screen',
                )
            ]
        )
    )

    arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['arm_controller'],
                    output='screen',
                )
            ]
        )
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        static_tf,
        spawn_entity,            
        joint_state_broadcaster_spawner, 
        controller_spawner,       
        arm_controller_spawner,
    ])
