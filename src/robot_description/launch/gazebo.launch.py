import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'robot_description'
    urdf_file = 'beam_agrobot_v2.urdf'
    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file)

    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        )
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
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
        arguments=['-entity', 'Beam-AgroBot', '-file', urdf_path],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        static_tf,
        spawn_entity
    ])
