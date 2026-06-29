from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def generate_launch_description():

    moveit_config_pkg = get_package_share_directory("moveit_config")

    # load SRDF
    srdf_path = os.path.join(moveit_config_pkg, "config", "agrobot.srdf")
    with open(srdf_path, "r") as f:
        robot_description_semantic = f.read()

    # load kinematics
    kinematics_path = os.path.join(moveit_config_pkg, "config", "kinematics.yaml")
    with open(kinematics_path, "r") as f:
        kinematics = yaml.safe_load(f)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[
            {"robot_description_semantic": robot_description_semantic},
            {"robot_description_kinematics": kinematics},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([rviz_node])