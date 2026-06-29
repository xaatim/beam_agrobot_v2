import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r") as f:
        return yaml.safe_load(f)


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r") as f:
        return f.read()


def generate_launch_description():

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("robot_description"),
            "urdf",
            "agrobot_v2.urdf.xacro"
        )
    )
    robot_description = {
        "robot_description": robot_description_config.toxml()  # type: ignore
    }

    robot_description_semantic = {
        "robot_description_semantic": load_file(
            "moveit_config",
            "config/agrobot.srdf"
        )
    }

    kinematics_yaml = load_yaml(
        "moveit_config",
        "config/kinematics.yaml"
    )

    joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            "moveit_config",
            "config/joint_limits.yaml"
        )
    }

    moveit_controllers = load_yaml(
        "moveit_config",
        "config/moveit_controllers.yaml"
    )

    ompl_planning_yaml = load_yaml(
        "moveit_config",
        "config/ompl_planning.yaml"
    )

    planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/move_group/planning_scene_monitor",
            "publish_planning_scene_topic": "/move_group/publish_planning_scene",
            "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            joint_limits_yaml,
            moveit_controllers,
            planning_pipeline_config,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([move_group_node])