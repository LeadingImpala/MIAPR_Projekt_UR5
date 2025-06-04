import launch
import os
import yaml

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as f:
        return yaml.safe_load(f)

def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=172.17.0.2",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=true",
            " ",
            "safety_pos_margin:=0.15",
            " ",
            "safety_k_position:=20",
            " ",
            "name:=ur",
            " ",
            "ur_type:=ur5e",
            " ",
            "prefix:=",
            '""',
        ]
    )

    return {"robot_description": robot_description_content}


def get_robot_description_semantic():
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=ur",
            " ",
            "prefix:=",
            '""',
        ]
    )
    return {"robot_description_semantic": robot_description_semantic_content}



def generate_launch_description():
    ompl_yaml_full = load_yaml("moveit_cpp_planners", "config/ompl_planning.yaml")
    ompl_yaml = ompl_yaml_full["moveit_cpp"]  # Only pass the actual nested dictionary

    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()

    kinematics_yaml = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    ompl_node = Node(
        package="moveit_cpp_planners",
        executable="planning_scene_ompl",
        name="planning_scene_ompl",
        output="screen",
        parameters=[
            {"planning_pipeline": "stomp"},  # ✅ Set primary planner
            {"planning_plugin": "stomp_interface/StompPlanner"},
            PathJoinSubstitution([FindPackageShare("moveit_cpp_planners"), "config", "stomp_planning.yaml"]),
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_yaml,
        ],

    )

    return launch.LaunchDescription([ompl_node])
