import launch
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        raise RuntimeError(f"Failed to load YAML file {absolute_file_path}: {str(e)}")

def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "visual_parameters.yaml"]
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
            "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur3e",
            " ",
            "prefix:=",
            '""',
            " ",
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
            "name:=",
            "ur",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )
    return {"robot_description_semantic": robot_description_semantic_content}

def generate_launch_description():
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()

    # Load kinematics.yaml properly
    kinematics_yaml_path = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'config',
        'kinematics.yaml'
    )
    
    if not os.path.exists(kinematics_yaml_path):
        raise FileNotFoundError(f"Kinematics config not found at: {kinematics_yaml_path}")

    try:
        with open(kinematics_yaml_path, 'r') as file:
            kinematics_yaml = yaml.safe_load(file)
            if not kinematics_yaml:
                raise ValueError("Loaded empty kinematics configuration")
    except Exception as e:
        raise RuntimeError(f"Failed to parse kinematics.yaml: {str(e)}")

    demo_node = Node(
        package="ros2_ur_moveit_examples",
        executable="kinematics",
        name="kinematics",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml}
        ],
    )

    return launch.LaunchDescription([demo_node])