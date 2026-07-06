import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml

def load_file(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, file_path)
    with open(abs_path, 'r') as f:
        return f.read()

def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, file_path)
    with open(abs_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    urdf = load_file('constructbot_description', 'urdf/snehbot.urdf')
    srdf = load_file('constructbot_moveit_config', 'config/constructbot.srdf')
    kinematics = load_yaml('constructbot_moveit_config', 'config/kinematics.yaml')
    joint_limits = load_yaml('constructbot_moveit_config', 'config/joint_limits.yaml')

    robot_description = {'robot_description': urdf}
    robot_description_semantic = {'robot_description_semantic': srdf}
    robot_description_kinematics = {'robot_description_kinematics': kinematics}
    joint_limits_param = {'robot_description_planning': joint_limits}

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits_param,
            {'use_sim_time': False},
            {'publish_robot_description_semantic': True},
            # Tell MoveIt2 which joint state topic to use
            {'joint_state_topic': 'joint_states'},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])
