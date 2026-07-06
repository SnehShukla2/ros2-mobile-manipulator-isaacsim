import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('constructbot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'snehbot.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([

        # 1. Publish robot_description to a topic so joint_state_publisher can find it
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': False,
                'publish_frequency': 50.0,
            }],
            output='screen'
        ),

        # 2. Publish joint states for wheel joints (continuous joints)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'use_sim_time': False,
                'robot_description': robot_desc,
            }],
            output='screen'
        ),

        # 3. Bridge constructbot → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='constructbot_to_base_link',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'constructbot',
                '--child-frame-id', 'base_link'
            ],
            output='screen'
        ),

        # 4. Also bridge odom → constructbot in case Isaac publishes odom separately
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_constructbot',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'odom',
                '--child-frame-id', 'constructbot'
            ],
            output='screen'
        ),

        # 5. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': False}],
            output='screen'
        ),
    ])
