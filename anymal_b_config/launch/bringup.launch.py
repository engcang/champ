import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    anymal_b_config_path = get_package_share_directory('anymal_b_config')
    description_file = os.path.join(anymal_b_config_path, 'urdf', 'anymal_b.urdf')
    # Process the Xacro file
    doc = xacro.process_file(description_file)
    robot_description = doc.toprettyxml(indent='  ')

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='/anymal_b'),
        DeclareLaunchArgument('joints_map_file', default_value=f'{anymal_b_config_path}/config/joints.yaml'),
        DeclareLaunchArgument('links_map_file', default_value=f'{anymal_b_config_path}/config/links.yaml'),
        DeclareLaunchArgument('gait_config_file', default_value=f'{anymal_b_config_path}/config/gait.yaml'),
        DeclareLaunchArgument('joint_controller_topic', default_value='joint_group_position_controller/command'),

        Node(
            package='champ_base',
            executable='quadruped_controller_node',
            namespace=LaunchConfiguration('robot_name'),
            parameters=[
                {'gazebo': False},
                {'publish_joint_states': False},
                {'publish_foot_contacts': False},
                {'publish_joint_control': True},
                {'joint_controller_topic': LaunchConfiguration('joint_controller_topic')},
                {'urdf': robot_description},
                LaunchConfiguration('joints_map_file'),
                LaunchConfiguration('links_map_file'),
                LaunchConfiguration('gait_config_file'),
            ],
            output='screen'
        ),

        Node(
            package='anymal_b_config',
            executable='champ_teleop.py',
            namespace=LaunchConfiguration('robot_name'),
            output='screen'
        ),
    ])
