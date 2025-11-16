import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    bt_xml = LaunchConfiguration('bt_xml')

    pkg_share = FindPackageShare('wheelchair_robot_description')
    urdf_path = os.path.join(get_package_share_directory('wheelchair_robot_description'), 'urdf', 'wheelchair.urdf')

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),

        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                pkg_share,
                'config',
                'nav2_params.yaml'
            ]),
            description='Full path to the Nav2 parameters file'),

        DeclareLaunchArgument(
            'bt_xml',
            default_value=PathJoinSubstitution([
                pkg_share,
                'bt_xml',
                'local_nav.xml'
            ]),
            description='Full path to the behavior tree XML'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                pkg_share,
                'launch',
                'navigation_launch.py'
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'bt_xml_filename': bt_xml
            }.items()
        )
    ])

