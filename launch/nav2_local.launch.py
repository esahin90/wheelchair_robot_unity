import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    bt_xml = LaunchConfiguration('bt_xml')

    pkg_share = FindPackageShare('wheelchair_robot_description')
    urdf_path = os.path.join(get_package_share_directory('wheelchair_robot_description'), 'urdf', 'wheelchair.urdf')

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
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
                'robot_description': robot_description
            }]
        ),

        Node(
            package='wheelchair_robot_description',
            executable='pose_to_odom_tf.py',
            name='pose_to_odom_tf',
            output='screen',
            parameters=[{
                'pose_topic': '/unity/robot_pose',
                'odom_topic': '/odom',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }]
        ),

        Node(
            package='wheelchair_robot_description',
            executable='unity_goal_bridge.py',
            name='unity_goal_bridge',
            output='screen',
            parameters=[{
                'goal_topic': '/unity/goal_pose',
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                pkg_share,
                'launch',
                'navigation_launch.py'
            ])),
            launch_arguments={
                'params_file': params_file,
                'bt_xml_filename': bt_xml
            }.items()
        ),

        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('rosbridge_server'),
                    'launch',
                    'rosbridge_websocket_launch.xml'
                ])
            ),
            launch_arguments={
                'port': '9090',          # optional: default is 9090
                'address': '0.0.0.0'     # optional: allow Unity to connect remotely
            }.items()
        )
    ])

