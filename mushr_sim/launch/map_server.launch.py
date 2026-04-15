import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart


def generate_launch_description():
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map')
    }])

    map_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
    }])

    wait_for_map_server = Node(
        package='mushr_sim',
        executable='wait_for_map_server',
        name='wait_for_map_server',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('mushr_sim'),
                'maps',
                'sandbox.yaml',
            ]),
        ),
        map_server_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=map_server_node,
                on_start=[wait_for_map_server],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_map_server,
                on_exit=[TimerAction(period=2.0, actions=[map_lifecycle_manager_node])],
            )
        ),
    ])
