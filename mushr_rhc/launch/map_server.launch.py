#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_arg = LaunchConfiguration("map")
    c2g_map_arg = LaunchConfiguration("c2g_map")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_arg}],
    )
    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="map_server_lifecycle_manager",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server"]}],
    )

    c2g_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="c2g_map",
        output="screen",
        parameters=[{"yaml_filename": c2g_map_arg}],
        remappings=[
            ("/map", "/c2g/map"),
            ("/map_metadata", "/c2g/map_metadata"),
        ],
    )
    c2g_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="c2g_map_lifecycle_manager",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["c2g_map"]}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("mushr_rhc"), "maps", "real-floor0-edited.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "c2g_map",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("mushr_rhc"), "maps", "real-floor0-edited-c2g-full-loop.yaml"]
                ),
            ),
            SetParameter(name="map_file", value=c2g_map_arg),
            map_server,
            c2g_map_server,
            TimerAction(period=2.0, actions=[map_lifecycle_manager]),  # c2g_lifecycle_manager
        ]
    )
