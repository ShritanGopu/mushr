#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tg = LaunchConfiguration("tg")
    share = FindPackageShare("mushr_rhc")
    trajgen_param_file = [
        PathJoinSubstitution([share, "launch", "params", "trajgen"]),
        "/",
        tg,
        ".yaml",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("tg", default_value="tl"),
            SetEnvironmentVariable("RHC_USE_CUDA", "0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([share, "launch", "map_server.launch.py"])
                )
            ),
            Node(
                package="mushr_rhc",
                executable="rhcdebug",
                name="rhcdebug",
                output="screen",
                parameters=[
                    {"inferred_pose_t": "/pf/inferred_pose"},
                    trajgen_param_file,
                    PathJoinSubstitution([share, "launch", "params", "all_params.yaml"]),
                    PathJoinSubstitution([share, "launch", "debug", "params.yaml"]),
                ],
            ),
        ]
    )
