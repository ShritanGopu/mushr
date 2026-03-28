#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tg = LaunchConfiguration("tg")
    car_name = LaunchConfiguration("car_name")
    map_server = LaunchConfiguration("map_server")
    share = FindPackageShare("mushr_rhc")
    trajgen_param_file = [
        PathJoinSubstitution([share, "launch", "params", "trajgen"]),
        "/",
        tg,
        ".yaml",
    ]

    controller = Node(
        package="mushr_rhc",
        executable="rhcnode",
        name="rhcontroller",
        output="screen",
        parameters=[
            {"inferred_pose_t": "particle_filter/inferred_pose", "car_name": car_name},
            trajgen_param_file,
            PathJoinSubstitution([share, "launch", "params", "all_params.yaml"]),
            PathJoinSubstitution([share, "launch", "real", "params.yaml"]),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("tg", default_value="dispersion"),
            DeclareLaunchArgument("car_name", default_value="car"),
            DeclareLaunchArgument("map_server", default_value="0"),
            SetEnvironmentVariable("RHC_USE_CUDA", "0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([share, "launch", "map_server.launch.py"])
                ),
                condition=IfCondition(map_server),
            ),
            GroupAction([PushRosNamespace(car_name), controller]),
        ]
    )
