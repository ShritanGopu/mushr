#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    out_path = LaunchConfiguration("out_path")
    trial_name = LaunchConfiguration("trial_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument("out_path"),
            DeclareLaunchArgument("trial_name"),
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    [
                        "ros2 param dump /rhcontroller > ",
                        out_path,
                        "/",
                        trial_name,
                        "-params.yaml",
                    ],
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "topic_tools",
                    "throttle",
                    "messages",
                    "/camera/color/image_raw",
                    "5",
                    "/debug_camera",
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    [out_path, "/", trial_name],
                    "/initialpose",
                    "/move_base_simple/goal",
                    "/pf/inferred_pose",
                    "/pf/viz/laserpose",
                    "/pf/viz/particles",
                    "/rhcontroller/traj_chosen",
                    "/scan",
                    "/debug_camera",
                ],
                output="screen",
            ),
        ]
    )
