#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("force_in_bounds", default_value="false"),
            DeclareLaunchArgument("tf_prefix", default_value=""),
            DeclareLaunchArgument("use_mocap", default_value="false"),
            Node(
                package="mushr_base",
                executable="racecar_state",
                name="racecar_state",
                output="screen",
                parameters=[
                    {"update_rate": 20.0},
                    {"speed_offset": 0.0},
                    {"speed_noise": 0.0001},
                    {"steering_angle_offset": 0.0},
                    {"steering_angle_noise": 0.000001},
                    {"forward_offset": 0.0},
                    {"forward_fix_noise": 0.0000001},
                    {"forward_scale_noise": 0.001},
                    {"side_offset": 0.0},
                    {"side_fix_noise": 0.000001},
                    {"side_scale_noise": 0.001},
                    {"theta_offset": 0.0},
                    {"theta_fix_noise": 0.000001},
                    {"force_in_bounds": LaunchConfiguration("force_in_bounds")},
                    {"tf_prefix": LaunchConfiguration("tf_prefix")},
                    {"use_mocap": LaunchConfiguration("use_mocap")},
                ],
            ),
        ]
    )
