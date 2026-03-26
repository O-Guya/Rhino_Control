"""
gripper_standalone.launch.py

Launch the gripper action server directly (no ros2_control).
Use this for bare-motor testing before the full ros2_control stack is needed.

Usage:
    ros2 launch dm_gripper_bringup gripper_standalone.launch.py
    ros2 launch dm_gripper_bringup gripper_standalone.launch.py can_interface:=can1
"""

import os
from ament_python_import_helper import get_package_share_directory  # type: ignore
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("dm_gripper_bringup")

    # ── Launch arguments ──────────────────────────────────────────────────────
    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="Linux SocketCAN interface name (e.g. can0, can1)"
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "gripper_params.yaml"]),
        description="Path to gripper parameters YAML"
    )

    # ── Nodes ─────────────────────────────────────────────────────────────────
    gripper_action_server = Node(
        package="dm_gripper_action_server",
        executable="gripper_action_server",
        name="gripper_action_server",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"can_interface": LaunchConfiguration("can_interface")},
        ],
    )

    return LaunchDescription([
        can_interface_arg,
        params_file_arg,
        gripper_action_server,
    ])
