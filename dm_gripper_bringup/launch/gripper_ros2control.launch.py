"""
gripper_ros2control.launch.py

Full ros2_control bringup:
  - controller_manager with DmGripperSystem hardware interface
  - gripper_controller (GripperActionController)
  - gripper_action_server node (optional, for standalone action interface)

Usage:
    ros2 launch dm_gripper_bringup gripper_ros2control.launch.py
    ros2 launch dm_gripper_bringup gripper_ros2control.launch.py can_interface:=can1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("dm_gripper_bringup")

    # ── Launch arguments ──────────────────────────────────────────────────────
    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="Linux SocketCAN interface name"
    )

    ros2_control_params_arg = DeclareLaunchArgument(
        "ros2_control_params",
        default_value=PathJoinSubstitution([pkg_share, "config", "ros2_control.yaml"]),
        description="Path to ros2_control parameters YAML"
    )

    # ── controller_manager ────────────────────────────────────────────────────
    # NOTE: robot_description parameter must be provided by the user's URDF
    # that includes the <ros2_control> block referencing dm_gripper_hardware/DmGripperSystem.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration("ros2_control_params"),
        ],
        output="screen",
    )

    # ── gripper_controller spawner ────────────────────────────────────────────
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawn controller only after controller_manager is running
    delay_gripper_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        can_interface_arg,
        ros2_control_params_arg,
        controller_manager,
        delay_gripper_controller,
    ])
