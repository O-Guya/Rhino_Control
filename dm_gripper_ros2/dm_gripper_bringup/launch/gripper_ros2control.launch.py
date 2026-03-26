"""
完整 ros2_control 启动文件
包括: controller_manager, gripper_controller, gripper_action_server
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("dm_gripper_bringup")

    ros2_control_params = PathJoinSubstitution([pkg, "config", "ros2_control.yaml"])
    gripper_params = PathJoinSubstitution([pkg, "config", "gripper_params.yaml"])

    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="SocketCAN network interface name",
    )

    # TODO: add controller_manager, spawner, gripper_action_server nodes

    return LaunchDescription([
        can_interface_arg,
        # TODO: nodes
    ])
