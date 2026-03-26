"""
裸电机测试启动文件（不依赖 ros2_control）
直接启动 gripper_action_server 节点，用于早期调试和电机验证。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("dm_gripper_bringup")

    params_file = PathJoinSubstitution([pkg, "config", "gripper_params.yaml"])

    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="SocketCAN network interface name",
    )

    # TODO: add gripper_action_server Node

    return LaunchDescription([
        can_interface_arg,
        # TODO: nodes
    ])
