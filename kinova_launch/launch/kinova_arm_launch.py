import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import (
    LaunchConfiguration,
)

from launch.actions import (
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("kinova_launch"), "config", "robot_parameters.yaml"
    )

    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="False",
        description="Launch the kinova_arm_node in simulation mode",
    )

    kinova_sim_node = Node(
        package="kinova_sim",
        namespace="arm",
        executable="kinova_sim",
        condition=IfCondition(LaunchConfiguration("sim")),
    )

    kinova_arm_node = Node(
        package="kinova_driver",
        namespace="arm",
        executable="kinova_arm_node",
        name="kinova_arm",
        output="screen",
        parameters=[config],
        # arguments=["--ros-args --log-level debug"],
    )

    kinova_tf_updater_node = Node(
        package="kinova_driver",
        namespace="arm",
        executable="kinova_tf_updater",
    )

    return LaunchDescription(
        [
            sim_arg,
            kinova_arm_node,
            kinova_tf_updater_node,
            kinova_sim_node,
        ]
    )
