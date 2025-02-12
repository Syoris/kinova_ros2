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

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("kinova_launch"), "config", "robot_parameters.yaml"
    )

    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="False",
        description="Launch the kinova_arm_node in simulation mode",
    )

    sim_arg = DeclareLaunchArgument(
        "rviz",
        default_value="False",
        description="Launch rviz",
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

    # Display launch
    rviz_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("kinova_launch"), "launch"),
                "/rviz_launch.py",
            ]
        )
    )

    return LaunchDescription(
        [
            sim_arg,
            kinova_arm_node,
            kinova_tf_updater_node,
            kinova_sim_node,
            rviz_launch_desc,
        ]
    )
