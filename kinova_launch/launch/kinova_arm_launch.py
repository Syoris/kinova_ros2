import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("kinova_launch"), "config", "robot_parameters.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="kinova_driver",
                namespace="arm",
                executable="kinova_arm_node",
                name="kinova_arm",
                output="screen",
                parameters=[config],
                # arguments=["--ros-args --log-level debug"],
            ),
        ]
    )
