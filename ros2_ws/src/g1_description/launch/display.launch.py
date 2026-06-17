"""Static display launch: robot_state_publisher + joint_state_publisher_gui + RViz.

Use this to inspect the URDF / try out joint angles in isolation. For the
real-robot or sim pipeline use ``g1_bringup`` instead — it replaces the GUI joint
publisher with the WS telemetry bridge driven by the live robot.

Run inside the ROS2 Humble Docker container (host runs Jazzy and is incompatible):
    ros2 launch g1_description display.launch.py
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("g1_description")
    default_urdf = PathJoinSubstitution([pkg_share, "urdf", "g1_29dof.urdf"])
    default_rviz = PathJoinSubstitution([pkg_share, "rviz", "g1_display.rviz"])

    urdf_arg = DeclareLaunchArgument(
        "urdf_file", default_value=default_urdf,
        description="Absolute path to the URDF to load.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz_config", default_value=default_rviz,
        description="Absolute path to the RViz config.",
    )

    # robot_description is the URDF text, expanded by xacro for safety even
    # though the file is plain XML — keeps the launch portable.
    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ", LaunchConfiguration("urdf_file"),
        ]),
    }

    return LaunchDescription([
        urdf_arg,
        rviz_arg,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ])
