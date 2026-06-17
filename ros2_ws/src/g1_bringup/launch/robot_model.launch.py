"""Publish the G1 robot model + TF, attached to FAST-LIO's pose.

    robot_state_publisher : g1_29dof.urdf -> /robot_description + link TFs
    joint_state_publisher : neutral (zero) joint angles so every link has a TF
    static_transform       : FAST-LIO `body` (the MID-360 IMU frame) -> `pelvis`

The static transform is inverse(pelvis -> mid360_link) from the URDF chain
(pelvis->waist_yaw->waist_roll->torso_link->mid360_link) at the neutral pose, so
the model's mid360_link coincides with FAST-LIO's `body` frame and the whole G1
rides the localization pose. RViz fixed frame stays `camera_init`.

Once Isaac publishes real /joint_states (via the DDS bridge), set
use_joint_state_publisher:=false so the model shows the actual sim pose.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_jsp = LaunchConfiguration('use_joint_state_publisher')
    attach_frame = LaunchConfiguration('attach_frame')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_use_jsp = DeclareLaunchArgument(
        'use_joint_state_publisher', default_value='true',
        description='Publish neutral joint angles (off once Isaac feeds /joint_states)')
    declare_attach = DeclareLaunchArgument(
        'attach_frame', default_value='body',
        description="FAST-LIO body frame the model's mid360_link is pinned to")

    urdf = PathJoinSubstitution(
        [FindPackageShare('g1_description'), 'urdf', 'g1_29dof.urdf'])
    # Plain URDF (no xacro macros) -> `cat` avoids a xacro dependency.
    robot_description = ParameterValue(Command(['cat ', urdf]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
        output='screen',
    )
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_jsp),
    )
    # body -> pelvis = inverse(pelvis -> mid360_link), computed from the URDF.
    body_to_pelvis = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_pelvis',
        arguments=[
            '--x', '0.022145', '--y', '-0.00003', '--z', '-0.459662',
            '--roll', '0', '--pitch', '-0.040143', '--yaw', '0',
            '--frame-id', attach_frame, '--child-frame-id', 'pelvis',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_jsp,
        declare_attach,
        rsp,
        jsp,
        body_to_pelvis,
    ])
