"""FAST-LIO online debugging session (simulation).

Brings up the full sim localization chain plus RViz so you can watch FAST-LIO
work in real time:

    g1_sim_bridge (Isaac /livox/lidar + /livox/imu_raw  ->  /livox/custom_msg + /livox/imu)
    FAST-LIO      (mid360.yaml, lidar_type:1, use_sim_time)
    RViz          (g1_fastlio.rviz: /Odometry_loc, /cloud_registered_1,
                   /Laser_map_1, /path_1, TF camera_init->body)

Run Isaac on the host first (Navigation/sim/launch_g1_sim.sh), then inside the
localization container (ws sourced):

    ros2 launch g1_bringup fastlio_debug.launch.py
    ros2 launch g1_bringup fastlio_debug.launch.py rviz:=false   # headless

NOTE: needs a NON-EMPTY lidar cloud. If FAST-LIO prints "No point, skip this
scan", the Isaac MID-360 profile isn't rendering yet -- launch Isaac with a
known-good built-in profile meanwhile:
    Navigation/sim/launch_g1_sim.sh --lidar-config Example_Rotary
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    fake_sweep_time = LaunchConfiguration('fake_sweep_time')
    robot_model = LaunchConfiguration('robot_model')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='FAST-LIO + RViz consume Isaac /clock when true')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='Open RViz')
    declare_fake_sweep = DeclareLaunchArgument(
        'fake_sweep_time', default_value='false',
        description='Synthesize per-point offset_time in the bridge')
    declare_robot_model = DeclareLaunchArgument(
        'robot_model', default_value='true',
        description='Show the G1 robot model (RSP + JSP + body->pelvis TF)')

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare('g1_bringup'), 'rviz', 'g1_fastlio.rviz'])

    # Reuse the sim localization bring-up (bridge + FAST-LIO).
    sim_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('g1_sim_bridge'),
            '/launch/sim_localization.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_fastlio': 'true',
            'fake_sweep_time': fake_sweep_time,
        }.items(),
    )

    robot_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('g1_bringup'), '/launch/robot_model.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(robot_model),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_fastlio',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        declare_fake_sweep,
        declare_robot_model,
        sim_localization,
        robot_model_launch,
        rviz_node,
    ])
