"""DLIO online debugging session (simulation).

Brings up the full sim localization chain plus RViz so you can watch DLIO
(direct_lidar_inertial_odometry) work in real time:

    g1_sim_bridge (Isaac /livox/lidar + /livox/imu_raw  ->  /livox/lidar_reliable + /livox/imu)
    DLIO          (dlio_sim.yaml, use_sim_time; odom -> base_link TF)
    RViz          (g1_dlio.rviz: /dlio/odom_node/odom,
                   /dlio/odom_node/pointcloud/deskewed, /dlio/map_node/map,
                   /dlio/odom_node/path, TF odom->base_link)

Run Isaac on the host first (Navigation/sim/launch_g1_sim.sh), then inside the
localization container (ws sourced):

    ros2 launch g1_bringup dlio_debug.launch.py
    ros2 launch g1_bringup dlio_debug.launch.py rviz:=false   # headless

NOTE: DLIO calibrates its IMU + gravity over the first few seconds, so keep the
robot STATIONARY at startup. It also needs a NON-EMPTY lidar cloud -- if DLIO
prints no odometry, the Isaac MID-360 profile may not be rendering yet; launch
Isaac with a known-good built-in profile meanwhile:
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
    robot_model = LaunchConfiguration('robot_model')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='DLIO + RViz consume Isaac /clock when true')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='Open RViz')
    declare_robot_model = DeclareLaunchArgument(
        'robot_model', default_value='true',
        description='Show the G1 robot model (RSP + JSP + base_link->pelvis TF)')

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare('g1_bringup'), 'rviz', 'g1_dlio.rviz'])

    # Reuse the sim localization bring-up (QoS relay + DLIO).
    sim_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('g1_sim_bridge'),
            '/launch/sim_localization.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_dlio': 'true',
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
        name='rviz_dlio',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        declare_robot_model,
        sim_localization,
        robot_model_launch,
        rviz_node,
    ])
