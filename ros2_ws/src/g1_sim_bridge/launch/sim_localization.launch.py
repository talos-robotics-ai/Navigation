"""Simulation localization bring-up.

Starts the Isaac->FAST-LIO bridge and (optionally) FAST-LIO itself, configured
for simulation:

    Isaac Sim  --(/livox/lidar PointCloud2)-->  g1_sim_bridge  --(/livox/custom_msg)-->  FAST-LIO
    Isaac Sim  --(/livox/imu_raw  BEST_EFFORT)->  g1_sim_bridge  --(/livox/imu RELIABLE)->  FAST-LIO

FAST-LIO runs with use_sim_time:=true so it consumes Isaac's /clock. The
mid360.yaml config is used unchanged (lidar_type:1, /livox/custom_msg,
/livox/imu) -- the bridge makes Isaac look exactly like the real Livox driver.

Run Isaac first (Navigation/sim/launch_g1_sim.sh), then:

    ros2 launch g1_sim_bridge sim_localization.launch.py
    ros2 launch g1_sim_bridge sim_localization.launch.py start_fastlio:=false   # bridge only
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_fastlio = LaunchConfiguration('start_fastlio')
    use_sim_time = LaunchConfiguration('use_sim_time')
    fake_sweep_time = LaunchConfiguration('fake_sweep_time')
    num_lines = LaunchConfiguration('num_lines')
    stride = LaunchConfiguration('stride')

    declare_start_fastlio = DeclareLaunchArgument(
        'start_fastlio', default_value='true',
        description='Also launch FAST-LIO (mapping.launch.py) with sim params')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='FAST-LIO consumes Isaac /clock when true')
    declare_fake_sweep = DeclareLaunchArgument(
        'fake_sweep_time', default_value='false',
        description='Synthesize per-point offset_time (de-skew). Off: Isaac '
                    'clouds are instantaneous snapshots.')
    declare_num_lines = DeclareLaunchArgument(
        'num_lines', default_value='4',
        description='Synthetic Livox scan lines (<= FAST-LIO scan_line)')
    declare_stride = DeclareLaunchArgument(
        'stride', default_value='1',
        description='Keep every Nth point (decimation)')

    bridge_node = Node(
        package='g1_sim_bridge',
        executable='isaac_livox_custom_adapter_node',
        name='isaac_livox_custom_adapter',
        output='screen',
        parameters=[{
            'input_topic': '/livox/lidar',
            'output_topic': '/livox/custom_msg',
            'imu_in_topic': '/livox/imu_raw',
            'imu_out_topic': '/livox/imu',
            'fake_sweep_time': fake_sweep_time,
            'num_lines': num_lines,
            'stride': stride,
            'use_sim_time': use_sim_time,
        }],
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fast_lio'), 'launch', 'mapping.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': 'mid360.yaml',
            'rviz': 'true',
        }.items(),
        condition=IfCondition(start_fastlio),
    )

    return LaunchDescription([
        declare_start_fastlio,
        declare_use_sim_time,
        declare_fake_sweep,
        declare_num_lines,
        declare_stride,
        bridge_node,
        fast_lio_launch,
    ])
