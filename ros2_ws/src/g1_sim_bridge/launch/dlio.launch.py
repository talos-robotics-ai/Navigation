"""Generic G1 DLIO bring-up: dlio_odom_node + dlio_map_node.

Shared by both localization launches so the DLIO node wiring lives in one place:
  * sim:  g1_sim_bridge/sim_localization.launch.py (config_file=dlio_sim.yaml,
          pointcloud_topic=/livox/lidar_reliable from the QoS relay)
  * real: g1_bringup/real_localization.launch.py   (config_file=dlio_mid360_real.yaml,
          pointcloud_topic=/livox/lidar from the Livox driver)

DLIO parameters are layered: the vendored cfg/dlio.yaml + cfg/params.yaml first,
then the G1 overlay (config_file, a filename in g1_sim_bridge/config). DLIO
publishes /dlio/odom_node/* and /dlio/map_node/map and broadcasts TF
odom -> base_link -> {livox, livox_imu}.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    config_file = LaunchConfiguration('config_file')
    start_map = LaunchConfiguration('start_map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='DLIO consumes /clock when true (sim).')
    declare_pointcloud = DeclareLaunchArgument(
        'pointcloud_topic', default_value='/livox/lidar',
        description='Input PointCloud2 topic for DLIO.')
    declare_imu = DeclareLaunchArgument(
        'imu_topic', default_value='/livox/imu',
        description='Input Imu topic for DLIO.')
    declare_config = DeclareLaunchArgument(
        'config_file', default_value='dlio_mid360_real.yaml',
        description='DLIO overlay yaml in g1_sim_bridge/config, layered over the '
                    'vendored cfg/dlio.yaml + cfg/params.yaml.')
    declare_start_map = DeclareLaunchArgument(
        'start_map', default_value='true',
        description='Also run dlio_map_node (accumulated /dlio/map_node/map).')

    # Layered parameters: vendored defaults first, then the G1 overlay.
    dlio_share = get_package_share_directory('direct_lidar_inertial_odometry')
    bridge_share = get_package_share_directory('g1_sim_bridge')
    dlio_params = [
        os.path.join(dlio_share, 'cfg', 'dlio.yaml'),
        os.path.join(dlio_share, 'cfg', 'params.yaml'),
        PathJoinSubstitution([bridge_share, 'config', config_file]),
        {'use_sim_time': use_sim_time},
    ]

    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        name='dlio_odom_node',
        output='screen',
        parameters=dlio_params,
        remappings=[
            ('pointcloud', pointcloud_topic),
            ('imu', imu_topic),
            ('odom', '/dlio/odom_node/odom'),
            ('pose', '/dlio/odom_node/pose'),
            ('path', '/dlio/odom_node/path'),
            ('kf_pose', '/dlio/odom_node/keyframes'),
            ('kf_cloud', '/dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', '/dlio/odom_node/pointcloud/deskewed'),
        ],
    )

    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        name='dlio_map_node',
        output='screen',
        condition=IfCondition(start_map),
        parameters=dlio_params,
        remappings=[
            ('keyframes', '/dlio/odom_node/pointcloud/keyframe'),
            ('map', '/dlio/map_node/map'),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_pointcloud,
        declare_imu,
        declare_config,
        declare_start_map,
        dlio_odom_node,
        dlio_map_node,
    ])
