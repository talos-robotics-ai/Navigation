"""Real-robot localization bring-up (DLIO + Livox MID-360).

The real-robot counterpart of g1_sim_bridge/sim_localization.launch.py. There is
no QoS relay here -- the stock Livox SDK driver publishes the data DLIO needs
directly:

    livox_ros_driver2  --(/livox/lidar  PointCloud2, xfer_format=0)-->  DLIO
    livox_ros_driver2  --(/livox/imu    Imu, RAW)-------------------->  DLIO

Key real-robot specifics:
  * xfer_format=0 -> the driver emits a PointCloud2 (PointXYZRTLT) with per-point
    timestamps, which DLIO auto-detects as SensorType::LIVOX and deskews.
  * The MID-360 is mounted upside-down. MID360_config.json applies roll:180 to
    the CLOUD (upright), and the driver now publishes the IMU RAW (inverted).
    DLIO corrects the IMU via extrinsics/baselink2imu/R = R_x(180) in
    dlio_mid360_real.yaml -- no driver patch.
  * DLIO must calibrate its IMU + gravity over the first ~3 s, so keep the robot
    STATIONARY at startup (e.g. during the AMO stabilize hold).

Run (inside the localization container, ws sourced, robot powered + on-network):

    ros2 launch g1_bringup real_localization.launch.py
    ros2 launch g1_bringup real_localization.launch.py rviz:=false robot_model:=false

See docs/DLIO_DEPLOYMENT_TESTING.md (phases 5-7) and docs/DLIO_G1_MID360_TUNING.md.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    robot_model = LaunchConfiguration('robot_model')
    livox_config = LaunchConfiguration('livox_config')
    start_map = LaunchConfiguration('start_map')
    local_map = LaunchConfiguration('local_map')

    default_livox_config = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'config', 'MID360_config.json')

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='Open RViz (g1_dlio.rviz)')
    declare_robot_model = DeclareLaunchArgument(
        'robot_model', default_value='true',
        description='Show the G1 robot model (RSP + JSP + base_link->pelvis TF)')
    declare_livox_config = DeclareLaunchArgument(
        'livox_config', default_value=default_livox_config,
        description='Livox MID360_config.json (host/lidar IPs + roll:180 mount).')
    declare_start_map = DeclareLaunchArgument(
        'start_map', default_value='true',
        description='Also run dlio_map_node (accumulated /dlio/map_node/map).')
    declare_local_map = DeclareLaunchArgument(
        'local_map', default_value='true',
        description='Run g1_local_map (rolling ground-removed voxel map for the planner).')

    # Livox MID-360 driver in PointCloud2 mode (xfer_format=0) for DLIO. Mirrors
    # the driver's own msg_MID360_launch.py params, but xfer_format 1 -> 0.
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 0},        # 0 = PointCloud2 (PointXYZRTLT) for DLIO
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': livox_config},
            {'cmdline_input_bd_code': 'livox0000000001'},
        ],
    )

    # Shared DLIO node bring-up, real variant: read the driver topics, real config.
    dlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('g1_sim_bridge'), '/launch/dlio.launch.py']),
        launch_arguments={
            'use_sim_time': 'false',
            'config_file': 'dlio_mid360_real.yaml',
            'pointcloud_topic': '/livox/lidar',
            'imu_topic': '/livox/imu',
            'start_map': start_map,
        }.items(),
    )

    # Local rolling voxel map: ground-removed obstacle cloud + costmap from the
    # DLIO deskewed scan, for the a_star_mpc planner. Fast (per-scan) unlike the
    # keyframe-driven /dlio/map_node/map.
    local_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('g1_local_map'), '/launch/local_map.launch.py']),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(local_map),
    )

    robot_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('g1_bringup'), '/launch/robot_model.launch.py']),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(robot_model),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_dlio',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('g1_bringup'), 'rviz', 'g1_dlio.rviz'])],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        declare_rviz,
        declare_robot_model,
        declare_livox_config,
        declare_start_map,
        declare_local_map,
        livox_driver,
        dlio,
        local_map_launch,
        robot_model_launch,
        rviz_node,
    ])
