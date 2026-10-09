"""X2 navigation + box pick-and-place on the laptop (ROS domain 77, localhost only -- see x2_run.sh).

  robojudo_link_node   TCP 127.0.0.1:8770 <-> /x2/odom, TF, /x2/crate_pose, /x2/cmd_vel_out, arm/hand
  dummy_obstacles_node far point so A* will plan
  a_star_mpc_planner   (bridge:=false, rviz:=false) + x2 overlay;  /mpc/cmd_vel
  pnp_fsm_node         the ONLY thing that forwards /mpc/cmd_vel to /x2/cmd_vel_out (the FSM gate)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('x2_bringup')
    planner = get_package_share_directory('a_star_mpc_planner')
    pnp = get_package_share_directory('x2_box_pnp')
    domain = LaunchConfiguration('ros_domain_id')

    # The planner launch takes ONE params_file, so merge default yaml + X2 overlay (overlay wins).
    import tempfile
    import yaml
    with open(os.path.join(planner, 'config', 'planner_params_default.yaml')) as f:
        merged = yaml.safe_load(f)
    with open(os.path.join(share, 'config', 'x2_planner_params.yaml')) as f:
        merged['/**']['ros__parameters'].update(yaml.safe_load(f)['/**']['ros__parameters'])
    merged_path = os.path.join(tempfile.mkdtemp(prefix='x2_planner_'), 'planner_merged.yaml')
    with open(merged_path, 'w') as f:
        yaml.safe_dump(merged, f)

    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id', default_value='77'),
        DeclareLaunchArgument('pnp', default_value='true', description='run the pick-and-place FSM'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('dummy_obstacles', default_value='true',
                              description='false when a real cloud feeds the obstacle topic (next step: '
                              '/kilvo/cloud_registered relayed to the laptop; set obstacle_topic in '
                              'x2_planner_params.yaml and ground-filter it)'),
        DeclareLaunchArgument('global_planner', default_value='false'),
        Node(package='x2_bringup', executable='robojudo_link_node', name='robojudo_link', output='screen'),
        Node(package='x2_bringup', executable='dummy_obstacles_node', name='dummy_obstacles', output='screen',
             condition=IfCondition(LaunchConfiguration('dummy_obstacles'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(planner, 'launch', 'planner.launch.py')),
            launch_arguments={
                'bridge': 'false', 'rviz': 'false',
                'global_planner': LaunchConfiguration('global_planner'),
                'ros_domain_id': domain,
                'params_file': merged_path,
            }.items()),
        Node(package='x2_box_pnp', executable='pnp_fsm_node', name='pnp_fsm', output='screen',
             condition=IfCondition(LaunchConfiguration('pnp')),
             parameters=[os.path.join(pnp, 'config', 'pnp_params.yaml'),
                         {'auto_start': LaunchConfiguration('auto_start')}]),
    ])
