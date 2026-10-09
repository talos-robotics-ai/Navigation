"""KILVO + the navigation stack, both ON the X2's PC2 (ROS 2 Humble, vendor DDS graph, default domain).

  ros2 launch x2_bringup x2_onboard.launch.py            (use ~/talos_nav_ws/run_nav_on_pc2.sh start)

  kilvo mapping_x2.launch.py (arg kilvo:=true; the COPY in ~/talos_nav_ws/src/kilvo)
        x2_leg_kinematics + KILVO  ->  /kilvo/aft_mapped_to_init, /kilvo/cloud_registered, /x2/foot_state
  x2_onboard_nav (ONE process, ONE DDS participant -- see onboard_nav_container.py)
        kilvo_base_odom -> /x2/odom      local_voxel_map -> /local_voxel_map/obstacles
        a_star_node + mpc_node -> /a_star/path, /mpc/cmd_vel
  Send a goal:  ros2 topic pub --once /global_goal geometry_msgs/PoseStamped ...   (frame odom = KILVO world)

walker:=none (default): NOTHING COMMANDS THE ROBOT. walker:=mc adds mc_velocity_node: pnp_fsm gate -> /x2/cmd_vel_out
-> vendor mc (input source talos_nav, priority 64 < the PS5 pad's 65, no mode changes).

DDS: the vendor profile on every node (without it PC2 does not see the HAL topics), the default
domain. Start KILVO first, the nav process a few seconds later: one discovery burst at a time.
`split:=true` runs the nav nodes as two processes (2 participants) if one process is CPU-bound.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VENDOR_DDS_PROFILE = '/agibot/software/entry/cfg/ros_dds_configuration.xml'


def merged_planner_params(walker='none'):
    """planner defaults <- x2_planner_params <- x2_onboard_planner [<- x2_planner_params_mc] (later wins), one file."""
    planner = get_package_share_directory('a_star_mpc_planner')
    share = get_package_share_directory('x2_bringup')
    with open(os.path.join(planner, 'config', 'planner_params_default.yaml')) as f:
        merged = yaml.safe_load(f)
    names = ['x2_planner_params.yaml', 'x2_onboard_planner.yaml'] + (['x2_planner_params_mc.yaml'] if walker == 'mc' else [])
    for name in names:
        with open(os.path.join(share, 'config', name)) as f:
            merged['/**']['ros__parameters'].update(yaml.safe_load(f)['/**']['ros__parameters'])
    path = os.path.join(tempfile.mkdtemp(prefix='x2_onboard_'), 'planner_merged.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(merged, f)
    return path


def nav_processes(context):
    share = get_package_share_directory('x2_bringup')
    cfg = lambda n: os.path.join(share, 'config', n)  # noqa: E731
    walker = LaunchConfiguration('walker').perform(context).lower()
    if walker not in ('none', 'mc'):
        raise RuntimeError(f"walker:={walker!r}: none | mc")
    params = ['--params-file', merged_planner_params(walker),
              '--params-file', os.path.join(get_package_share_directory('x2_box_pnp'), 'config', 'pnp_params.yaml'),
              '--params-file', cfg('x2_onboard_fsm.yaml'),
              '--params-file', cfg('x2_onboard_local_map.yaml'),
              '--params-file', cfg('x2_onboard_base_odom.yaml'),
              '--params-file', cfg('x2_onboard_mc.yaml')]
    gp = LaunchConfiguration('global_planner').perform(context).lower() in ('true', '1')
    nav = 'planner,fsm' + (',mc' if walker == 'mc' else '')
    if LaunchConfiguration('split').perform(context).lower() in ('true', '1'):
        groups = ['base_odom,local_map', nav]
    else:
        groups = ['base_odom,local_map,' + nav]
    actions = [Node(package='x2_bringup', executable='x2_onboard_nav', output='screen',
                    # No `name=`: launch_ros would add a GLOBAL `-r __node:=name` renaming every node of the process.
                    arguments=['--nodes', nodes, *(['--global-planner'] if gp else []), *params])
               for nodes in groups]
    if walker == 'mc':
        actions.insert(0, LogInfo(msg=LOUD_MC))
    return actions


LOUD_MC = (
    '\n' + '!' * 78 + '\n'
    '!! walker:=mc  THIS WILL COMMAND THE ROBOT\n'
    '!! mc_velocity -> /aima/mc/locomotion/velocity, source talos_nav, priority 64 (PS5 pad = 65 overrides).\n'
    '!! Path: planner /mpc/cmd_vel -> pnp_fsm gate -> /x2/cmd_vel_out -> mc. Only while the FSM is\n'
    '!! navigating (after /pnp/start); caps vx 0.5 vy 0.3 wz 0.5; zeros when stale (0.3 s) or on /estop.\n'
    '!! No mode change is ever requested: put the robot in STAND_DEFAULT with the pad first.\n'
    + '!' * 78)


def generate_launch_description():
    kilvo_share = None
    try:
        kilvo_share = get_package_share_directory('kilvo')
    except Exception:   # laptop without the kilvo copy: nav-only launches still work (kilvo:=false)
        pass
    return LaunchDescription([
        DeclareLaunchArgument('kilvo', default_value='true', description='start the KILVO copy (mapping_x2.launch.py)'),
        DeclareLaunchArgument('kilvo_config', default_value='x2.yaml'),
        DeclareLaunchArgument('legs', default_value='true'),
        DeclareLaunchArgument('walker', default_value='none', description='none: NOTHING commands the robot | mc: vendor walker via mc_velocity_node'),
        DeclareLaunchArgument('global_planner', default_value='false'),
        DeclareLaunchArgument('split', default_value='false', description='two nav processes instead of one'),
        DeclareLaunchArgument('nav_delay', default_value='8.0', description='s between KILVO and the nav process starting'),
        DeclareLaunchArgument('dds_profile', default_value=VENDOR_DDS_PROFILE),
        DeclareLaunchArgument('use_dds_profile', default_value='true', description='false: keep the environment DDS settings'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', LaunchConfiguration('dds_profile'),
                               condition=IfCondition(LaunchConfiguration('use_dds_profile'))),
        *([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(kilvo_share, 'launch', 'mapping_x2.launch.py')),
            launch_arguments={'config': LaunchConfiguration('kilvo_config'), 'legs': LaunchConfiguration('legs'),
                              'dds_profile': LaunchConfiguration('dds_profile'),
                              'use_dds_profile': LaunchConfiguration('use_dds_profile')}.items(),
            condition=IfCondition(LaunchConfiguration('kilvo')))] if kilvo_share else []),
        TimerAction(period=LaunchConfiguration('nav_delay'), actions=[OpaqueFunction(function=nav_processes)]),
    ])
