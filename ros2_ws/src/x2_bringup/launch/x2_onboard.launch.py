"""KILVO + the navigation stack, both ON the X2's PC2 (ROS 2 Humble, vendor DDS graph, default domain).

  ros2 launch x2_bringup x2_onboard.launch.py            (use ~/talos_nav_ws/run_nav_on_pc2.sh start)

  kilvo mapping_x2.launch.py (arg kilvo:=true; the COPY in ~/talos_nav_ws/src/kilvo)
        x2_leg_kinematics + KILVO  ->  /kilvo/aft_mapped_to_init, /kilvo/cloud_registered, /x2/foot_state
  mode:=relay (DEFAULT) x2_onboard_nav --nodes base_odom,relay[,mc]  (ONE process, ONE DDS participant)
        kilvo_base_odom -> /x2/odom, /x2/crate_pose      nav_relay_server: TCP :5596 <-> the laptop, which runs
        local map + A* + MPC + FSM (x2_laptop_nav.launch.py); its commands come back as /x2/cmd_vel_out -> mc.
  mode:=full  x2_onboard_nav (ONE process, ONE DDS participant -- see onboard_nav_container.py)
        kilvo_base_odom -> /x2/odom      local_voxel_map -> /local_voxel_map/obstacles
        a_star_node + mpc_node -> /a_star/path, /mpc/cmd_vel      (~1 core of PC2: that is why relay is the default)
  Send a goal (full mode):  ros2 topic pub --once /global_goal geometry_msgs/PoseStamped ...   (frame odom = KILVO world)

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

from x2_bringup.launch_utils import LOUD_MC, merged_planner_params

VENDOR_DDS_PROFILE = '/agibot/software/entry/cfg/ros_dds_configuration.xml'


def nav_processes(context):
    share = get_package_share_directory('x2_bringup')
    cfg = lambda n: os.path.join(share, 'config', n)  # noqa: E731
    walker = LaunchConfiguration('walker').perform(context).lower()
    if walker not in ('none', 'mc', 'onrobot'):
        raise RuntimeError(f"walker:={walker!r}: none | mc | onrobot")

    mode = LaunchConfiguration('mode').perform(context).lower()
    if mode not in ('relay', 'full'):
        raise RuntimeError(f"mode:={mode!r}: relay | full")
    if walker == 'onrobot' and mode != 'relay':
        raise RuntimeError('walker:=onrobot needs mode:=relay (the relay is what talks to the walker)')
    gp = LaunchConfiguration('global_planner').perform(context).lower() in ('true', '1')
    if mode == 'relay':
        # PC2 keeps only the cheap things: base odom + the TCP relay (+ the mc link). The planner stack
        # runs on the laptop (x2_laptop_nav.launch.py) and talks to nav_relay_server over TCP 5596.
        params = ['--params-file', cfg('x2_onboard_base_odom.yaml'),
                  '--params-file', cfg('x2_onboard_relay.yaml'),
                  '--params-file', cfg('x2_onboard_mc.yaml')]
        if LaunchConfiguration('crate_on_pc2').perform(context).lower() not in ('true', '1'):
            # the detector runs on the laptop: no /fpose/crate_pose -> /x2/crate_pose on PC2 (the laptop converts it)
            off = os.path.join(tempfile.mkdtemp(prefix='x2_onboard_'), 'crate_off.yaml')
            with open(off, 'w') as f:
                yaml.safe_dump({'kilvo_base_odom': {'ros__parameters': {'crate_topic': ''}}}, f)
            params += ['--params-file', off]
        ihz = LaunchConfiguration('image_hz').perform(context)
        if ihz:
            # head RGB-D pairs/s for the laptop box tracker (camera max 10; > ~5 needs the tracker's --mode track)
            fhz = os.path.join(tempfile.mkdtemp(prefix='x2_onboard_'), 'image_hz.yaml')
            with open(fhz, 'w') as f:
                yaml.safe_dump({'nav_image_server': {'ros__parameters': {'image_hz': float(ihz)}}}, f)
            params += ['--params-file', fhz]
        if walker == 'onrobot':
            # the relay also drives the on-robot RL walker (127.0.0.1:8770) and returns its phase to the laptop
            on = os.path.join(tempfile.mkdtemp(prefix='x2_onboard_'), 'walker_on.yaml')
            with open(on, 'w') as f:
                yaml.safe_dump({'nav_relay_server': {'ros__parameters': {'walker': 'onrobot'}}}, f)
            params += ['--params-file', on]
        groups = ['base_odom,relay' + (',image' if LaunchConfiguration('images').perform(context).lower() in ('true', '1') else '')
                  + (',mc' if walker == 'mc' else '')]
        gp = False
    else:
        params = ['--params-file', merged_planner_params(walker),
                  '--params-file', os.path.join(get_package_share_directory('x2_box_pnp'), 'config', 'pnp_params.yaml'),
                  '--params-file', cfg('x2_onboard_fsm.yaml'),
                  '--params-file', cfg('x2_onboard_local_map.yaml'),
                  '--params-file', cfg('x2_onboard_base_odom.yaml'),
                  '--params-file', cfg('x2_onboard_mc.yaml')]
        nav = 'planner,fsm' + (',mc' if walker == 'mc' else '')
        if LaunchConfiguration('split').perform(context).lower() in ('true', '1'):
            groups = ['base_odom,local_map', nav]
        else:
            groups = ['base_odom,local_map,' + nav]
    actions = [Node(package='x2_bringup', executable='x2_onboard_nav', output='screen',
                    # No `name=`: launch_ros would add a GLOBAL `-r __node:=name` renaming every node of the process.
                    arguments=['--nodes', nodes, *(['--global-planner'] if gp else []),
                               *(['--threads', '1'] if mode == 'relay' and walker != 'mc' else []), *params])
               for nodes in groups]
    # C++ rate limiter in front of the Python container: KILVO odom (~930 Hz) and the HAL waist/head joint groups
    # (~1 kHz each) would otherwise wake Python thousands of times a second (100% of an Orin core).
    actions.insert(0, Node(package='x2_throttle', executable='x2_throttle', output='screen',
                           ros_arguments=['--disable-rosout-logs'],
                           parameters=[{'inputs': ['/kilvo/aft_mapped_to_init', '/aima/hal/joint/waist/state',
                                                   '/aima/hal/joint/head/state'],
                                        'outputs': ['/x2/throttled/kilvo_odom', '/x2/throttled/joint/waist',
                                                    '/x2/throttled/joint/head'],
                                        'types': ['nav_msgs/msg/Odometry', 'aimdk_msgs/msg/JointStateArray',
                                                  'aimdk_msgs/msg/JointStateArray'],
                                        'rates': [50.0, 25.0, 25.0]}]))
    if walker == 'mc':
        actions.insert(0, LogInfo(msg=LOUD_MC))
    return actions


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
        DeclareLaunchArgument('walker', default_value='none', description='none: NOTHING commands the robot | mc: vendor walker via mc_velocity_node | onrobot: on-robot RL walker via the relay (mode:=relay only)'),
        DeclareLaunchArgument('mode', default_value='relay', description='relay: base odom + TCP relay to the laptop planner (cheap) | full: whole stack on PC2'),
        DeclareLaunchArgument('crate_on_pc2', default_value='false', description='relay mode: keep PC2-side /fpose/crate_pose -> /x2/crate_pose (detector still on PC2)'),
        DeclareLaunchArgument('image_hz', default_value='', description='override nav_image_server image_hz (default: config, 4)'),
        DeclareLaunchArgument('images', default_value='true', description='relay mode: head RGB-D server on :5597 (idle without a client)'),
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
