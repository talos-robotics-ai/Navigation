"""LAPTOP half of the PC2/laptop navigation split (use x2_laptop_run.sh: domain 77, localhost only).

  x2_onboard_nav --nodes relay_client,local_map,planner,fsm,crate_conv   (one process)
     crate_conv: /fpose/crate_pose of the LAPTOP detector (rgbd_head_front) -> /x2/crate_pose (odom), using the
                 head-camera pose PC2 streams (/x2/cam_pose, also TF odom->rgbd_head_front)
     nav_relay_client  <-TCP 10.0.1.41:5596->  nav_relay_server on PC2 (x2_onboard.launch.py mode:=relay)
         /x2/odom, /x2/crate_pose, /kilvo/cloud_registered_ds   in;   /x2/cmd_vel_out (20 Hz), /estop   out
     g1_local_map (cloud_registered_ds -> /local_voxel_map/obstacles), a_star + mpc, pnp_fsm (the gate)
  [foxglove_bridge on :8765 if installed (foxglove:=true)]

NOTHING here reaches the robot's DDS network; the only path to the robot is the TCP link. Whether the robot
walks is decided on PC2 (run_nav_on_pc2.sh start mc); the FSM gate forwards /mpc/cmd_vel only after `x2_nav go`.
"""
import os
import subprocess
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from x2_bringup.launch_utils import merged_planner_params


def processes(context):
    share = get_package_share_directory('x2_bringup')
    cfg = lambda n: os.path.join(share, 'config', n)  # noqa: E731
    host = LaunchConfiguration('pc2_host').perform(context)
    port = LaunchConfiguration('pc2_port').perform(context)
    detector_on_pc2 = LaunchConfiguration('detector_on_pc2').perform(context).lower() in ('true', '1')
    gp = LaunchConfiguration('global_planner').perform(context).lower() in ('true', '1')
    walker = LaunchConfiguration('walker').perform(context).lower()
    if walker not in ('mc', 'onrobot', 'none'):
        raise RuntimeError(f"walker:={walker!r}: mc | onrobot | none")
    link_params = os.path.join(tempfile.mkdtemp(prefix='x2_laptop_'), 'link.yaml')   # last file: wins over the config
    with open(link_params, 'w') as f:
        yaml.safe_dump({'nav_relay_client': {'ros__parameters': {'host': host, 'port': int(port)}},
                        # onrobot: the FSM runs the teleop's hanging-to-walking bring-up on the walker's phases
                        'pnp_fsm': {'ros__parameters': {'bringup': walker == 'onrobot'}}}, f)
    # mc: the vendor mc walks the robot (sidestep allowed). onrobot: the AnyTrack policy (no vy: X2 base overlay).
    params = ['--params-file', merged_planner_params('mc' if walker == 'mc' else 'none'),
              '--params-file', os.path.join(get_package_share_directory('x2_box_pnp'), 'config', 'pnp_params.yaml'),
              '--params-file', cfg('x2_laptop_fsm.yaml'),
              '--params-file', cfg('x2_laptop_relay_client.yaml'),
              '--params-file', link_params]
    actions = [Node(package='x2_bringup', executable='x2_onboard_nav', output='screen',
                    arguments=['--nodes', 'relay_client,local_map,planner,fsm' + ('' if detector_on_pc2 else ',crate_conv'),
                               *(['--global-planner'] if gp else []), *params])]
    if LaunchConfiguration('foxglove').perform(context).lower() in ('true', '1'):
        have = subprocess.run(['ros2', 'pkg', 'prefix', 'foxglove_bridge'], capture_output=True).returncode == 0
        if have:
            actions.append(Node(package='foxglove_bridge', executable='foxglove_bridge', name='nav_foxglove', output='screen',
                                parameters=[{'port': 8765, 'address': '127.0.0.1',
                                             'topic_whitelist': ['/x2/.*', '/local_voxel_map/.*', '/pnp/.*', '/mpc/.*',
                                                                 '/navigation/.*', '/global_goal', '/global_path', '/a_star/.*',
                                                                 '/kilvo/cloud_registered_ds', '/estop', '/fpose/.*', '/tf', '/tf_static'],
                                             'service_whitelist': ['^$'], 'param_whitelist': ['^$']}]))
        else:
            print('x2_laptop_nav: foxglove_bridge not installed (sudo apt install ros-jazzy-foxglove-bridge); skipping')
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pc2_host', default_value='10.0.1.41'),
        DeclareLaunchArgument('pc2_port', default_value='5596'),
        DeclareLaunchArgument('detector_on_pc2', default_value='false', description='true: /x2/crate_pose comes from PC2 (no laptop crate_to_odom)'),
        DeclareLaunchArgument('global_planner', default_value='false'),
        DeclareLaunchArgument('walker', default_value='mc', description='mc: vendor walker | onrobot: on-robot AnyTrack walker (FSM bring-up on, no vy) | none: watch only -- the robot is driven by something else (e.g. the PS5 pad), PC2 consumes nothing (no vy, no bring-up)'),
        DeclareLaunchArgument('foxglove', default_value='true', description='foxglove_bridge on ws://localhost:8765 if installed'),
        OpaqueFunction(function=processes),
    ])
