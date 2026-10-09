"""Helpers shared by the x2 launch files (importable: launch files themselves are not modules)."""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

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


LOUD_MC = (
    '\n' + '!' * 78 + '\n'
    '!! walker:=mc  THIS WILL COMMAND THE ROBOT\n'
    '!! mc_velocity -> /aima/mc/locomotion/velocity, source talos_nav, priority 64 (PS5 pad = 65 overrides).\n'
    '!! Path: planner /mpc/cmd_vel -> pnp_fsm gate -> /x2/cmd_vel_out -> mc (relay mode: gate on the laptop,\n'
    '!! TCP 5596 -> nav_relay_server -> /x2/cmd_vel_out). Only while the FSM is\n'
    '!! navigating (after /pnp/start); caps vx 0.5 vy 0.3 wz 0.5; zeros when stale (0.3 s) or on /estop.\n'
    '!! No mode change is ever requested: put the robot in STAND_DEFAULT with the pad first.\n'
    + '!' * 78)
