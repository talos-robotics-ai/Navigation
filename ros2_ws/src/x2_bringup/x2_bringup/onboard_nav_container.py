"""All the on-robot Python navigation nodes in ONE process (= ONE DDS participant).

New participants on the X2's vendor graph (PC2) have made the robot fall, and every ROS process is
one. rmw_fastrtps shares a participant among the nodes of a context, so instead of five processes
(base odom, local map, A*, MPC [, global planner]) this runs them as nodes of one rclpy context in a
MultiThreadedExecutor:

    kilvo_base_odom  /kilvo/aft_mapped_to_init + waist/head joints  -> /x2/odom (pelvis)
    local_voxel_map  /kilvo/cloud_registered + /x2/odom             -> /local_voxel_map/obstacles
    a_star_node      obstacles + odom + /global_goal                -> /a_star/path
    mpc_node         /a_star/path + odom + obstacles                -> /mpc/cmd_vel
    pnp_fsm          the gate: /mpc/cmd_vel -> /x2/cmd_vel_out (only while navigating; zeros otherwise)
    [mc_velocity]    (--nodes ...,mc) /x2/cmd_vel_out -> vendor mc via McLocomotionVelocity: THE ONLY COMMANDER
    [global_planner_node]                                           -> /global_path   (--global-planner)

`--nodes` picks a subset, so the same script can be started twice (two participants, two GIL's) if
one process turns out CPU-bound: `--nodes base_odom,local_map` and `--nodes planner`.

ROS parameters are the usual `--ros-args --params-file` ones, exactly as `ros2 launch` would pass
them (the planner's merged `/**` file, then the local map's, then the odom node's); extra `--ros-args`
after `--` pass through. The nodes run without /rosout and parameter services (fewer DDS endpoints).
"""
import argparse
import os
import sys

import signal

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

ALL = ('base_odom', 'local_map', 'planner', 'fsm', 'mc')
DEFAULT = ('base_odom', 'local_map', 'planner', 'fsm')


def build_nodes(which, global_planner, node_kw):
    nodes = []
    if 'base_odom' in which:
        from .kilvo_base_odom_node import KilvoBaseOdom
        nodes.append(KilvoBaseOdom(**node_kw))
    if 'local_map' in which:
        from g1_local_map.local_voxel_map_node import LocalVoxelMapNode
        nodes.append(LocalVoxelMapNode(**node_kw))
    if 'fsm' in which:
        from x2_box_pnp.pnp_fsm_node import PnpNode
        nodes.append(PnpNode(**node_kw))
    if 'mc' in which:
        from .mc_velocity_node import McVelocity
        nodes.append(McVelocity(**node_kw))   # registers its mc input source here, before the executor exists
    if 'planner' in which:
        from a_star_mpc_planner.a_star_node import AStarNode
        from a_star_mpc_planner.mpc_node import MPCNode
        nodes += [AStarNode(**node_kw), MPCNode(**node_kw)]
        if global_planner:
            from a_star_mpc_planner.global_planner_node import GlobalPlannerNode
            nodes.append(GlobalPlannerNode(**node_kw))
    return nodes


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    ros_args = []
    if '--ros-args' in argv:
        i = argv.index('--ros-args')
        argv, ros_args = argv[:i], argv[i:]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--nodes', default=','.join(DEFAULT), help=f'comma list of {ALL}')
    ap.add_argument('--global-planner', action='store_true')
    ap.add_argument('--params-file', action='append', default=[], help='ROS params yaml (repeatable, later wins)')
    ap.add_argument('--threads', type=int, default=4)
    ap.add_argument('--rosout', action='store_true', help='keep /rosout + parameter services (default: off)')
    a = ap.parse_args(argv)
    which = [w for w in a.nodes.split(',') if w]
    bad = set(which) - set(ALL)
    if bad:
        ap.error(f'unknown --nodes {sorted(bad)}')

    args = list(ros_args) if ros_args else ['--ros-args']
    for f in a.params_file:
        args += ['--params-file', os.path.abspath(f)]
    # Our own SIGINT/SIGTERM handling: rclpy's default handler invalidates the context at once, and
    # mc_velocity must still send INPUT_DELETE (a service call) while shutting down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node_kw = {} if a.rosout else {'enable_rosout': False, 'start_parameter_services': False}
    nodes = build_nodes(which, a.global_planner, node_kw)
    ex = MultiThreadedExecutor(num_threads=a.threads)
    for n in nodes:
        ex.add_node(n)
    nodes[0].get_logger().info(f'onboard nav container: {[n.get_name() for n in nodes]} in one process '
                               f'(one DDS participant), {a.threads} executor threads')
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: ex.shutdown())
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            ex.remove_node(n)
        for n in nodes:
            if hasattr(n, 'on_shutdown'):
                n.on_shutdown()     # e.g. mc_velocity: INPUT_DELETE, context still valid
        for n in nodes:
            n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
