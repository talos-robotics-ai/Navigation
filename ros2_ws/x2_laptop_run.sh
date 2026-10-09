#!/usr/bin/env bash
# LAPTOP half of the PC2/laptop navigation split: relay client + local map + A* + MPC + pnp FSM.
# PC2 runs only KILVO + base odom + nav_relay_server (run_nav_on_pc2.sh start [mc]); this connects to it over
# TCP 10.0.1.41:5596 and reconnects by itself.
#
# SAFETY (CRITICAL): nothing started here may reach the robot's DDS network -- new participants on PC2's graph
# have made the robot fall. Domain 77, discovery LOCALHOST only, vendor DDS profile variables unset; the only
# path to the robot is the TCP link.
#
# One-time setup, venv for casadi: see x2_run.sh.   Build: colcon build --symlink-install --packages-select
#   a_star_mpc_planner g1_local_map x2_box_pnp x2_bringup
# Usage: ./x2_laptop_run.sh [launch args, e.g. pc2_host:=10.0.1.41 global_planner:=true foxglove:=false]
# Then, from any terminal:  ros2_ws/x2_nav go | reset | estop | clear     (file triggers in ~/.x2_nav/run)
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI RMW_FASTRTPS_USE_QOS_FROM_XML ROS_STATIC_PEERS
export ROS_DOMAIN_ID="${X2_ROS_DOMAIN_ID:-77}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
set +u
source /opt/ros/jazzy/setup.bash
source "$HERE/install/setup.bash"
export PYTHONPATH="$HERE/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
echo "x2_laptop_run: ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE"
exec ros2 launch x2_bringup x2_laptop_nav.launch.py "$@"
