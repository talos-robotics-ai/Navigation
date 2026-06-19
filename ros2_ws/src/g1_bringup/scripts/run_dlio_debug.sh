#!/usr/bin/env bash
# Clean-restart wrapper for the DLIO debug session.
#
# `ros2 launch` orphans nodes on a hard Ctrl-C; they pile up across runs and
# cause duplicate-name warnings + "lidar loop back" (multiple relays/DLIOs on
# the same topics with different clocks). This kills any stale stack from a
# previous run, then launches g1_bringup dlio_debug.launch.py fresh.
#
# Run (inside the localization container):
#   ros2 run g1_bringup run_dlio_debug.sh
#   ros2 run g1_bringup run_dlio_debug.sh rviz:=false robot_model:=false
#
# NOTE: this only cleans the ROS-side stack in THIS container. Isaac Sim runs on
# the host and is left untouched (it keeps publishing /livox/lidar + /clock).
set -uo pipefail

# Best-effort source so it works whether or not the caller sourced the ws.
set +u
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
set -u

PROCS=(
  dlio_odom_node
  dlio_map_node
  isaac_dlio_qos_relay
  robot_state_publisher
  joint_state_publisher
  static_transform_publisher
  rviz2
)

echo "[run_dlio_debug] stopping any stale stack from a previous run..."
for p in "${PROCS[@]}"; do pkill -2 -f "$p" 2>/dev/null || true; done
sleep 2
# Force-kill anything that ignored SIGINT.
for p in "${PROCS[@]}"; do pkill -9 -f "$p" 2>/dev/null || true; done
sleep 1

# Refresh the ROS 2 graph so duplicate-name warnings clear.
ros2 daemon stop  >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "[run_dlio_debug] launching: ros2 launch g1_bringup dlio_debug.launch.py $*"
exec ros2 launch g1_bringup dlio_debug.launch.py "$@"
