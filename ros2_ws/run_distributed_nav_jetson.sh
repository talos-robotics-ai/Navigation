#!/usr/bin/env bash
# Jetson side of the distributed-nav split (docs/planning/DISTRIBUTED_NAV_PLAN.md,
# Variant A): perception + the SONIC gait bridge + the ROS<->ZMQ relay. A*/MPC run
# OFF-BOARD on the laptop, so the Orin only carries DLIO + local map + the policy.
#
# ORDER (see the doc): start the SONIC controller FIRST in its own terminal, then run
# this, then start the LAPTOP side, then set a /global_goal in Foxglove.
#
#   LAPTOP_IP=10.251.100.88 ./run_distributed_nav_jetson.sh
#
# Everything here is pinned to cores 0,1 (the controller owns 2-5). e-stop runs in the
# FOREGROUND of this terminal (s=stop g=go q=quit) — the primary software stop stays
# on the robot, never only on the laptop.
set -uo pipefail
LAPTOP_IP="${LAPTOP_IP:?set LAPTOP_IP=<laptop wifi ip>  (e.g. LAPTOP_IP=10.251.100.88)}"
HOLD_ARMS="${HOLD_ARMS:-true}"

HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE"
# ROS/colcon setup scripts reference unset vars — disable nounset around BOTH sources.
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_DOMAIN_ID=42
NAV=(taskset -c "${NAV_CPUS:-0,1}")

cleanup() {
  echo ">> stopping distributed-nav (Jetson) ..."
  tmux kill-session -t dnav_loc 2>/dev/null || true
  tmux kill-session -t dnav_gaitbridge 2>/dev/null || true
  tmux kill-session -t dnav_zmq 2>/dev/null || true
  pkill -9 -f "[l]ivox_ros_driver2_node" 2>/dev/null || true
  exit 0
}
trap cleanup INT TERM

# 1. Perception: DLIO + local_voxel_map (NO A*/MPC here anymore)
echo ">> [1/3] localization (DLIO + local map), pinned to CPUs ${NAV_CPUS:-0,1} ..."
tmux kill-session -t dnav_loc 2>/dev/null || true
tmux new-session -d -s dnav_loc "${NAV[*]} ros2 launch g1_bringup real_localization.launch.py rviz:=false > /tmp/dnav_localization.log 2>&1"
sleep 4   # DLIO IMU/gravity init — keep the robot still

# 2. SONIC gait bridge: subscribes /mpc/cmd_vel (republished by the relay below) and
#    drives the policy over ZMQ :5556. hold_arms keeps the arms still.
echo ">> [2/3] SONIC gait bridge (/mpc/cmd_vel -> policy ZMQ :5556) ..."
tmux kill-session -t dnav_gaitbridge 2>/dev/null || true
tmux new-session -d -s dnav_gaitbridge "${NAV[*]} ros2 run g1_sim_bridge cmd_vel_to_sonic_node --ros-args -p hold_arms:=${HOLD_ARMS} > /tmp/dnav_gaitbridge.log 2>&1"

# 3. ROS<->ZMQ relay: ship odom/TF/obstacles (+ viz) to the laptop planner, receive cmd_vel back.
# Base = what the PLANNER needs + the 2D costmap (cheap, ~145 KB/s). We deliberately do
# NOT relay /dlio/odom_node/path: DLIO republishes the ENTIRE accumulating trajectory every
# scan, so it grows without bound and hit ~6 MB/s here — enough to saturate WiFi and starve
# everything else (and the network softirqs can jitter the controller's LowState thread).
# RELAY_CLOUDS=1 ships the BOUNDED lidar clouds (live deskewed scan + local voxel grid) so
# CloudRegistered + LocalVoxelMap show in g1_dlio.rviz — ~2.5 MB/s, WiFi-safe. OFF by default.
# RELAY_MAP=1 ALSO ships /dlio/map_node/map, which ACCUMULATES without bound (same failure
# class as the path) and creeps up WiFi over a long run — enable only for a quick look.
SEND_TOPICS="/dlio/odom_node/odom:nav_msgs/msg/Odometry,/tf:tf2_msgs/msg/TFMessage,/tf_static:tf2_msgs/msg/TFMessage,/local_voxel_map/obstacles:sensor_msgs/msg/PointCloud2,/local_voxel_map/costmap:nav_msgs/msg/OccupancyGrid"
if [ "${RELAY_CLOUDS:-0}" = "1" ]; then
  echo ">> RELAY_CLOUDS=1: ALSO shipping deskewed scan + voxel grid (~2.5 MB/s)"
  SEND_TOPICS="${SEND_TOPICS},/dlio/odom_node/pointcloud/deskewed:sensor_msgs/msg/PointCloud2,/local_voxel_map/voxel_grid:sensor_msgs/msg/PointCloud2"
fi
if [ "${RELAY_MAP:-0}" = "1" ]; then
  echo ">> RELAY_MAP=1: ALSO shipping the accumulated 3D map (UNBOUNDED — watch WiFi)"
  SEND_TOPICS="${SEND_TOPICS},/dlio/map_node/map:sensor_msgs/msg/PointCloud2"
fi
echo ">> [3/3] ROS<->ZMQ relay -> laptop ${LAPTOP_IP} (PUB :5601, SUB laptop:5602)  clouds=${RELAY_CLOUDS:-0} map=${RELAY_MAP:-0} ..."
tmux kill-session -t dnav_zmq 2>/dev/null || true
tmux new-session -d -s dnav_zmq "${NAV[*]} python3 ${HERE}/zmq_ros_bridge.py \
  --send '${SEND_TOPICS}' \
  --pub-bind 'tcp://*:5601' \
  --recv '/mpc/cmd_vel:geometry_msgs/msg/Twist' \
  --sub-connect 'tcp://${LAPTOP_IP}:5602' > /tmp/dnav_zmq.log 2>&1"

echo ""
echo ">> Jetson up. Logs: /tmp/dnav_localization.log /tmp/dnav_gaitbridge.log /tmp/dnav_zmq.log"
echo ">> Now start the LAPTOP:  JETSON_IP=<this jetson ip> ./run_distributed_nav_laptop.sh"
echo ">> Then set a goal in Foxglove (Publish -> /global_goal)."
echo ""
echo ">> SAFETY E-STOP active in THIS terminal:  s=stop  g=go  q=quit"
"${NAV[@]}" ros2 run g1_sim_bridge estop_keyboard_node
cleanup
