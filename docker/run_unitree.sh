#!/usr/bin/env bash
# Run the Unitree G1 NATIVE (factory) walking policy — the counterpart to
# run_amo.sh, but the "gait" is the robot's on-board LocoClient controller
# instead of the RoboJuDo AMO joint policy.
#
# Unlike AMO (a separate policy PROCESS that we run), the native gait lives in
# the robot's firmware. So what this script starts is the SDK-side driver that
# talks to it: either the cmd_vel_to_unitree_loco BRIDGE (autonomous — consumes
# the A*+MPC /mpc/cmd_vel and calls LocoClient.SetVelocity) or the standalone
# unitree_gait_test tool (manual teleop / stand). Both apply the same velocity
# smoothing (the high-level analog of AMO's joint filtering) — see
# docs/locomotion/UNITREE_GAIT.md and g1_sim_bridge/unitree_loco.py.
#
# Runs the g1_sim_bridge nodes inside the localization image (which has the built
# workspace); it needs the Unitree Python SDK installed there — see PREREQS.
#
# All extra args are forwarded to the underlying node/tool, e.g.:
#   ./run_unitree.sh                       # stand in place (bring-up, then hold)
#   ./run_unitree.sh --vx 0.3 --duration 5 # (default/JOYSTICK tool) walk fwd 5 s
#   AUTONOMOUS=1 ./run_unitree.sh          # track the A*+MPC /mpc/cmd_vel
#   JOYSTICK=1  ./run_unitree.sh           # keyboard teleop (w/s/a/d/q/e)
#
# MODES (mutually exclusive; pick one, or neither = bring-up & stand):
#   AUTONOMOUS=1 ./run_unitree.sh          # FULL AUTONOMY: run cmd_vel_to_unitree_loco,
#                                          #   which subscribes /mpc/cmd_vel and drives the
#                                          #   native gait. Start the planner FIRST with the
#                                          #   gait bridge OFF so there is only one driver:
#                                          #     ros2 launch a_star_mpc_planner planner.launch.py bridge:=false
#                                          #   then set a goal in RViz.
#   JOYSTICK=1   ./run_unitree.sh          # MANUAL: unitree_gait_test --teleop (keyboard).
#
# PREREQS (one-time, in the localization image/container):
#   colcon build --packages-select g1_sim_bridge          # new entry points
#   export CYCLONEDDS_HOME=/opt/ros/humble
#   pip3 install cyclonedds
#   pip3 install git+https://github.com/unitreerobotics/unitree_sdk2_python.git
#
# SAFETY: the native gait and AMO both command the motors — run only ONE (do NOT
# run run_amo.sh at the same time). Keep the Unitree hardware remote in hand.
#
# Env overrides:
#   NET_IF=enp3s0    ./run_unitree.sh ...  # robot NIC (Unitree SDK DDS)
#   UNITREE_LOCO_FSM=501 ./run_unitree.sh  # walking FSM (501 = expert 3DoF waist)
#   AUTO_BRING_UP=1  ./run_unitree.sh ...  # (AUTONOMOUS) stand the robot up automatically
#   SERVICE=localization ./run_unitree.sh
#   BUILD=1          ./run_unitree.sh ...  # (re)build the image first
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"     # Navigation/docker
cd "${HERE}"

SERVICE="${SERVICE:-localization}"

# NIC the robot is on. Exported so the SDK (and the node's net_if default) bind
# CycloneDDS to the same interface as the AMO flow.
export UNITREE_NET_IFACE="${NET_IF:-${UNITREE_NET_IFACE:-eth0}}"

# The planner/ROS side runs on ROS_DOMAIN_ID=42; the Unitree SDK uses its own DDS
# domain 0 on the robot NIC (handled inside the node). Force 42 for ROS so this
# driver sees /mpc/cmd_vel + /estop from the planner.
export ROS_DOMAIN_ID=42

# Pick `docker compose` (v2) or fall back to `docker-compose` (v1).
if docker compose version >/dev/null 2>&1; then
    DC=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
    DC=(docker-compose)
else
    echo "error: neither 'docker compose' nor 'docker-compose' is available" >&2
    exit 1
fi

if [[ "${BUILD:-0}" == "1" ]]; then
    echo ">> building ${SERVICE} image ..."
    "${DC[@]}" build "${SERVICE}"
fi

if [[ "${AUTONOMOUS:-0}" == "1" && "${JOYSTICK:-0}" == "1" ]]; then
    echo "error: set only one of AUTONOMOUS=1 or JOYSTICK=1, not both" >&2
    exit 1
fi

# Build the ros2 command for the selected mode.
if [[ "${AUTONOMOUS:-0}" == "1" ]]; then
    echo ">> AUTONOMOUS: cmd_vel_to_unitree_loco tracking /mpc/cmd_vel -> native gait ..."
    echo ">>   (start the planner with bridge:=false so this is the only gait driver)"
    NODE=(ros2 run g1_sim_bridge cmd_vel_to_unitree_loco_node --ros-args
          -p "net_if:=${UNITREE_NET_IFACE}"
          -p "control_fsm:=${UNITREE_LOCO_FSM:-501}"
          -p "auto_bring_up:=$([[ "${AUTO_BRING_UP:-0}" == "1" ]] && echo true || echo false)")
elif [[ "${JOYSTICK:-0}" == "1" ]]; then
    echo ">> JOYSTICK: unitree_gait_test keyboard teleop (w/s a/d q/e, space=stop, z=quit) ..."
    NODE=(ros2 run g1_sim_bridge unitree_gait_test --net_if "${UNITREE_NET_IFACE}" --teleop)
else
    echo ">> default: unitree_gait_test bring-up & stand (add --vx/--vy/--yaw to walk) ..."
    NODE=(ros2 run g1_sim_bridge unitree_gait_test --net_if "${UNITREE_NET_IFACE}")
fi

echo ">> running Unitree gait on ${SERVICE} (NIC=${UNITREE_NET_IFACE}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, UNITREE_LOCO_FSM=${UNITREE_LOCO_FSM:-501})"
echo ">> forwarded args: $*"

# -it: the gait test tool + e-stop need a TTY. Source the overlay, then exec the
# node so signals (Ctrl-C) reach it and it damps the robot on exit.
exec "${DC[@]}" run --rm -it \
    -e "UNITREE_NET_IFACE=${UNITREE_NET_IFACE}" \
    -e "UNITREE_LOCO_FSM=${UNITREE_LOCO_FSM:-501}" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    "${SERVICE}" \
    bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && exec ${NODE[*]} $*"
