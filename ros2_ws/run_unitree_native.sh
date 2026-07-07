#!/usr/bin/env bash
# Run the Unitree G1 NATIVE (factory) walking-gait driver DIRECTLY on the Jetson —
# the no-Docker counterpart of docker/run_unitree.sh. The "gait" itself lives in the
# robot's firmware (on-board LocoClient); this just runs the g1_sim_bridge SDK-side
# driver that talks to it. The native workspace already builds the entry points; you
# only need the Unitree Python SDK + cyclonedds in the HOST Python (one-time, below).
#
# MODES (mutually exclusive; pick one, or neither = bring-up & stand):
#   AUTONOMOUS=1 ./run_unitree_native.sh   # cmd_vel_to_unitree_loco_node consumes the
#                                          #   A*+MPC /mpc/cmd_vel. Start the planner with
#                                          #   the gait bridge OFF so this is the only driver:
#                                          #     ros2 launch a_star_mpc_planner planner.launch.py bridge:=false
#   JOYSTICK=1   ./run_unitree_native.sh   # unitree_gait_test --teleop (keyboard w/s a/d q/e)
#   (neither)    ./run_unitree_native.sh   # unitree_gait_test bring-up & stand
#
# PREREQS (one-time, in the HOST Python — the container had these, the host may not):
#   export CYCLONEDDS_HOME=/opt/ros/humble
#   pip3 install cyclonedds
#   pip3 install git+https://github.com/unitreerobotics/unitree_sdk2_python.git
#
# SAFETY: the native gait commands the motors. Run only ONE gait driver — do NOT run
# the SONIC controller or run_amo at the same time. Keep the Unitree remote in hand.
# (The native gait balances on the robot's firmware, so the Jetson LowState-starvation
# issue that affects SONIC does not apply here.)
#
# Env overrides:
#   NET_IF=enP8p1s0        robot NIC the Unitree SDK DDS binds to (default enP8p1s0)
#   UNITREE_LOCO_FSM=501   walking FSM (501 = expert 3DoF waist)
#   AUTO_BRING_UP=1        (AUTONOMOUS) stand the robot up automatically
#   ROS_DOMAIN_ID=42       ROS domain for /mpc/cmd_vel + /estop (forced to 42 if unset/0)
set -uo pipefail

# ── Resolve + source the native workspace (same discovery as autonomy.sh) ──
HERE="$(cd "$(dirname "$0")" && pwd)"
WS=""
d="${HERE}"
while [[ "${d}" != "/" ]]; do
    if [[ -f "${d}/install/setup.bash" ]]; then WS="${d}"; break; fi
    d="$(dirname "${d}")"
done
if [[ -z "${WS}" ]]; then
    echo "error: no install/setup.bash at or above ${HERE} — build the workspace first" >&2
    exit 1
fi
# ROS + workspace setup scripts reference unset vars — disable nounset around them.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u

# The Unitree SDK's DDS needs the CycloneDDS C library; ROS ships it here.
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/opt/ros/humble}"

# NIC the robot is on (Unitree SDK DDS binds here) — same knob as the docker path.
export UNITREE_NET_IFACE="${NET_IF:-${UNITREE_NET_IFACE:-enP8p1s0}}"

# ROS side runs on domain 42; the Unitree SDK uses its own DDS domain 0 on the NIC.
if [[ -z "${ROS_DOMAIN_ID:-}" || "${ROS_DOMAIN_ID}" == "0" ]]; then
    export ROS_DOMAIN_ID=42
fi
UNITREE_LOCO_FSM="${UNITREE_LOCO_FSM:-501}"

# ── Preflight: the SDK Python deps must import in the host Python ──
if ! python3 -c 'import unitree_sdk2py, cyclonedds' >/dev/null 2>&1; then
    echo "error: the Unitree Python SDK / cyclonedds are not installed in the host Python." >&2
    echo "Install once, then re-run:" >&2
    echo "    export CYCLONEDDS_HOME=/usr/local   # has lib/libddsc.so + headers on this Jetson" >&2
    echo "    pip3 install cyclonedds" >&2
    echo "    pip3 install git+https://github.com/unitreerobotics/unitree_sdk2_python.git" >&2
    exit 1
fi

if [[ "${AUTONOMOUS:-0}" == "1" && "${JOYSTICK:-0}" == "1" ]]; then
    echo "error: set only one of AUTONOMOUS=1 or JOYSTICK=1, not both" >&2
    exit 1
fi

# ── Build the node command for the selected mode (mirrors docker/run_unitree.sh) ──
if [[ "${AUTONOMOUS:-0}" == "1" ]]; then
    echo ">> AUTONOMOUS: cmd_vel_to_unitree_loco tracking /mpc/cmd_vel -> native gait ..."
    echo ">>   (start the planner with bridge:=false so this is the only gait driver)"
    NODE=(ros2 run g1_sim_bridge cmd_vel_to_unitree_loco_node --ros-args
          -p "net_if:=${UNITREE_NET_IFACE}"
          -p "control_fsm:=${UNITREE_LOCO_FSM}"
          -p "auto_bring_up:=$([[ "${AUTO_BRING_UP:-0}" == "1" ]] && echo true || echo false)")
elif [[ "${JOYSTICK:-0}" == "1" ]]; then
    echo ">> JOYSTICK: unitree_gait_test keyboard teleop (w/s a/d q/e, space=stop, z=quit) ..."
    NODE=(ros2 run g1_sim_bridge unitree_gait_test --net_if "${UNITREE_NET_IFACE}" --teleop)
else
    echo ">> default: unitree_gait_test bring-up & stand (add --vx/--vy/--yaw to walk) ..."
    NODE=(ros2 run g1_sim_bridge unitree_gait_test --net_if "${UNITREE_NET_IFACE}")
fi

echo ">> running Unitree gait NATIVELY (NIC=${UNITREE_NET_IFACE}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, FSM=${UNITREE_LOCO_FSM})"
echo ">> forwarded args: $*"
# exec so Ctrl-C reaches the node and it damps the robot on exit.
exec "${NODE[@]}" "$@"
