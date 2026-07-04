#!/usr/bin/env bash
# Run the SONIC whole-body walking policy bridge — the counterpart to
# run_amo.sh / run_unitree.sh, for NVIDIA GEAR-SONIC as the low-level gait.
#
# Like the Unitree native gait (and UNLIKE AMO, a policy PROCESS we run here),
# the SONIC policy runtime is EXTERNAL to this repo: the C++/TensorRT deploy
# controller (g1_deploy_onnx_ref) runs separately and SUBs a ZMQ socket. So what
# this script starts is the ROS 2 -> SONIC BRIDGE (cmd_vel_to_sonic_node), which
# PUBs planner commands to that controller over ZMQ :5556. It runs in the
# localization image (which has the built workspace). See docs/locomotion/SONIC_POLICY.md.
#
# The bridge converts the MPC's BODY-frame /mpc/cmd_vel into SONIC's WORLD-frame
# movement/facing using measured DLIO yaw (closed loop) — the only thing it does
# beyond the AMO/Unitree bridges. It also honours the same /estop latch + cmd
# watchdog.
#
# All extra args are forwarded to the node, e.g. -p speed_gain:=1.25 :
#   ./run_sonic.sh                          # MANUAL: bridge /cmd_vel -> SONIC (idle until teleop)
#   AUTONOMOUS=1 ./run_sonic.sh             # track the A*+MPC /mpc/cmd_vel
#   JOYSTICK=1  ./run_sonic.sh              # MANUAL: teleop via /cmd_vel (run joy_to_cmdvel too)
#   HOLD_ARMS=1 ARM_PRESET=carry ./run_sonic.sh   # pin the 17-DOF upper body (carry pose)
#
# MODES (mutually exclusive; pick one, or neither = manual on /cmd_vel):
#   AUTONOMOUS=1 ./run_sonic.sh  # FULL AUTONOMY: cmd_vel_to_sonic_node subscribes /mpc/cmd_vel
#                                #   and drives SONIC. Start the planner FIRST with the gait
#                                #   bridge OFF so this is the only driver:
#                                #     ros2 launch a_star_mpc_planner planner.launch.py bridge:=false
#                                #   then set a goal in RViz.
#   JOYSTICK=1   ./run_sonic.sh  # MANUAL: the bridge subscribes /cmd_vel; publish /cmd_vel from a
#                                #   teleop source in another terminal, e.g.
#                                #     ros2 run g1_sim_bridge joy_to_cmdvel_node
#                                #   (the G1 pad) or any keyboard teleop.
#
# PREREQ (one-time, in the localization image/container):
#   build_ws   # or: colcon build --packages-select g1_sim_bridge  (new entry point)
#   pyzmq is baked into Dockerfile.localization; BUILD=1 rebuilds if it is missing.
#
# The SONIC deploy controller (C++/TensorRT) is NOT started here — bring it up
# separately (it SUBs tcp://<host>:5556). It is not part of this repo.
#
# SAFETY: SONIC, AMO, and the native gait all command the motors — run only ONE
# (do NOT run run_amo.sh / run_unitree.sh at the same time). Keep the hardware
# e-stop in hand; /estop + the cmd watchdog zero the robot.
#
# Env overrides:
#   SONIC_HOST=*     ./run_sonic.sh ...   # ZMQ PUB bind address (default *, all NICs)
#   SONIC_PORT=5556  ./run_sonic.sh ...   # ZMQ PUB port (the SONIC controller SUBs it)
#   CMD_VEL_TOPIC=/x ./run_sonic.sh ...   # override the command topic for the selected mode
#   HOLD_ARMS=1 / ARM_PRESET=carry        # pin the upper body while walking (default|carry)
#   SERVICE=localization ./run_sonic.sh
#   BUILD=1          ./run_sonic.sh ...   # (re)build the image first
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"     # Navigation/docker
cd "${HERE}"

SERVICE="${SERVICE:-localization}"
SONIC_HOST="${SONIC_HOST:-*}"             # ZMQ wildcard bind (all interfaces)
SONIC_PORT="${SONIC_PORT:-5556}"

# The planner/ROS side runs on ROS_DOMAIN_ID=42; force it so the bridge sees
# /mpc/cmd_vel + /estop + /dlio odom from the planner (matches run_unitree.sh).
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

# Command source selection. The bridge is the SAME node in every mode; only the
# topic it subscribes changes (autonomy = the MPC's /mpc/cmd_vel, manual =
# /cmd_vel driven by a teleop publisher).
if [[ "${AUTONOMOUS:-0}" == "1" ]]; then
    echo ">> AUTONOMOUS: cmd_vel_to_sonic_node tracking /mpc/cmd_vel -> SONIC ZMQ :${SONIC_PORT} ..."
    echo ">>   (start the planner with bridge:=false so this is the only gait driver)"
    CMD_TOPIC="/mpc/cmd_vel"
elif [[ "${JOYSTICK:-0}" == "1" ]]; then
    echo ">> JOYSTICK: manual — bridge subscribes /cmd_vel; drive it with joy_to_cmdvel_node ..."
    CMD_TOPIC="/cmd_vel"
else
    echo ">> default: manual — bridge subscribes /cmd_vel (idle until you publish a command) ..."
    CMD_TOPIC="/cmd_vel"
fi
CMD_TOPIC="${CMD_VEL_TOPIC:-${CMD_TOPIC}}"

# Optional fixed 17-DOF upper-body hold (carry an object while the legs walk).
# Enabled by HOLD_ARMS=1 or a non-empty ARM_PRESET.
ARM_ARGS=()
if [[ "${HOLD_ARMS:-0}" == "1" || -n "${ARM_PRESET:-}" ]]; then
    ARM_ARGS+=(-p "hold_arms:=true")
    [[ -n "${ARM_PRESET:-}" ]] && ARM_ARGS+=(-p "arm_preset:=${ARM_PRESET}")
    echo ">> holding upper body (preset=${ARM_PRESET:-default})"
fi

NODE=(ros2 run g1_sim_bridge cmd_vel_to_sonic_node --ros-args
      -p "cmd_vel_topic:=${CMD_TOPIC}"
      -p "zmq_host:=${SONIC_HOST}"
      -p "zmq_port:=${SONIC_PORT}"
      "${ARM_ARGS[@]}")

echo ">> running SONIC bridge on ${SERVICE} (ZMQ tcp://${SONIC_HOST}:${SONIC_PORT}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, topic=${CMD_TOPIC})"
echo ">> forwarded args: $*"

# -it: Ctrl-C must reach the node so it sends SONIC stop on exit. The node args
# (and any user args) are passed as a real argv array to `exec "$@"` so the ZMQ
# wildcard bind '*' survives literally instead of being glob-expanded by the shell.
exec "${DC[@]}" run --rm -it \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    "${SERVICE}" \
    bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && exec "$@"' \
    _ "${NODE[@]}" "$@"
