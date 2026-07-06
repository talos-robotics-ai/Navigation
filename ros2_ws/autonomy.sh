#!/usr/bin/env bash
# Autonomous navigation launcher: bring up BOTH the perception/localization stack
# and the A*+MPC planner with one command, as two SEPARATE processes.
#
# Each `ros2 launch` is its own process tree and runs all of its nodes
# concurrently (ROS 2 launch already manages that — no manual threading needed),
# so this just starts two launch files and ties their lifetimes together: Ctrl-C
# tears BOTH down cleanly (the plain `cmd & cmd` form would orphan the first).
#
# This is the all-in-one alternative to starting the two launch files by hand
# (see README "Autonomous navigation"). The gait that consumes /mpc/cmd_vel is
# selected with GAIT= (forwarded to planner.launch.py as gait:=):
#   GAIT=amo (default) — the AMO bridge is launched here; the AMO policy runs
#                        separately: AUTONOMOUS=1 NET_IF=<nic> ./docker/run_amo.sh
#   GAIT=sonic         — the SONIC bridge is launched here; the SONIC controller
#                        (g1_deploy_onnx_ref) must ALREADY be running FIRST, or it
#                        misses the one-shot start handshake (docs/locomotion/SONIC_REAL_BRINGUP.md).
#   GAIT=unitree       — the native-gait bridge is launched here; bring the robot
#                        to walking control first (docs/locomotion/UNITREE_GAIT.md).
#
# Env overrides:
#   GAIT=amo            gait consuming /mpc/cmd_vel: amo | sonic | unitree
#   ROS_DOMAIN_ID=42    DDS domain (default 42; matches both launch files)
#   PLANNER_DELAY=3     seconds to wait after localization before the planner,
#                       so DLIO finishes its IMU/gravity init (hold the robot
#                       STILL during this window). Set 0 to start them together.
#   USE_RVIZ=1          open the on-board RViz (localization launch). Set 0 to run
#                       headless — visualize remotely instead (Foxglove on the
#                       laptop, or remote RViz), avoiding the Jetson GPU/NvMap load.
set -uo pipefail

# Resolve the workspace root: the nearest ancestor (or the script's own dir) that
# holds install/setup.bash. Works whether this script sits in ros2_ws/ or
# ros2_ws/src/, and whether the workspace is mounted at /ws or anywhere else.
HERE="$(cd "$(dirname "$0")" && pwd)"
WS=""
d="${HERE}"
while [[ "${d}" != "/" ]]; do
    if [[ -f "${d}/install/setup.bash" ]]; then WS="${d}"; break; fi
    d="$(dirname "${d}")"
done
if [[ -z "${WS}" ]]; then
    echo "error: no install/setup.bash found at or above ${HERE} — build the workspace first" >&2
    echo "       (inside the localization container: build_ws)" >&2
    exit 1
fi
cd "${WS}"
# Disable nounset around the ROS/colcon sourcing: install/setup.bash references
# unset vars (COLCON_TRACE, AMENT_TRACE, …) and is not `set -u`-safe.
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u

# The launch files force ROS_DOMAIN_ID=42 for their nodes, but the estop keyboard
# helper + the bag recorder started BELOW run in THIS shell's domain. A container
# shell often has a stray ROS_DOMAIN_ID=0, which would put the e-stop on the wrong
# domain (bridge never hears it) and record an empty bag. Empty/0 → force 42.
if [[ -z "${ROS_DOMAIN_ID:-}" || "${ROS_DOMAIN_ID}" == "0" ]]; then
    export ROS_DOMAIN_ID=42
fi
PLANNER_DELAY="${PLANNER_DELAY:-3}"
# On-board RViz on the Jetson renders with OGRE/OpenGL and eats the GPU/NvMap
# carveout (NvMapMemAllocInternalTagged ... error 12). USE_RVIZ=0 runs the stack
# headless — visualize remotely instead (foxglove_bridge -> Foxglove on the laptop).
USE_RVIZ="${USE_RVIZ:-1}"
if [[ "${USE_RVIZ}" == "0" ]]; then RVIZ_ARG="rviz:=false"; else RVIZ_ARG="rviz:=true"; fi

# ── CPU isolation from the SONIC controller ──────────────────────────────────
# On the 6-core Orin Nano the SONIC controller pins its RT threads to cores 2-5.
# If the nav stack (+ DDS + any remote viz) competes for those cores it can starve
# the controller's 500 Hz loop / LowState DDS thread -> "Lost LowState data
# connection" -> safety-stop -> the robot FALLS (this happened once). So pin every
# process THIS script starts to the nav cores 0,1 only. Pair it with starting the
# controller as  SONIC_CPU_MAIN=2 scripts/start_deploy_real.sh  (moves its main/
# LowState thread off core 0 onto the isolated set), and optionally isolcpus=2-5 at
# boot. See docs/locomotion/SONIC_REAL_BRINGUP.md §6a. NAV_CPUS="" disables pinning.
NAV_CPUS="${NAV_CPUS:-0,1}"
TASKSET=()
if [[ -n "${NAV_CPUS}" ]] && command -v taskset >/dev/null 2>&1; then
    TASKSET=(taskset -c "${NAV_CPUS}")
    echo ">> pinning nav stack to CPUs ${NAV_CPUS} (keeps cores 2-5 free for the SONIC controller)"
fi

# Which gait consumes /mpc/cmd_vel (forwarded to planner.launch.py). The bridge
# for the selected gait is launched as part of the planner below; the reminder
# printed later depends on it (the SONIC/Unitree gaits need a process this script
# does NOT start).
# Pin the SONIC upper body to a neutral standing pose so the arms stay still instead
# of swinging with the policy's gait (gait:=sonic only). HOLD_ARMS=1 ./autonomy.sh
HOLD_ARMS="${HOLD_ARMS:-false}"
[[ "${HOLD_ARMS}" == "1" ]] && HOLD_ARMS=true
GAIT="${GAIT:-amo}"
case "${GAIT}" in
    amo)     GAIT_NOTE="Start the AMO gait:  AUTONOMOUS=1 NET_IF=<nic> ./docker/run_amo.sh" ;;
    sonic)   GAIT_NOTE="SONIC controller must ALREADY be running (start it FIRST): cd ~/groot/sonic-g1-locomotion && scripts/start_deploy_real.sh" ;;
    unitree) GAIT_NOTE="Bring the robot to walking control first (see docs/locomotion/UNITREE_GAIT.md)." ;;
    *)       GAIT_NOTE="gait:=${GAIT}" ;;
esac

# ── Separate logs for localization vs planner ────────────────────────────────
# Both launches used to share this terminal, so DLIO/g1_local_map and the
# A*+MPC planner logs interleaved. Now each launch's stdout+stderr goes to its
# OWN file so you can read them independently:
#     tail -f logs/localization_latest.log     # DLIO + g1_local_map
#     tail -f logs/planner_latest.log          # A* node + MPC node (+ bridge)
# Override the directory with LOG_DIR=/path ./autonomy.sh. Set LOG_TO_CONSOLE=1
# to ALSO mirror both streams to this terminal (they will interleave again).
LOG_DIR="${LOG_DIR:-${WS}/logs}"
mkdir -p "${LOG_DIR}"
TS="$(date +%Y%m%d_%H%M%S)"
LOCALIZATION_LOG="${LOG_DIR}/localization_${TS}.log"
PLANNER_LOG="${LOG_DIR}/planner_${TS}.log"
# Stable "latest" symlinks so you can tail without knowing the timestamp.
ln -sfn "$(basename "${LOCALIZATION_LOG}")" "${LOG_DIR}/localization_latest.log"
ln -sfn "$(basename "${PLANNER_LOG}")"      "${LOG_DIR}/planner_latest.log"
LOG_TO_CONSOLE="${LOG_TO_CONSOLE:-0}"

pids=()
cleanup() {
    trap - INT TERM EXIT
    echo ""
    echo ">> stopping localization + planner ..."
    # SIGINT lets each `ros2 launch` shut its own nodes down gracefully.
    kill -INT "${pids[@]}" 2>/dev/null || true
    wait 2>/dev/null || true
    # Backstop: the livox_ros_driver2 node IGNORES SIGINT, so it survives the
    # graceful shutdown above and gets orphaned — spinning at ~50% CPU until the
    # next reboot. Left unchecked, successive runs stack drivers (5 seen once =
    # ~2.5 wasted cores). SIGKILL any survivor. Everything else (DLIO, planner,
    # Python nodes) exits on the SIGINT; the SONIC policy is a separate process
    # and is deliberately NOT touched here.
    pkill -9 -f livox_ros_driver2_node 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

# Launch a `ros2 launch`, sending its output to a dedicated logfile (and, when
# LOG_TO_CONSOLE=1, also to this terminal via tee). Records the launch PID — NOT
# tee's — so cleanup signals the launch directly.
run_launch() {
    local logfile="$1"; shift
    # stdin from /dev/null: these run in the BACKGROUND while the foreground
    # e-stop keyboard node owns the terminal. Without this, `ros2 bag record`
    # (which reads stdin for its SPACE pause) would steal the s/g/q keystrokes
    # meant for the e-stop, making the safety stop unreliable.
    if [[ "${LOG_TO_CONSOLE}" == "1" ]]; then
        "${TASKSET[@]}" "$@" < /dev/null > >(tee -a "${logfile}") 2>&1 &
    else
        "${TASKSET[@]}" "$@" < /dev/null > "${logfile}" 2>&1 &
    fi
    pids+=($!)
}

# Preflight: clear any leaked Livox driver before starting a fresh one. It ignores
# SIGINT, so a prior run killed hard (e.g. an RViz crash taking the launch down)
# leaves it spinning at ~50% CPU, and runs stack them. Only the Livox driver leaks;
# DLIO/planner/Python nodes exit cleanly. Never touches the SONIC policy.
if pgrep -f livox_ros_driver2_node >/dev/null 2>&1; then
    echo ">> [preflight] stale livox_ros_driver2_node found (leaked from a prior run) — killing ..."
    pkill -9 -f livox_ros_driver2_node 2>/dev/null || true
    sleep 1
fi

echo ">> [1/2] localization (DLIO + g1_local_map) on ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ..."
echo ">>       logs -> ${LOCALIZATION_LOG}"
run_launch "${LOCALIZATION_LOG}" ros2 launch g1_bringup real_localization.launch.py "${RVIZ_ARG}"

if (( PLANNER_DELAY > 0 )); then
    echo ">> waiting ${PLANNER_DELAY}s for DLIO IMU/gravity init — keep the robot STILL ..."
    sleep "${PLANNER_DELAY}"
fi

echo ">> [2/2] A*+MPC planner (gait:=${GAIT}, its cmd_vel bridge) ..."
echo ">>       logs -> ${PLANNER_LOG}"
run_launch "${PLANNER_LOG}" ros2 launch a_star_mpc_planner planner.launch.py gait:=${GAIT} hold_arms:=${HOLD_ARMS}

# ── Auto-record a ROS bag for troubleshooting ────────────────────────────────
# Every autonomy run captures the nav topics to a timestamped bag (shares TS with
# the logs) so the run can be replayed / plotted afterwards with
# scripts/analyze_nav_bag.py. Reuses record_nav_bag.sh (forces domain 42 +
# preflight-checks that topics are live), backgrounded and tied into cleanup()
# via run_launch so Ctrl-C / 'q' finalises the bag gracefully.
#   RECORD_BAG=0   ./autonomy.sh   # disable recording for this run
#   RECORD_FULL=1  ./autonomy.sh   # also grab obstacle cloud + costmaps (big;
#                                  #   needed for obstacle-avoidance debugging)
#   BAG_DIR=/path  ./autonomy.sh   # override the bag root (default ${WS}/nav_bags)
RECORD_BAG="${RECORD_BAG:-1}"
RECORD_FULL="${RECORD_FULL:-0}"
BAG_ROOT="${BAG_DIR:-${WS}/nav_bags}"
REC_SCRIPT="${WS}/scripts/record_nav_bag.sh"
BAG_OUT=""
if [[ "${RECORD_BAG}" == "1" ]]; then
    if [[ -x "${REC_SCRIPT}" ]]; then
        BAG_OUT="${BAG_ROOT}/nav_${TS}"
        BAG_LOG="${LOG_DIR}/rosbag_${TS}.log"
        ln -sfn "$(basename "${BAG_LOG}")" "${LOG_DIR}/rosbag_latest.log"
        rec_args=("${BAG_OUT}")
        [[ "${RECORD_FULL}" == "1" ]] && rec_args+=(--full)
        echo ">> [rec] recording nav bag -> ${BAG_OUT}  (full=${RECORD_FULL})"
        echo ">>       rec log -> ${BAG_LOG}"
        run_launch "${BAG_LOG}" "${REC_SCRIPT}" "${rec_args[@]}"
        ln -sfn "${BAG_OUT}" "${BAG_ROOT}/nav_latest"   # stable handle for analysis
    else
        echo ">> [rec] WARNING: ${REC_SCRIPT} not executable — NOT recording." >&2
    fi
fi

echo ""
echo ">> both launches running. Read their logs SEPARATELY (each in its own terminal):"
echo ">>     tail -f ${LOG_DIR}/localization_latest.log"
echo ">>     tail -f ${LOG_DIR}/planner_latest.log"
echo ">> ${GAIT_NOTE}"
echo ">> then set a goal in RViz (2D Goal Pose -> /global_goal)."
if [[ -n "${BAG_OUT}" ]]; then
    echo ">> recording -> ${BAG_OUT}"
    echo ">>   analyse after:  python3 ${WS}/scripts/analyze_nav_bag.py ${BAG_ROOT}/nav_latest --no-show"
fi

# ── Foreground keyboard e-stop ───────────────────────────────────────────────
# This owns the terminal's stdin (the two launches stream to logfiles), so you
# can SAFELY stop and later re-enable navigation without killing anything:
#     s + Enter  ->  STOP  (zero velocity to the AMO gait; robot holds)
#     g + Enter  ->  GO    (resume navigation)
#     q + Enter  ->  quit autonomy.sh (stops everything)
# DISABLE_ESTOP_KEYS=1 falls back to the old "Ctrl-C only / wait" behaviour.
#
# If a launch itself crashes, the velocity command to the gait still goes to
# ZERO automatically — the MPC fail-safe (stale pose/path) and the bridge
# cmd_vel watchdog both zero it — so a crash stops the robot even before you
# press q / Ctrl-C here.
echo ""
if [[ "${DISABLE_ESTOP_KEYS:-0}" == "1" ]]; then
    echo ">> e-stop keys disabled — Ctrl-C stops everything."
    wait -n 2>/dev/null || wait
else
    echo ">> SAFETY E-STOP active in THIS terminal:  s=stop  g=go  q=quit"
    "${TASKSET[@]}" ros2 run g1_sim_bridge estop_keyboard_node
fi
cleanup
