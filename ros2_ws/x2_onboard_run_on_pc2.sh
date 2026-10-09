#!/usr/bin/env bash
# Runs ON PC2 (deployed by x2_onboard_deploy.sh as ~/talos_nav_ws/run_nav_on_pc2.sh).
# Starts / stops / shows KILVO + the navigation stack on the X2 itself:
#
#   run_nav_on_pc2.sh start [mc]|stop|status|log
#     start        KILVO + navigation; NOTHING commands the robot (walker:=none)
#     start mc     also the vendor-walker link (mc_velocity_node, input source talos_nav, priority 64, below
#                  the PS5 bridge's talos_gamepad 65): the planner then walks the robot, through the pnp gate
#   NAV_MODE=full run_nav_on_pc2.sh start        # whole planner stack ON PC2 (~1 core); default is relay
#   NAV_ARGS="split:=true global_planner:=true"  NAV_MODE=full run_nav_on_pc2.sh start
#   NAV_KILVO=false run_nav_on_pc2.sh start      # nav only (KILVO started some other way)
#
# DEFAULT (NAV_MODE=relay): PC2 runs only the cheap things -- KILVO + base odom + nav_relay_server (TCP 0.0.0.0:5596).
# The local map, A*, MPC and the pnp FSM run on the laptop (ros2_ws/x2_laptop_run.sh), which connects to
# 10.0.1.41:5596; its velocity commands come back through the relay as /x2/cmd_vel_out. Operator commands
# (go/reset/estop/clear) are then given on the LAPTOP (ros2_ws/x2_nav ...); the ones below only apply in full mode.
#
# One `ros2 launch x2_bringup x2_onboard.launch.py` (setsid, nohup, pid file), which starts
#   x2_leg_kinematics + KILVO   (the COPY in ~/talos_nav_ws/src/kilvo; ~/kilvo_ws is never used or touched)
#   x2_onboard_nav, 8 s later   (relay: base odom + nav_relay_server [+ mc]; full: base odom + local voxel map
#                                + A* + MPC + FSM; one process, one DDS participant either way)
# on the vendor DDS graph (domain 0, vendor profile: KILVO needs it for the HAL topics).
#
# With plain `start` NOTHING COMMANDS THE ROBOT: /mpc/cmd_vel has no consumer. Give the planner a goal with
#   ros2 topic pub --once /global_goal geometry_msgs/msg/PoseStamped "{header: {frame_id: odom}, pose: {position: {x: 1.5}}}"
# (from a laptop on the robot LAN, same domain -- another participant: do that with the robot on the gantry).
#
# Start refuses while the original KILVO (~/kilvo_ws) or ANY kilvo / x2_leg_kinematics process runs: two
# KILVOs on one HAL graph, and a second /x2/foot_state publisher. This script never starts a second
# ROS process and its `status` never calls the ros2 CLI (that too would be a new participant).
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR="$DIR/logs"
mkdir -p "$LOGDIR"
VENDOR_PROFILE=/agibot/software/entry/cfg/ros_dds_configuration.xml
NAV_ARGS="${NAV_ARGS:-}"
NAV_KILVO="${NAV_KILVO:-true}"
NAV_MODE="${NAV_MODE:-relay}"   # relay | full
case "$NAV_MODE" in relay|full) ;; *) echo "NAV_MODE must be relay or full" >&2; exit 2 ;; esac
NAME=nav

say() { echo "[run_nav_on_pc2] $*"; }
pidfile() { echo "$DIR/$1.pid"; }
pid_of() { [[ -f "$(pidfile "$1")" ]] && cat "$(pidfile "$1")" || true; }
pid_alive() { [[ -n "${1:-}" ]] && kill -0 "$1" 2>/dev/null; }

# Lines of `ps` naming a KILVO-ish process, minus this script and the shells/ssh around it.
foreign_kilvo() {
  ps -eo pid=,args= | grep -E 'lib/kilvo/|x2_leg_kinematics|mapping_x2|launch kilvo|/kilvo_ws/|kilvo_node' \
    | grep -vE 'grep|run_nav_on_pc2|ssh |sshd' || true
}

ros_env() {
  set +u   # ROS setup scripts reference unset variables
  source /opt/ros/humble/setup.bash
  source "$HOME"/lx2501*/install/setup.bash                 # aimdk_msgs
  source "$DIR/install/setup.bash"                          # kilvo (copy), planner, local map, x2_bringup
  set -u
  # casadi (offline wheel installed by the deploy script)
  export PYTHONPATH="$DIR/pydeps${PYTHONPATH:+:$PYTHONPATH}"
  # The vendor graph: its profile, its (default) domain, no localhost-only restriction.
  unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE CYCLONEDDS_URI
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="$VENDOR_PROFILE"
  export ROS_DOMAIN_ID="${NAV_ROS_DOMAIN_ID:-0}"
  # One BLAS/OpenMP thread: numpy's threads contend with KILVO and the box detector for the Orin's cores.
  export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
}

cmd_start() {
  local walker=none
  case "${1:-}" in
    "") ;;
    mc) walker=mc ;;
    *) echo "usage: run_nav_on_pc2.sh start [mc]" >&2; exit 2 ;;
  esac
  if pid_alive "$(pid_of $NAME)"; then
    say "FAILED: already running (pid $(pid_of $NAME)); stop first"; exit 1
  fi
  if [[ "$NAV_KILVO" == true ]]; then
    local other; other="$(foreign_kilvo)"
    if [[ -n "$other" ]]; then
      say "FAILED: a KILVO / leg-kinematics process is already running; stop it first (or NAV_KILVO=false):"
      echo "$other" | cut -c1-200 | sed 's/^/    /'
      exit 1
    fi
  fi
  [[ -f "$DIR/install/x2_bringup/share/x2_bringup/launch/x2_onboard.launch.py" ]] \
    || { say "FAILED: x2_bringup not built in $DIR/install (run x2_onboard_deploy.sh on the laptop)"; exit 1; }
  [[ -d "$DIR/pydeps/casadi" ]] || { say "FAILED: no $DIR/pydeps/casadi (run x2_onboard_deploy.sh)"; exit 1; }
  local ts logfile pid
  ts="$(date +%Y%m%d_%H%M%S)"
  logfile="$LOGDIR/${NAME}_${ts}.log"
  ( ros_env
    if [[ "$walker" == mc ]]; then
      say "!!!! walker:=mc -- THIS COMMANDS THE ROBOT: mc input source 'talos_nav' (priority 64; the PS5 pad, 65, overrides)."
      if [[ "$NAV_MODE" == relay ]]; then
        say "!!!! path: LAPTOP planner -> pnp_fsm gate (only after 'x2_nav go') -> TCP 5596 -> /x2/cmd_vel_out -> mc. Robot must be in STAND_DEFAULT (pad)."
        say "!!!! laptop gone / silent 0.3 s -> zeros, then silence."
      else
        say "!!!! path: /mpc/cmd_vel -> pnp_fsm gate (only after /pnp/start) -> /x2/cmd_vel_out -> mc. Robot must be in STAND_DEFAULT (pad)."
      fi
    else
      say "walker:=none -- nothing commands the robot"
    fi
    say "starting: ros2 launch x2_bringup x2_onboard.launch.py kilvo:=$NAV_KILVO mode:=$NAV_MODE walker:=$walker $NAV_ARGS (domain $ROS_DOMAIN_ID)"
    # shellcheck disable=SC2086  # NAV_ARGS is a word list on purpose
    setsid nohup ros2 launch x2_bringup x2_onboard.launch.py kilvo:="$NAV_KILVO" mode:="$NAV_MODE" walker:="$walker" $NAV_ARGS \
      > "$logfile" 2>&1 < /dev/null &
    echo $! > "$(pidfile $NAME)" )
  pid="$(pid_of $NAME)"
  ln -sf "$(basename "$logfile")" "$LOGDIR/${NAME}_latest.log"
  sleep 5
  if ! pid_alive "$pid"; then
    say "FAILED: launch exited within 5 s, see $logfile"; tail -n 25 "$logfile" || true
    rm -f "$(pidfile $NAME)"; exit 1
  fi
  say "running (pid $pid), log $logfile"
  say "KILVO is up first; the nav process starts ~8 s later. Watch: run_nav_on_pc2.sh log"
}

cmd_stop() {
  local pid waited=0
  pid="$(pid_of $NAME)"
  if ! pid_alive "$pid"; then
    say "not running"; rm -f "$(pidfile $NAME)"; return 0
  fi
  say "stopping pid $pid: SIGINT (ros2 launch shuts its children down)"
  kill -INT "$pid" 2>/dev/null || true
  while pid_alive "$pid" && [[ $waited -lt 20 ]]; do sleep 1; waited=$((waited + 1)); done
  if pid_alive "$pid"; then
    say "still alive after ${waited}s: SIGTERM to its session"
    kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    sleep 3
  fi
  if pid_alive "$pid"; then
    say "SIGKILL to its session"; kill -KILL "-$pid" 2>/dev/null || true; sleep 1
  fi
  pid_alive "$pid" && { say "FAILED: pid $pid still alive"; exit 1; }
  rm -f "$(pidfile $NAME)"
  say "stopped"
}

cmd_status() {
  local pid; pid="$(pid_of $NAME)"
  if pid_alive "$pid"; then
    say "running (pid $pid); its processes:"
    ps -eo pid=,pgid=,etimes=,pcpu=,pmem=,args= | awk -v g="$pid" '$2==g' | cut -c1-170 | sed 's/^/    /'
  else
    say "not running"
  fi
  local other; other="$(foreign_kilvo)"
  if ! pid_alive "$pid" && [[ -n "$other" ]]; then
    say "other KILVO-ish processes (not ours):"; echo "$other" | cut -c1-170 | sed 's/^/    /'
  fi
  [[ -e "$LOGDIR/${NAME}_latest.log" ]] && { say "log tail:"; tail -n 8 "$LOGDIR/${NAME}_latest.log" | sed 's/^/    /'; }
  return 0
}

cmd_log() {
  [[ -e "$LOGDIR/${NAME}_latest.log" ]] || { say "FAILED: no $LOGDIR/${NAME}_latest.log yet"; exit 1; }
  exec tail -n 60 -f "$LOGDIR/${NAME}_latest.log"
}

trigger() {  # consumed by pnp_fsm within 0.2 s; no ros2 CLI process joins the vendor graph
  mkdir -p "$HOME/talos_nav_ws/run" && touch "$HOME/talos_nav_ws/run/$1" && say "trigger: $1"
}

case "${1:-}" in
  go) trigger start ;;
  reset) trigger reset ;;
  estop) trigger estop ;;
  clear) trigger clear ;;
  engage) trigger engage ;;
  released) trigger released ;;
  start) shift; cmd_start "$@" ;;
  stop) cmd_stop ;;
  status) cmd_status ;;
  log) cmd_log ;;
  *) echo "usage: run_nav_on_pc2.sh start [mc]|stop|status|log|go|estop|clear|reset|engage|released   (env: NAV_MODE=relay|full, NAV_ARGS, NAV_KILVO, NAV_ROS_DOMAIN_ID)" >&2; exit 2 ;;
esac
