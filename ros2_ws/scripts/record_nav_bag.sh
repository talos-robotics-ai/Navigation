#!/usr/bin/env bash
#
# record_nav_bag.sh — record a ROS 2 bag for troubleshooting the A*+MPC(C) navigation
# stack on the real G1 (Navigation / DLIO stack).
#
# Captures everything needed to compare, offline:
#   * the GLOBAL route         (/global_path)
#   * the LOCAL A* plan         (/a_star/path)
#   * the MPC predicted traj    (/mpc/predicted_path)
#   * the ACTUAL trajectory     (/dlio/odom_node/odom)
#   * the gait velocity command (/mpc/cmd_vel — the input the AMO policy tracks)
#   * goals / setpoints / state (/global_goal, /mpc/next_setpoint, /navigation/state)
#   * solver diagnostics        (/mpc/diagnostics: success, cost, solve_ms, fails, ...)
#
# Analyse the result with:  scripts/analyze_nav_bag.py <bag_dir>
#
# Usage:
#   scripts/record_nav_bag.sh [OUTPUT_DIR] [--full]
#
#   OUTPUT_DIR   destination bag dir (default: ~/nav_bags/nav_<UTC-timestamp>)
#   --full       also record the heavy costmaps + obstacle cloud + RViz markers
#                (bigger bags; use when debugging obstacle avoidance specifically)
#
# The stack runs on ROS_DOMAIN_ID=42 (matches real_localization.launch.py); this
# script sets it so `ros2 bag record` joins the same DDS domain. Override by
# exporting ROS_DOMAIN_ID before calling.
#
set -euo pipefail

# The whole nav stack is hardcoded to ROS_DOMAIN_ID=42 (see planner.launch.py /
# real_localization.launch.py). A container shell often has a stray
# ROS_DOMAIN_ID=0, which would make the recorder join the WRONG domain and record
# an EMPTY bag. So: empty OR 0 → force 42; any other explicit value is respected.
if [[ -z "${ROS_DOMAIN_ID:-}" || "${ROS_DOMAIN_ID}" == "0" ]]; then
  export ROS_DOMAIN_ID=42
fi

OUT_DIR=""
FULL=0
for arg in "$@"; do
  case "$arg" in
    --full) FULL=1 ;;
    *)      OUT_DIR="$arg" ;;
  esac
done

if [[ -z "$OUT_DIR" ]]; then
  STAMP="$(date -u +%Y%m%d_%H%M%S)"
  OUT_DIR="${HOME}/nav_bags/nav_${STAMP}"
fi
mkdir -p "$(dirname "$OUT_DIR")"

# ── Core troubleshooting topics (light) ─────────────────────────────────────
CORE_TOPICS=(
  /dlio/odom_node/odom          # actual trajectory + odometry velocity (ground truth)
  /mpc/cmd_vel                  # velocity command → AMO gait input (the "speed")
  /a_star/path                  # local A* plan
  /mpc/predicted_path           # MPC/MPCC predicted trajectory
  /global_path                  # global route the local A* follows as a carrot
  /global_goal                  # commanded goal (from RViz)
  /a_star/local_goal            # local carrot / goal
  /mpc/next_setpoint            # MPC lookahead setpoint
  /mpc/diagnostics              # [success, COST(J), solve_ms, avg_ms, fails, security, vx_eff]
                                #   index [1] is the MPC cost function J — recorded here
                                #   and plotted by analyze_nav_bag.py (solver panel, twin axis)
  /navigation/state             # IDLE | NAVIGATING | ALIGNING | GOAL_REACHED | SECURITY | STOPPED
)

# ── Heavy topics (only with --full) ─────────────────────────────────────────
FULL_TOPICS=(
  /local_voxel_map/obstacles    # ground-removed obstacle cloud (large PointCloud2)
  /a_star/occupancy_grid        # local costmap
  /global_planner/costmap       # global costmap
  /mpc/predicted_obstacles      # RViz obstacle spheres
  /mpc/obstacle_velocities      # RViz dynamic-obstacle velocity arrows
)

TOPICS=("${CORE_TOPICS[@]}")
if [[ "$FULL" -eq 1 ]]; then
  TOPICS+=("${FULL_TOPICS[@]}")
fi

echo "=================================================================="
echo " Recording navigation bag"
echo "   ROS_DOMAIN_ID = ${ROS_DOMAIN_ID}"
echo "   output        = ${OUT_DIR}"
echo "   mode          = $([[ $FULL -eq 1 ]] && echo 'FULL (+costmaps/cloud/markers)' || echo 'core')"
echo "   topics        = ${#TOPICS[@]}"
echo "-----------------------------------------------------------------"
printf '   %s\n' "${TOPICS[@]}"
echo "=================================================================="

# ── Preflight: is the stack actually publishing on this domain? ──────────────
# The #1 cause of an empty bag is a domain mismatch (recorder on a different
# ROS_DOMAIN_ID than the nav stack) — the recorder happily writes 0 messages for
# minutes. Verify the key topic has a publisher before we commit to recording.
KEY_TOPIC="/dlio/odom_node/odom"
if command -v ros2 >/dev/null 2>&1; then
  echo " preflight: checking ${KEY_TOPIC} has a publisher on domain ${ROS_DOMAIN_ID} ..."
  if timeout 6 ros2 topic list 2>/dev/null | grep -qx "${KEY_TOPIC}"; then
    echo " preflight: OK — stack is publishing."
  else
    echo "!! WARNING: ${KEY_TOPIC} NOT found on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}." >&2
    echo "!! The nav stack is probably not running, or is on a different domain." >&2
    echo "!! Recording now would produce an EMPTY bag. Start the stack first, or" >&2
    echo "!! re-run with the correct domain, e.g.:  ROS_DOMAIN_ID=42 $0 $*" >&2
    echo "!! Continuing in 5 s (Ctrl-C to abort) ..." >&2
    sleep 5
  fi
fi

echo " Ctrl-C to stop."
echo

exec ros2 bag record -o "$OUT_DIR" "${TOPICS[@]}"
