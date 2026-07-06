#!/usr/bin/env bash
# Laptop side of the distributed-nav split (docs/planning/DISTRIBUTED_NAV_PLAN.md):
# runs A* + MPC + the ROS<->ZMQ relay in a HUMBLE environment (matches the Jetson's
# ROS distro so serialized CDR is compatible — the laptop's native Jazzy must NOT
# touch these messages). Receives odom/TF/obstacles from the Jetson, sends cmd_vel back.
#
# Run it on the laptop HOST — it launches the Humble container for you:
#   JETSON_IP=10.251.101.176 ./run_distributed_nav_laptop.sh
# If you're already inside a Humble container (with this ros2_ws mounted), it detects
# that and runs the planner directly instead of nesting docker.
#
# ORDER: start the Jetson side first (run_distributed_nav_jetson.sh), then this.
set -uo pipefail
JETSON_IP="${JETSON_IP:?set JETSON_IP=<jetson wifi ip>  (e.g. JETSON_IP=10.251.101.176)}"
IMG="${HUMBLE_IMG:-dnav-laptop:humble}"   # baked from Dockerfile.dnav (deps pre-installed)
WS="$(cd "$(dirname "$0")" && pwd)"

# ── On the host (docker present, not already in a container): relaunch inside Humble ──
if [ ! -f /.dockerenv ] && command -v docker >/dev/null 2>&1; then
  # Build the planner image once (Humble + casadi/pyzmq/scipy). No context — just the Dockerfile.
  if ! docker image inspect "${IMG}" >/dev/null 2>&1; then
    echo ">> building planner image ${IMG} (one-time, ~2-3 min) ..."
    docker build -t "${IMG}" - < "${WS}/Dockerfile.dnav"
  fi
  echo ">> launching A*/MPC + relay + RViz in ${IMG} (network host), planner <- Jetson ${JETSON_IP}"
  xhost +local:root >/dev/null 2>&1 || true   # let the container reach the X server
  exec docker run --rm -it --network host \
    -v "${WS}":/ws -w /ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e ROS_DOMAIN_ID=42 -e JETSON_IP="${JETSON_IP}" \
    "${IMG}" bash /ws/run_distributed_nav_laptop.sh
fi

# ── Inside the container (or any Humble host): build + run ──
export ROS_DOMAIN_ID=42
set +u   # ROS/colcon setup scripts reference unset vars
source /opt/ros/humble/setup.bash
set -u

# Deps are baked into dnav-laptop:humble (Dockerfile.dnav); sanity-check them.
python3 -c "import casadi, zmq, scipy" 2>/dev/null || {
  echo "!! casadi/zmq/scipy missing — not the baked dnav-laptop:humble image?"
  echo "   rebuild: docker build -t dnav-laptop:humble - < Dockerfile.dnav"; }

# Build a_star_mpc_planner once (idempotent; install/ persists on the host via the mount).
if [ ! -f install/a_star_mpc_planner/share/a_star_mpc_planner/package.xml ]; then
  echo ">> building a_star_mpc_planner (one-time) ..."
  colcon build --packages-select a_star_mpc_planner --symlink-install
fi

# Hand off to autonomy.sh: JETSON_IP is in the env, which puts it in OFF-BOARD mode
# (ZMQ relay + A*/MPC with bridge:=false + goal-bound RViz, no localization). One
# orchestrator, with autonomy.sh's logs + bag recording + safe wait_for_goal params.
exec bash "${WS}/autonomy.sh"
