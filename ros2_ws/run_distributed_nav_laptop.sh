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
IMG="${HUMBLE_IMG:-osrf/ros:humble-desktop}"
WS="$(cd "$(dirname "$0")" && pwd)"

# ── On the host (docker present, not already in a container): relaunch inside Humble ──
if [ ! -f /.dockerenv ] && command -v docker >/dev/null 2>&1; then
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

# One-time deps + build (idempotent; install/ persists on the host via the bind mount).
python3 -c "import casadi, zmq, scipy" 2>/dev/null || pip3 install --quiet casadi pyzmq scipy
if [ ! -f install/a_star_mpc_planner/share/a_star_mpc_planner/package.xml ]; then
  echo ">> building a_star_mpc_planner (one-time) ..."
  colcon build --packages-select a_star_mpc_planner --symlink-install
fi
set +u; source install/setup.bash; set -u

echo ">> relay: recv odom/TF/obstacles from Jetson ${JETSON_IP}:5601, send /mpc/cmd_vel :5602"
python3 zmq_ros_bridge.py \
  --recv "/dlio/odom_node/odom:nav_msgs/msg/Odometry,/tf:tf2_msgs/msg/TFMessage,/tf_static:tf2_msgs/msg/TFMessage,/local_voxel_map/obstacles:sensor_msgs/msg/PointCloud2" \
  --sub-connect "tcp://${JETSON_IP}:5601" \
  --send "/mpc/cmd_vel:geometry_msgs/msg/Twist" \
  --pub-bind "tcp://*:5602" &

echo ">> A* + MPC ..."
ros2 run a_star_mpc_planner a_star_node &
ros2 run a_star_mpc_planner mpc_node &

# RViz (in-container, same Humble domain as A*) — visualize the relayed odom/obstacles
# and A*/MPC output, and publish the goal. The 'SetGoal' tool is bound to /global_goal
# in this config, so a "2D Goal Pose" click IS the goal A* consumes. No goal auto-published.
if [ -f /tmp/.X11-unix/X0 ] || [ -n "${DISPLAY:-}" ]; then
  echo ">> RViz (Fixed Frame=odom; SetGoal -> /global_goal) ..."
  rviz2 -d /ws/distributed_nav.rviz &
else
  echo ">> no X display detected — skipping RViz (publish /global_goal yourself)."
fi

echo ">> distributed planner up. In RViz: '2D Goal Pose' -> publishes /global_goal to A*."
trap "kill 0" INT TERM
wait
