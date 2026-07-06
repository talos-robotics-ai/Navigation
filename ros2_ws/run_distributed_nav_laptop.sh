#!/usr/bin/env bash
# Laptop side of the distributed-nav split (docs/planning/DISTRIBUTED_NAV_PLAN.md):
# runs A* + MPC + the ROS<->ZMQ relay in a HUMBLE container (matches the Jetson's ROS
# distro so serialized CDR is compatible — the laptop is Jazzy, which must NOT touch
# these messages natively). Receives odom/TF/obstacles from the Jetson and sends
# /mpc/cmd_vel back.
#
# ORDER: start the Jetson side first (run_distributed_nav_jetson.sh), then this.
#
#   JETSON_IP=10.251.101.176 ./run_distributed_nav_laptop.sh
#
# The planner is built once inside the container (cached in ./build+install on the
# host via the bind mount); subsequent runs skip the build.
set -euo pipefail
JETSON_IP="${JETSON_IP:?set JETSON_IP=<jetson wifi ip>  (e.g. JETSON_IP=10.251.101.176)}"
IMG="${HUMBLE_IMG:-osrf/ros:humble-desktop}"
WS="$(cd "$(dirname "$0")" && pwd)"     # this repo's ros2_ws (mounted into the container)

echo ">> launching A*/MPC + relay in ${IMG} (network host), planner <- Jetson ${JETSON_IP}"
exec docker run --rm -it --network host \
  -v "${WS}":/ws -w /ws \
  -e ROS_DOMAIN_ID=42 -e JETSON_IP="${JETSON_IP}" \
  "${IMG}" bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    # One-time deps + build (idempotent; install/ persists on the host via the mount).
    python3 -c "import casadi, zmq, scipy" 2>/dev/null || pip3 install --quiet casadi pyzmq scipy
    if [ ! -f install/a_star_mpc_planner/share/a_star_mpc_planner/package.xml ]; then
      echo ">> building a_star_mpc_planner (one-time) ..."
      colcon build --packages-select a_star_mpc_planner --symlink-install
    fi
    source install/setup.bash

    echo ">> relay: recv odom/TF/obstacles from Jetson '${JETSON_IP}':5601, send /mpc/cmd_vel :5602"
    python3 zmq_ros_bridge.py \
      --recv "/dlio/odom_node/odom:nav_msgs/msg/Odometry,/tf:tf2_msgs/msg/TFMessage,/tf_static:tf2_msgs/msg/TFMessage,/local_voxel_map/obstacles:sensor_msgs/msg/PointCloud2" \
      --sub-connect "tcp://'${JETSON_IP}':5601" \
      --send "/mpc/cmd_vel:geometry_msgs/msg/Twist" \
      --pub-bind "tcp://*:5602" &
    ZMQ_PID=$!

    echo ">> A* + MPC ..."
    ros2 run a_star_mpc_planner a_star_node &
    ros2 run a_star_mpc_planner mpc_node &

    echo ">> distributed planner up. Set a goal on /global_goal (Foxglove Publish)."
    trap "kill 0" INT TERM
    wait
  '
