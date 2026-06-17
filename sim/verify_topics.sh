#!/usr/bin/env bash
# Verify the simulated MID-360 topics are actually flowing — run this INSIDE the
# localization container (or any Humble shell on the same host + ROS_DOMAIN_ID as
# Isaac Sim) to confirm Isaac's data reaches the localization stack over DDS.
#
#   # 1) Isaac on the host:
#   Navigation/sim/launch_g1_sim.sh
#   # 2) the bridge (for the *_custom / RELIABLE imu topics), e.g. in the container:
#   ros2 launch g1_sim_bridge sim_localization.launch.py start_fastlio:=false
#   # 3) this script, in the container (sim/ is mounted at /sim by docker-compose):
#   docker compose run --rm localization bash -lc 'bash /sim/verify_topics.sh'
#   #   (or just run it from any sourced Humble shell on the same host + domain)
#
# Pass a topic name to stream it in full:  ./verify_topics.sh /livox/imu_raw
set -uo pipefail

source /opt/ros/humble/setup.bash 2>/dev/null || true
# /ws/install has livox_ros_driver2 (CustomMsg) + g1_sim_bridge after `build_ws`.
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash 2>/dev/null || true

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
echo "RMW=$RMW_IMPLEMENTATION  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo

# Full-stream mode for a single topic.
if [[ $# -ge 1 ]]; then
    echo ">> ros2 topic echo $1"
    exec ros2 topic echo "$1"
fi

echo "=== ros2 topic list ==="
ros2 topic list
echo

check() {  # name  : info + a few seconds of 'hz'
    local t="$1"
    echo "--- $t ---"
    if ! ros2 topic info "$t" >/dev/null 2>&1; then
        echo "  NOT ADVERTISED (publisher not seen on this domain/RMW)"
        return
    fi
    ros2 topic info "$t" | sed 's/^/  /'
    timeout 5 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" \
        || echo "  (no messages in 5 s)"
}

echo "=== Isaac-published (raw) ==="
for t in /clock /livox/lidar /livox/imu_raw; do check "$t"; done
echo
echo "=== g1_sim_bridge output (what FAST-LIO consumes) ==="
for t in /livox/custom_msg /livox/imu; do check "$t"; done
echo

echo "=== IMU Z sanity — linear_acceleration.z should be ~ +9.81 at rest ==="
timeout 5 ros2 topic echo /livox/imu_raw --once --field linear_acceleration 2>/dev/null \
    || echo "  (no IMU message received)"
