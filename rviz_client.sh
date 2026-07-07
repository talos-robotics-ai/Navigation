#!/usr/bin/env bash
# Launch RViz on THIS laptop to visualize the Jetson's navigation stack over WiFi.
# The Jetson runs the ROS2 stack (localization container) on ROS_DOMAIN_ID=42 with
# CycloneDDS pinned to its WiFi IP + this laptop as a unicast peer. No ethernet needed.
#
# Prereq (once per Jetson boot, inside the localization container, BEFORE autonomy.sh):
#   export ROS_LOCALHOST_ONLY=0
#   export CYCLONEDDS_URI=file:///workspace/config/cyclonedds_jetson.xml
#   export ROS_DOMAIN_ID=42
#
# Usage: ./rviz_client.sh          # opens RViz
#        ./rviz_client.sh topics   # just list discovered topics (connectivity test)
set -eo pipefail   # NOT -u: ROS setup scripts reference unbound vars

# Source ONLY the system ROS (Jazzy). Do NOT source ros2_ws/install — it was built
# for Humble inside the container and is ABI-incompatible with this laptop's Jazzy.
source /opt/ros/jazzy/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://${HOME}/.ros/cyclonedds_client.xml"

if [[ "${1:-}" == "topics" ]]; then
    echo ">> discovering Jetson topics on DOMAIN 42 (peer 10.251.101.176) ..."
    ros2 daemon stop >/dev/null 2>&1 || true
    timeout 8 ros2 topic list
    exit 0
fi

exec rviz2 "$@"
