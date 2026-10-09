#!/usr/bin/env bash
# Run the X2 navigation + box pick-and-place stack on the LAPTOP.
#
# SAFETY (CRITICAL): nothing started here may ever reach the robot's DDS network -- new participants
# on PC2's graph have made the robot fall. So: dedicated domain 77, discovery restricted to
# localhost, no vendor/robot DDS profile, and the stack talks to the walking policy only over
# TCP 127.0.0.1:8770 (RoboJuDo link).
#
# One-time setup (python3.12 venv; casadi is not in apt/rosdep here; python3-venv/ensurepip is
# missing on this laptop so uv is used; --no-deps keeps the system numpy 1.26 / scipy 1.11):
#   uv venv --system-site-packages --python /usr/bin/python3.12 ros2_ws/.venv
#   uv pip install --python ros2_ws/.venv/bin/python --no-deps casadi
#   cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install \
#       --packages-select a_star_mpc_planner x2_box_pnp x2_bringup
# Usage: ./x2_run.sh [extra ros2 launch args, e.g. auto_start:=true global_planner:=true]
# Trigger:  ros2 service call /pnp/start std_srvs/srv/Trigger   (same env as below)
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI RMW_FASTRTPS_USE_QOS_FROM_XML ROS_STATIC_PEERS
export ROS_DOMAIN_ID=77
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
set +u
source /opt/ros/jazzy/setup.bash
source "$HERE/install/setup.bash"
# colcon entry points use /usr/bin/python3, so expose the venv's casadi through PYTHONPATH
# (venv holds ONLY casadi: numpy 1.26 / scipy 1.11 come from the system -- numpy 2 would break scipy).
export PYTHONPATH="$HERE/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
echo "x2_run: ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE"
exec ros2 launch x2_bringup x2_nav.launch.py "$@"
