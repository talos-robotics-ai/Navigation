#!/usr/bin/env bash
# Stream the on-Jetson navigation topics to Foxglove Studio on a remote laptop over a
# SINGLE WebSocket (TCP :8765) — no DDS on the WiFi. This is the reliable way to
# visualise the stack remotely on an enterprise/campus WiFi where native RViz-over-DDS
# fails (blocked multicast, unroutable robot-eth locator, Jazzy/Humble mismatch, UDP
# fragment drops). See docs/system/REMOTE_VISUALIZATION.md.
#
#   Jetson:  ros2_ws/start_foxglove.sh
#   Laptop:  open Foxglove Studio -> ws://<jetson-wifi-ip>:8765   (URL is printed below)
#
# ┌─────────────────────────────────────────────────────────────────────────────────┐
# │ ⚠ SAFETY: on the 6-core Orin Nano, running this ALONGSIDE the SONIC controller    │
# │   while it is actively balancing can starve the controller's LowState DDS thread  │
# │   -> "Lost LowState data connection" -> safety-stop -> the robot FALLS. Foxglove  │
# │   only streams topics a client subscribes to, so raw /livox/lidar + point clouds  │
# │   are the heavy ones. Prefer remote viz when the robot is NOT under active         │
# │   balance control, or subscribe only to light topics (odom / path / TF / costmap). │
# └─────────────────────────────────────────────────────────────────────────────────┘
set -eo pipefail

PORT="${FOXGLOVE_PORT:-8765}"
DOMAIN="${ROS_DOMAIN_ID:-42}"

# Detect the Jetson's WiFi IP (the address the laptop reaches it on). Override with
# JETSON_WIFI_IP=... if you have more than one wireless NIC or detection is wrong.
WIFI_IP="${JETSON_WIFI_IP:-$(ip -4 -o addr show up 2>/dev/null | awk '$2 ~ /^wl/ {print $4}' | cut -d/ -f1 | head -1)}"
if [[ -z "$WIFI_IP" ]]; then
    echo "!! could not auto-detect a WiFi (wl*) interface. Set JETSON_WIFI_IP=<ip> and re-run." >&2
    ip -br addr | grep -v '^lo' >&2
    exit 1
fi

# CycloneDDS config for the bridge so it DISCOVERS the local nav nodes. The nav stack
# runs DEFAULT DDS (on-host multicast, ephemeral participant index), so the bridge must
# match: keep multicast ON (loopback multicast is on-host, NOT blocked by the AP) and
# use ParticipantIndex=none so we don't collide with / get capped by the well-known
# index scheme the nav stack fills. NO interface binding / NO unicast peer — that
# would only see WiFi-pinned nodes and miss default-DDS ones (the common case).
CFG="$(mktemp /tmp/cyclonedds_foxglove.XXXXXX.xml)"
cat > "$CFG" <<EOF
<?xml version='1.0' encoding='UTF-8' ?>
<CycloneDDS xmlns='https://cdds.io/config'>
  <Domain id='any'>
    <Discovery>
      <ParticipantIndex>none</ParticipantIndex>
      <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
trap 'rm -f "$CFG"' EXIT

# System ROS + the workspace overlay (the overlay carries the custom message types —
# livox CustomMsg, planner msgs — so the bridge can advertise those channels too).
source /opt/ros/humble/setup.bash
WS_INSTALL="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install/setup.bash"
[[ -f "$WS_INSTALL" ]] && source "$WS_INSTALL"

export ROS_DOMAIN_ID="$DOMAIN"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$CFG"

# Pin the bridge to the nav cores (0,1,2) so it can NEVER compete with the SONIC
# controller's RT cores (3-5). Serialising heavy topics (point clouds) is exactly
# what starved the controller's LowState thread and dropped the robot. NAV_CPUS=""
# disables. See docs/system/REMOTE_VISUALIZATION.md.
NAV_CPUS="${NAV_CPUS:-0,1,2}"
TASKSET=()
if [[ -n "${NAV_CPUS}" ]] && command -v taskset >/dev/null 2>&1; then
    TASKSET=(taskset -c "${NAV_CPUS}")
fi

echo ">> foxglove_bridge on 0.0.0.0:${PORT}  (ROS_DOMAIN_ID=${DOMAIN}${NAV_CPUS:+, CPUs ${NAV_CPUS}})"
echo ">> On the laptop, open Foxglove Studio and connect to:  ws://${WIFI_IP}:${PORT}"
echo ">> (Ctrl-C to stop.)"
exec "${TASKSET[@]}" ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:="${PORT}" address:=0.0.0.0
