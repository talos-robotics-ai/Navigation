#!/usr/bin/env bash
# Watchdog for the intermittent Livox MID-360 stall: the lidar occasionally stops
# publishing while its DRIVER PROCESS stays alive, silently blinding DLIO + local_voxel_map
# (obstacles/costmap go empty — we hit exactly this: the ZMQ relay forwarded odom/tf but
# zero obstacles). This detects the stall and KILLS the driver node so the launch's
# respawn=True (real_localization.launch.py) restarts it — automating the manual "restart
# the driver" you do today.
#
# Rate source: DLIO's own "Sensor Rates: Livox @ X Hz" log line — NOT `ros2 topic hz
# /livox/lidar`, because a fresh CLI subscriber often can't pull the large PointCloud2
# over CycloneDDS here (QoS/large-msg quirk) and would report false stalls.
#
# Run it alongside the stack in its own tmux/terminal:  ros2_ws/scripts/livox_watchdog.sh
# Env: RATE_MIN (Hz, default 5)  STALL_CHECKS (consecutive bad before restart, default 3)
#      PERIOD (s between checks, default 5)  LOG (localization logfile; auto-detected)
set -uo pipefail

RATE_MIN="${RATE_MIN:-5}"
STALL_CHECKS="${STALL_CHECKS:-3}"
PERIOD="${PERIOD:-5}"

# Auto-detect the localization log written by the run scripts, unless LOG is given.
LOG="${LOG:-}"
if [[ -z "${LOG}" ]]; then
  for c in /tmp/dnav_localization.log /tmp/teleop_localization.log \
           "${HOME}/Navigation/ros2_ws/logs/localization_latest.log" \
           /tmp/stage2_perception.log /tmp/v_perc.log /tmp/p1_perc.log; do
    [[ -f "${c}" ]] && LOG="${c}" && break
  done
fi
[[ -n "${LOG}" && -f "${LOG}" ]] || { echo "!! no localization log found (set LOG=<path>); can't read Livox rate" >&2; exit 1; }

echo ">> livox_watchdog: reading '${LOG}', restart driver if Livox < ${RATE_MIN} Hz for ${STALL_CHECKS} checks (every ${PERIOD}s)"
bad=0
while true; do
  # last reported Livox rate from DLIO's telemetry line
  rate=$(grep -aoE "Sensor Rates: Livox @ [0-9.]+" "${LOG}" 2>/dev/null | tail -1 | grep -oE "[0-9.]+$")
  rate="${rate:-0}"
  if awk "BEGIN{exit !(${rate} < ${RATE_MIN})}"; then
    bad=$((bad + 1))
    echo "$(date +%T) Livox=${rate} Hz < ${RATE_MIN}  (bad ${bad}/${STALL_CHECKS})"
    if [[ "${bad}" -ge "${STALL_CHECKS}" ]]; then
      echo "$(date +%T) !! Livox stalled — killing driver so respawn recovers it"
      pkill -f "[l]ivox_ros_driver2_node" 2>/dev/null || true
      bad=0
      sleep 6   # let respawn_delay + reconnect settle before checking again
    fi
  else
    [[ "${bad}" -gt 0 ]] && echo "$(date +%T) Livox recovered (${rate} Hz)"
    bad=0
  fi
  sleep "${PERIOD}"
done
