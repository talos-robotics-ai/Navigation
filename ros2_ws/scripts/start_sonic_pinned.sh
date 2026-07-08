#!/usr/bin/env bash
# Start the SONIC controller (g1_deploy_onnx_ref) with the VALIDATED 3/3 core pinning.
#
# Why this wrapper exists (measured on the real G1, 2026-07):
#   The controller's CycloneDDS LowState receive threads (recvUC / recvMC / dq.*) are
#   NOT pinned by SONIC_CPU_MAIN. If they float onto the nav cores (0,1,2), which run
#   at 60-90% under DLIO + local_voxel_map + the ZMQ relay, they suffer 5-15 ms
#   scheduling stalls (vs 72 us on a clean core) -> miss LowState frames (500 Hz) ->
#   "Lost LowState data connection" -> safety-stop -> the robot FALLS. The ONLY thing
#   that reliably confines every controller thread (incl. those DDS threads) to the RT
#   cores is the outer `taskset -c 3-5`. Starting SONIC WITHOUT it is the #1 fall cause.
#
# 3/3 split (verified strictly better than the old 2/4): perception owns cores 0,1,2,
# SONIC owns 3,4,5. SONIC is GPU-bound (~11% CPU on its cores), so 3 cores keep its
# 500 Hz loop and LowState healthy — measured LowState age was IDENTICAL to 2/4 (~5 ms
# p50, 0 losses) while the nav side dropped from ~94% to ~80% per core. The 4 RT
# workers map onto 3 cores as: control(80)->5 alone, command_writer(90)->4 alone,
# main(99)+input(60)+planner(40)->3 (all light, priority-ordered).
#
# Pair with: isolcpus=3-5 nohz_full=3-5 rcu_nocbs=3-5 at boot (extlinux.conf) and
#            scripts/pin_network_irqs.sh (keep WiFi softirqs off 3-5).
# Docs: docs/locomotion/SONIC_REAL_BRINGUP.md §6a.
#
# SAFETY: SONIC drives the motors. Robot on a hoist + hardware E-stop in hand for the
# first runs. Exactly one g1_deploy_onnx_ref may write rt/lowcmd (this script kills any
# stale one first); Unitree's stock high-level/sport service must be released.
#
# Usage:
#   scripts/start_sonic_pinned.sh                 # foreground
#   TMUX=1 scripts/start_sonic_pinned.sh          # detached tmux session 'sdeploy'
# Env overrides:
#   SONIC_DIR   (default ~/groot/sonic-g1-locomotion)
#   GR00T_DIR   (default ~/groot/GR00T-WholeBodyControl)
#   SONIC_CORES (default 3-5)   SONIC_DEPLOY (default scripts/start_deploy_real.sh)
set -uo pipefail

SONIC_DIR="${SONIC_DIR:-$HOME/groot/sonic-g1-locomotion}"
GR00T_DIR_DEFAULT="$HOME/groot/GR00T-WholeBodyControl"
SONIC_CORES="${SONIC_CORES:-3-5}"
SONIC_DEPLOY="${SONIC_DEPLOY:-scripts/start_deploy_real.sh}"
LOG="${SONIC_LOG:-/tmp/sonic_deploy_real.log}"

[[ -d "$SONIC_DIR" ]] || { echo "!! SONIC_DIR not found: $SONIC_DIR" >&2; exit 1; }
cd "$SONIC_DIR"
[[ -f "$SONIC_DEPLOY" ]] || { echo "!! deploy script not found: $SONIC_DIR/$SONIC_DEPLOY" >&2; exit 1; }

# Per-boot prep: free the unified memory TensorRT needs; ensure exactly one controller.
sudo -n bash -c "sync; echo 3 > /proc/sys/vm/drop_caches" 2>/dev/null || \
  echo ">> note: could not drop_caches (needs sudo) — continuing" >&2
pkill -9 -f "[g]1_deploy_onnx_ref" 2>/dev/null || true

# GR00T_DIR MUST be passed INTO the launched command: a detached tmux/non-interactive
# shell does not inherit ad-hoc exports or ~/.bashrc, so env.sh would use its wrong
# default. SONIC_CPU_* map each RT worker to a core (see header). taskset -c ${SONIC_CORES}
# confines ALL threads — including the DDS LowState receive threads — to the RT cores.
CMD="GR00T_DIR=${GR00T_DIR:-$GR00T_DIR_DEFAULT} \
SONIC_CPU_MAIN=3 SONIC_CPU_INPUT=3 SONIC_CPU_PLANNER=3 SONIC_CPU_WRITER=4 SONIC_CPU_CONTROL=5 \
taskset -c ${SONIC_CORES} ${SONIC_DEPLOY}"

echo ">> starting SONIC pinned to cores ${SONIC_CORES} (3/3 split); log -> ${LOG}"
if [[ "${TMUX:-0}" == "1" ]]; then
  tmux kill-session -t sdeploy 2>/dev/null || true
  tmux new-session -d -s sdeploy "$CMD > ${LOG} 2>&1"
  echo ">> detached tmux 'sdeploy'. Waiting for Init Done ..."
  for _ in $(seq 1 25); do grep -qE "Init Done|out of memory|LowState is not available" "$LOG" 2>/dev/null && break; sleep 2; done
else
  # Foreground: run in background just long enough to verify pinning, then re-attach output.
  bash -c "$CMD" > "$LOG" 2>&1 &
  BGPID=$!
  echo ">> pid $BGPID; waiting for Init Done ..."
  for _ in $(seq 1 25); do grep -qE "Init Done|out of memory|LowState is not available" "$LOG" 2>/dev/null && break; sleep 2; done
fi

# ── Verify the pinning actually took (fail loudly if a thread escaped to 0,1,2) ──
PID=$(pgrep -f "[g]1_deploy_onnx_ref" | head -1)
if [[ -n "$PID" ]] && grep -q "Init Done" "$LOG" 2>/dev/null; then
  bad=0
  for t in /proc/$PID/task/*; do
    allowed=$(awk '/Cpus_allowed_list/{print $2}' "$t/status" 2>/dev/null)
    case "$allowed" in ""|3|4|5|3-5|3,4,5|4,5|3,4) : ;; *) echo "!! thread $(cat $t/comm) allowed on $allowed (expected within 3-5)"; bad=1 ;; esac
  done
  if [[ $bad -eq 0 ]]; then
    echo ">> OK: all controller threads (incl. LowState recvUC/dq) confined to cores 3-5."
  else
    echo "!! WARNING: some controller threads are NOT confined to 3-5 — LowState fall risk!" >&2
  fi
  grep -E "\[RT\]" "$LOG" 2>/dev/null | tail -4
else
  echo "!! SONIC did not reach 'Init Done' — check ${LOG}:" >&2; tail -8 "$LOG" >&2
  exit 1
fi

# Foreground mode: stream the controller log so this terminal owns its lifetime.
[[ "${TMUX:-0}" == "1" ]] || { echo ">> streaming ${LOG} (Ctrl-C stops the controller) ..."; wait ${BGPID:-} 2>/dev/null || tail -f "$LOG"; }
