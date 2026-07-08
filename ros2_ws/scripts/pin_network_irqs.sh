#!/usr/bin/env bash
# Keep WiFi network interrupt + softirq processing on the nav cores (0,1,2) and OFF the
# SONIC controller's RT cores (3-5). Complements the 3/3 CPU pinning + isolcpus:
# taskset confines PROCESS threads, but hardware IRQs and their softirqs are steered
# separately. On the distributed split, pulling odom/obstacles over WiFi drives NIC
# RX/TX softirqs; if they fire on cores 3-5 they can delay SONIC's 500 Hz loop /
# LowState receive -> jitter/fall. This pins the WiFi IRQs and RPS/XPS to 0,1,2.
#
# Idempotent. Needs root (sudo). Resets on reboot — run from a boot service or after
# each boot (or rely on isolcpus + kernel irqaffinity; this is belt-and-suspenders).
#
# Usage:  sudo ros2_ws/scripts/pin_network_irqs.sh            # auto-detect wl* iface
#         sudo NAV_CPUS=0,1,2 IFACE=wlP1p1s0 ros2_ws/scripts/pin_network_irqs.sh
set -uo pipefail

NAV_CPUS="${NAV_CPUS:-0,1,2}"
IFACE="${IFACE:-$(ls /sys/class/net 2>/dev/null | grep -E '^wl' | head -1)}"
[[ -n "${IFACE}" && -d "/sys/class/net/${IFACE}" ]] || { echo "!! no WiFi iface found (set IFACE=)"; exit 1; }

# CPU list -> hex mask (for rps_cpus/xps_cpus, which take a hex bitmask).
mask=0
IFS=',' read -ra _c <<< "${NAV_CPUS//-/ }"   # note: expand ranges below if used
for part in ${NAV_CPUS//,/ }; do
  if [[ "$part" == *-* ]]; then
    for ((i=${part%-*}; i<=${part#*-}; i++)); do mask=$(( mask | (1<<i) )); done
  else
    mask=$(( mask | (1<<part) ))
  fi
done
hexmask=$(printf "%x" "$mask")

echo ">> pinning WiFi '${IFACE}' IRQs + RPS/XPS to CPUs ${NAV_CPUS} (mask 0x${hexmask}); keeping off SONIC cores 3-5"

# 1) Hardware IRQ affinity: every IRQ line that mentions the iface (or its PCI device).
irqs=$(grep -iE "${IFACE}|iwlwifi|mmc|wlan" /proc/interrupts 2>/dev/null | awk -F: '{gsub(/ /,"",$1); print $1}')
# also the device's MSI irqs, if present
if [[ -d "/sys/class/net/${IFACE}/device/msi_irqs" ]]; then
  irqs="$irqs $(ls /sys/class/net/${IFACE}/device/msi_irqs 2>/dev/null)"
fi
n=0
for irq in $(echo "$irqs" | tr ' ' '\n' | sort -u); do
  [[ -w "/proc/irq/${irq}/smp_affinity_list" ]] || continue
  if echo "${NAV_CPUS}" > "/proc/irq/${irq}/smp_affinity_list" 2>/dev/null; then
    n=$((n+1))
  fi
done
echo "   set affinity on ${n} IRQ line(s): $(echo $irqs | tr ' ' '\n' | sort -un | tr '\n' ' ')"

# 2) RPS (receive-side softirq steering) + XPS (transmit) per queue.
for q in /sys/class/net/${IFACE}/queues/rx-*; do
  [[ -w "$q/rps_cpus" ]] && echo "${hexmask}" > "$q/rps_cpus" 2>/dev/null || true
done
for q in /sys/class/net/${IFACE}/queues/tx-*; do
  [[ -w "$q/xps_cpus" ]] && echo "${hexmask}" > "$q/xps_cpus" 2>/dev/null || true
done
echo "   RPS/XPS set to 0x${hexmask} on $(ls -d /sys/class/net/${IFACE}/queues/rx-* 2>/dev/null | wc -l) rx / $(ls -d /sys/class/net/${IFACE}/queues/tx-* 2>/dev/null | wc -l) tx queue(s)"

echo ">> done. Verify:  for i in \$(grep -iE '${IFACE}|iwlwifi' /proc/interrupts | awk -F: '{print \$1}'); do echo -n \"irq\$i: \"; cat /proc/irq/\${i// /}/smp_affinity_list; done"
