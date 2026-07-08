# Real-robot bring-up: autonomous navigation on the SONIC gait

End-to-end procedure to run the A\*+MPC navigation stack on the **real Unitree G1**
with **SONIC** as the walking policy (`gait:=sonic`). Everything runs **natively on
the Jetson** (no Docker) — the build set up by `setup_jetson.sh`.

> Read this fully before the first run. SONIC drives the motors; treat every step
> as safety-critical. First feet-down tests happen on a hoist with the hardware
> E-stop in hand.
>
> Prerequisites: the workspace builds ([BUILDING.md](../system/BUILDING.md)), and the SONIC
> deploy runtime (`g1_deploy_onnx_ref`) is built and sim-validated
> ([SONIC_POLICY.md](SONIC_POLICY.md)). This doc is the *real-robot* counterpart
> of the SONIC repo's own `docs/bringup.md` (which covers sim + the SONIC side).

---

## 0. Architecture — three separate comms planes

Nothing in the navigation stack talks to the robot's motors directly. It produces
a velocity command; the bridge translates it to ZMQ; the SONIC controller owns the
motors. Know which plane each process lives on:

```
          ROS 2  (CycloneDDS, ROS_DOMAIN_ID=42)                 ZMQ TCP           Unitree DDS
 ┌───────────────────────────────────────────────┐          (localhost:5556)   (robot ethernet)
 │ livox_ros_driver2 ─► DLIO ─► /dlio/…/odom       │                                enP8p1s0
 │ g1_local_map ─► /local_voxel_map/obstacles      │                             192.168.123.161
 │ a_star_node ─► /a_star/path                      │                                   │
 │ mpc_node ─► /mpc/cmd_vel  (BODY-frame Twist)     │                                   │
 │ estop_keyboard ─► /estop (latched Bool)          │                                   ▼
 │ cmd_vel_to_sonic_node  ◄─ /mpc/cmd_vel + odom + /estop │  ── planner{} ──►  g1_deploy_onnx_ref ──► rt/lowcmd ─► motors
 └───────────────────────────────────────────────┘   PUB :5556 │ SUB :5556           ▲
                                                                                 rt/lowstate ◄─ robot
```

| Plane | Who | Transport | Key setting |
|---|---|---|---|
| **ROS 2 nav** | DLIO, g1_local_map, A\*, MPC, the bridge, e-stop | CycloneDDS | `ROS_DOMAIN_ID=42` on **every** ROS terminal |
| **Bridge → SONIC** | `cmd_vel_to_sonic_node` → `g1_deploy_onnx_ref` | ZMQ PUB/SUB | `:5556`, both on localhost |
| **SONIC → robot** | `g1_deploy_onnx_ref` ↔ G1 | Unitree DDS | iface `enP8p1s0`, robot `192.168.123.161` |

The **bridge is the only bridge between planes** — it subscribes ROS (domain 42)
and PUBs ZMQ. It never touches the robot's DDS. The SONIC controller never sees
ROS. So DDS-domain isolation between the nav stack (42) and the robot link is
automatic.

---

## 1. Safety gates — ALL true before any motion

- [ ] G1 powered, clear of people, **on a hoist/gantry** for first tests.
- [ ] **Hardware E-stop verified and in hand** (Unitree remote / power cutoff). The
      software `/estop` and watchdog are backstops, not a substitute.
- [ ] Robot ethernet up: `ping -c2 192.168.123.161` replies.
- [ ] **Unitree's stock high-level / sport service is released/stopped**, so only
      `g1_deploy_onnx_ref` writes `rt/lowcmd`. Two writers on `rt/lowcmd` is
      dangerous.
- [ ] Exactly **one** `g1_deploy_onnx_ref` will run (`pkill -9 -f "[g]1_deploy_onnx_ref"` first).
- [ ] Livox MID-360 powered and on its network (DLIO needs it).
- [ ] RT limits active in the launching shell (`ulimit -r` = 99) — from a fresh
      login/tmux after `setup_rt_limits.sh`, or SONIC's `[RT]` lines won't appear.
- [ ] Conservative caps for the first runs (see [§5](#5-velocity-caps--first-run-tuning)).

---

## 2. One-time / per-boot prep

```bash
# free the unified memory SONIC/TensorRT needs (frees ~4-5 GB); do this per boot
sudo bash -c "sync; echo 3 > /proc/sys/vm/drop_caches"

# make sure no stale controller is holding the GPU / robot link
pkill -9 -f "[g]1_deploy_onnx_ref" 2>/dev/null || true

# SONIC deploy env: env.sh defaults GR00T_DIR to $HOME/GR00T-WholeBodyControl, but on
# this Jetson it lives under groot/ — set it (add to ~/.bashrc to make it permanent):
export GR00T_DIR=$HOME/groot/GR00T-WholeBodyControl

# ROS side: load ROS + workspace + LIVOX_SDK2_ROOT in every ROS terminal
source nav_env            # (or: source /opt/ros/humble/setup.bash && source ~/Navigation/ros2_ws/install/setup.bash)
export ROS_DOMAIN_ID=42   # nav_env sets 0 by default — the stack REQUIRES 42
```

> `ROS_DOMAIN_ID=42` is mandatory on every ROS terminal (localization, planner,
> e-stop). A node left on domain 0 won't see the others: the bridge won't hear
> `/mpc/cmd_vel` or `/estop`, and DLIO's large PointCloud2 can be corrupted by a
> stray cross-distro participant. The launch files force 42 for *their* nodes; the
> e-stop and any manual `ros2` command you run must set it too.

---

## 3. Bring-up sequence

Run each in its **own terminal/tmux** (long-lived processes; a dropped shell
shouldn't kill them). Verify each step before starting the next.

### Step A — SONIC controller FIRST (the walking policy)

Robot **hoisted**. On the real robot the state source is the **robot itself**
(`rt/lowstate` over ethernet) — there is **no `start_sim.sh`**. When it comes up the
policy takes control and the robot assumes a standing stance, so keep it suspended
and let it settle.

```bash
# terminal 1 — use the wrapper: it sets the 3/3 SONIC_CPU_* mapping (control->5,
# command_writer->4, main+input+planner->3), wraps the launch in `taskset -c 3-5` so
# EVERY thread — including the unpinned CycloneDDS LowState receive threads
# (recvUC/dq.*) — is confined to the RT cores, and VERIFIES that before returning
# (loud warning if any thread escaped to the nav cores). See §6a.
TMUX=1 ~/Navigation/ros2_ws/scripts/start_sonic_pinned.sh
# (the wrapper waits for Init Done and prints the four [RT] lines itself)
```
**Verify:** `Init Done`, four `[RT]` lines, and NOT repeating `LowState is not
available` (that means the robot link/state is down — recheck `ping 192.168.123.161`
and that the robot isn't E-stopped). CRC stays **enabled**, state logging **off**.
The controller binds its ZMQ SUB on `localhost:5556` and now waits for the start
handshake.

> **Why the controller goes first:** `autonomy.sh` (Step B) starts the SONIC
> **bridge**, which sends `command{start=1}` **once** — and a ZMQ PUB **drops it if
> no SUB is connected yet**. Controller up → *then* `autonomy.sh`.

### Step B — `autonomy.sh` with `GAIT=sonic` (localization + planner + bridge + e-stop)

One command brings up localization (DLIO + Livox + local map), waits ~3 s for DLIO's
IMU/gravity init (**keep the robot still**), then the A\*+MPC planner **with the SONIC
bridge** (`gait:=sonic`), records a nav bag, and hands you a **foreground e-stop**.
It self-sources the workspace and forces `ROS_DOMAIN_ID=42`.

```bash
# terminal 2
cd ~/Navigation/ros2_ws
GAIT=sonic ./autonomy.sh
```
The e-stop lives in this terminal: **`s`=STOP `g`=GO `q`=quit** (publishes `/estop`,
which the bridge honours → IDLE). Useful env: `PLANNER_DELAY=<s>` (DLIO settle wait,
default 3), `RECORD_BAG=0` (skip the bag).

**Verify** (each log in its own terminal):
```bash
tail -f logs/localization_latest.log   # DLIO odom live, cloud registering
tail -f logs/planner_latest.log        # "bridging /mpc/cmd_vel (+ yaw ...) -> SONIC ZMQ ..."
                                        #   and "anchored SONIC world frame at yaw0=..."
```
The SONIC controller log (`/tmp/sonic_deploy_real.log`) should start showing planner
ticks / `Loop timing`. With no goal yet the bridge sends **IDLE** — the robot holds
its stance.

> **First feet-down tests:** `autonomy.sh` uses the default velocity caps. To cap
> harder for the first runs, skip it and use the manual conservative path in
> [§5](#5-velocity-caps--first-run-tuning) instead (planner with `bridge:=false` +
> a standalone low-cap bridge).

### Step C — Send a goal

In RViz use **2D Goal Pose → `/global_goal`**. The MPC emits `/mpc/cmd_vel`; the
bridge converts it to world-frame `movement`/`facing` and the robot walks the path.

---

## 3d. Distributed variant — A*/MPC off-board on the laptop

Same SONIC gait, but the **A\*/MPC planner runs on a laptop** instead of the Orin
(docs/planning/DISTRIBUTED_NAV_PLAN.md, Variant A). Use it to keep the Orin light, to
iterate on the planner without a robot rebuild, or when on-board planning risks OOM.
Nothing changes for SONIC — Step A is identical. The split is:

```
 JETSON  perception (DLIO + local_voxel_map) + SONIC gait bridge + ZMQ relay   [cores 0,1,2]
         SONIC controller                                                       [cores 3,4,5]
   │  ZMQ over WiFi  (NO DDS crosses the network — CDR bytes over one TCP socket)
   │   down :5601  odom@20 + /tf + /local_voxel_map/{obstacles,costmap}   (+ clouds if RELAY_CLOUDS=1)
   │   up   :5602  /mpc/cmd_vel
 LAPTOP  A*/MPC planner + ZMQ relay + RViz/Foxglove   (Humble container, matches the Jetson CDR)
```

**Order:** Step A (SONIC controller, pinned 3–5) → the Jetson side → the laptop side →
set a goal on the laptop.

```bash
# terminal 1 (Jetson) — SONIC controller first, exactly as Step A
~/Navigation/ros2_ws/scripts/start_sonic_pinned.sh          # wait "Init Done"

# terminal 2 (Jetson) — perception + gait bridge + relay (NO A*/MPC here). Pins to 0,1,2;
# the FOREGROUND e-stop stays HERE (s=stop g=go q=quit) — the real stop is on the robot.
LAPTOP_IP=<laptop wifi ip> ~/Navigation/ros2_ws/run_distributed_nav_jetson.sh

# terminal 3 (laptop) — A*/MPC + relay + viz (auto-launches the Humble container)
JETSON_IP=10.251.101.176 ./run_distributed_nav_laptop.sh     # add FOXGLOVE=1 for Foxglove
```

Then set a goal on the **laptop** (RViz 2D Goal Pose, or a Pose on `/global_goal` in
Foxglove). `/mpc/cmd_vel` is relayed back to the Jetson bridge → SONIC.

**Seeing the voxel map / obstacles on the laptop.** The Jetson relays
`/local_voxel_map/obstacles` (+ `/costmap`) **by default** — that is the obstacle
data the off-board A\*/MPC consume, and what shows in RViz/Foxglove. The heavier 3D
clouds (DLIO deskewed scan + `local_voxel_map/voxel_grid`) are **OFF by default** to
keep WiFi clear; ship them for a look with:
```bash
RELAY_CLOUDS=1 LAPTOP_IP=<laptop ip> ~/Navigation/ros2_ws/run_distributed_nav_jetson.sh
```
If obstacles/voxels stay **empty** on the laptop, it is almost always upstream on the
Jetson (`local_voxel_map` producing nothing — usually a stalled MID-360), not a
transport problem — see [§7 Troubleshooting](#7-troubleshooting).

> **Safety in the split:** the e-stop is on the **Jetson** terminal (term 2). A laptop
> Ctrl-C only stops the planner → `/mpc/cmd_vel` stops → the bridge's 0.5 s watchdog
> zeros the gait. Never rely on the laptop as the only stop.

---

## 4. What actually "starts" the walking

The SONIC policy *process* is already running after Step A. The **gait starts** the
moment the bridge (started by `autonomy.sh` in Step B) sends
`command{start=1, planner=1}` — one ZMQ message.
From then on each bridge tick sends `planner{mode, movement, facing, speed}`:

```
/mpc/cmd_vel (body vx,vy,wz) + measured DLIO yaw
   └─ cmd_vel_to_sonic_node: rotate body→world by measured yaw, facing = yaw + wz·lookahead
        └─ planner{ movement=[world dir], facing=[cos,sin], speed=|v|·speed_gain }  ──ZMQ──►
             └─ planner_sonic.onnx → reference motion → policy → rt/lowcmd → motors
```

Because `facing` is anchored on **measured** yaw (not open-loop integration), the
heading loop is closed through DLIO — no heading drift over long runs.

---

## 5. Velocity caps & first-run tuning

The `gait:=sonic` launch applies these bridge params (see
[planner.launch.py](../ros2_ws/src/a_star_mpc_planner/launch/planner.launch.py)):

| param | default | meaning |
|---|---|---|
| `max_forward_vel` | 0.5 | forward clamp (m/s) |
| `max_lateral_vel` | 0.12 | strafe clamp (kept at `mpc_vy_max`) |
| `max_yaw_rate` | 0.8 | yaw clamp (rad/s) |
| `speed_gain` | 1.18 | SONIC realises ~0.85× commanded m/s; this corrects it |
| `facing_lookahead_sec` | 0.4 | how far `facing` leads measured heading (**>0 or it won't turn**) |
| `cmd_timeout_sec` | 0.5 | watchdog → IDLE if `/mpc/cmd_vel` goes stale |

**For first feet-down tests, cap harder.** Start the planner with its bridge OFF and
run the bridge standalone with reduced limits, then raise gradually:

```bash
# planner only, no gait bridge (start the SONIC controller first, as in Step A)
ros2 launch a_star_mpc_planner planner.launch.py gait:=sonic bridge:=false

# the SONIC bridge with conservative caps (separate terminal)
ros2 run g1_sim_bridge cmd_vel_to_sonic_node --ros-args \
    -p cmd_vel_topic:=/mpc/cmd_vel -p odom_topic:=/dlio/odom_node/odom \
    -p max_forward_vel:=0.20 -p max_speed:=0.25 -p max_yaw_rate:=0.4
```
Feet-down progression (sign off between stages): hoisted idle → hoisted forward
0.20 m/s 3–4 s → hoisted stop → **pause** → feet-down idle → feet-down tiny
forward/turn at 0.15–0.25 → raise only after clean starts/stops/turns.

**Carry a payload while walking** (optional): add `-p hold_arms:=true` or
`-p arm_preset:=carry`. Verify any arm pose on the hoist first — a pose the policy
can't balance can destabilise the gait.

---

## 6. Stopping & E-stop layers

From fastest/softest to hardest:

1. **`s` (software e-stop)** → `/estop` latched true → bridge sends `MODE_IDLE`,
   robot holds. `g` resumes. Non-destructive.
2. **Command watchdog** → if `/mpc/cmd_vel` stops (MPC dies, planner killed) for
   >0.5 s the bridge auto-sends IDLE.
3. **SONIC watchdog (~0.3 s)** → if the *bridge* dies (ZMQ silent), the controller
   stops advancing the target and soft-holds.
4. **Kill the controller** → `pkill -9 -f "[g]1_deploy_onnx_ref"`; the robot goes
   limp/soft-hold. Follow with a controlled damp/sit.
5. **Hardware E-stop / power** → always the primary. Use it whenever in doubt.

Graceful full shutdown:
```bash
# Ctrl-C / q in the e-stop terminal, then stop the planner launch (Ctrl-C in term 3),
# then the controller, then localization:
pkill -9 -f "[g]1_deploy_onnx_ref"
# (Ctrl-C terminal 1 for localization)
```
On its own exit the bridge sends IDLE then `command{stop=1}` to the controller.

---

## 6a. CPU isolation — prevent LowState starvation (falls)

The Orin Nano has **6 cores**. We use a **3/3 split**: perception + relay own cores
**0,1,2**, the SONIC controller owns **3,4,5**. The controller pins its four RT worker
threads with `SONIC_CPU_*` (in the SONIC repo's `scripts/env.sh`) — but **only** those
four. Two other sets of threads are **not** pinned by `SONIC_CPU_*`:

- the **main thread** (`SONIC_CPU_MAIN`), and
- the **CycloneDDS worker threads** (`recvUC`, `recv`, `dq.builtins`, `rlsnr`, …) that
  actually **deliver LowState** from the robot — these float on the general pool
  (observed on CPUs 0,1 when the controller is started without an outer `taskset`).

So if the nav stack / DDS / remote viz saturate the nav cores, the LowState receive
threads starve and the controller trips:

```
[ERROR] Lost LowState data connection from robot!
[ERROR] Safety check failed, stopping control.
```
→ it stops writing `rt/lowcmd` → **the robot falls.** This is *not* a cable fault
(`ping 192.168.123.161` stays 0% loss); it is CPU contention. It happened once with
the full nav stack + a Foxglove bridge feeding a laptop subscribed to `/livox/lidar`.

**Measured (real G1, 2026-07).** A normal-priority thread (like a DDS LowState receive
thread) on a nav core under load sees **p99 5.6 ms / max 15 ms** scheduling stalls;
on a clean controller core it sees **72 µs**. LowState arrives at ~500 Hz (2 ms), so a
5–15 ms stall drops 2–7 frames → the trip above. With the controller confined to its
own cores, LowState age stayed ~5 ms (p50) with **0 losses** even while the nav cores
ran at 80–94% — so the fix is purely *isolation*, not more compute.

**Why 3/3 and not the old 2/4 (nav on 0,1 · SONIC on 2–5).** Perception load is
motion-dependent: when the robot moves, `local_voxel_map` populates and becomes the
top nav-core cost, pushing 2 nav cores to **94/87%** (near-saturation → relay jitter →
gait stutter). SONIC is GPU-bound (~11% CPU across its cores), so giving it 3 cores
instead of 4 costs it **nothing** (LowState age was identical, 0 losses) while the nav
side dropped to ~80%. Net: hand SONIC's 4th core to the starved nav side.

**The fix — keep the controller's cores exclusively for the controller:**

1. **Confine the *whole* controller (RT loop + main + DDS/LowState threads) to cores
   3–5.** Use the wrapper — it sets `SONIC_CPU_*` (control→5, command_writer→4,
   main+input+planner→3), wraps the launch in `taskset -c 3-5`, and **verifies** every
   thread (incl. `recvUC`/`dq.*`) landed on 3–5 before returning:
   ```bash
   ros2_ws/scripts/start_sonic_pinned.sh          # foreground
   TMUX=1 ros2_ws/scripts/start_sonic_pinned.sh   # detached tmux 'sdeploy'
   ```
   The outer `taskset -c 3-5` is the load-bearing part: `SONIC_CPU_*` alone leaves the
   unpinned DDS/main threads floating onto the nav cores. **Never start SONIC without it.**
2. **Pin everything else to cores 0,1,2.** `autonomy.sh`, `run_distributed_nav_jetson.sh`,
   `run_teleop_jetson.sh` and `start_foxglove.sh` do this automatically
   (`taskset -c 0,1,2`, override/disable with `NAV_CPUS`). Nothing the nav side runs
   touches cores 3–5.
3. **(Optional hardening — needs a reboot) Hard-isolate the controller cores** so even
   kernel housekeeping (kworkers, RCU callbacks, timer ticks) stays off them, shaving
   the last rare tail-jitter spikes. Steps 1+2 (pinning + IRQ steering) already gave
   **0 LowState losses** in testing even at 94% nav load, so this is a "last 5%" layer,
   not a requirement — add it only if you still see rare LowState blips under heavy real
   load. It does **not** cost nav any throughput (nav is pinned to 0,1,2 either way) and
   does **not** slow SONIC (its threads are explicitly pinned; inference is on the GPU).
   Add to the Jetson boot args (`/boot/extlinux/extlinux.conf`, append to **both**
   `APPEND` lines — `DEFAULT` is `JetsonIO`): `isolcpus=3-5 nohz_full=3-5 rcu_nocbs=3-5`,
   then `sudo reboot`. Verify with `cat /sys/devices/system/cpu/isolated` → `3-5`.

Verify the isolation on a running controller (all threads should show PSR 3–5):
```bash
ps -L -o tid,psr,comm -p "$(pgrep -f '[g]1_deploy_onnx_ref')"
```

!!! tip "Heavy viz is safer now, but still be conservative"
    With `taskset -c 3-5` the LowState threads no longer share cores with the nav
    stack, so remote viz is far safer. Cores 0,1,2 are still finite, though — while
    balancing, prefer light Foxglove topics over raw `/livox/lidar` / point clouds.
    See [Remote visualization](../system/REMOTE_VISUALIZATION.md).

---

## 7. Troubleshooting

| symptom | check / fix |
|---|---|
| robot stands but **won't walk** on a goal | bridge started **before** the controller → it missed `start=1`. Bring the controller up first (Step A), then restart the bridge (re-run `autonomy.sh`, or the §5 standalone bridge). |
| bridge logs but robot **doesn't move at all** | controller not SUBbed to `:5556` (check it's up, `--zmq-host localhost`), or `/mpc/cmd_vel` is zero — check `ros2 topic echo /mpc/cmd_vel`. |
| bridge never hears goals / e-stop does nothing | a terminal is on the wrong DDS domain — `export ROS_DOMAIN_ID=42` everywhere. |
| controller hangs at `LowState is not available` | robot link/state down — `ping 192.168.123.161`, robot not E-stopped, Unitree sport service released. |
| controller **dies mid-run** on `Lost LowState data connection from robot` → **robot falls** | The controller's LowState DDS thread was **starved**, not a cable fault (verify: `ping 192.168.123.161` is still 0% loss). Cause is **Jetson overload** — too many heavy processes competing with the 500 Hz control loop. The trigger seen in practice: running **Foxglove remote viz** (`foxglove_bridge` + a laptop subscribed to `/livox/lidar` / point clouds) alongside the full nav stack pushed Orin load to ~3.7 and the controller lost LowState ~20 s later. **Don't run heavy viz/logging while balancing** — see [Remote visualization](../system/REMOTE_VISUALIZATION.md). Keep the robot on the hoist for any run that adds load. |
| `CUDA … out of memory` at controller launch | Jetson memory pressure — `sudo bash -c "sync; echo 3 > /proc/sys/vm/drop_caches"`, relaunch. |
| robot **slower** than commanded | expected ~0.85×; `speed_gain` (default 1.18) compensates, and the MPC closes the speed loop from odom. |
| robot **won't turn** | `facing_lookahead_sec` must be > 0 (default 0.4). It's the whole turning mechanism in the closed-loop bridge. |
| DLIO pose drifts / diverges at start | robot moved during the ~3 s IMU/gravity init — restart localization with the robot held still. |
| **no voxel map / obstacles on the laptop** (distributed) | Debug upstream→downstream, NOT the WiFi. 1) Is `local_voxel_map` producing? Grep its log for the `in=… obstacles=…` heartbeat; `in=0` (or no line) = it's getting no deskewed cloud. 2) Is the MID-360 alive? DLIO log `Sensor Rates: Livox @ ~10 Hz`; if it dropped, the lidar stalled — restart the driver (`pkill -f livox_ros_driver2_node`; it now `respawn`s, or run `scripts/livox_watchdog.sh`). 3) `local_voxel_map` only emits obstacles for **non-ground** points in ±8 m; a bare hoist area near the floor can legitimately yield few. Move something into view. 4) Only then suspect transport: `ros2 topic hz /local_voxel_map/obstacles` **on the Jetson** (may read empty even when live — large-cloud CLI/QoS quirk; trust the relay + laptop RViz). The 3D `voxel_grid` needs `RELAY_CLOUDS=1`. |
| `soft-holding` / `MotorCommand stale` at startup | benign: the controller soft-holds until the gait bridge sends the planner handshake — start `autonomy.sh` promptly after `Init Done`. If it recurs *mid-run*: SONIC state logging left on (keep OFF on hardware) or memory pressure. |
| `[RT]` lines missing | launched from a stale shell — `ulimit -r` must be 99; re-login/fresh tmux after `setup_rt_limits.sh`. |

---

## 8. Quick reference (happy path)

```bash
# per boot
sudo bash -c "sync; echo 3 > /proc/sys/vm/drop_caches"
pkill -9 -f "[g]1_deploy_onnx_ref" 2>/dev/null || true

# term 1 — SONIC controller FIRST (robot HOISTED). The wrapper pins ALL its threads
# (incl. DDS/LowState) to cores 3-5 and verifies it, so it can't be starved -> fall (§6a).
~/Navigation/ros2_ws/scripts/start_sonic_pinned.sh                # wait "Init Done" (wrapper does)

# term 2 — localization + planner + SONIC bridge + e-stop (auto-pinned to cores 0,1,2).
# Start it PROMPTLY after Init Done: the controller soft-holds ("MotorCommand stale")
# until the bridge sends the planner handshake — a long gap is a benign soft-hold.
cd ~/Navigation/ros2_ws && GAIT=sonic ./autonomy.sh               # e-stop: s=STOP g=GO q=quit

# then: RViz "2D Goal Pose" -> /global_goal
```

Related: [SONIC_POLICY.md](SONIC_POLICY.md) (integration + sim), [UNITREE_GAIT.md](UNITREE_GAIT.md)
(gait selection), [BUILDING.md](../system/BUILDING.md) (Jetson build), and the SONIC repo's
`docs/bringup.md` (the controller side).
