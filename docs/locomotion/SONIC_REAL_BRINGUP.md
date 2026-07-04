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
# terminal 1  (in the SONIC deploy repo)
cd ~/groot/sonic-g1-locomotion
tmux new-session -d -s sdeploy "scripts/start_deploy_real.sh > /tmp/sonic_deploy_real.log 2>&1"
until grep -qE "Init Done|out of memory" /tmp/sonic_deploy_real.log; do sleep 2; done
grep "\[RT\]" /tmp/sonic_deploy_real.log     # expect FIFO+pin lines for all 4 workers
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

## 7. Troubleshooting

| symptom | check / fix |
|---|---|
| robot stands but **won't walk** on a goal | bridge started **before** the controller → it missed `start=1`. Bring the controller up first (Step A), then restart the bridge (re-run `autonomy.sh`, or the §5 standalone bridge). |
| bridge logs but robot **doesn't move at all** | controller not SUBbed to `:5556` (check it's up, `--zmq-host localhost`), or `/mpc/cmd_vel` is zero — check `ros2 topic echo /mpc/cmd_vel`. |
| bridge never hears goals / e-stop does nothing | a terminal is on the wrong DDS domain — `export ROS_DOMAIN_ID=42` everywhere. |
| controller hangs at `LowState is not available` | robot link/state down — `ping 192.168.123.161`, robot not E-stopped, Unitree sport service released. |
| `CUDA … out of memory` at controller launch | Jetson memory pressure — `sudo bash -c "sync; echo 3 > /proc/sys/vm/drop_caches"`, relaunch. |
| robot **slower** than commanded | expected ~0.85×; `speed_gain` (default 1.18) compensates, and the MPC closes the speed loop from odom. |
| robot **won't turn** | `facing_lookahead_sec` must be > 0 (default 0.4). It's the whole turning mechanism in the closed-loop bridge. |
| DLIO pose drifts / diverges at start | robot moved during the ~3 s IMU/gravity init — restart localization with the robot held still. |
| periodic `soft-holding` / `MotorCommand stale` | SONIC state logging left on (keep OFF on hardware) or memory pressure. |
| `[RT]` lines missing | launched from a stale shell — `ulimit -r` must be 99; re-login/fresh tmux after `setup_rt_limits.sh`. |

---

## 8. Quick reference (happy path)

```bash
# per boot
sudo bash -c "sync; echo 3 > /proc/sys/vm/drop_caches"
pkill -9 -f "[g]1_deploy_onnx_ref" 2>/dev/null || true

# term 1 — SONIC controller FIRST (robot HOISTED)
cd ~/groot/sonic-g1-locomotion && scripts/start_deploy_real.sh    # wait for "Init Done"

# term 2 — localization + planner + SONIC bridge + e-stop, one command (robot STILL ~3 s)
cd ~/Navigation/ros2_ws && GAIT=sonic ./autonomy.sh               # e-stop: s=STOP g=GO q=quit

# then: RViz "2D Goal Pose" -> /global_goal
```

Related: [SONIC_POLICY.md](SONIC_POLICY.md) (integration + sim), [UNITREE_GAIT.md](UNITREE_GAIT.md)
(gait selection), [BUILDING.md](../system/BUILDING.md) (Jetson build), and the SONIC repo's
`docs/bringup.md` (the controller side).
