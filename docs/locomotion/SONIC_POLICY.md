# SONIC walking policy — setup & integration plan (G1, sim2sim first)

Goal: run **NVIDIA GEAR-SONIC** as the **walking (locomotion) policy** for the G1,
first in **sim2sim**, driven by velocity commands — eventually replacing the AMO
gait so the A\*+MPC nav stack can drive SONIC exactly as it drives AMO today.

Status: **nav→SONIC bridge IMPLEMENTED; sim2sim/hardware validation pending.**
The ZMQ command schema is now resolved (the SONIC deploy repo is on this machine
and its `planner`/`command` wire format is sim2sim-validated), and the Navigation
stack ships a `gait:=sonic` bridge (`cmd_vel_to_sonic_node`, §5). What remains is
operator-side: build the SONIC C++/TensorRT deploy runtime, then run the plan's
validation sequence (drive the bridge in sim2sim, then on hardware). Milestones in
§6 track this.

> Priority note: the SONIC **deploy runtime** (C++/TensorRT) is a separate build
> that is NOT part of this repo — the bridge here only produces the ZMQ commands
> it consumes. Test the **current** AMO-based navigation first
> (see [Appendix B](#appendix-b--test-the-current-nav-first)); bring SONIC up in
> sim2sim before any hardware use.

---

## 1. Why SONIC is not a drop-in for AMO

| | AMO (current) | SONIC (GEAR-SONIC / GR00T-WBC) |
|---|---|---|
| Nature | velocity-conditioned **gait** | motion-tracking **foundation model** + kinematic **locomotion planner** |
| Runtime | Python driver `amo/amo_inference.py` | **C++ / TensorRT** stack (`deploy.sh` / `just run …`) |
| Native command | `{vx,vy,yaw}` JSON on WebSocket `:8766` | keyboard / gamepad / **ZMQ** / ROS 2, fed into the planner |
| Control rate | ~50 Hz | ~50 Hz policy, ~10 Hz planner |
| Setup | existing `amo_policy` container | clone + build C++, download ONNX checkpoints, obs config; JetPack 6 on the robot for real deploy |

The key enabler: SONIC's **kinematic planner** (`planner_sonic.onnx`) accepts
velocity + heading commands, and there are `zmq` / `zmq_manager` / `ros2` input
modes. So the nav stack *can* drive SONIC's walking — the AMO WebSocket bridge is
replaced by a **`/mpc/cmd_vel` → SONIC movement-command** bridge (see §5).

Sources: <https://huggingface.co/nvidia/GEAR-SONIC>,
<https://github.com/NVlabs/GR00T-WholeBodyControl>,
<https://nvlabs.github.io/GR00T-WholeBodyControl/>.

---

## 2. Prerequisites (sim2sim, from scratch)

Host: an x86 box with an NVIDIA GPU + recent driver, Docker, and (for building the
C++ stack) the toolchain the repo expects (`just`, CMake, TensorRT). SONIC uses
Isaac Lab for training/eval, but **sim2sim deployment only needs the ONNX runtime
path**, not full Isaac training.

1. **Clone the deploy repo**
   ```bash
   cd ~/TalosRoboticsAI/g1
   git clone https://github.com/NVlabs/GR00T-WholeBodyControl.git
   cd GR00T-WholeBodyControl
   ```
2. **Download the deployment checkpoints** (policy ONNX + planner):
   ```bash
   python download_from_hf.py            # policy + planner → gear_sonic_deploy/
   # or the low-latency variant used by the deploy examples:
   python download_from_hf.py --low-latency
   ```
   Produces (names per the model card):
   `model_encoder.onnx`, `model_decoder.onnx`, `planner_sonic.onnx`,
   `low_latency/{model_encoder,model_decoder}.onnx`, `low_latency/last.pt`,
   and `policy/low_latency/observation_config.yaml`.
3. **Build the C++ deployment stack** — follow the repo's Installation (Deployment)
   page: <https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/installation_deploy.html>
   (TensorRT + `just`/CMake). **TODO:** capture the exact build commands here once run.

---

## 3. Sim2sim smoke test (confirm it walks, no nav yet)

Bring SONIC up in simulation and drive it by hand first, to confirm the checkpoints
+ obs config + planner produce stable walking before wiring in autonomy.

```bash
# from the deploy dir; encoder/decoder share the --cp prefix (deploy.sh appends
# _encoder.onnx / _decoder.onnx). --obs-config selects the G1 observation layout.
./deploy.sh --cp policy/low_latency/model \
            --obs-config policy/low_latency/observation_config.yaml \
            --input-type keyboard \
            --planner-file <path/to/planner_sonic.onnx> \
            sim
```
- Keyboard (from the docs): `T` play motion, `N`/`P` switch sequence, `Q`/`E`
  heading, `O` **emergency stop**.
- Gamepad mode (`--input-type gamepad`): left stick = movement direction, right
  stick = facing. Confirms the **locomotion planner** takes velocity/heading — the
  interface we'll target for autonomy.

**Exit criterion:** SONIC walks in sim under keyboard/gamepad without falling.

---

## 4. Command interface for autonomy (RESOLVED)

We inject the MPC's `(vx, vy, wz)` into SONIC's planner over **ZMQ**. The wire
format is now pinned down (from the SONIC deploy repo, sim2sim-validated) and
encoded in [`sonic_wire.py`](../ros2_ws/src/g1_sim_bridge/g1_sim_bridge/sonic_wire.py):

- **Transport:** ZMQ **PUB** socket bound at `tcp://*:5556`; the SONIC deploy
  controller SUBs it (`--input-type zmq_manager`).
- **Message layout:** `topic_bytes + 1280-byte null-padded JSON header +
  little-endian payload`. The header field order MUST match the payload order.
- **Topics / fields:**
  - `command` — `start:u8, stop:u8, planner:u8` (enter/leave planner mode).
  - `planner` — `mode:i32, movement:f32[3], facing:f32[3], speed:f32, height:f32`
    (+ optional `upper_body_position/velocity:f32[17]` to hold arms while walking).
    `movement`/`facing` are **world-frame direction unit vectors**; `speed=-1` /
    `height=-1` mean "use the mode default".
- **Frame/sign conventions** (sim2sim-validated): `vx>0` forward, `vy>0` left,
  `wz>0` CCW; **turning requires world frame** (a body-frame command alone never
  updates `facing`). The planner realises ~0.85x commanded m/s (`speed_gain`
  corrects).

Real-robot DDS wiring (`rt/lowcmd` etc.) lives inside the SONIC deploy runtime,
not this repo — the bridge only produces the ZMQ commands above.

---

## 5. The nav → SONIC bridge (IMPLEMENTED)

[`cmd_vel_to_sonic_node`](../ros2_ws/src/g1_sim_bridge/g1_sim_bridge/cmd_vel_to_sonic_node.py)
is the third sibling of the AMO/Unitree bridges — same `/estop` latch and 0.5 s
watchdog, same "run exactly one gait" contract — but its sink is ZMQ (§4) and it
does one extra thing the others don't: a **closed-loop body→world frame
conversion**. The whole A\*+MPC stack upstream is untouched; only the last hop
changes.

**Why the frame conversion (the crux):** the MPC emits a **body-frame** Twist
(vx forward, vy left, wz yaw-rate), but SONIC's `planner` wants **world-frame**
direction vectors, and turning only happens via `facing`. The stock SONIC bridge
derives world heading open-loop (`theta += wz*dt`), which drifts. This node
instead anchors on the **measured DLIO yaw** (`/dlio/odom_node/odom`):

```
dth      = wrap(yaw_meas - yaw0)          # heading vs SONIC's start frame (yaw0 latched at start)
movement = R(dth) . [vx, vy]              # body velocity -> world direction, using MEASURED yaw
face     = dth + wz * facing_lookahead    # inject the MPC's turn intent AHEAD of truth
facing   = [cos(face), sin(face)]
speed    = min(hypot(vx,vy) * speed_gain, max_speed)
```

`facing_lookahead` (default 0.4 s) is **required** — without it `facing` == the
current heading and the robot never turns; it converges back to `dth` as the
measured yaw catches up and the MPC shrinks `wz`. `yaw0` is latched at control
start, so the DLIO odom frame and SONIC's policy-start frame agree.

The 17-DOF **upper-body hold** (carry an object while the legs walk) is exposed as
optional node params `hold_arms` / `arm_preset` (`default` | `carry`) — a
locomanipulation capability the AMO/Unitree bridges don't have.

**Run it** — two equivalent ways:
```bash
# A) one-launch (bridge inside the planner):
ros2 launch a_star_mpc_planner planner.launch.py gait:=sonic
#   sonic_host:=*  sonic_port:=5556  by default (the SONIC deploy controller SUBs it)

# B) standalone bridge via docker/run_sonic.sh (the run_amo.sh / run_unitree.sh
#    equivalent). Start the planner with its own bridge OFF so this is the only
#    driver, then run the bridge:
ros2 launch a_star_mpc_planner planner.launch.py bridge:=false
AUTONOMOUS=1 ./docker/run_sonic.sh          # track /mpc/cmd_vel -> SONIC ZMQ :5556
#   JOYSTICK=1 ./docker/run_sonic.sh        # manual: teleop via /cmd_vel
#   HOLD_ARMS=1 ARM_PRESET=carry ./docker/run_sonic.sh   # carry pose while walking
```
Start the SONIC deploy runtime (which SUBs :5556) first, then set a goal in RViz.
`gait:=sonic` is mutually exclusive with `amo`/`unitree` — run only one.

**Prerequisite in the container:** the bridge needs `pyzmq` in the localization
container (rosdep key `python3-zmq`, declared in `g1_sim_bridge/package.xml`; also
baked into `docker/Dockerfile.localization`). Rebuild the image / run `build_ws`
so the dependency and the new `cmd_vel_to_sonic_node` entry point are present.

The SONIC deploy runtime itself (C++/TensorRT) is out of scope for this repo — it
would be its own `docker-compose` service (mirroring `amo_policy`) once built.

---

## 6. Staged milestones

1. ☐ Clone GR00T-WBC, download checkpoints, build C++ (§2).
2. ☐ Sim2sim keyboard/gamepad walk (§3) — SONIC walks, doesn't fall.
3. ☑ Read the exact ZMQ command schema from source (§4) — encoded in `sonic_wire.py`.
4. ◐ `cmd_vel_to_sonic_node` **written** and wired into `planner.launch.py`
   (`gait:=sonic`); wire-format + body→world math unit-checked. **Still pending:**
   drive SONIC in sim from a manual `/mpc/cmd_vel` (needs the deploy runtime up).
5. ☐ Full sim2sim autonomy: A\*+MPC → bridge → SONIC, goal set in RViz. Confirm
   `facing` tracks DLIO yaw with **no drift** over a long loop and `/estop`/watchdog
   zero the robot.
6. ☐ (later) Real-G1 promotion: JetPack 6 flash, calibration, safety bring-up.

---

## Appendix A — real-robot notes (later)

Real deploy is `./deploy.sh … real` and needs the G1 on **JetPack 6**
(<https://nvlabs.github.io/GR00T-WholeBodyControl/references/jetpack6.html>),
TensorRT on the Jetson, calibration, and its own safety bring-up (SONIC has an
`O`-key e-stop; keep our `/estop` latch in the bridge too). Do **not** promote to
real until sim2sim autonomy (milestone 5) is solid.

## Appendix B — test the current nav FIRST

Before any SONIC work, validate the current AMO-based stack with the changes
already landed (soft-hold, heading blend, sector obstacle selection, y-nav):

```bash
# ROS 2 / localization container
ros2 launch g1_bringup real_localization.launch.py
ros2 launch a_star_mpc_planner planner.launch.py
# amo_policy container
AUTONOMOUS=1 NET_IF=<nic> ./docker/run_amo.sh
# record while navigating, then analyse
/ws/scripts/record_nav_bag.sh run1 --full
python3 /ws/scripts/analyze_nav_bag.py ~/nav_bags/run1
```
Use `nav_analysis.png` to confirm: stop/go gone, no spin at the goal, obstacle
avoidance (which layer, if any, still fails), and lateral motion working.

## Appendix C — future phase: semantic goals (SAM) + frontier search

Out of scope for now, recorded so it isn't lost: prompt a segmentation model
(e.g. SAM) with "find the brown box" → derive a **manipulation-ready stance**
(position + heading offset from the object) → publish it as `/global_goal`; if the
object isn't visible, emit **frontier-exploration goals** until it is. This layers
on top of the existing `/global_goal` interface (see the AgenticNav repo) and needs
the semantic map + a detector-to-goal node — a separate effort after nav + SONIC.
