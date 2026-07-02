# SONIC walking policy — setup & integration plan (G1, sim2sim first)

Goal: run **NVIDIA GEAR-SONIC** as the **walking (locomotion) policy** for the G1,
first in **sim2sim**, driven by velocity commands — eventually replacing the AMO
gait so the A\*+MPC nav stack can drive SONIC exactly as it drives AMO today.

Status: **PLAN ONLY.** The SONIC deploy stack is not yet on this machine and a few
runtime details (exact ZMQ command schema, ROS 2 topic names, G1 obs/DDS wiring)
must be confirmed against the cloned source — those points are flagged **TODO**
below and must not be guessed at before deployment.

> Priority note: this is deferred. Test the **current** AMO-based navigation first
> (see [Appendix B](#appendix-b--test-the-current-nav-first)). Only start SONIC once
> the current stack behaves.

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

## 4. Command interface for autonomy (the crux)

We need to inject the MPC's `(vx, vy, yaw)` into SONIC's planner programmatically.
Two candidate channels the docs expose:

- **`--input-type zmq_manager`** — the manager accepts "motion index, frame,
  operator state, planner state, **movement commands**" over ZMQ (default port
  **5556**, topic `pose`). This is the most likely velocity-injection path.
- **`--input-type ros2`** (only if the stack was built with ROS 2 support) —
  publishes/subscribes DDS topics directly; could take a Twist-like command with no
  extra bridge.

**TODO (blocking, from cloned source):**
- exact ZMQ **socket type** (PUB/SUB vs PUSH/PULL) and the **movement-command
  message schema** — field names/layout for velocity x/y, yaw/heading, gait, height,
  mode. The docs do not publish it; it's in the C++ manager/ZMQ code.
- for the `ros2` path: the exact **topic names + message types** it consumes/emits,
  and whether the build has ROS 2 enabled.
- the G1 **DDS wiring** it uses on the real robot (`rt/lowstate` / `rt/lowcmd`?),
  for the eventual real-robot promotion.

Until these are read from source, do not write the bridge payload — it would be a
guess, and a wrong joint/command mapping on a humanoid is unsafe.

---

## 5. The nav → SONIC bridge (mirrors `cmd_vel_to_amo`)

Once §4 is pinned down, add a bridge analogous to
[`cmd_vel_to_amo_node.py`](../ros2_ws/src/g1_sim_bridge/g1_sim_bridge/cmd_vel_to_amo_node.py):

- **`cmd_vel_to_sonic_node`** (new node in `g1_sim_bridge`): subscribe `/mpc/cmd_vel`
  (Twist), clip to caps, and **publish the SONIC movement-command over ZMQ**
  (PUB → SONIC's `zmq_manager` SUB on :5556) at ~20 Hz. Keep the **same fail-safe
  watchdog + `/estop` latch** already in the AMO bridge — critical on hardware.
- Reuse the existing planner unchanged: it still emits `/mpc/cmd_vel`. Only the
  *last hop* changes (WebSocket → ZMQ), so the whole A\*+MPC stack is untouched.
- If the `ros2` input path works, we may skip the bridge entirely and have the
  planner's Twist consumed directly — decide after reading §4.

Launch integration: add a `sonic` bridge option to
[`planner.launch.py`](../ros2_ws/src/a_star_mpc_planner/launch/planner.launch.py)
mirroring the `cmd_vel_to_amo` node (arg `gait:=sonic|amo`), and a `sonic_policy`
service in [`docker/docker-compose.yml`](../docker/docker-compose.yml) mirroring
`amo_policy` (its own Dockerfile with the SONIC C++ runtime + TensorRT).

---

## 6. Staged milestones

1. ☐ Clone GR00T-WBC, download checkpoints, build C++ (§2).
2. ☐ Sim2sim keyboard/gamepad walk (§3) — SONIC walks, doesn't fall.
3. ☐ Read the exact `zmq_manager`/`ros2` command schema from source (§4 TODOs).
4. ☐ Write `cmd_vel_to_sonic_node`; drive SONIC in sim from a manual `/mpc/cmd_vel`.
5. ☐ Full sim2sim autonomy: A\*+MPC → bridge → SONIC, goal set in RViz.
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
