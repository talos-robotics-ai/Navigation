# Unitree native walking policy — navigation integration

The A*+MPC stack can drive **either** gait, selected at launch:

| gait | how /mpc/cmd_vel reaches the robot | when |
|---|---|---|
| **amo** (default) | `cmd_vel_to_amo_node` → WebSocket :8766 → RoboJuDo AMO **joint** policy | current stack |
| **unitree** | `cmd_vel_to_unitree_loco_node` → Unitree SDK DDS → **native LocoClient** | this doc |

Nothing upstream changes — A*, MPC, the global planner, RViz, `/estop`, the
watchdog are all identical. Only the **last hop** (which gait consumes
`/mpc/cmd_vel`) differs. Run **exactly one** gait (both command the motors).

```
/global_goal ─► A* ─► MPC ─► /mpc/cmd_vel ─┬─(gait:=amo)──► cmd_vel_to_amo ──► WS :8766 ──► AMO joint policy
                                           └─(gait:=unitree)► cmd_vel_to_unitree_loco ─► LocoClient.SetVelocity ─► native gait
```

## "Same joint filtering as AMO" — what it maps to here

AMO is a **low-level** policy: it outputs per-joint position targets, which we
smooth with `amo/joint_filters.py` (S-curve blend + gain ramp + slew clamp +
low-pass) before writing `rt/lowcmd`. The Unitree native gait is **high-level**:
you send it a *velocity* and the firmware owns the joints — **there are no joint
targets to filter**. The correct analog is **velocity-command smoothing**
(ramp-in from zero + slew/accel limit + EMA low-pass), implemented in
`VelocitySmoother` ([unitree_loco.py](../ros2_ws/src/g1_sim_bridge/g1_sim_bridge/unitree_loco.py))
and shared by both the bridge and the test tool — mirroring how AMO shares
`joint_filters.py`.

## Prerequisite: Unitree Python SDK

`gait:=unitree` needs `unitree_sdk2py` in the container that runs the bridge
(the localization container). It's optional (not needed for AMO), so it's left
out of the image by default. Install it (the node prints this if it's missing):

```bash
export CYCLONEDDS_HOME=/opt/ros/humble          # reuse ROS's CycloneDDS (cmake config is there)
pip3 install cyclonedds
pip3 install git+https://github.com/unitreerobotics/unitree_sdk2_python.git
```

**Where to install matters** — `run_unitree.sh` uses `docker compose run --rm`, a
**fresh ephemeral container** each run, so a pip install inside it is discarded.
Pick one:

- **For `run_unitree.sh` (repeatable):** bake it into the image — uncomment the
  ready-made block in `docker/Dockerfile.localization` and `docker compose build
  localization`. This is the AMO parallel (RoboJuDo is baked into the amo_policy image).
- **For a quick test (this session):** install in your **running** `localization`
  container and run the node there via `exec` (not `run_unitree.sh`):
  ```bash
  docker compose exec -it localization bash
  export CYCLONEDDS_HOME=/opt/ros/humble
  pip3 install cyclonedds git+https://github.com/unitreerobotics/unitree_sdk2_python.git
  source install/setup.bash && export ROS_DOMAIN_ID=42
  ros2 run g1_sim_bridge unitree_gait_test --net_if <nic> --bring-up
  ```
  (Persists until that container is `docker rm`'d — survives stop/restart.)

## Enable the native gait on the robot (remote) — REQUIRED FIRST

`LocoClient` only responds when the G1 is in its **factory high-level controller**.
That is the opposite of the low-level (`rt/lowcmd`) mode AMO uses — the two are
**mutually exclusive**, so you switch the robot between them. If the loco RPC
times out (`GetFsmId code=3102`, `send request error`), the robot is still in
low-level mode.

On this G1, the remote sequence to enter the native walking policy is:

```
L2 + B      # 1. damp / release
L2 + Up     # 2. stand up
R2 + A      # 3. enter main (walking) control
```

**Confirm it worked:** the remote's left stick should now walk the robot. Once it
does, the loco service is up and `run_unitree.sh` / `unitree_gait_test` will drive
it. (Stop any AMO/low-level program first; suspend or clear the robot for the
stand-up.) To go back to AMO, return the robot to low-level mode.

## Run via `docker/run_unitree.sh` (the run_amo.sh equivalent)

`run_unitree.sh` mirrors `run_amo.sh` — same env-var UX. The two commands you'll
use most:

### 1. Test the walking policy by hand (impose a reference speed)

Bring the robot up and drive it with a velocity you set by hand — the quickest
way to confirm the gait tracks a commanded speed, with no planner involved.

```bash
cd docker

# walk forward at 0.3 m/s for 5 s (smoothed ramp up → hold → ramp down → stand):
NET_IF=<nic> ./run_unitree.sh --vx 0.3 --duration 5

# other hand-set references (any combination):
NET_IF=<nic> ./run_unitree.sh --vx 0.2 --yaw 0.3 --duration 6   # arc
NET_IF=<nic> ./run_unitree.sh --vy 0.1 --duration 4             # sidestep
NET_IF=<nic> ./run_unitree.sh                                   # just stand (no vel)

# live keyboard control instead of a fixed reference:
JOYSTICK=1 NET_IF=<nic> ./run_unitree.sh    # w/s a/d q/e, space=stop, z=quit
```
`<nic>` = the robot network interface (same one you pass as `NET_IF` to AMO).
The velocity passes through the same `VelocitySmoother` the autonomous bridge
uses, so what you test by hand matches what navigation will feed the gait.

### 2. Autonomous mode (exactly like AMO)

Same as `AUTONOMOUS=1 ./run_amo.sh`, but driving the native gait. Start the
planner **with its own gait bridge OFF** so `run_unitree.sh` is the only driver,
then let it stand and set a goal:

```bash
# terminal 1 — localization + planner (planner's built-in bridge disabled):
ros2 launch g1_bringup real_localization.launch.py
ros2 launch a_star_mpc_planner planner.launch.py bridge:=false

# terminal 2 — the gait driver: track /mpc/cmd_vel → native LocoClient.
#   AUTO_BRING_UP=1 stands the robot automatically; omit it to bring the robot
#   up yourself first (safer): ./run_unitree.sh --bring-up, Ctrl-C, then this.
AUTONOMOUS=1 AUTO_BRING_UP=1 NET_IF=<nic> ./run_unitree.sh

# then set a goal in RViz (2D Goal Pose → /global_goal).
```

Equivalent one-launch alternative (bridge inside the planner, no run_unitree.sh):
`ros2 launch a_star_mpc_planner planner.launch.py gait:=unitree net_if:=<nic>`.

`s`/`g` e-stop and the cmd watchdog work exactly as with AMO.

---

## 1. Test the native gait ALONE first (no navigation)

Confirm the robot stands and walks under the native controller before wiring in
autonomy. Run `unitree_gait_test` (needs the SDK + robot NIC). **Stop AMO first**
— only one controller may drive the motors — and keep the hardware e-stop ready.

```bash
# stand up ready, then hold (Ctrl-C to stop)
ros2 run g1_sim_bridge unitree_gait_test --net_if <nic> --bring-up

# bring up, walk forward 0.3 m/s for 5 s (smoothed), ramp down, stand
ros2 run g1_sim_bridge unitree_gait_test --net_if <nic> --vx 0.3 --duration 5

# keyboard teleop:  w/s=fwd/back  a/d=left/right  q/e=turn  space=stop  z=quit
ros2 run g1_sim_bridge unitree_gait_test --net_if <nic> --teleop

# just damp (release) the robot
ros2 run g1_sim_bridge unitree_gait_test --net_if <nic> --damp-only
```
FSM bring-up sequence used: `Damp → StandUp → SetFsmId(501)`. Unitree documents
FSM `501` as the expert `Walk Motion-3Dof-waist` interface; FSM `500` is the
plain `Walk Motion` interface. The tool verifies the requested FSM before
sending velocity; override with `--control-fsm 500` or `UNITREE_LOCO_FSM=500`
only if you intentionally want the plain walk interface.

## 2. Drive it from the navigation stack

Launch the planner with `gait:=unitree` (and the robot NIC). Do **not** start the
AMO container in this mode.

```bash
ros2 launch a_star_mpc_planner planner.launch.py gait:=unitree net_if:=<nic>
# or via autonomy.sh (planner launch args pass through if you add them there)
```
Then bring the robot to walking control (either `unitree_gait_test --bring-up`
once, or set `auto_bring_up:=true` on the node — off by default for safety), and
set a goal in RViz. The MPC's velocity now flows to the native gait.

The `/estop` keyboard (`s`=stop / `g`=go) works unchanged: on `s` the bridge
sends zero velocity to `LocoClient`; the cmd watchdog also zeros it if the MPC
stops. (Ensure the e-stop node runs on the same ROS domain — autonomy.sh forces
42.)

## Safety

- **One controller only.** AMO and the native gait both write the motors. Never
  run both. `auto_bring_up` is OFF by default so the robot never stands
  unexpectedly.
- The software e-stop is **not** a substitute for the Unitree hardware remote /
  power cutoff — keep it in hand during any real-robot test.
- On shutdown the bridge/test call `StopMove` then `Damp` (release) as a fail-safe.

## Key files

- [unitree_loco.py](../ros2_ws/src/g1_sim_bridge/g1_sim_bridge/unitree_loco.py) — LocoClient wrapper + VelocitySmoother + FSM bring-up
- [cmd_vel_to_unitree_loco_node.py](../ros2_ws/src/g1_sim_bridge/g1_sim_bridge/cmd_vel_to_unitree_loco_node.py) — the bridge (mirrors cmd_vel_to_amo)
- [unitree_gait_test.py](../ros2_ws/src/g1_sim_bridge/g1_sim_bridge/unitree_gait_test.py) — standalone stand/walk tester
- [planner.launch.py](../ros2_ws/src/a_star_mpc_planner/launch/planner.launch.py) — `gait:=amo|unitree`, `net_if:=`
