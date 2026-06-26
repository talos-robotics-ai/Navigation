# DLIO + AMO bring-up & testing sequence

The ordered task list for deploying the G1 navigation stack: get **localization
+ mapping (DLIO)** solid first, *then* layer in **AMO gait** testing. Do the
phases in order — each one's pass criteria are the entry condition for the next.
Don't move a phase forward until its checklist is green.

Legend: 🖥️ = host, 📦 = `localization` container, 🤖 = `amo_policy` container / robot.
Config tuning referenced here lives in
[`DLIO_G1_MID360_TUNING.md`](DLIO_G1_MID360_TUNING.md).

---

## Phase 0 — Build & sanity (do once)

1. 📦 Build the workspace:
   ```bash
   docker compose run --rm localization bash
   build_ws            # rosdep + colcon (DLIO + g1_* + livox_ros_driver2)
   source /ws/install/setup.bash
   ```
   **Pass:** `direct_lidar_inertial_odometry`, `g1_sim_bridge`, `g1_bringup`,
   `g1_description`, `livox_ros_driver2` all build with no errors;
   `ros2 pkg executables direct_lidar_inertial_odometry` lists
   `dlio_odom_node` and `dlio_map_node`.

---

## Phase 1 — Simulated sensor flow (no DLIO yet)

Goal: confirm Isaac is actually emitting a non-empty cloud + IMU + clock.

2. 🖥️ Start Isaac with the G1 + MID-360:
   ```bash
   Navigation/sim/launch_g1_sim.sh
   ```
3. 📦 In the container, check the raw topics:
   ```bash
   bash /sim/verify_topics.sh
   ```
   **Pass:** `/clock`, `/livox/lidar` (PointCloud2, ~10 Hz, **non-empty**),
   `/livox/imu_raw` (Imu, ~100–200 Hz) all advertised and flowing. IMU
   `linear_acceleration.z ≈ +9.81` at rest (sim IMU is upright).
   - If `/livox/lidar` is empty → relaunch Isaac with a known-good profile:
     `launch_g1_sim.sh --lidar-config Example_Rotary`.

---

## Phase 2 — DLIO localization in sim, **stationary**

Goal: DLIO initializes (IMU/gravity calibration) and holds a stable pose with
the robot standing still. This is the make-or-break correctness gate.

4. 🖥️ Keep Isaac running with the robot held still
   (`g1_sim_scene.py --hold-pose` pins the pelvis so the unactuated robot can't
   collapse).
5. 📦 Launch the relay + DLIO + RViz:
   ```bash
   ros2 run g1_bringup run_dlio_debug.sh
   # or: ros2 launch g1_bringup dlio_debug.launch.py
   ```
   Keep the robot **stationary for the first ~3 s** (DLIO IMU + gravity
   calibration, see tuning §2.2).
6. Verify:
   ```bash
   bash /sim/verify_topics.sh        # now also shows /dlio/odom_node/* + /dlio/map_node/map
   ros2 topic echo --once /dlio/odom_node/odom
   ros2 run tf2_ros tf2_echo odom base_link
   ```
   **Pass:**
   - `/dlio/odom_node/odom`, `/dlio/odom_node/pointcloud/deskewed`,
     `/dlio/map_node/map` all publishing.
   - TF `odom → base_link → {livox, livox_imu}` present.
   - With the robot still, the odom pose **does not drift** (position stable to a
     few mm, no yaw creep) over 30–60 s.
   - In RViz (fixed frame `odom`), the deskewed cloud is stable and the robot
     model sits in it correctly.
   - **Fail → drift while standing:** suspect IMU calibration / gyro bias →
     increase `calibration/time` (tuning §2.2); or self-points in the scan →
     `cropBoxFilter/size` (tuning §2.1).

---

## Phase 3 — DLIO localization + mapping while **moving the sensor** (no gait yet)

Goal: confirm odometry tracks translation/rotation and the map accumulates
consistently, decoupled from gait dynamics.

7. 🖥️ Move the sensor gently: either nudge the held pose / teleport the robot
   in Isaac, or drive a few joints, so the MID-360 translates and rotates slowly
   (walking-speed-ish, but smooth).
8. Watch RViz + `/dlio/odom_node/path`.
   **Pass:**
   - Odometry follows the motion with no jumps; returning the sensor near its
     start closes the loop with small drift.
   - `/dlio/map_node/map` grows a coherent (non-smeared, non-double-walled) map.
   - Turning in place does not blow up the estimate (deskew sanity).
9. Save a map to prove the round-trip (sim runs on the default `ROS_DOMAIN_ID=0`,
   so no export is needed here; if the call hangs on `waiting for service`, your
   shell is on a different domain than `dlio_map_node`):
   ```bash
   ros2 daemon stop && ros2 daemon start   # clear a stale CLI daemon if needed
   ros2 service call /save_pcd direct_lidar_inertial_odometry/srv/SavePCD \
     "{leaf_size: 0.05, save_path: '/ws/maps/'}"
   ```
   **Pass:** a `.pcd` is written and looks like the scene in a viewer (see
   [`DLIO_MAP_SAVE_LOAD.md`](DLIO_MAP_SAVE_LOAD.md) § Viewing the saved map).

---

## Phase 4 — AMO gait policy in sim (the first coupled test)

Goal: confirm DLIO stays converged **while the AMO gait runs** — foot-strike
impacts and gait sway are the new stressor.

10. 🤖 Start the AMO policy against the sim (drives the G1 joints):
    ```bash
    docker compose run --rm amo_policy \
      python /workspace/amo/amo_inference.py --config /workspace/config/amo_g1.yaml
    ```
    Sequence matters: **DLIO must already be initialized (Phase 2) and the robot
    standing still when DLIO calibrates**, then let AMO start walking.
11. Command slow forward walking first, then turning, then faster gaits.
    **Pass:**
    - DLIO odometry tracks the walk without divergence; **Z does not ramp** and
      yaw does not creep per step.
    - The map stays consistent (no per-step smearing).
    - **Fail → Z ramp / bias wander during gait:** lower `geo/Kab`,`Kgb`
      (tuning §2.3); re-check `cropBoxFilter` removes the swinging arms.
12. (Optional) Drive a goal/waypoint to exercise the full loop once a planner is
    wired in (planner is out of scope for this repo today — deferred).

> Phases 1–4 fully validate the **sim** path. Do not go to the real robot until
> Phase 4 is green.

---

## Phase 5 — Real robot: sensor bring-up + IMU/extrinsic sanity

Goal: the physical MID-360 produces the data DLIO expects, and the inverted-IMU
fix is correct **before** any motion.

13. 🤖 Bring up the Livox driver on the real robot. The real localization launch
    starts the driver in `xfer_format=0` (PointCloud2 `/livox/lidar` with
    per-point timestamps) + raw `/livox/imu`, then DLIO with the real config:
    ```bash
    # driver only, to inspect the sensor first:
    ros2 launch g1_bringup real_localization.launch.py rviz:=false robot_model:=false
    #   (or launch just the driver: ros2 launch livox_ros_driver2 msg_MID360_launch.py
    #    after setting xfer_format=0 — real_localization.launch.py already does this)
    ```
14. **Inverted-IMU check (critical):**
    ```bash
    ros2 topic echo --once /livox/imu      # linear_acceleration.z should be ≈ -9.8 at rest
    ```
    `-9.8` confirms the IMU is raw/inverted at the source → DLIO's
    `baselink2imu = R_x(180)` in `dlio_mid360_real.yaml` will correct it. If it
    reads `+9.8`, the driver already rotates the IMU → set `baselink2imu/R` to
    identity (tuning §1.1) to avoid a double rotation.
15. Confirm the cloud has timestamps (DLIO detects `SensorType::LIVOX`):
    ```bash
    ros2 topic echo --once /livox/lidar    # fields include 'timestamp'
    ```
    **Pass:** cloud ~10 Hz non-empty with timestamps; IMU ~200 Hz; gyro at rest
    is small/stable (watch for the MID-360's known steady bias — tuning §2.2).

---

## Phase 6 — Real robot: DLIO localization, **stationary**

16. 🤖 Run the full real localization stack (driver + DLIO with
    `dlio_mid360_real.yaml`, `use_sim_time:=false`, deskew on):
    ```bash
    ros2 launch g1_bringup real_localization.launch.py
    ```
    Keep the robot standing still for the first ~3 s (IMU + gravity calibration).
    **Pass:** same criteria as Phase 2 — no drift while standing, TF tree
    correct, deskewed cloud stable. Tune `cropBoxFilter/size` to the real body
    extent (tuning §2.1).

---

## Phase 7 — Real robot: DLIO + AMO gait, **teleoperated with the Unitree joystick** (the deployment goal)

Goal: drive the G1 around by hand with the Unitree gamepad while DLIO localizes
and maps. The velocity-command path is the **same `/cmd_vel` pipeline used in
sim**, with a WebSocket bridge into the (ROS-free) AMO container:

```
Unitree gamepad ──js0──> joy_node ──/joy──> joy_to_cmdvel ──/cmd_vel──> cmd_vel_to_amo ──ws:8766──> amo_inference ──DDS──> G1
```

All three containers use host networking, so DDS (domain 0) and the WebSocket
(`127.0.0.1:8766`) are shared across them and the host. (In sim the Isaac
`--stabilize` AMO loop subscribes `/cmd_vel` directly, so no bridge is needed —
see [`simulation_stack.md`](simulation_stack.md).)

**Prerequisites**
- Phase 6 green: DLIO localizes stably while the robot stands.
- The Unitree controller is paired to the PC as a standard gamepad — confirm
  `ls /dev/input/js*` shows `js0`. The `localization` container is `privileged`,
  so it already has device access; no extra mount needed.

17. 📦 Start the real localization stack (Livox driver + DLIO + local map + RViz):
    ```bash
    ros2 launch g1_bringup real_localization.launch.py
    ```
    Keep the robot still for the first ~3 s (IMU + gravity calibration).

    > **No pre-DLIO ground filter.** DLIO consumes the **raw** `/livox/lidar`
    > cloud — the old `ground_removal` node (and the `ground_removal:=` arg) were
    > removed, because stripping the ground upstream robs the LiDAR-inertial
    > odometry of its pitch/roll/Z constraint. Ground removal now lives
    > **downstream** in `g1_local_map`, gravity-aware on the accumulated cloud
    > (see [`GROUND_REMOVAL_PLAN.md`](GROUND_REMOVAL_PLAN.md) /
    > [`LOCAL_VOXEL_MAP.md`](LOCAL_VOXEL_MAP.md) §3).

18. 🤖 Start the AMO policy in **joystick (WebSocket) mode**. `JOYSTICK=1` makes
    `run_amo.sh` both (a) launch the gamepad→`/cmd_vel`→WebSocket nodes detached
    in the `localization` container (`real_teleop.launch.py`) and (b) run
    `amo_inference` with `--command_source websocket`:
    ```bash
    NET_IF=<robot-nic> JOYSTICK=1 ./docker/run_amo.sh
    ```
    Equivalent manual form (two terminals) — useful for a held deadman:
    ```bash
    # amo_policy container — AMO listens for velocity on ws:8766:
    NET_IF=<robot-nic> ./docker/run_amo.sh --command_source websocket
    # localization container — gamepad -> /cmd_vel -> bridge:
    ros2 launch g1_sim_bridge real_teleop.launch.py                 # always-on
    ros2 launch g1_sim_bridge real_teleop.launch.py deadman_button:=4   # SAFER: hold LB to move
    ```
    The AMO **activation sequence runs automatically** (low-gain posture hold →
    S-curve ramp to standing → gains soft→full → `stabilize_s` settle); the
    velocity command is **held at zero** through that window and only eases in
    afterwards, so DLIO calibrates during the stationary hold *before* the robot
    can move. Do not touch the sticks until it logs the walk phase.

19. Drive: **left stick** = translate (forward / strafe), **right stick X** =
    turn. Start slow, then turning, then normal gait.
    **Pass:**
    - DLIO stays converged through gait impacts (no Z ramp, no per-step yaw
      creep); odometry matches the physical path on a known loop.
    - Returning to start closes the loop within a small drift.
    - The gamepad reliably drives `(vx, vy, yaw)` — verify with
      `ros2 topic echo /cmd_vel` while moving the sticks. If signs feel mirrored,
      flip `invert_vx/vy/yaw` on `joy_to_cmdvel` (axes follow the SDL/Xbox layout).
    - **Fail (odometry):** apply §2.1 (crop — must remove the swinging arms),
      §2.2 (calibration/gyro bias), §2.3 (geo gains) in that order; re-verify a
      stationary run first after any change.

**Stop / e-stop:** Ctrl-C `amo_inference` — it damps the motors on exit. Then
stop the teleop nodes: `docker exec localization pkill -f real_teleop`.

---

## Phase 8 — Mapping & (future) map-based localization

20. 🤖 Joystick the robot along a full route, then `save_pcd` the accumulated map.
    The **real** stack runs on `ROS_DOMAIN_ID=42` (set by
    `real_localization.launch.py`), so the saving shell must join that domain or
    `/save_pcd` is invisible and the call hangs on `waiting for service`:
    ```bash
    export ROS_DOMAIN_ID=42                 # real stack domain
    ros2 daemon stop && ros2 daemon start   # drop any stale domain-0 CLI daemon
    ros2 service list | grep save_pcd       # sanity: /save_pcd must appear
    ros2 service call /save_pcd direct_lidar_inertial_odometry/srv/SavePCD \
      "{leaf_size: 0.05, save_path: '/ws/maps/'}"
    ```
    Watch `/dlio/map_node/map` and `/local_voxel_map/*` grow coherently in RViz
    as you drive.
    **Pass:** the saved map is metrically consistent with the environment.
21. *(Deferred)* Map-based re-localization: DLIO has **no** prebuilt-map
    localization mode (only odometry + online map + `save_pcd`). If drift-free
    re-localization against a fixed map is later required, that is a separate
    add-on (e.g. a scan-to-map matcher) — not part of this stack today.

---

## Quick gate summary

| Phase | Gate (must be green to proceed) |
|---|---|
| 0 | Workspace builds; DLIO executables present |
| 1 | Isaac cloud non-empty + IMU + clock flowing |
| 2 | **Sim, stationary: DLIO pose stable, no drift** |
| 3 | Sim, moving: odom tracks, map coherent, `save_pcd` works |
| 4 | **Sim + AMO gait: DLIO stays converged while walking** |
| 5 | Real sensor: `/livox/imu` z ≈ −9.8, cloud has timestamps |
| 6 | **Real, stationary: DLIO pose stable** |
| 7 | **Real + joystick AMO gait: gamepad drives `/cmd_vel`, DLIO converged on a known loop** |
| 8 | Map saved + metrically consistent |
