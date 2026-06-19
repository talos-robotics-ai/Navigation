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
9. Save a map to prove the round-trip:
   ```bash
   ros2 service call /save_pcd direct_lidar_inertial_odometry/srv/SavePCD \
     "{leaf_size: 0.2, save_path: '/ws/maps/'}"
   ```
   **Pass:** a `.pcd` is written and looks like the scene in a viewer.

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

## Phase 7 — Real robot: DLIO + AMO gait (the deployment goal)

17. 🤖 Follow the **safe AMO activation sequence** (low-gain posture hold →
    S-curve ramp to standing → gains soft→full → stabilize) — DLIO must
    calibrate during the stationary stabilize hold, *before* the first walking
    command.
18. Walk slowly, then turn, then normal gait.
    **Pass:**
    - DLIO stays converged through gait impacts (no Z ramp, no per-step yaw
      creep); odometry matches the physical path on a known loop.
    - Returning to start closes the loop within a small drift.
    - **Fail:** apply §2.1 (crop), §2.2 (calibration/gyro bias), §2.3 (geo gains)
      in that order; re-verify a stationary run first after any change.

---

## Phase 8 — Mapping & (future) map-based localization

19. 🤖 Walk a full route and `save_pcd` the accumulated map (Phase 3 step 9).
    **Pass:** the saved map is metrically consistent with the environment.
20. *(Deferred)* Map-based re-localization: DLIO has **no** prebuilt-map
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
| 7 | **Real + AMO gait: DLIO converged on a known loop** |
| 8 | Map saved + metrically consistent |
