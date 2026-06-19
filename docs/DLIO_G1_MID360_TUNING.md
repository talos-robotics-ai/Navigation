# DLIO config tuning for the Unitree G1 + Livox MID-360

Which parameters in the DLIO stack you **must** set, **should** review, and can
leave alone for the Unitree G1 wearing a Livox MID-360. DLIO ships sane defaults
tuned for car/handheld Ouster/Velodyne rigs; a torso-mounted, upside-down
MID-360 on a walking humanoid needs a few deliberate changes.

## Where the parameters live

DLIO reads three layered files (later overrides earlier), wired up in
[`sim_localization.launch.py`](../ros2_ws/src/g1_sim_bridge/launch/sim_localization.launch.py):

| File | Owner | Holds |
|------|-------|-------|
| `direct_lidar_inertial_odometry/cfg/dlio.yaml` | upstream (vendored) | feature toggles, IMU intrinsics, extrinsics |
| `direct_lidar_inertial_odometry/cfg/params.yaml` | upstream (vendored) | frames, GICP/keyframe/submap/geo-observer tuning |
| `g1_sim_bridge/config/dlio_sim.yaml` | **ours (sim)** | sim overrides (identity extrinsics, deskew off, use_sim_time) |
| `g1_sim_bridge/config/dlio_mid360_real.yaml` | **ours (real)** | real overrides (R_x(180) IMU, MID-360 extrinsics, deskew on) |

**Rule of thumb:** do not edit the vendored `cfg/*.yaml`. Put every G1/MID-360
change in `dlio_sim.yaml` / `dlio_mid360_real.yaml` so upstream stays clean and
re-cloneable. The tables below say which file each change belongs in.

---

## 1. MUST change — get these right or odometry diverges

### 1.1 Extrinsics (the single most important block) — `dlio_mid360_real.yaml`

DLIO expects two transforms, both expressed **from `base_link`**:
`extrinsics/baselink2lidar` and `extrinsics/baselink2imu`. We define `base_link`
to coincide with the MID-360 mount (the URDF `mid360_link`), so:

```yaml
# baselink2lidar: base_link IS the lidar mount -> identity.
extrinsics/baselink2lidar/t: [ 0.0, 0.0, 0.0 ]
extrinsics/baselink2lidar/R: [ 1.,0.,0., 0.,1.,0., 0.,0.,1. ]

# baselink2imu: the MID-360's internal lidar->IMU offset, PLUS the upside-down
# mount rotation R_x(180) = diag(1,-1,-1).
extrinsics/baselink2imu/t: [ -0.011, -0.02329, 0.04412 ]   # VERIFY (see below)
extrinsics/baselink2imu/R: [ 1., 0., 0.,
                             0., -1., 0.,
                             0., 0., -1. ]
```

- **Rotation `R = R_x(180)`** corrects the upside-down mount. The MID-360 on the
  G1 has `MID360_config.json: extrinsic_parameter.roll = 180`, so the stock
  driver rotates the **cloud** upright but leaves the **IMU raw (inverted)**.
  DLIO's `baselink2imu` rotation genuinely re-orients the IMU into `base_link`
  (FAST-LIO's `extrinsic_R` could not — that is the whole reason this works at
  config level now, with no driver patch).
  - **Verify on the robot:** `ros2 topic echo --once /livox/imu` →
    `linear_acceleration.z ≈ -9.8` at rest means the IMU is still inverted at the
    source (correct — DLIO's R_x(180) is then doing the fix). If it reads
    `+9.8`, the driver is already rotating the IMU and you must set `R = identity`
    instead, or you will double-rotate.
- **Translation `t`** is the MID-360's lidar→IMU lever arm. The `[-0.011,
  -0.02329, 0.04412]` value is carried over from the FAST-LIO config; **confirm
  it against the Livox MID-360 user manual** (the IMU sits a few cm below the
  optical center) and the sign convention DLIO uses (base_link→imu). A wrong
  lever arm shows up as small position wobble that scales with angular rate.

> **If you re-define `base_link = pelvis` instead** (so DLIO's `odom→base_link`
> drives the robot model directly): set `baselink2lidar` and `baselink2imu` to
> the full `pelvis→mid360` / `pelvis→imu` transforms from the URDF, and update
> `robot_model.launch.py`'s `attach_frame`/static TF accordingly. The current
> setup keeps `base_link = mid360_link` (simpler, sensor-centric).

### 1.2 Deskew (motion undistortion) — already split sim vs real

```yaml
# dlio_mid360_real.yaml
pointcloud/deskew: true     # real MID-360 cloud has per-point timestamps
# dlio_sim.yaml
pointcloud/deskew: false    # Isaac clouds are instantaneous snapshots
```

DLIO auto-detects `SensorType::LIVOX` from the per-point `timestamp` field
(>1e14 ns) and deskews correctly on the **real** robot. In **sim**, Isaac's
plain-XYZ cloud has no time field → DLIO would set sensor `UNKNOWN` and disable
deskew anyway; we set `false` explicitly so it never synthesizes motion that did
not happen (which injects distortion while the robot turns).

---

## 2. SHOULD review — defaults work but a humanoid benefits from tuning

### 2.1 Self-point crop — `cropBoxFilter/size` (in `params.yaml`; override per file)

```yaml
odom/preprocessing/cropBoxFilter/size: 1.0   # default: removes a 1 m cube around the sensor
```

DLIO drops every point inside a `±size` cube centered on the lidar to reject the
carrier's own body. On a car that is the roof; on the **G1 the MID-360 sits on
the torso and stares at the robot's own arms, head and (looking down) legs**.

- Too **small** → the robot's body leaks into the scan and GICP locks onto
  self-motion (the arms move with the gait) → jitter/drift while walking.
- Too **large** → you also delete legitimate nearby obstacles (a 1 m cube blinds
  the planner to anything within 1 m).
- **Recommendation:** measure the farthest body part the MID-360 sees from its
  mount and set `size` just beyond it (often `0.5–0.8`). Validate by viewing
  `/dlio/odom_node/pointcloud/deskewed` in RViz while the AMO policy swings the
  arms — no body returns should remain. If the robot's body is asymmetric, note
  that DLIO's crop is a symmetric cube (no per-axis box); if that removes too
  much forward space, keep it tight and rely on the planner's own filtering
  downstream.

### 2.2 IMU calibration + a stationary start — `params.yaml`

```yaml
odom/imu/calibration/gyro: true
odom/imu/calibration/accel: true
odom/imu/calibration/time: 3.0     # seconds of STILL data to estimate bias + gravity
odom/imu/approximateGravity: false # false = full calibration (needs stationary start)
```

- DLIO averages the first `calibration/time` seconds of IMU to estimate gyro
  bias, accel bias and the gravity direction. **The robot MUST be standing still
  for that window** (this is the same constraint FAST-LIO had). The AMO startup
  sequence already holds the robot stationary at the start — make sure DLIO is
  launched *before* the robot starts walking, during that hold.
- The MID-360's built-in IMU has been observed to carry a **large steady gyro
  bias** (~0.3 rad/s). 3 s of calibration usually captures it; if odometry yaws
  while standing still, **increase `calibration/time` to 5–8 s** or verify the
  bias is stationary.
- If you ever cannot guarantee a still start, set `approximateGravity: true`
  (uses a coarse gravity estimate) — but prefer a clean stationary init.

### 2.3 Geometric-observer gains for foot-strike impacts — `params.yaml`

```yaml
odom/geo/Kp: 4.5     # position correction
odom/geo/Kv: 11.25   # velocity correction
odom/geo/Kq: 4.0     # orientation correction
odom/geo/Kab: 2.25   # accel-bias adaptation
odom/geo/Kgb: 1.0    # gyro-bias adaptation
odom/geo/abias_max: 5.0
odom/geo/gbias_max: 0.5
```

Walking produces sharp vertical accelerations at every foot-strike. DLIO's
nonlinear geometric observer is generally robust, but if you see the **Z
estimate ramp or the bias terms wander while walking**:
- Lower `Kab`/`Kgb` (e.g. `Kab: 1.0`, `Kgb: 0.5`) so the bias estimates do not
  chase the impact spikes.
- Keep `abias_max`/`gbias_max` as safety clamps.
Change these only after you have a clean stationary run and a clean slow-walk
run — they are a last resort, not a first knob.

### 2.4 Map / voxel resolution — `params.yaml`

```yaml
odom/preprocessing/voxelFilter/res: 0.25   # scan voxel size fed to GICP
map/sparse/leafSize: 0.25                  # published /dlio/map_node/map voxel size
map/dense/filtered: false
```

`0.25 m` is a good indoor default. For tight indoor navigation you may want
finer detail (`0.10–0.15`) at higher CPU cost; for large/outdoor runs go coarser
(`0.3–0.5`) to bound memory. Keep `voxelFilter/res` and `map/sparse/leafSize`
in the same ballpark.

---

## 3. Usually leave alone (review only if you have a problem)

| Param (`params.yaml`) | Default | Note for G1/MID-360 |
|---|---|---|
| `odom/gravity` | `9.80665` | Standard g; fine. |
| `odom/computeTimeOffset` | `true` | Estimates lidar↔IMU time offset; keep on (good for Livox). |
| `odom/keyframe/threshD` / `threshR` | `1.0 m` / `45°` | Keyframe spawn thresholds; fine for slow humanoid motion. Lower `threshD` (~0.5) if mapping feels sparse indoors. |
| `odom/submap/keyframe/{knn,kcv,kcc}` | `10/10/10` | Submap construction; fine. |
| `odom/gicp/*` | (see file) | GICP registration; defaults are robust. Raise `maxIterations` only if registration is failing. |
| `map/waitUntilMove` | `true` | Don't accumulate map until the robot moves; fine. |
| `adaptive` (`dlio.yaml`) | `true` | Adaptive keyframing by spaciousness; keep on. |
| `imu/intrinsics/*` (`dlio.yaml`) | zeros / identity | Leave at zero — `imu/calibration: true` estimates bias online. Only hard-code if you have a bench calibration of the MID-360 IMU. |

---

## 4. Quick checklist

- [ ] `dlio_mid360_real.yaml`: `baselink2imu/R = R_x(180)` and the `t` lever arm
      verified against the MID-360 manual.
- [ ] `/livox/imu` reads `acc.z ≈ -9.8` at rest (confirms the inverted-IMU path).
- [ ] `cropBoxFilter/size` set so the robot's body is removed but close obstacles
      are not (check the deskewed cloud in RViz with the arms moving).
- [ ] Robot stationary for `calibration/time` s before DLIO/odometry start.
- [ ] Stationary run: no yaw drift, no Z ramp. Then slow-walk run before tuning
      `geo/*` gains.

See [`DLIO_DEPLOYMENT_TESTING.md`](DLIO_DEPLOYMENT_TESTING.md) for the ordered
bring-up + test sequence that exercises each of these.
