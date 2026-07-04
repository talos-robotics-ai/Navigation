# Distributed navigation: offload A*+MPC to an off-board computer

Plan for splitting the navigation stack so that **perception/odometry runs on the
Jetson** and **global+local planning (A*+MPC) runs on an off-board computer**, with
reference velocities returned to the robot over ZMQ.

> **Status:** proposal. Read the "Do you actually need this?" section first — as of
> 2026-07-03 the Jetson has ~65% idle CPU once the Livox-driver leak is fixed, so
> this is an *optional* architecture change, not a fix for a real-time problem.

---

## 0. Do you actually need this? (read before building)

Measured on the Orin Nano (8 GB) with the full stack + SONIC running, headless, one
Livox driver (leak cleared):

| Metric | Value | Read |
|---|---|---|
| Load average | 3.14 | Healthy for 6 cores |
| CPU idle | **65.7%** | Large headroom |
| SONIC `g1_deploy_onnx` | 40% (rt) | GPU inference cheap (~0.5 ms) |
| `mpc_node` | 30% | 1/3 of one core |
| `a_star_node` | 21% | — |
| `dlio_odom_node` | **6%** | Not a bottleneck |
| GPU (GR3D) | 4–11% | Nearly idle |
| RAM | 3.9 GB used / 7.6 GB | Comfortable headless |

**Conclusion:** the board is *not* the constraint today. The earlier saturation was
5 leaked `livox_ros_driver2_node` processes (~2.5 cores of waste), not the workload.

**Legitimate reasons to still do the split** (none about current compute):
- **Dev iteration speed** — edit/tune A*+MPC on the laptop without rebuilding in the
  Jetson's root-owned container each time.
- **Future headroom** — reserve the Jetson for heavier perception (denser maps,
  learned costmaps, a second sensor) later.
- **Richer planning** — run a heavier planner (sampling-based MPPI, longer horizons)
  that the Nano couldn't sustain.

**Reasons NOT to** (real costs):
- Puts **WiFi in the reactive control path** (obstacle → costmap → MPC → cmd_vel).
  WiFi jitter/dropouts directly perturb the velocity command driving locomotion.
- Adds a serialization/transport layer to build and maintain.
- Two machines to keep in sync and launch in order.

**Recommendation:** don't do the full A*+MPC offload yet. If you want the dev-speed
win, offload only **A\*** (global, slow, replans at ~1–5 Hz) and keep **MPC on the
Jetson** next to the policy (see §6, Variant B). Revisit the full split only if a
future planner genuinely exceeds the Nano.

---

## 1. Current architecture (all on the Jetson)

```
MID-360 ─▶ livox_ros_driver2 ─┬▶ /livox/lidar ───────────────▶ DLIO ─▶ /dlio/odom_node/odom (Odometry)
                              └▶ /livox/imu ─▶ imu_rescale ─▶ /livox/imu_ms2 ─▶ DLIO   + TF odom→base_link
                                                                                 └▶ /dlio/odom_node/pointcloud/deskewed
DLIO deskewed cloud ─▶ local_voxel_map ─▶ obstacle PointCloud2 (ground-removed, ±8 m)
                                          │
      /global_goal (from Foxglove/RViz) ─┤
      odom ───────────────────────────── ┼▶ a_star_node ─▶ /a_star/path
      obstacle cloud ───────────────────-┘        │
                                        /a_star/path + odom ─▶ mpc_node ─▶ /mpc/cmd_vel
                                                                              │
                                            cmd_vel_to_sonic.py (ROS sub) ─▶ ZMQ tcp://*:5556 ─▶ g1_deploy_onnx_ref (SONIC)
```

Verified I/O:
- **A\*** subscribes: `odom` (Odometry), obstacle `PointCloud2` (from local map),
  `/global_goal`; optional external costmap (`Float32MultiArray`) or SLAM
  `OccupancyGrid`. Publishes `/a_star/path` (+ debug grids).
- **MPC** subscribes: `/a_star/path`, `/global_goal`, odom. Publishes `/mpc/cmd_vel`.
- **cmd_vel_to_sonic.py** already bridges ROS `/mpc/cmd_vel` → ZMQ `tcp://*:5556`.

## 2. Proposed split

```
┌───────────────── JETSON (on-robot, real-time, sensor-coupled) ─────────────────┐
│ livox_ros_driver2 → DLIO → odom + TF                                            │
│ DLIO deskewed cloud → local_voxel_map → obstacle cloud                          │
│ g1_deploy_onnx_ref (SONIC policy, 50 Hz, wired to robot)                        │
│ cmd_vel_to_sonic.py (ZMQ bridge into the policy)                               │
└───────▲─────────────────────────────────────────────────────────────▲─────────┘
        │ odom + obstacle cloud + TF  (Jetson → laptop)                │ cmd_vel (laptop → Jetson)
        │                                                              │
┌───────┴────────────── LAPTOP (off-board, planning) ──────────────────┴─────────┐
│ a_star_node → /a_star/path → mpc_node → /mpc/cmd_vel                            │
│ /global_goal set here (Foxglove Publish panel)                                 │
└────────────────────────────────────────────────────────────────────────────────┘
```

What stays on the Jetson (hardware/latency-bound): Livox, DLIO, local_voxel_map,
SONIC policy, the ZMQ→policy bridge. What moves to the laptop: `a_star_node`,
`mpc_node`, goal input.

## 3. Data flows, bandwidth, latency

| Link | Payload | Rate | Size | Notes |
|---|---|---|---|---|
| Jetson→laptop | `odom` (Odometry) | ~10 Hz | ~½ KB | trivial |
| Jetson→laptop | TF `odom→base_link` | ~10–50 Hz | tiny | needed for planning frame |
| Jetson→laptop | obstacle `PointCloud2` | ~10 Hz | ~10s–100s KB | **the heavy link**; already ground-removed & windowed, ≪ raw deskewed cloud |
| laptop→Jetson | `cmd_vel` (Twist) | ~20–50 Hz | tiny | the control-relevant return path |

Budget check: on 5 GHz WiFi the obstacle cloud at 10 Hz is easily within bandwidth.
The concern is **latency/jitter**, not throughput — see §5 (safety).

**Optimization:** instead of shipping the obstacle `PointCloud2`, have the Jetson
compute the 2D costmap and send it via A*'s existing **external-costmap hook**
(`Float32MultiArray`). A fixed-size grid is smaller and constant-bandwidth. This
also keeps the (cheap) costmap build on the Jetson and sends the laptop a
ready-to-plan grid.

## 4. Transport: ZMQ vs ROS-2-over-WiFi

**Do NOT bridge the large clouds over DDS to a Jazzy laptop.** Per
`real_localization.launch.py`, a Jazzy participant on `ROS_DOMAIN_ID=42` corrupts
CycloneDDS deserialization of large PointCloud2 (`serdata.cpp:384`). Two safe options:

- **Option A — ZMQ everywhere (recommended, consistent with existing bridge).**
  Add small ZMQ bridge nodes:
  - Jetson: publish `odom`, TF, and obstacle costmap (`Float32MultiArray`) over ZMQ
    PUB (e.g. `tcp://*:5557`). Reuse the `cmd_vel_to_sonic.py` PUB/SUB pattern.
  - Laptop: a SUB node that re-publishes them as ROS topics into the laptop's ROS
    graph so `a_star_node`/`mpc_node` run unmodified.
  - Laptop: publish `/mpc/cmd_vel` over ZMQ PUB → Jetson SUB → into ROS →
    `cmd_vel_to_sonic.py` (or send straight into the policy's ZMQ if you collapse
    the hop). No DDS crosses the network.
- **Option B — ROS 2 over WiFi in a matching-distro container.** Run the laptop
  planner in a **Humble** container on `ROS_DOMAIN_ID=42` and only cross-publish the
  small topics (odom, TF, costmap grid — not the raw cloud). Less code, but you own
  the DDS-over-WiFi discovery/reliability and must avoid the large-cloud corruption.

Recommendation: **Option A** — it extends what already works and never puts DDS on
the laptop side.

## 5. Safety — WiFi in the loop

- The reactive loop **obstacle → costmap → MPC → cmd_vel → policy** now crosses WiFi.
  A dropout stalls `cmd_vel`. The existing **cmd_vel watchdog** zeros the command on
  staleness (robot stops) — safe, but a flaky link = stuttering gait.
- Keep the **cmd_vel watchdog short** and verify it zeros on link loss (test: pull
  WiFi mid-walk, robot must stop, not run stale).
- Keep the **e-stop on the Jetson side** (it already runs in the autonomy shell), not
  only on the laptop, so a network loss never disables the stop.
- Prefer offloading **A\* only** (Variant B, §6) so the *fast* loop (MPC→cmd_vel)
  stays on-robot and only the *slow* global replan crosses WiFi.

## 6. Implementation plan (phased)

**Variant A — full A*+MPC off-board** (what you asked for):
1. **Bridge out (Jetson→laptop):** ZMQ PUB node for `odom` + TF + obstacle costmap
   (`Float32MultiArray`). Start with odom+TF only; add the costmap once verified.
2. **Bridge in (laptop):** ZMQ SUB → ROS re-publisher so the planners see native topics.
3. **Move launch:** run `a_star_node` + `mpc_node` on the laptop (Humble container,
   domain isolated). Point A* at the external-costmap topic; point MPC at odom.
4. **Bridge cmd_vel back:** laptop ZMQ PUB `/mpc/cmd_vel` → Jetson SUB → feed
   `cmd_vel_to_sonic.py`.
5. **Goal:** set `/global_goal` on the laptop (Foxglove Publish).
6. **Verify:** §7 checklist end-to-end; pull-the-WiFi safety test.

**Variant B — A\* off-board, MPC on Jetson (recommended if splitting at all):**
1. ZMQ bridge out: obstacle costmap + odom to laptop.
2. Laptop runs `a_star_node` only → publishes `/a_star/path`.
3. Bridge `/a_star/path` back to the Jetson (small, path is a few hundred poses).
4. `mpc_node` stays on the Jetson, consumes local `/a_star/path` + odom → `/mpc/cmd_vel`
   → policy. The fast reactive loop never leaves the robot.

## 7. Verification checklist

Run after wiring the bridges (see also `docs/` bringup guides):
- `ros2 topic hz` on the laptop for the bridged `odom` / costmap — matches the
  Jetson's source rate (no decimation from the transport).
- `tf2_echo odom base_link` **on the laptop** succeeds (planning frame present).
- `/a_star/path` and (Variant A) `/mpc/cmd_vel` publish on a set goal.
- End-to-end: goal → robot walks; measure added latency vs on-board baseline.
- **Fault test:** drop WiFi mid-walk → robot stops within the watchdog timeout.

---

## 8. Should you rewrite in CUDA?

**Short answer: no — not now, and not "everything."** It would be premature
optimization of code that isn't the bottleneck.

Grounded reasoning against a wholesale CUDA rewrite:
- **Nothing is CPU-bound.** 65% idle, DLIO at 6%, MPC at 30% of one core. There is no
  real-time deadline being missed. CUDA buys latency only where a *parallel* workload
  is the bottleneck — you don't have one.
- **The real-time-critical path is already optimal.** The 50 Hz SONIC policy runs ONNX
  on the GPU at ~0.5 ms/inference. That's the loop that must never miss, and it's fine.
- **MPC is a *small* problem.** Humanoid velocity MPC is a small QP/NLP. Good CPU
  solvers (OSQP/qpOASES/acados) solve these in well under the control period, often
  *faster* than GPU due to kernel-launch overhead on small matrices. GPU wins only for
  **large batched/sampling** MPC (MPPI with thousands of rollouts) — a different
  algorithm, not a port of the current one.
- **DLIO is CPU by design** (nano-GICP). At 6% it's a non-issue; a CUDA-GICP rewrite is
  a large project for negative practical return here.
- **Cost/risk:** CUDA code is far harder to write, debug, and maintain, ties you to the
  Jetson toolchain, and is easy to get subtly wrong on real-time paths.

**Where CUDA *would* pay off (only if a bottleneck appears):**
- Point-cloud voxelization / ground removal in `local_voxel_map` **if** it grows to
  dense, large-window maps and shows up as the CPU hog (it's a Python node today —
  first move it to C++/PCL, which likely closes the gap without CUDA).
- A future **MPPI / sampling-based** local planner (thousands of parallel rollouts) —
  genuinely GPU-shaped.
- Learned perception (segmentation/costmap) inference — already GPU via ONNX/TensorRT.

**Order of optimization if you ever need it** (cheapest, highest-return first):
1. Fix leaks / correct process hygiene (done — the Livox leak was the whole "problem").
2. Rewrite the **Python** nodes (`local_voxel_map`, `imu_rescale`) in C++ — biggest
   easy win on Arm, no CUDA.
3. Tune DLIO input density / voxel sizes.
4. Offload planning off-board (this doc) for dev speed / headroom.
5. **Only then**, and only for a proven parallel bottleneck, reach for CUDA/TensorRT —
   targeted kernels, not a rewrite.

**Bottom line:** invest in C++ rewrites of the Python nodes and clean process
management before any CUDA work. The Nano is not your limit today.

---

## 9. §8 optimization pass — IMPLEMENTED

Everything below runs **on the Jetson**; no off-board compute, no ZMQ link, no
CUDA. The Python nodes were vectorised instead of rewritten in C++ — measured
on the dev host (Orin Nano is ~3-5× slower per core; ratios carry):

| Hot path | Before | After | Change |
|---|---|---|---|
| `local_voxel_map` accumulator (per scan, 18k pts) | 29.8 ms | 2.4 ms | packed-int64 keys, bulk `np.unique` merge — **12.6×**, output bit-identical |
| MPC obstacle clustering (8k pts, per solve @10 Hz) | 4.4 ms | 0.3 ms | `scipy.ndimage.label` on the rasterised cell patch — **16×**, identical partitions |
| MPCC IPOPT solve (N=50, 14 obs, warm) | p50 32 / p95 38 ms | p50 23 / p95 27 ms | `tol 1e-3` + early-accept (default 1e-8 was polishing digits a 10 Hz Twist can't use); `max_cpu_time 0.3` hang guard. 50/50 success |
| Message serialisation (all nodes) | 15-40k-elem `.tolist()` per cycle | buffer-adopting `array('b'/'f')` / direct `PointCloud2.data = arr.tobytes()` | removes per-element Python conversion; `voxel_grid` (duplicate of `obstacles`) now published only when subscribed |

Benchmarked and **rejected**: IPOPT `mu_init 1e-4` + warm-start bound pushes
(slower recovery solves, 0/50 success when combined with a tight CPU wall) and
a `max_cpu_time` at the 100 ms control period (cold solves on the Nano take
~150 ms → permanent failure cascade; 0.3 s guards hangs without that).

**DLIO deskewed cloud vs local voxel map — decision: keep the map.**
`/dlio/odom_node/pointcloud/deskewed` is a *single* MID-360 scan (further
voxel-downsampled to 0.25 m by DLIO's input filter) — too sparse for per-cell
ground removal (~1 pt/cell → everything fails open) and with no temporal decay
(a person who walked past would persist or vanish scan-to-scan). DLIO's only
other dense product, `/dlio/map_node/map`, updates per keyframe (~1 m / 45°) —
too laggy for reactive avoidance. The local map's accumulate→decay→segment
pipeline is exactly the part DLIO does not provide; at ~2.4 ms/scan it is no
longer a meaningful cost, so replacing it would save nothing and lose the
densified, ground-removed, decaying obstacle cloud every consumer depends on.

**Safety added with this pass** (all in `mpc_node`, on-robot, no network):
- **YIELD state**: a CONFIRMED-dynamic cluster (existing tracker + temporal
  consistency gate) inside — or predicted within `yield_lookahead_sec` to
  enter — the forward corridor (`yield_corridor_length` ×
  ±`yield_corridor_halfwidth`) stops the robot until it has passed (debounced
  both ways). Takes precedence over the security escape: sidestepping around a
  walking person could step into their path.
- **Blind-stop**: past `lidar_max_age_sec` the MPC now *holds the last cloud*
  (odom-frame static structure stays valid) instead of the old behaviour of
  dropping ALL obstacles and planning blind; past `lidar_blind_stop_sec`
  (2.5 s, or if no cloud ever arrived) it hard-stops until the feed recovers.

**Global+local fusion (quality-gated)** — `global_fusion_mode: auto`:
`global_planner_node` now also publishes its **pre-inflation confirmed-hit
cells** (`/global_planner/known_obstacles`: hit-thresholded ≥2 observations,
decay-faded, breadcrumb-carved — the anti-ghost layer). Once ≥
`global_fusion_min_cells` cells are confirmed ("map is well constructed"),
`a_star_node` fuses them into the local costmap as direct obstacles — but
**never within `global_fusion_min_range` (3 m) of the robot**, so the live
local map owns the near field and a stale global cell can re-route but never
trap the robot. This is the drift-tolerant replacement for the raw
`enable_dlio_map` fusion that injected phantom obstacles (that path remains,
still off by default).

Tests: `g1_local_map/test/test_voxel_accumulator.py` (equivalence vs the old
dict implementation) and `a_star_mpc_planner/test/test_yield_corridor.py`
(corridor geometry + clustering-partition equivalence); full suites pass
(53/53).
