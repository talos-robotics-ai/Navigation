# Static-Map Deployment Runbook

How to navigate over a **pre-built map** instead of the default online-built
`GlobalCostmap`. Off by default (`use_static_map: false`) — use this only when you
want the planner to know the *whole* environment from cycle 1 (route to a goal
across the building, not just within sensor range).

> **Default vs static.** Online mode (default) builds obstacle memory as you drive
> and needs no map/localization — use it for cold-start and exploration. Static
> mode needs a map **and** localization in it, but gives full-environment routing
> immediately.

---

## Prerequisites (localization host)

```bash
sudo apt install ros-humble-nav2-map-server ros-humble-nav2-amcl \
                 ros-humble-nav2-lifecycle-manager ros-humble-pointcloud-to-laserscan
```

---

## Step 1 — Build a dense map OFFLINE (keeps it off the Orin)

Dense DLIO settings overload the Orin (CPU starvation → falls), so build offline:
record DLIO's **raw inputs** on the Orin, then replay them through DLIO on the
laptop with dense settings. Record the RAW `/livox/lidar` + `/livox/imu_ms2`
(DLIO's inputs) — **not** the deskewed cloud, which is DLIO's *output* and can't
be fed back in.

> DLIO has **no loop closure** — it drifts over large loops. Keep the mapped area
> bounded, or use a loop-closing SLAM (LIO-SAM / DLIOM) if the map must close.

### 1a — Record on the Orin, with live viz on the laptop

**Orin, terminal 1** — localization + map + relay (ships the DLIO map/odom to the laptop):
```bash
cd ~/Navigation/ros2_ws
LAPTOP_IP=<laptop-ip> RELAY_CLOUDS=1 RELAY_MAP=1 ./run_distributed_nav_jetson.sh
```
`RELAY_MAP=1` ships the accumulating `/dlio/map_node/map` (what you watch build).
The gait bridge / e-stop come up idle — you're not nav-driving while mapping.

**Orin, terminal 2** — record the raw inputs:
```bash
export ROS_DOMAIN_ID=42
ros2 bag record /livox/lidar /livox/imu_ms2 -o map_run
```

**Laptop** — RViz (`g1_dlio.rviz` already has a `CloudMap` display on
`/dlio/map_node/map` — tick it on):
```bash
cd ~/TalosRoboticsAI/g1/Navigation/ros2_ws
JETSON_IP=<orin-ip> ./run_distributed_nav_laptop.sh
```
(Foxglove instead: `FOXGLOVE=1`, load `foxglove/nav_command_center.json`, then add
topic `/dlio/map_node/map` to the 3D panel.) Then walk the robot **slowly**,
multiple passes / angles, keeping areas in view. `RELAY_MAP=1` is heavy on WiFi,
but there is no off-board control loop during mapping so it only affects viz.

### 1b — Replay the bag through DLIO on the laptop (dense)

Use a laptop container that **has DLIO built** (`localization-runtime:humble` —
`dnav-laptop:humble` does not). First set the dense knobs in
`src/direct_lidar_inertial_odometry/cfg/params.yaml`:
```yaml
map/dense/filtered: true
map/sparse/leafSize: 0.05
odom/preprocessing/voxelFilter/res: 0.10
odom/keyframe/threshD: 0.5
```
Launch the container (mount the ws, the bag, and a maps dir):
```bash
docker run --rm -it --network host \
  -v ~/TalosRoboticsAI/g1/Navigation/ros2_ws:/ws \
  -v /path/to/map_run:/bag -v $PWD/maps:/maps \
  -e ROS_DOMAIN_ID=42 localization-runtime:humble bash
# inside: source ROS + the DLIO workspace, then two shells:
```
**Shell A — DLIO + RViz, on the bag's topics:**
```bash
ros2 launch direct_lidar_inertial_odometry dlio.launch.py \
    rviz:=true pointcloud_topic:=/livox/lidar imu_topic:=/livox/imu_ms2 use_sim_time:=true
```
**Shell B — play the bag SLOWLY** (dense DLIO can't keep real-time; slow it or it
drops scans → holes):
```bash
ros2 bag play /bag --clock --rate 0.3
```
When it finishes and the map looks good, **save it**:
```bash
ros2 service call /save_pcd direct_lidar_inertial_odometry/srv/SavePCD \
    "{leaf_size: 0.0, save_path: '/maps'}"     # -> /maps/dlio_map.pcd (leaf 0.0 = full density)
```
Iterate freely — tweak `leafSize` / `threshD` / `--rate`, re-run, never touch the
Orin. If `/save_pcd` is missing, `dlio.launch.py` didn't start the map node; if the
map has gaps, lower `--rate` (0.2).

---

## Step 2 — Convert the 3D map to a 2D occupancy grid

```bash
python3 ros2_ws/scripts/dlio_map_to_occupancy.py maps/dlio_map.pcd -o maps/map \
    --resolution 0.05 --ground-band 0.15 --max-height 2.0 --min-points 2
# → maps/map.pgm + maps/map.yaml (Nav2 format)
```
Knobs: `--resolution` (cell size), `--ground-band` (m above floor before a point
counts — raise if the floor leaks in), `--max-height` (drop ceiling),
`--min-points` (points/cell to call it occupied — raise to reject noise).
Inputs: `.pcd` (ascii/uncompressed-binary), `.npy`, `.txt/.csv` (x y z).

---

## Step 3 — Serve the map + localize in it

```bash
ros2 launch a_star_mpc_planner static_map_localization.launch.py map:=$PWD/maps/map.yaml
```
Brings up: `pointcloud_to_laserscan` (MID-360 3D → 2D `/scan`) → `map_server`
(`/map`) → `amcl` (`map→odom` + `/amcl_pose`) → lifecycle manager.

**Localization is the crux** — AMCL must converge or the planner routes in a
wrong frame:
- In RViz set fixed frame `map`, then use **"2D Pose Estimate"** to seed AMCL at
  the robot's true spot; drive a little so it locks on.
- Tune `scan_min_height` / `scan_max_height` (base_link metres) to a wall-height
  slice so AMCL sees walls, not floor/ceiling.

---

## Step 4 — Point the planner at the map + run

`ros2_ws/src/a_star_mpc_planner/config/planner_params_default.yaml`:
```yaml
    use_static_map: true          # ← the only change
    static_map_topic: /map
    map_pose_topic: /amcl_pose
    static_occupied_thresh: 50
```
Then start the planner as usual. The global planner seeds
`GlobalCostmap.load_static_map()` from `/map`, takes the robot pose from
`/amcl_pose`, and routes in the `map` frame. Live obstacles stay with the local
planner (not fused into the static grid).

---

## Step 5 — Navigate

Set the RViz fixed frame to `map` (or load the Foxglove layout and change the
frame), then click a **2D Goal Pose anywhere in the map** — the global planner
routes across the full environment and the local A\*+MPC executes + dodges
dynamics. No exploration needed.

---

## Reverting to online mode

Set `use_static_map: false` and stop the localization launch. Nothing else
changes — online map-building is the default and is unaffected.

## Gotchas

| Symptom | Cause / fix |
|---|---|
| Robot/plan in the wrong place | AMCL not converged → re-seed with 2D Pose Estimate; tune scan height band |
| Floor shows as obstacles in `map.pgm` | raise `--ground-band` / `--min-points` in the converter |
| Doubled / smeared walls in the DLIO map | DLIO drift (no loop closure) → smaller mapped area or a loop-closing SLAM |
| `/scan` empty | wrong `cloud_topic` or height band excludes all points |
| Map reused across sessions is misaligned | needs relocalization (AMCL 2D handles it; a raw DLIO cloud reload does not) |
