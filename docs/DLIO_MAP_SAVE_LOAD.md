# Saving & loading the DLIO map (G1 + MID-360)

## TL;DR

- **Save:** call the `/save_pcd` service on the running `dlio_map_node`. It writes
  `<save_path>/dlio_map.pcd`.
- **Load to resume / continue mapping:** **not possible with DLIO.** DLIO is pure
  online LiDAR-inertial *odometry* — it rebuilds its map from scratch every run
  and resets the world origin to wherever the robot starts. The saved PCD is an
  **output artifact** (visualization, a global costmap, or input to a *separate*
  localization node), never a DLIO input. See [§ Loading](#loading-a-map-next-time--the-reality).

---

## Saving the map

### What actually gets saved

`dlio_map_node` accumulates keyframe clouds (from `/dlio/odom_node/pointcloud/keyframe`)
into one cloud in the **`odom` frame**, held **in memory**. `/save_pcd` voxel-downsamples
that cloud at the request `leaf_size` and writes it
([`map.cc:76-101`](../ros2_ws/src/direct_lidar_inertial_odometry/src/dlio/map.cc#L76-L101)).

- **Fixed filename:** always `dlio_map.pcd` inside the directory you pass.
  `save_path: '/ws/maps'` → `/ws/maps/dlio_map.pcd`.
- `leaf_size` is an independent downsample of the *saved* file (not
  `map/sparse/leafSize`). `0.1–0.2` is typical; smaller = bigger file/more detail.

### Procedure

Run **inside the same container** where `dlio_map_node` runs, with the **same DDS
env** (RMW / `ROS_DOMAIN_ID`):

```bash
mkdir -p /ws/maps        # make the maps dir if not already present
ros2 service call /save_pcd direct_lidar_inertial_odometry/srv/SavePCD \
  "{leaf_size: 0.2, save_path: '/ws/maps'}"
```

Confirm: the `dlio_map_node` console prints `Saving map to /ws/maps/dlio_map.pcd ... done`,
and the service returns `success: true`. Then `ls -la /ws/maps/`.

### Gotchas (learned the hard way)

- **`/save_pcd` lives only on `dlio_map_node`, not the odom node**
  ([`map.cc:16,29`](../ros2_ws/src/direct_lidar_inertial_odometry/src/dlio/map.cc#L16-L29)).
  If your bring-up launches only `dlio_odom_node`, the service does not exist.
- **"waiting for service to become available" while the node is clearly running**
  = your shell is on a different DDS view than the launched node (different RMW /
  domain / `ROS_LOCALHOST_ONLY`, or a stale CLI daemon showing duplicate node
  names). Adopt the node's env, then refresh the daemon:
  ```bash
  export $(tr '\0' '\n' </proc/$(pgrep -f dlio_map_node)/environ \
            | grep -E '^(RMW_IMPLEMENTATION|ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY)=')
  ros2 daemon stop; ros2 daemon start
  ros2 service list | grep save_pcd
  ```
- **The directory must exist and be writable in the container that runs
  `dlio_map_node`** — the file is written by *that* process, in *its* filesystem.
- **Save before the node dies.** The map is in-memory only. If `dlio_odom_node`
  crashes (e.g. the `malloc(): invalid size` heap fault), `dlio_map_node` often
  stays alive holding the map — save *immediately*. Once `dlio_map_node` exits the
  map is gone; there is no on-disk autosave.

### Make it persist past the container

`save_path` must land on a host bind-mount or the PCD dies with the container. The
`localization` service mounts `../ros2_ws:/ws`
([`docker-compose.yml`](../docker/docker-compose.yml)), so `/ws/maps/dlio_map.pcd`
appears on the host at `Navigation/ros2_ws/maps/dlio_map.pcd` and survives.

---

## Loading a map next time — the reality

**DLIO cannot load a prior map to "not start from scratch."** There is no
`map_path` / load parameter and no relocalization anywhere in `dlio_odom_node`
([`odom.h`](../ros2_ws/src/direct_lidar_inertial_odometry/include/dlio/odom.h):
only online keyframe/submap building; the *first scan* defines the origin). Every
run therefore:

- sets the world origin at the robot's **start pose** (so two runs are **not** in
  the same coordinate frame), and
- rebuilds the keyframe map from zero.

So you cannot continue or extend a saved DLIO map across sessions. What to do
instead depends on your goal:

### 1. You only want the map as a reference / global costmap / RViz layer
Keep the saved `dlio_map.pcd`. View it (`pcl_viewer dlio_map.pcd`) or publish it as
a latched `PointCloud2` in the `odom`/`map` frame to seed a global costmap. It does
**not** feed back into DLIO.

### 2. You want to localize against a known map (deploy without re-mapping)
Use a **localization-against-prior-map** node, not DLIO. The sibling stack already
has one: **`FAST_LIO_LOCALIZATION_HUMANOID`** (in the `G1_navigation` repo), which
loads a saved PCD and relocalizes against it. Workflow:

1. **Map once** (DLIO mapping run), `/save_pcd`.
2. **Deploy** with the localization node loading that PCD + an initial-pose guess;
   it estimates the robot's pose in the prior map's frame.
3. Mind frame/PCD conventions — a FAST-LIO relocalizer expects its own map format/
   frame; convert the DLIO PCD if needed, or build the deployment map with the same
   tool you localize with.

### 3. You want consistent multi-session maps / loop closure
DLIO has **no loop closure** and no map serialization (your test showed ~1 m
residual returning near the origin after a 71 m loop). If that matters, do the
*mapping* phase with a pose-graph SLAM that saves/loads its graph, and switch to a
localization mode for deployment.

### Recommended G1 workflow
- **Mapping session:** DLIO from a stationary start (see
  [`DLIO_G1_MID360_TUNING.md`](DLIO_G1_MID360_TUNING.md) §2.2), drive the area,
  `/save_pcd`.
- **Deployment sessions:** localize against the saved PCD with a relocalization
  node (option 2) so you start in the *known* map frame instead of a fresh DLIO
  origin — that is the only way to "not start from scratch."

---

See also: [`DLIO_G1_MID360_TUNING.md`](DLIO_G1_MID360_TUNING.md),
[`LOCAL_VOXEL_MAP.md`](LOCAL_VOXEL_MAP.md).
