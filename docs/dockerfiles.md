# Docker framework — `Navigation/docker/`

The Navigation stack is split into **three independent container images**, one
per concern. They are deliberately decoupled: each can be built, run, and
debugged on its own, and they communicate at run time over **CycloneDDS** (the
robot bus) and ROS 2 topics — never by sharing a Python environment. This keeps
the heavy, conflicting dependency sets (ROS 2 + DLIO/PCL vs. RoboJuDo + torch vs.
Unitree SDK) out of each other's way.

```mermaid
flowchart TB
    robot(["Unitree G1 robot<br/>CycloneDDS lowstate / lowcmd"])

    subgraph unitree["unitree — Dockerfile.unitree (ROS 2)"]
        u["Unitree SDK2 (C++) + unitree_ros2 msgs<br/>teleop · joystick · RViz"]
    end

    subgraph amo["amo_policy — Dockerfile.amo_policy (no ROS 2)"]
        a["RoboJuDo AMO RL gait<br/>pure-Python DDS control<br/>WebSocket :8766 to MPC"]
    end

    subgraph loc["localization — Dockerfile.localization (ROS 2)"]
        l["DLIO localization<br/>PCL · Eigen · OpenMP · Livox-SDK2 · RViz"]
    end

    robot <-->|DDS| unitree
    robot <-->|DDS| amo
    unitree -->|ROS 2 / WS| loc
    amo -->|WS velocity_target| loc
    loc -->|odometry| amo
```

## The three images at a glance

| Image | Dockerfile | Base | Purpose | ROS 2? |
|---|---|---|---|---|
| **unitree** | `Dockerfile.unitree` | `ros:humble-desktop` | Unitree SDK2 (C++) + `unitree_ros2` message packages, CycloneDDS, teleop, joystick, RViz. The ROS 2 ↔ robot bridge layer. | ✅ |
| **localization** | `Dockerfile.localization` | `ros:humble-desktop` | DLIO LiDAR-inertial odometry/localization for the MID-360: PCL, Eigen, OpenMP, `pcl_ros`, plus Livox-SDK2 (for the real-robot `livox_ros_driver2`) and the `direct_lidar_inertial_odometry` workspace. | ✅ |
| **amo_policy** | `Dockerfile.amo_policy` | `python:3.11-slim-bookworm` | The RoboJuDo **AMO RL gait** that actually drives the joints, via `real_g1_walking_policy.py`. Pure Python over CycloneDDS — **no ROS 2**. | ❌ |

## Shared conventions

All three honour the same DDS knobs so they can be wired together identically:

- **`UNITREE_NET_IFACE`** — the NIC CycloneDDS binds to (default `lo`; set to
  e.g. `eth0` to reach the robot). The `unitree` and `amo_policy` images build
  `CYCLONEDDS_URI` from it **in their entrypoint** so it can be changed at
  `docker run` time without rebuilding.
- **`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`** and **`ROS_DOMAIN_ID`** — set in
  the two ROS 2 images so the ROS middleware and the raw DDS traffic share one
  transport. (Not set in `amo_policy`: it uses CycloneDDS directly, not via the
  ROS rmw layer.)
- **Host networking + IPC** at run time (`--network host --ipc host`) so DDS
  multicast discovery and shared-memory transport reach the robot.
- Source/workspaces are **bind-mounted at run time**, not copied at build time,
  so you can iterate on code without rebuilding the image.

## 1. `Dockerfile.unitree` — robot bus + teleop

`ros:humble-desktop` base. Builds and installs:

- **Unitree SDK2** (C++) from source → `/opt/unitree_sdk2`.
- **`unitree_ros2`**: the `cyclonedds_ws` and `example` colcon workspaces under
  `/opt/unitree_ros2`, providing the Unitree message types and examples.
- ROS 2 tooling for driving the robot by hand: `teleop_twist_keyboard`, `joy`,
  `joy_teleop`, `robot_state_publisher`, `xacro`, `rviz2`, TF tools.

Entrypoint (`unitree_entrypoint.sh`) synthesises `CYCLONEDDS_URI` from
`UNITREE_NET_IFACE`, then sources ROS 2 + both Unitree workspaces.

> Note: this image installs the **C++** SDK and the ROS 2 message packages. The
> **Python** binding `unitree_sdk2py` needed by the AMO policy is *not* here —
> it lives in the `amo_policy` image, which talks to the robot directly.

## 2. `Dockerfile.localization` — DLIO odometry/localization

`ros:humble-desktop` base. DLIO's build dependencies are lightweight system
packages — no from-source Open3D, no Livox-SDK1:

- **PCL, Eigen, and OpenMP** via `libpcl-dev`, `libeigen3-dev`, `libomp-dev`,
  plus `ros-humble-pcl-ros`. These are DLIO's only real build deps and live in
  the base layer.
- **Livox-SDK2** from source — still needed for the real-robot
  `livox_ros_driver2`. **Baked into the image** (installed libs in `/usr/local`;
  `LIVOX_SDK2_ROOT` points the driver build at them). Build-once C library, not
  edited. (Open3D 0.14.1 and Livox-SDK1 were removed — they only served the old
  FAST-LIO / `open3d_loc` path, which DLIO does not use.)
- The ROS 2 perception stack (`pcl_ros`, `cv_bridge`, `image_transport`,
  message packages, `rviz2`).

The ROS 2 workspace is **not** baked in — it is a locally-editable checkout at
[`../ros2_ws/`](../ros2_ws/), bind-mounted to `/ws` by the compose file, and
built **inside the container** with the bundled `build_ws` helper. It holds both
the `direct_lidar_inertial_odometry` packages (DLIO, with `dlio_odom_node` +
`dlio_map_node`) and the `livox_ros_driver2` MID-360 ROS 2 driver (with the
tuned `MID360_config.json`); the driver links against the baked-in Livox-SDK2.
`build_ws` runs rosdep + colcon and selects the driver's ROS 2 build
(`-DROS_EDITION=ROS2`); it no longer passes `-DOpen3D_DIR`:

```bash
cd Navigation/docker
docker compose run --rm localization bash
# inside the container, first time only:
build_ws            # rosdep install + colcon build --symlink-install
```

`build/`, `install/`, and `log/` are written back to the host under `ros2_ws/`
(git-ignored). New shells auto-source `/ws/install/setup.bash` once it exists.

This is the perception/state-estimation image; it publishes the odometry the
planner and AMO policy consume.

## 3. `Dockerfile.amo_policy` — RoboJuDo AMO gait

`python:3.11-slim-bookworm` base — **not** a ROS 2 image. The AMO policy
(`policy/real_g1_walking_policy.py`) is pure Python and talks to the robot
directly over CycloneDDS via `unitree_sdk2py` (see `policy/README.md`). The
image bundles:

- **Python 3.11** — RoboJuDo requires `python >= 3.11`.
- A **CPU build of torch** (the deployment box has no CUDA).
- **RoboJuDo's runtime deps** (`scipy`, `onnxruntime`, `mujoco`, `pydantic`,
  `python-box`, `msgpack`/`msgpack-numpy`, `colorlog`, …) plus `websockets`
  for the policy's `:8766` state/command server. `mujoco` is **mandatory**:
  `G1RealEnvCfg.update_with_fk=True` makes RoboJuDo's `MujocoKinematics` a hard
  requirement, not an optional viewer.
- **Eclipse Cyclone DDS** C library built from source (`releases/0.10.x`) — the
  `cyclonedds==0.10.2` Python binding pulled in by `unitree_sdk2py` is a source
  dist that needs the matching C library present first. Installed to
  `/usr/local`; `CYCLONEDDS_HOME` points the binding at it.
- **`unitree_sdk2py`** from GitHub (`unitree_sdk2_python`), with two upstream
  packaging bugs worked around at build time:
  1. robot-family subpackages (`b2`, `g1`, `h1`, `h2`, `comm`) ship without
     `__init__.py` — they get touched so `from . import b2` resolves.
  2. `setup.py` omits `package_data`, so the CRC native `.so`s are copied into
     site-packages manually; otherwise `CRC().Crc()` fails at `send_cmd` time.

A build-time **import smoke test** imports every critical module and
instantiates `CRC()` (whose native lib loads lazily) so packaging failures
surface at build, not on the robot.

Entrypoint (`amo_entrypoint.sh`):

- builds `CYCLONEDDS_URI` from `UNITREE_NET_IFACE` unless one is already set
  (e.g. a mounted `cyclonedds.xml`);
- adds `ROBOJUDO_ROOT` (default `/workspace/policy/RoboJuDo`) to `PYTHONPATH`;
- runs `POLICY_SCRIPT` (default `/workspace/policy/real_g1_walking_policy.py`),
  forwarding any args; a bare command (e.g. `bash`) is run as-is instead.

### Build & run

```bash
# from Navigation/
docker build -t g1-amo-policy:latest -f docker/Dockerfile.amo_policy docker/

docker run --rm -it --network host --ipc host \
    -e UNITREE_NET_IFACE=eth0 \
    -v "$PWD":/workspace:rw \
    g1-amo-policy:latest --observe_only
```

> `policy/RoboJuDo` is a symlink to the RoboJuDo checkout. Make sure the real
> directory is mounted into the container (mount the parent that resolves the
> symlink, or bind-mount the RoboJuDo checkout to `ROBOJUDO_ROOT` explicitly).

## Why three images, not one

RoboJuDo wants Python 3.11 and a specific torch/mujoco/cyclonedds stack;
DLIO wants PCL/Eigen/OpenMP against ROS 2 Humble (Python 3.10);
the Unitree bridge wants the C++ SDK and `unitree_ros2`. Forcing these into one
image creates version conflicts (notably the Python 3.10 vs 3.11 split) and a
giant, slow-to-build image. Splitting by concern keeps each build reproducible
and lets you restart just the layer you're iterating on.
