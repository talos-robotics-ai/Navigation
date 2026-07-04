# Building the workspace on the Jetson (after you make changes)

How to (re)build `ros2_ws` **natively on the Jetson Orin** (bare metal, no
container) after editing code — the flow set up by `setup_jetson.sh`. The golden
rule fixes the `LIVOX_SDK2_ROOT` failure this stack hits on a clean build; the
rest is about **not** rebuilding when you don't have to.

> This whole page is the **bare-metal Jetson** flow. Building inside the
> `localization` Docker container instead (e.g. on an x86 dev box)? Skip to
> [In the container](#in-the-container).

---

## TL;DR

```bash
cd ~/Navigation/ros2_ws
source /opt/ros/humble/setup.bash
export LIVOX_SDK2_ROOT=/opt/navigation        # <-- REQUIRED (see Golden rule)

# Most edits (Python only) need NO build — just re-source + run:
source install/setup.bash

# Structural change (new node, setup.py/package.xml/launch, or any C++)? Build
# ONLY the package you touched:
colcon build --symlink-install --packages-select g1_sim_bridge \
    --cmake-args -DROS_EDITION=ROS2 -DLIVOX_SDK2_ROOT=/opt/navigation
source install/setup.bash
```

If you installed the helper (`setup_jetson.sh` → `install_helpers`), the one-liner
`build_ws` does the full build with the env already set. `source nav_env` sets up
a fresh shell (ROS + overlays + `LIVOX_SDK2_ROOT`).

---

## Golden rule: always set `LIVOX_SDK2_ROOT=/opt/navigation`

`livox_ros_driver2`'s CMake looks for the Livox-SDK2 shared lib under
`${LIVOX_SDK2_ROOT}/lib`. On this Jetson the SDK lives at `/opt/navigation` (put
there by `setup_jetson.sh`). If the variable is **unset**, CMake falls back to a
container-only path (`/ros_ws/Livox-SDK2`) and the build dies with:

```
CMake Error at CMakeLists.txt (find_library):
  Could not find LIVOX_LIDAR_SDK_LIBRARY ... liblivox_lidar_sdk_shared.so
```

Set it every build. The safest form passes it **both** as an env var and as a
`-D` cmake arg (the `-D` also force-overrides a poisoned cache — see
[the stale-cache trap](#the-stale-cache-trap)):

```bash
export LIVOX_SDK2_ROOT=/opt/navigation
colcon build --symlink-install \
    --cmake-args -DROS_EDITION=ROS2 -DLIVOX_SDK2_ROOT=/opt/navigation
```

`build_ws` and `nav_env` set `LIVOX_SDK2_ROOT` for you — prefer them so you can't
forget. If they aren't installed, run `sudo ./setup_jetson.sh` once (idempotent;
it skips the already-built SDKs) or add the export to `~/.bashrc`.

---

## Do I even need to build? (usually not)

The workspace is built with `--symlink-install`, so each package's Python source
dir is **symlinked** into the install space. Imports resolve straight to your
`src/` files. That means:

### No rebuild needed — just re-run the node

- Editing the **body** of an existing `.py` (logic, params, constants) in a
  Python package: `g1_sim_bridge`, `a_star_mpc_planner`, `g1_local_map`.
  Example: tweaking `cmd_vel_to_sonic_node.py` or `sonic_wire.py`, changing an
  MPC gain in `mpc_node.py`.
- Just re-run the node (and `source install/setup.bash` in a fresh shell). The
  change is already live.

### Rebuild needed (`--packages-select <pkg>`)

- **New executable / entry point** — you added a line to `setup.py`
  `console_scripts` (e.g. registering `cmd_vel_to_sonic_node`). The wrapper
  script in the install space is generated at build time.
- **`package.xml` dependency change** — also run `rosdep install` (below).
- **`launch/` or `config/` file added or renamed** — these are installed by the
  `setup.py` `data_files` globs at build time.
- **Any C++ edit** — `direct_lidar_inertial_odometry`, `livox_ros_driver2` (and
  `g1_description`/`g1_bringup` CMake bits) are compiled; edits always require a
  build.

> Rule of thumb: **content of an existing Python file = live; anything that
> changes the package's _shape_ (executables, deps, installed files) or any C++ =
> build.** When unsure, a `--packages-select` build is cheap.

---

## Build just what you changed

Full `colcon build` recompiles DLIO + livox in C++ (~3.5 min on the Orin). After
the first successful build you almost never need that. Build only the touched
package (and, with `--packages-up-to`, its dependents):

```bash
# after editing g1_sim_bridge (e.g. the SONIC node's structure):
colcon build --symlink-install --packages-select g1_sim_bridge \
    --cmake-args -DROS_EDITION=ROS2 -DLIVOX_SDK2_ROOT=/opt/navigation

# rebuild a package AND everything that depends on it:
colcon build --symlink-install --packages-up-to a_star_mpc_planner \
    --cmake-args -DROS_EDITION=ROS2 -DLIVOX_SDK2_ROOT=/opt/navigation
```

Then always: `source install/setup.bash`.

### After a `package.xml` change

New `<depend>`/`<exec_depend>` entries are only pulled in by rosdep:

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

(e.g. the SONIC node added `nav_msgs` + `python3-zmq`; `python3-zmq` also needs
to actually be installed on the host — `sudo apt install -y python3-zmq`.)

---

## The stale-cache trap

If a build **fails on the Livox SDK path**, do **not** just re-run `colcon build`
with the variable set — it will keep failing. The failed run writes an empty
`LIVOX_SDK2_ROOT` into `build/<pkg>/CMakeCache.txt`, and CMake's
`set(... CACHE PATH ...)` will **not** overwrite a cached value, so your corrected
env var is ignored. Break out of it one of two ways:

```bash
# A) force it past the cache with -D (overrides the cached value):
colcon build --symlink-install --cmake-args -DLIVOX_SDK2_ROOT=/opt/navigation -DROS_EDITION=ROS2

# B) or wipe the poisoned build dir(s) and rebuild clean:
rm -rf build/livox_ros_driver2 build/direct_lidar_inertial_odometry
#   (or a full reset:)  rm -rf build install log
```

A full-reset build is also when you must have the env var set from the start —
that is the exact situation that bites people.

---

## After building: source the overlay

A build changes files in `install/`; a running shell won't see new executables or
messages until you re-source:

```bash
source install/setup.bash          # this workspace's overlay
```

New shells: `source nav_env` loads ROS + the Unitree overlays + this workspace +
`LIVOX_SDK2_ROOT` in one shot.

---

## Runtime deps (not build deps)

Some packages build fine but fail at **run** time on a missing dep:

- **`gait:=sonic` / `cmd_vel_to_sonic_node`** needs `pyzmq`. Build succeeds
  without it; the node crashes with `ModuleNotFoundError: No module named 'zmq'`.
  Install once: `sudo apt install -y python3-zmq` (also in `setup_jetson.sh`).

The Livox shared lib resolves at run time with no action needed — CMake bakes
`/opt/navigation/lib` into the driver library's RUNPATH.

---

## In the container

Inside the `localization` image the workspace is bind-mounted at `/ws` and the
Livox SDK2 is baked into the image, so the helper does everything:

```bash
docker compose run --rm localization bash
build_ws                    # rosdep + colcon (LIVOX_SDK2_ROOT already set)
source /ws/install/setup.bash
```

Same symlink-install rules apply, so Python edits are live inside the container
too (the `../ros2_ws` bind mount makes them editable from the host).

---

## Cheat sheet

| You changed… | Action |
|---|---|
| Body of an existing `.py` (Python pkg) | none — re-run the node (`source install/setup.bash` in a new shell) |
| `setup.py` entry point / new executable | `colcon build --packages-select <pkg>` |
| `package.xml` deps | `rosdep install …` + `colcon build --packages-select <pkg>` |
| `launch/` or `config/` file | `colcon build --packages-select <pkg>` |
| C++ (`direct_lidar_inertial_odometry`, `livox_ros_driver2`) | `colcon build --packages-select <pkg>` |
| Build failed on `LIVOX_LIDAR_SDK_LIBRARY` | set/`-D` `LIVOX_SDK2_ROOT=/opt/navigation`; if it persists, `rm -rf build/<pkg>` |
| Nothing builds after `rm -rf build` | you forgot `export LIVOX_SDK2_ROOT=/opt/navigation` |

Every build command on this stack carries `--symlink-install --cmake-args
-DROS_EDITION=ROS2 -DLIVOX_SDK2_ROOT=/opt/navigation`. If that's a mouthful, use
`build_ws`.
