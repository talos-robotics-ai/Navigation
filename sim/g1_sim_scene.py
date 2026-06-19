#!/usr/bin/env python3
"""Standalone Isaac Sim scene: Unitree G1 + Livox MID-360 LiDAR + IMU, published
to ROS 2 for the DLIO (direct_lidar_inertial_odometry) localization stack.

This script publishes (consumed by the g1_sim_bridge QoS relay, see below):

    /livox/lidar     sensor_msgs/PointCloud2   frame_id = livox_frame  (RTX lidar)
    /livox/imu_raw   sensor_msgs/Imu           frame_id = livox_frame
    /clock           rosgraph_msgs/Clock

DLIO consumes a plain PointCloud2 + Imu directly (no Livox CustomMsg needed),
but its cloud subscriber is RELIABLE while Isaac's ROS 2 bridge only publishes
BEST_EFFORT. The companion ROS 2 node `g1_sim_bridge` (Humble workspace) is a
thin QoS relay: /livox/lidar -> /livox/lidar_reliable and /livox/imu_raw ->
/livox/imu (both RELIABLE) so DLIO's subscribers match.

Run via launch_g1_sim.sh (which sets RMW_IMPLEMENTATION=rmw_cyclonedds_cpp so it
talks to the rest of the navigation stack), or directly:

    ./isaac-sim/python.sh g1_sim_scene.py [--headless] [--usd PATH] ...
"""

from __future__ import annotations

import argparse
import os
import sys

# ---------------------------------------------------------------------------
# CLI -- parsed before SimulationApp so --headless can be honoured.
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
# The saved G1 stage lives in the g1-isaac-sim repo (not in Navigation).
# Override with --usd or the ISAAC_G1_STAGE env var.
_DEFAULT_USD = os.environ.get(
    "ISAAC_G1_STAGE",
    "/home/lorenzo/TalosRoboticsAI/g1/g1-isaac-sim/isaac_projects/g1_basic_world.usd",
)

parser = argparse.ArgumentParser(description="G1 + MID-360 + IMU -> ROS 2 sim")
parser.add_argument("--usd", default=os.path.normpath(_DEFAULT_USD),
                    help="Stage to open (default: the G1 warehouse stage)")
parser.add_argument("--mount-path", default="/World/g1/torso_link/mid360_link",
                    help="Prim the LiDAR + IMU are parented to")
parser.add_argument("--lidar-config", default="Livox_Mid360",
                    help="RTX lidar config name (file in lidar_configs/)")
parser.add_argument("--lidar-topic", default="/livox/lidar")
# IMU goes to an intermediate topic; g1_sim_bridge republishes it RELIABLE on
# /livox/imu (Isaac's bridge publishes BEST_EFFORT; the relay makes it RELIABLE
# so any reliable consumer matches -- DLIO's IMU sub itself is BEST_EFFORT).
parser.add_argument("--imu-topic", default="/livox/imu_raw")
parser.add_argument("--frame-id", default="livox_frame")
parser.add_argument("--clock-topic", default="/clock")
# The G1 is unactuated in this scene -- with physics on and no controller it
# sags/collapses under gravity and DLIO tracks the falling sensor (Z drifts).
# --hold-pose pins the pelvis to the world (fixed base) so the robot stays put
# for a clean static localization test. Turn OFF once AMO drives the joints.
parser.add_argument("--hold-pose", action="store_true",
                    help="Pin the pelvis to the world (fixed base) so the unactuated G1 doesn't collapse")
parser.add_argument("--robot-root", default="/World/g1/pelvis",
                    help="Robot root link prim to pin when --hold-pose is set")
# MID-360 is mounted upside-down on the real G1, but the livox driver already
# rotates cloud+IMU upright at the source, so the rest of the stack sees an
# upright sensor. We therefore mount it upright in sim too (identity by
# default); override if your mid360_link frame needs an extra rotation.
parser.add_argument("--lidar-rpy-deg", default="0,0,0",
                    help="Extra roll,pitch,yaw (deg) applied to the sensor at the mount")
parser.add_argument("--headless", action="store_true")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

# enable_motion_bvh: transform each lidar ray by the sensor pose at that ray's
# sample time, not one pose per frame. Without it a moving/walking robot smears
# the world-frame cloud (the "MotionBVH for lidar model not enabled" warning).
simulation_app = SimulationApp({"headless": args.headless, "enable_motion_bvh": True})

# ---------------------------------------------------------------------------
# Imports that only exist inside a running Kit app.
# ---------------------------------------------------------------------------
import carb  # noqa: E402
import omni  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.prims import set_targets  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdPhysics  # noqa: E402

# RTX lidar + ROS 2 bridge + physics IMU sensor.
for _ext in ("isaacsim.sensors.rtx", "isaacsim.sensors.physics", "isaacsim.ros2.bridge"):
    enable_extension(_ext)
simulation_app.update()

# ---------------------------------------------------------------------------
# Register our lidar_configs/ folder so config="Livox_Mid360" resolves.
# ---------------------------------------------------------------------------
_LIDAR_CFG_DIR = os.path.join(_HERE, "lidar_configs") + "/"
_settings = carb.settings.get_settings()
_PROFILE_KEY = "/app/sensors/nv/lidar/profileBaseFolder"
_folders = list(_settings.get(_PROFILE_KEY) or [])
if _LIDAR_CFG_DIR not in _folders:
    _folders.append(_LIDAR_CFG_DIR)
    _settings.set_string_array(_PROFILE_KEY, _folders)
carb.log_warn(f"[g1_sim] lidar config folder registered: {_LIDAR_CFG_DIR}")

# Per-ray motion compensation for the lidar (belt-and-suspenders with the
# SimulationApp enable_motion_bvh flag above).
_settings.set_bool("/rtx/sceneOptimizationBVH/enableMotion", True)

# ---------------------------------------------------------------------------
# Open the G1 stage.
# ---------------------------------------------------------------------------
if not os.path.isfile(args.usd):
    carb.log_error(f"[g1_sim] stage not found: {args.usd}")
    simulation_app.close()
    sys.exit(1)

ctx = omni.usd.get_context()
ctx.open_stage(args.usd)
simulation_app.update()
stage = ctx.get_stage()
carb.log_warn(f"[g1_sim] opened stage: {args.usd}")

# ---------------------------------------------------------------------------
# Make sure the mount prim exists (it lives inside the G1 payload; create an
# Xform fallback so the script also works on stages without it).
# ---------------------------------------------------------------------------
mount_path = args.mount_path
if not stage.GetPrimAtPath(mount_path).IsValid():
    carb.log_warn(f"[g1_sim] mount {mount_path} missing -> creating Xform fallback")
    UsdGeom.Xform.Define(stage, mount_path)

# Extra orientation at the mount (XYZW order from roll,pitch,yaw).
_r, _p, _y = (float(v) for v in args.lidar_rpy_deg.split(","))
_q = (
    Gf.Rotation(Gf.Vec3d(0, 0, 1), _y)
    * Gf.Rotation(Gf.Vec3d(0, 1, 0), _p)
    * Gf.Rotation(Gf.Vec3d(1, 0, 0), _r)
).GetQuat()
sensor_orient = Gf.Quatd(_q.GetReal(), _q.GetImaginary())

# ---------------------------------------------------------------------------
# RTX LiDAR (Livox MID-360 approximation) -> PointCloud2.
# ---------------------------------------------------------------------------
lidar_path = mount_path + "/mid360_lidar"
# Custom JSON profiles (not built-in USD assets in SUPPORTED_LIDAR_CONFIGS) take
# the command's _call_replicator_api fallback, which forwards extra kwargs to
# the prim at CREATION time -- so pass `sensorModelConfig` here, not after. A
# post-creation Set() is too late: the omni.sensors plugin builds the sensor
# model from sensorModelConfig when the prim is created, so a later value is
# ignored and the lidar renders nothing (empty cloud, width:0). The
# "Config '...' not found" warning that still prints is cosmetic (it's the
# USD-asset-name lookup; the JSON profile loads via sensorModelConfig + the
# search-folder symlink set up in launch_g1_sim.sh).
_, lidar_prim = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path=lidar_path,
    parent=None,
    config=args.lidar_config,
    translation=(0.0, 0.0, 0.0),
    orientation=sensor_orient,
    sensorModelConfig=args.lidar_config,
)
# Belt-and-suspenders: ensure the attribute is present/correct on the prim.
_cfg_attr = lidar_prim.GetAttribute("sensorModelConfig")
if not _cfg_attr or not _cfg_attr.IsValid():
    _cfg_attr = lidar_prim.CreateAttribute("sensorModelConfig", Sdf.ValueTypeNames.String, False)
_cfg_attr.Set(args.lidar_config)
carb.log_warn(f"[g1_sim] sensorModelConfig = {_cfg_attr.Get()!r} "
              f"(profile JSON from lidar config search folders)")

lidar_render_product = rep.create.render_product(lidar_prim.GetPath(), [1, 1], name="mid360_rp")

pc_writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
pc_writer.initialize(topicName=args.lidar_topic, frameId=args.frame_id)
pc_writer.attach([lidar_render_product])

# A visible debug splat of the cloud in the viewport (no-op when headless).
try:
    dbg = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
    dbg.attach([lidar_render_product])
except Exception as exc:  # pragma: no cover - cosmetic only
    carb.log_warn(f"[g1_sim] debug draw unavailable: {exc}")

carb.log_warn(f"[g1_sim] MID-360 lidar @ {lidar_path} -> {args.lidar_topic} ({args.frame_id})")

# ---------------------------------------------------------------------------
# IMU sensor (co-located with the lidar) -> sensor_msgs/Imu via OmniGraph.
# ---------------------------------------------------------------------------
imu_path = mount_path + "/mid360_imu"
omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor",
    path=imu_path,
    parent=None,
    translation=Gf.Vec3d(0.0, 0.0, 0.0),
    orientation=sensor_orient,
)

GRAPH = "/G1SimGraph"
og.Controller.edit(
    {"graph_path": GRAPH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
            ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "ReadIMU.inputs:execIn"),
            ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
            ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
            ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ReadIMU.inputs:readGravity", True),
            ("PublishIMU.inputs:topicName", args.imu_topic),
            ("PublishIMU.inputs:frameId", args.frame_id),
            ("PublishClock.inputs:topicName", args.clock_topic),
        ],
    },
)
# imuPrim is a USD relationship -> set the target prim explicitly.
set_targets(
    prim=stage.GetPrimAtPath(GRAPH + "/ReadIMU"),
    attribute="inputs:imuPrim",
    target_prim_paths=[imu_path],
)
carb.log_warn(f"[g1_sim] IMU @ {imu_path} -> {args.imu_topic} ({args.frame_id})")

# ---------------------------------------------------------------------------
# Optional fixed base: pin the pelvis to the world so the unactuated robot does
# not collapse during static localization tests.
# ---------------------------------------------------------------------------
if args.hold_pose:
    if not stage.GetPrimAtPath(args.robot_root).IsValid():
        carb.log_error(f"[g1_sim] --hold-pose: root prim {args.robot_root} not found; "
                       f"robot will collapse. Pass --robot-root <pelvis prim path>.")
    else:
        fj_path = "/World/g1/hold_pose_fixed_joint"
        fj = UsdPhysics.FixedJoint.Define(stage, fj_path)
        # body0 unset = world; body1 = robot root -> pins it at its current pose.
        fj.CreateBody1Rel().SetTargets([args.robot_root])
        carb.log_warn(f"[g1_sim] --hold-pose: pinned {args.robot_root} to world "
                      f"(fixed base). Joints below the pelvis still settle once, "
                      f"then the robot is static.")

# ---------------------------------------------------------------------------
# Run. Sim time drives /clock; use_sim_time:=true on the ROS side.
# ---------------------------------------------------------------------------
sim_ctx = SimulationContext(physics_dt=1.0 / 200.0, rendering_dt=1.0 / 60.0,
                            stage_units_in_meters=1.0)
simulation_app.update()
sim_ctx.play()
carb.log_warn("[g1_sim] playing. Publishing /livox/lidar, /livox/imu_raw, /clock. "
              "Run 'ros2 launch g1_sim_bridge sim_localization.launch.py' to get "
              "/livox/lidar_reliable + /livox/imu for DLIO. Drive the G1 joints "
              "with your locomotion policy/companion to walk.")

while simulation_app.is_running():
    simulation_app.update()

sim_ctx.stop()
simulation_app.close()
