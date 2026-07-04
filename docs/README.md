# Navigation documentation

Docs are grouped by **macro-topic**. Start with
[system/system_architecture.md](system/system_architecture.md) for the
end-to-end data flow, then dive into the area you need.

```
LiDAR ─▶ DLIO odometry ─▶ local voxel map ─▶ A* + MPCC planner ─▶ cmd_vel ─▶ gait policy
        └── perception ──┘                  └──── planning ────┘            └ locomotion ┘
```

## perception/ — sensing, odometry, mapping
The LiDAR-inertial front end and the obstacle representation the planner consumes.

| Doc | What it covers |
|---|---|
| [perception/DLIO_DEPLOYMENT_TESTING.md](perception/DLIO_DEPLOYMENT_TESTING.md) | Ordered DLIO bring-up + tests on the real robot |
| [perception/DLIO_G1_MID360_TUNING.md](perception/DLIO_G1_MID360_TUNING.md) | DLIO parameters for the G1 + Livox MID-360 |
| [perception/DLIO_MAP_SAVE_LOAD.md](perception/DLIO_MAP_SAVE_LOAD.md) | Saving / reloading the DLIO global map |
| [perception/LOCAL_VOXEL_MAP.md](perception/LOCAL_VOXEL_MAP.md) | Rolling ground-removed obstacle cloud that feeds the planner |
| [perception/GROUND_SEGMENTATION.md](perception/GROUND_SEGMENTATION.md) | Per-cell local-minimum ground filter (current method) |
| [perception/GROUND_REMOVAL_PLAN.md](perception/GROUND_REMOVAL_PLAN.md) | Superseded SVD ground-removal plan (history) |

## planning/ — global + local planning and control
How a goal becomes a velocity command, and how obstacles bend the trajectory.

| Doc | What it covers |
|---|---|
| [planning/A_STAR_MPC_PLANNER.md](planning/A_STAR_MPC_PLANNER.md) | The A* + MPC goal→velocity planner, global+local fusion |
| [planning/MPCC_CONTOURING_MPC.md](planning/MPCC_CONTOURING_MPC.md) | Time-optimal contouring MPC (the default tracker) |
| [planning/DYNAMIC_OBSTACLE_AVOIDANCE.md](planning/DYNAMIC_OBSTACLE_AVOIDANCE.md) | **Dynamic + static obstacle safety: detect → track → YIELD / escape / barrier / blind-stop** |
| [planning/DISTRIBUTED_NAV_PLAN.md](planning/DISTRIBUTED_NAV_PLAN.md) | Off-board split proposal + the §8/§9 on-Jetson optimization pass |

## locomotion/ — gait policies and last-hop bridges
The RL gaits that consume `cmd_vel` and drive the robot's joints.

| Doc | What it covers |
|---|---|
| [locomotion/SONIC_REAL_BRINGUP.md](locomotion/SONIC_REAL_BRINGUP.md) | Full on-robot bring-up with the SONIC walking policy |
| [locomotion/SONIC_POLICY.md](locomotion/SONIC_POLICY.md) | The SONIC policy + its ZMQ command wire |
| [locomotion/UNITREE_GAIT.md](locomotion/UNITREE_GAIT.md) | Unitree native high-level gait path |
| [locomotion/amo_inference_plan.md](locomotion/amo_inference_plan.md) | AMO inference + joint-smoothing filter design |

## autonomy/ — high-level / LLM navigation
| Doc | What it covers |
|---|---|
| [autonomy/AGENTIC_LLM_NAV.md](autonomy/AGENTIC_LLM_NAV.md) | Gemini-driven LLM nav layer over the A*+MPC stack (`/global_goal`) |

## system/ — architecture, build, infra, sim
| Doc | What it covers |
|---|---|
| [system/system_architecture.md](system/system_architecture.md) | End-to-end stack (LiDAR → DLIO → MPC → gait) |
| [system/BUILDING.md](system/BUILDING.md) | Building `ros2_ws` on the Jetson — what to rebuild, the `LIVOX_SDK2_ROOT` rule |
| [system/dockerfiles.md](system/dockerfiles.md) | The three-image framework |
| [system/simulation_stack.md](system/simulation_stack.md) | Wrapping the stack into Isaac Sim |
