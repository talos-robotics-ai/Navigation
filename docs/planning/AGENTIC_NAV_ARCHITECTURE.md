# Agentic Navigation — Architecture & Test-First Plan

This branch (`feat/agentic-nav-stack`) adds an **agentic navigation layer** on top
of the existing geometric planner, structured as **skills + a clean interface +
composable blueprints**. It is built **test-first**: the low-level (geometric)
navigation is exposed and validated on its own *before* any VLM or agent is added.

## Design principle

> One thin seam between "intelligence" and "geometry." Everything smart talks to
> navigation only through `NavigationInterface`; the geometric planner can be
> swapped, tuned, or run off-board without any skill/agent code changing.

## Layers (bottom → top)

```
  ┌─ agent / LLM (Phase 3+, NOT in this branch) ─────────────────┐
  │        calls skills as tools; resolves language → goals       │
  ├─ skills/         NavigationSkillContainer                     │
  │     navigate_to_pose ✅   stop_navigation ✅                   │
  │     navigate_with_text ⏸ (deferred: needs memory + VLM)       │
  ├─ spatial_memory/ SpatialMemory                                │
  │     tag_location / query_tagged_location ✅                    │
  │     query_by_text ⏸ (deferred: needs VLM detections)          │
  ├─ control/        NavigationCoordinator (goal-source priority) │
  │                  blueprints/mobile.py (composition)           │
  ├─ interfaces/     NavigationInterface  ◀── the one seam        │
  │     set_goal / get_state / is_goal_reached / cancel           │
  └─ GEOMETRIC STACK (unchanged, this is what we preserve) ───────┘
        /global_goal → global A* (persistent map) → local A* → MPCC
        → /mpc/cmd_vel → gait ;  /navigation/state out
```

`ros2_ws/src/agentic_nav/` mirrors this: `interfaces/`, `skills/`,
`spatial_memory/`, `control/` + `control/blueprints/`, `nodes/`, `test/`.

## What we PRESERVE from the current stack (evaluated, kept as-is)

The geometric stack is already strong; the agentic layer wraps it, it does not
replace it:

- **Local A\* + MPCC** (`a_star_mpc_planner`) — reactive avoidance + smooth
  contouring control. Comparable to any framework's local planner; kept.
- **Persistent global planner** (`global_planner_node` + `GlobalCostmap`) — now
  with whole-environment memory: growable map, dead-end/stuck penalty, drift
  re-anchoring, and **2.5D height-graded cost** (tall structure lethal, low
  returns soft). Kept and extended.
- **DLIO odometry + `local_voxel_map`** (ground-removed obstacles). Kept.
- **`/navigation/state` + `/global_goal` + `/estop` contract** — this is exactly
  the seam `NavigationInterface` wraps, so no planner change was needed.
- **Distributed compute split** (Jetson perception ↔ laptop planning ↔ Foxglove).
  The agentic layer runs wherever the planner does; VLM/LLM go off-board.

## Test-first: validate low-level nav BEFORE the agent

The whole reason the interface/skill layer ships first: prove goal-in → drive →
`GOAL_REACHED` works through the clean seam with nothing intelligent above it.

- **Offline logic** (no robot): `src/agentic_nav/test/test_agentic_nav.py` —
  skill registry, memory, coordinator priority, goal parsing, skill polling.
- **On-robot geometric** (planner + robot up): the harness node —
  ```bash
  ros2 run agentic_nav low_level_nav_test --ros-args \
      -p goals:="2.0,0.0,0.0; 0.0,0.0,3.14"
  ```
  It tags the start pose as `home`, drives each goal via
  `NavigationSkillContainer.navigate_to_pose`, and reports reached/timeout. This
  is the acceptance gate for "the geometric nav works through the agentic seam."

Only once this passes do we add Phase 2 (VLM perception + spatial memory) and
Phase 3 (the agent that resolves language to goals). See
`SEMANTIC_SPATIAL_NAV_ROADMAP.md` for the full phase plan.

## Deferred (scaffolded but not implemented here)

- `navigate_with_text` — control flow written, perception/memory calls raise
  `NotImplementedError` so the shape is fixed but nothing fakes working.
- `SpatialMemory.query_by_text` — returns `None` until the VLM detector +
  embedding store exist (Phase 2). Its memory model intentionally mirrors
  `GlobalCostmap` (world-anchored, hit-counted, decayed, drift-re-anchored).
- The **agent / LLM** node — not in this branch by design.
