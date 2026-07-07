# Semantic-Spatial Navigation Roadmap

**Goal.** Evolve the current geometric A\*+MPC navigation stack into a
*semantically aware* one: a robot that builds a persistent **spatial-semantic
memory** of its environment, can be commanded in **natural language**
("go to the kitchen", "find a chair", "go back to where you saw the door"),
**reasons about where to go** from what it has already seen, and **explores
autonomously** to find things it hasn't seen yet.

This document is a phased plan. Each phase is independently shippable, plugs into
the *existing* components, and is testable in isolation.

---

## 0. What we already have (the geometric foundation)

| Capability | Component | Status |
|---|---|---|
| LiDAR-inertial odometry | DLIO (`direct_lidar_inertial_odometry`) | ✅ |
| Live local obstacle map | `g1_local_map` (rolling 8 m voxel window + costmap) | ✅ |
| Reactive local planning | `a_star_mpc_planner` (local A\* + MPCC) | ✅ |
| **Persistent geometric memory** | `GlobalCostmap` (growable, dead-end penalty, drift-anchored) | ✅ (new) |
| Long-horizon routing | `global_planner_node` (A\* over the global map) | ✅ |
| Whole-body gait | SONIC policy via the ZMQ gait bridge | ✅ |
| Off-board compute split | distributed relay (Jetson perception ↔ laptop planning) | ✅ |
| LLM goal layer | AgenticNav (language → `/global_goal`) | ✅ (sibling repo) |

**The missing half is semantic.** Everything above reasons about *free vs
occupied space*. Nothing knows what a "kitchen" or a "chair" *is* or *where* it
was seen. The phases below add that, reusing the geometric memory patterns
(hit-counting, drift-anchoring, growable world grid) we already built.

---

## Phase 1 — Semantic perception layer

**Build:** an open-vocabulary / VLM object detector on the robot's camera
stream that localizes objects and landmarks in 3D.

- New node `semantic_perception_node`: subscribes to the camera image (+ depth /
  the DLIO deskewed cloud for range), runs an **open-vocabulary detector**
  (text-promptable, so classes aren't fixed) and back-projects each detection to
  a 3D point in the `odom` frame using the camera→lidar extrinsics.
- Publishes `/semantic/detections` — per frame: `{label, confidence, xyz (odom),
  bbox, embedding}`.
- Runs **off-board** (laptop/GPU) — the Orin can't carry a VLM alongside DLIO +
  the controller. Feed it the camera stream over the existing relay (a new image
  topic, compressed) or a dedicated WebRTC/GStreamer path.

**Deliverable:** live labeled 3D detections visible in RViz/Foxglove.
**Effort:** medium. **Depends on:** a camera on the G1 + extrinsics.

---

## Phase 2 — Spatial-semantic memory

**Build:** a persistent object-level map — the "where things are" brain.

- New module `semantic_map.py` + `semantic_memory_node`: accumulates
  `/semantic/detections` into a set of **object instances**, each with:
  `{id, label, embedding, position (odom), first_seen, last_seen, hit_count,
  permanence}`.
- **Reuse the `GlobalCostmap` design directly:**
  - *hit-counting* → an object needs ≥N consistent detections before it's
    "confirmed" (anti-hallucination, same as the anti-ghost gate).
  - *decay / permanence* → objects not re-observed fade (a moved chair is
    forgotten), but static landmarks (door, wall) persist.
  - *drift re-anchoring* → on a DLIO loop-closure jump, shift the object
    positions by the same delta (the T3 pattern) so memory doesn't smear.
  - *world-anchored & growable* → the semantic map covers the whole explored
    scene, not a window.
- Publishes `/semantic/map` (MarkerArray for viz) + exposes a **query API**
  (service or in-process): "nearest object matching `<text>`", "all objects of
  class X", "objects seen near pose P". This is the retrieval layer.

**Deliverable:** ask "where is the nearest chair?" and get a coordinate.
**Effort:** medium. **Depends on:** Phase 1.

---

## Phase 3 — Language → coordinate goal resolution

**Build:** extend AgenticNav so a natural-language command resolves to a
`/global_goal` **using the semantic memory**.

- The LLM agent is given a **tool interface** (MCP-style / function-calling) over
  Phase 2's query API: `find_object(text) → pose`, `list_known(text) → [poses]`,
  `robot_pose()`, `set_goal(pose)`, `explore()`.
- Flow for "go to the kitchen":
  1. Agent calls `find_object("kitchen")` against spatial memory.
  2. **Hit** → resolve to the object/region pose → `set_goal(pose)` → the
     existing global planner + MPC drive there.
  3. **Miss** → the target isn't in memory yet → trigger **Phase 4 exploration**
     with the text as the search target, then set the goal once found.
- Add spatio-temporal retrieval ("go back to where you saw the red door") by
  querying memory on `{label/embedding, time, place}`.

**Deliverable:** natural-language semantic goals that route via known memory.
**Effort:** medium (mostly agent/tool plumbing; AgenticNav already emits goals).
**Depends on:** Phase 2, AgenticNav.

---

## Phase 4 — Autonomous exploration

**Build:** frontier-based exploration so the robot can *go look* for something
it hasn't seen.

- New `exploration_node`: derives **frontiers** (boundaries between free and
  unknown) from the `GlobalCostmap`'s free/unknown layers — the map already
  distinguishes driven-free, confirmed-occupied, and unknown.
- Exploration goal policy: pick the frontier that best trades off *information
  gain* vs *travel cost*; while exploring, Phase 1/2 keep filling the semantic
  map. Stop when the search target appears (Phase 3 miss-path) or coverage is
  saturated.
- Emits `/global_goal` like any other goal source (arbitration: explicit user
  goal > semantic goal > exploration).

**Deliverable:** "find a fire extinguisher" with an empty map → the robot
explores until it finds one.
**Effort:** medium-high. **Depends on:** `GlobalCostmap` (done), Phase 2.

---

## Phase 5 — Modular skill / stream architecture

**Build:** refactor the growing set of nodes into **composable skills** with
typed stream interfaces, so behaviors compose and the agent can orchestrate them.

- Define a thin **skill contract**: each capability (perceive, remember, plan,
  gait, explore, speak) is a module with declared typed inputs/outputs (streams)
  and a small set of agent-callable actions.
- A **blueprint/compose** layer wires skills into a behavior graph (config, not
  code) so a run is "perception → semantic memory → agent → planner → gait."
- Keep the transport as-is (ROS 2 / DDS on-robot, ZMQ relay off-board); the skill
  layer sits *above* transport so it's transport-agnostic.

**Deliverable:** swap/rewire behaviors from config; agent orchestrates skills.
**Effort:** high (refactor). **Depends on:** Phases 1–4 existing as modules.

---

## Phase 6 — Closed-loop VLM reasoning

**Build:** continuous scene reasoning, not just goal resolution.

- A VLM periodically answers task-relevant questions about the live view ("is the
  door open?", "is the path blocked by people?", "does this match the target?")
  and feeds structured results back to the agent, which can **re-plan
  semantically** (choose another route, abandon, re-target).
- Grounds the agent's decisions in current perception instead of only the map.

**Deliverable:** the robot adapts mid-task to semantic changes.
**Effort:** high. **Depends on:** Phases 1–5.

---

## Cross-cutting concerns

- **Compute placement.** VLM/LLM run off-board (laptop GPU or cloud); perception +
  gait + geometric planning stay on the split we already have. Add the new
  semantic topics (`/semantic/*`, camera stream) to the ZMQ relay, and keep the
  control-critical topics (odom, cmd_vel) on a **separate socket** so bulk
  semantic/image traffic never head-of-line-blocks control (the lesson from the
  path-flood incident).
- **Frames & drift.** Every semantic layer is anchored in `odom` and must reuse
  the drift re-anchoring (T3) — a loop-closure jump has to shift objects *and*
  costmap *and* the semantic map together, or memory desyncs.
- **Simulation.** Stand up a MuJoCo (or Isaac) G1 sim to develop Phases 1–4
  without the robot — semantic perception + memory + exploration are all testable
  in sim, and it de-risks the on-robot time.
- **Safety.** Language goals pass through the same `/global_goal` → planner → MPC
  → e-stop chain; the semantic layer only *proposes* goals, it never bypasses the
  geometric safety envelope.

---

## Suggested order & first milestone

1. **Phase 1 + 2** first — semantic perception + spatial memory are the
   foundation everything else queries. Milestone: "point at a chair on the map."
2. **Phase 3** — wire memory into AgenticNav → first natural-language known-goal.
3. **Phase 4** — exploration for unknown targets.
4. **Phase 5/6** — architecture + closed-loop reasoning once the capabilities exist.

The geometric memory work already done (growable `GlobalCostmap`, dead-end
penalty, drift anchoring) is the template: the semantic map is the *same idea*
applied to objects instead of occupancy.
