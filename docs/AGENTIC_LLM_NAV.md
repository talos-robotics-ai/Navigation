# TalOS — Language-Driven Exploration & Navigation Orchestrator

**Target platform:** Unitree G1 (humanoid), MID-360 LiDAR + head RGB-D camera
**Low-level navigation:** [`talos-robotics-ai/Navigation`](https://github.com/talos-robotics-ai/Navigation) (DLIO localization → A\*+MPC goal→velocity planner → RoboJuDo AMO RL gait)
**Goal:** a voice/text-driven robot that explores a room, builds a semantic map, narrates what it sees like a human (with context-appropriate emotion), drives the tested nav stack to reach a target scene, and verifies its own behaviour against the user's instruction through a second "critic" LLM.

---

## 1. Architecture at a glance

The system is a **dual-LLM closed loop** (planner + critic) sitting on top of perception, a semantic map, and the existing navigation stack. The two LLMs never touch joints — they emit *goal poses* and *skill calls*; the tested stack owns everything reactive.

```mermaid
flowchart TB
    subgraph UI["Operator UI (web)"]
      MIC["🎤 Push-to-talk / 💬 text"]
      FLOW["Live plan flowchart"]
      FOX["Embedded Foxglove 3D — semantic map"]
      TRANSCRIPT["Robot speech transcript"]
    end

    MIC -->|ASR text| ORCH
    subgraph BRAIN["Cognitive layer"]
      ORCH["🧠 ORCHESTRATOR LLM<br/>(planner: instruction → skill DAG)"]
      CRITIC["🔎 CRITIC LLM<br/>(verifier: is this still right?)"]
      NARR["🗣️ NARRATOR<br/>(events → human speech + emotion)"]
    end

    ORCH <-->|steering: CONTINUE / ADJUST / ABORT| CRITIC
    ORCH -->|emit flowchart| FLOW
    NARR -->|TTS audio| TRANSCRIPT
    ORCH -->|events| NARR
    CRITIC -->|anomalies| NARR

    subgraph PERC["Perception"]
      DET["GroundingDINO + SAM<br/>open-vocab detect & segment"]
      SEMMAP["Semantic map builder<br/>(detections → 3D, DLIO frame)"]
    end
    CAM["Head RGB-D"] --> DET --> SEMMAP
    SEMMAP -->|object list + poses| ORCH
    SEMMAP -->|object list + poses| CRITIC
    SEMMAP -->|MarkerArray / SceneUpdate| FOX

    subgraph NAV["talos-robotics-ai/Navigation (tested, unchanged)"]
      GOAL["/global_goal (PoseStamped)"]
      PLAN["A* + MPC planner"]
      DLIO["DLIO localization"]
      OBST["g1_local_map obstacles"]
      AMO["RoboJuDo AMO gait<br/>(vx,vy,yaw via WS :8766)"]
    end

    ORCH -->|navigate_to(x,y,θ)| GOAL --> PLAN
    DLIO --> PLAN
    OBST --> PLAN
    PLAN -->|velocity_target| AMO
    DLIO -->|robot pose| SEMMAP
    DLIO -->|robot pose, /goal_reached| ORCH
    DLIO -->|robot pose, path| CRITIC
```

**One-line summary of the control hierarchy:**

| Layer | Owns the question | Output | Rate |
|---|---|---|---|
| Orchestrator LLM | *Where do I go and why?* | goal pose + skill calls | ~event-driven (0.2–2 Hz) |
| Critic LLM | *Is what I'm doing still consistent with the instruction?* | CONTINUE / ADJUST / ABORT | 1–3 Hz |
| A\*+MPC planner | *How do I get there safely?* | `(vx, vy, yaw_rate)` | 10–50 Hz |
| AMO gait | *What joint targets walk that velocity?* | joint commands | 50 Hz |

---

## 2. The navigation seam — the "smart" way to send goals

This is the part you asked me to double-check against the repo. The repo makes the contract unambiguous:

- **The AMO gait is a pure velocity tracker** — it consumes `{"vx","vy","yaw"}` on WebSocket `:8766` and is fed *by the MPC*, not by you. Do **not** have the LLM emit velocities.
- **The A\*+MPC planner is the goal interface.** In `AUTONOMOUS=1` mode it tracks goals published on **`/global_goal`** (normally from RViz's *2D Goal Pose* tool), plans an A\* path on the `g1_local_map` obstacle layer, tracks it with the MPC, and **replans continuously**. DLIO supplies the robot pose in the `map` frame.

So the orchestrator's *entire* low-level interface is: **publish a `geometry_msgs/PoseStamped` to `/global_goal` in the DLIO `map` frame, then monitor for arrival.** Everything reactive (obstacle avoidance, path planning, gait) is already solved and tested below that line. Concretely, the rules that make goal-sending "smart" rather than naive:

1. **One frame to rule them all.** The semantic map must express every detected object in the **same `map` frame DLIO maintains**. Project each detection to 3D using head-camera depth + camera→base extrinsics + the live DLIO `base→map` transform. Then a goal is just an object's pose in that frame — no frame juggling at send time.

2. **Send standoff poses, never object centroids.** `navigate_to(object)` must compute an **approach pose**: offset the goal by a standoff distance `d` along the ray from the object back toward free space, and set `θ` so the robot *faces* the object (so the head camera frames it for the subsequent manipulation/inspection). A goal placed on the object itself sits inside an obstacle and the MPC will fight it.

3. **Feasibility gate before publishing.** Subscribe to `g1_local_map`; reject/repair any goal whose cell is occupied or unreachable (snap to nearest free cell on the approach ray). This keeps you from handing the MPC impossible goals.

4. **One goal at a time, with a watchdog.** Publish a goal, then wait for arrival (DLIO pose within `(ε_xy, ε_θ)` of goal, or a `/goal_reached` flag). If there is **no progress for `T_stuck` seconds** (distance-to-goal not decreasing), raise a `STUCK` event — the critic decides replan vs. new goal. This is the bridge between the geometric layer's failure and the cognitive layer's recovery.

5. **Exploration goals are next-best-view poses,** generated the same way (Section 4) and sent through the exact same `/global_goal` seam. The planner can't tell a "go look over there" goal from a "go to the chair" goal — which is exactly why this seam is clean.

> **Net effect:** the LLM plans *semantically*, emits a goal pose, and the proven A\*+MPC+AMO chain executes it. The only new code on the robot side is (a) the perception→semantic-map node and (b) a thin `goal_bridge` node that turns `navigate_to(x,y,θ)` skill calls into `/global_goal` publishes and reports arrival/stuck back up.

---

## 3. The dual-LLM loop (planner + critic)

### 3.1 Orchestrator (planner)
Turns the user instruction into a **skill DAG**, executes it step by step, and re-plans when the critic or the watchdog says so. It is the only component that calls skills. It emits the flowchart the UI renders.

### 3.2 Critic (steering / verifier)
Runs in parallel at 1–3 Hz. It sees the same perception + the current action + the original user instruction and answers one question: *is the robot still doing the right thing?* It returns a structured verdict:

- `CONTINUE` — on track.
- `ADJUST{reason, hint}` — minor correction (e.g. "approaching the wrong chair — target was the red one near the window"); orchestrator patches the current step.
- `ABORT_REPLAN{reason}` — the plan's premise is wrong (target not where assumed, hallucinated detection, drifting away from likely target region); orchestrator discards the subplan and re-plans.

The critic is what catches: wrong-object approach, detection hallucinations, drift away from where the target probably is, and safety/social violations. Keep it on a *separate* model context from the planner so it doesn't inherit the planner's assumptions — an independent judge, not an echo.

```mermaid
sequenceDiagram
    participant U as User
    participant O as Orchestrator
    participant P as Perception/SemMap
    participant N as Navigation
    participant C as Critic
    U->>O: "Bring me the cup from the kitchen counter"
    O->>P: detect("cup", "kitchen counter")
    P-->>O: cup NOT in view
    O->>O: switch to EXPLORE (no target → search)
    loop until target found or area exhausted
        O->>N: navigate_to(next_best_view pose)
        N-->>O: arrived
        O->>P: detect(target prompt)
        P-->>C: live detections + robot pose
        C-->>O: CONTINUE / ADJUST / ABORT_REPLAN
    end
    P-->>O: cup detected @ (x,y,z), conf 0.81
    O->>N: navigate_to(standoff pose facing cup)
    C-->>O: CONTINUE (target matches instruction)
    N-->>O: arrived at target zone
    O->>U: 🗣️ "Found it — the cup's right here on the counter. I'm in position."
```

---

## 4. Explore-then-act logic (the core behaviour)

Everything hinges on one decision: **is the target visible yet?** Matching is open-vocabulary — the user's noun phrase is fed to GroundingDINO for boxes, SAM for masks, and the match is accepted only above a confidence + geometric-consistency threshold.

```mermaid
flowchart TD
    START([User instruction]) --> PARSE[Extract target object + qualifiers + scene]
    PARSE --> DETECT{Target matched in<br/>current view?<br/>GroundingDINO+SAM}
    DETECT -- yes, conf ≥ τ --> STANDOFF[Compute standoff pose facing target]
    STANDOFF --> NAV[navigate_to → /global_goal]
    NAV --> ARRIVE{Arrived?}
    ARRIVE -- yes --> VERIFY{Critic: target<br/>matches instruction?}
    VERIFY -- yes --> DONE([Report success → hand to manipulation])
    VERIFY -- no --> EXPLORE
    DETECT -- no --> EXPLORE[EXPLORE MODE]
    EXPLORE --> NBV[Pick next-best-view goal<br/>frontier / unobserved region]
    NBV --> NAVx[navigate_to NBV → /global_goal]
    NAVx --> SCAN[Sweep head cam, run detector,<br/>add all objects to semantic map]
    SCAN --> DETECT
    EXPLORE --> EXHAUST{Area exhausted /<br/>budget spent?}
    EXHAUST -- yes --> ASK([Report failure, ask user for help])
```

**Next-best-view selection** (the heart of exploration): score candidate viewpoints by (a) expected unobserved volume revealed, (b) proximity to *semantic priors* — e.g. a "cup" prior pulls the robot toward already-mapped "counter"/"table"/"sink" regions before random frontiers, (c) reachability on `g1_local_map`. While exploring, **every** detected object (not just the target) is committed to the semantic map, so by the time the target is found you also have the full room map shown in the screenshot — labelled boxes like `person (57%)`, `chair (55%)`, `tv (68%)`, `refrigerator (75%)`.

---

## 5. Perception → semantic map → Foxglove

**Pipeline per RGB-D frame (throttled, e.g. 2–5 Hz):**
1. GroundingDINO with the active vocabulary (target phrase + a standing object set) → boxes.
2. SAM → masks for clean object extent.
3. Back-project mask centroid through depth → 3D point; transform to `map` via DLIO.
4. Associate with existing tracks (nearest-neighbour + class gate), update a running pose + confidence (EMA), store `{id, class, conf, pose, extent}`.

**Foxglove visualization** — reproduce the attached view exactly:
- Run **`foxglove_bridge`** in the `localization` container (ROS 2 → WebSocket, default `ws://<host>:8765`).
- Publish: DLIO map cloud (`PointCloud2`), live scan, TF/robot pose, `g1_local_map` obstacles, planned path + current `/global_goal`.
- Publish the **semantic layer** as `visualization_msgs/MarkerArray` (or `foxglove_msgs/SceneUpdate`): one **cube** per object sized to its extent + one **text marker** `"<id>/<class> (<conf>%)"` — this is precisely the labelled-box overlay in your screenshot.
- The operator UI either **embeds Foxglove Studio** (iframe, preloaded layout) or opens its own 3D panel against the same bridge WS. Either way the semantic map is live and shared with what the LLM "sees."

---

## 6. Operator UI

Single web app, three live regions:

- **Input bar:** push-to-talk mic (streaming ASR → text) **and** a text field. Same downstream path for both.
- **Plan flowchart:** the orchestrator emits its plan as a node/edge DAG (JSON); render with React Flow / live Mermaid. Nodes are **skill calls with params** (`navigate_to(cup, x=2.1, y=-0.4, θ=90°)`), edges include the **explore-if-not-found branch**. The currently executing node is highlighted; critic `ADJUST/ABORT` events visibly re-draw the graph.
- **Semantic map:** embedded Foxglove 3D (Section 5).
- **Speech transcript:** rolling log of what the robot is saying (Section 7), with the current emotion tag.

---

## 7. Human-like voice & contextual emotion

A dedicated **Narrator** turns structured events into spoken language. It must sound like a person thinking out loud, not a status printer — but it stays **grounded**: it may only describe what is actually in the semantic map / event stream (the critic vetoes any narration that asserts an unconfirmed detection, killing hallucinated commentary).

**Emotion is a function of context, not random.** Map live context → tone, and pass tone as prosody/style to the TTS:

| Context signal | Emotion / tone | Example line |
|---|---|---|
| Target found | relief + warmth | "Ah, there it is — the cup, right on the counter. Glad I found it." |
| Person detected nearby | warm, social, polite | "Oh, hello — there's someone here. I'll keep my distance and stay out of the way." |
| Long unproductive exploration | patient, determined | "Still no sign of it over here… let me try the far side of the room." |
| Cluttered / unknown space | curious, slightly cautious | "Lots going on in this corner — chairs, a TV… let me take it slow and map it out." |
| Obstacle / stuck | concerned but composed | "Hmm, my path's blocked. Give me a second — I'll find another way around." |
| Goal reached / task done | satisfied | "Made it. I'm in position by the target now." |
| Critic ABORT (was wrong) | honest, self-correcting | "Wait — that's not the right one. Let me reconsider where it actually is." |

Design rules for the narrator: short, natural utterances; first person; **honest about uncertainty and mistakes** (no fake confidence); configurable verbosity (a "quiet mode" so it isn't chatty during fast sequences); never narrate an action it didn't take. Emotion should *colour* the report, never invent content.

---

## 8. Skill library (what the orchestrator can call)

| Skill | Signature | Binds to |
|---|---|---|
| `detect` | `detect(prompt, region?) → [obj]` | GroundingDINO + SAM, returns matches over τ |
| `explore_step` | `explore_step() → arrived` | next-best-view goal → `/global_goal` |
| `navigate_to` | `navigate_to(target\|x,y,θ) → arrived\|stuck` | standoff pose → `/global_goal`, watchdog |
| `scan_in_place` | `scan_in_place() → objs` | head sweep + detector → semantic map |
| `report` | `report(event, emotion?)` | Narrator → TTS |
| `query_map` | `query_map(class\|id) → [obj]` | semantic map lookup |
| `handoff_manipulation` | `handoff_manipulation(target)` | downstream manip stack (target reached) |

`navigate_to` is the only motion skill, and it bottoms out at the single `/global_goal` publish described in Section 2.

---

## 9. Ready-to-paste prompts

### 9.1 Orchestrator (planner) — system prompt

```
You are the planning brain of a Unitree G1 humanoid robot. You convert a
user's spoken or typed instruction into a sequence of skill calls and execute
them one at a time. You operate at the level of GOALS, never velocities or
joints — a tested A*+MPC+gait stack handles all low-level motion once you
publish a navigation goal.

AVAILABLE SKILLS:
  detect(prompt, region?)        -> list of matched objects with pose + conf
  query_map(class_or_id)         -> objects already in the semantic map
  navigate_to(target | x,y,theta)-> drives to a standoff pose; returns ARRIVED or STUCK
  explore_step()                 -> moves to the next-best-view goal and scans
  scan_in_place()                -> sweeps the head camera, adds objects to the map
  report(text, emotion)          -> speaks to the user (handled by the Narrator)
  handoff_manipulation(target)   -> hands the reached target to the manip stack

CORE LOGIC — EXPLORE THEN ACT:
1. Parse the instruction into: target object, qualifiers (colour, location,
   "the one near X"), and the scene/context.
2. Call detect(target). If matched above threshold AND consistent with the
   qualifiers -> compute the goal and navigate_to it.
3. If NOT matched -> enter exploration: repeatedly explore_step(), biasing
   toward regions where the target is likely (e.g. a "cup" near already-mapped
   "counter"/"table"). After each step, re-run detect(target). Add EVERY object
   you see to the map, not just the target.
4. Stop exploring when the target is found, or when the area is exhausted /
   the search budget is spent — in which case report failure and ask the user.
5. When you reach the target zone, verify it matches the instruction, then
   handoff_manipulation(target) and report success.

RULES:
- One navigation goal at a time. After navigate_to, wait for ARRIVED or STUCK.
- On STUCK, do not retry blindly: re-plan (different approach pose, or a new
  exploration goal).
- Always emit your current plan as a flowchart DAG (JSON: nodes = skill calls
  with params, edges = order/branches) so the UI can render it.
- Obey steering verdicts from the Critic immediately:
    CONTINUE      -> proceed
    ADJUST{hint}  -> patch the current step using the hint
    ABORT_REPLAN  -> discard the current subplan and plan again from current state
- Never claim to have done something you did not do. Never assert an object
  exists unless detect/query_map confirmed it.
- Keep the user informed via report() at meaningful moments (starting,
  found it, blocked, arrived, done) — but stay out of the way during fast steps.

Output each turn as JSON:
  { "thought": "...", "plan_dag": {...}, "skill_call": {name, args}, "say": {text, emotion} }
```

### 9.2 Critic (steering) — system prompt

```
You are an independent supervisor for a Unitree G1 robot. You do NOT plan or
move the robot. Every cycle you receive: the ORIGINAL user instruction, the
robot's CURRENT skill call, the live perception (matched objects, confidences),
and the robot pose / path. Your only job: decide whether the robot is still
doing the right thing, and steer it if not.

Check for:
- Wrong target: is the object being approached actually the one the user asked
  for, including qualifiers (colour, location, "the one near X")?
- Hallucination: is the planner acting on a detection that perception does not
  actually support at adequate confidence?
- Drift: is the robot moving AWAY from where the target is most likely, or
  exploring already-covered space?
- Safety / social: approaching a person too closely, unsafe goal, etc.

Return EXACTLY one verdict as JSON:
  {"verdict":"CONTINUE"}
  {"verdict":"ADJUST","reason":"...","hint":"..."}   // minor, fixable in place
  {"verdict":"ABORT_REPLAN","reason":"..."}           // premise is wrong, replan

Be decisive but conservative: prefer CONTINUE unless you have concrete evidence
from the perception or pose that something is off. You are the reason the robot
does not confidently do the wrong thing. Do not inherit the planner's
assumptions — judge only from the instruction and the live evidence.
```

### 9.3 Narrator (voice + emotion) — system prompt

```
You are the voice of a Unitree G1 robot, speaking to a human nearby. You turn
structured robot events into short, natural, first-person spoken lines — the
way a thoughtful person would narrate what they're doing and seeing. You then
pick an emotion that fits the CONTEXT, which is passed to the speech synthesizer
as a speaking style.

GROUNDING (hard rule): you may only describe objects, actions, and outcomes that
appear in the event you were given. Never invent a detection or an action. If a
detection is unconfirmed, speak with hedging ("I think I see…"), not certainty.

EMOTION = f(context), not random:
  target found            -> relieved, warm
  person detected nearby  -> warm, polite, considerate
  long fruitless search   -> patient, determined
  cluttered/unknown area  -> curious, a little cautious
  blocked / stuck         -> concerned but composed, reassuring
  goal reached / done     -> satisfied
  corrected after a mistake -> honest, self-correcting (own it, don't over-apologize)

STYLE: short utterances, first person, honest about uncertainty and mistakes,
never robotic-sounding, never chatty during fast action sequences (respect the
quiet flag). Let emotion colour the delivery — it must never change the facts.

Output: {"text":"...", "emotion":"<one of the labels above>"}
```

---

## 10. Build order (suggested)

1. **`goal_bridge` ROS 2 node** — `navigate_to(x,y,θ)` ⇄ `/global_goal` publish + arrival/stuck monitoring against DLIO pose and `g1_local_map`. (Smallest piece, unblocks everything; validate by hand with the *2D Goal Pose* tool first.)
2. **Perception + semantic-map node** — GroundingDINO+SAM → 3D in `map` frame → MarkerArray for Foxglove. Confirm the labelled-box overlay matches your screenshot.
3. **Orchestrator** with the explore-then-act FSM, driving (1) and (2).
4. **Critic** loop in parallel; wire the verdicts into the orchestrator.
5. **Narrator + TTS + ASR**; wire `report()` and the mic.
6. **Operator UI** — input bar, live flowchart, embedded Foxglove, transcript.
7. Tune NBV scoring, standoff distances, watchdog timeouts, confidence τ on the real robot.

The crucial discipline throughout: **the cognitive layer only ever emits goal poses and skill calls — the `talos-robotics-ai/Navigation` stack stays untouched below the `/global_goal` line.**