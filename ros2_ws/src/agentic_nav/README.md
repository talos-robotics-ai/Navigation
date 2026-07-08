# agentic_nav

Agentic navigation layer over the geometric A\*+MPC planner — **skills + a clean
interface + composable blueprints**, built **test-first**. Phase 1 (this branch)
exposes and validates the *low-level* navigation before any VLM/agent is added.

See `docs/planning/AGENTIC_NAV_ARCHITECTURE.md` for the full design and
`docs/planning/SEMANTIC_SPATIAL_NAV_ROADMAP.md` for the phased plan.

## Layout

```
agentic_nav/
  interfaces/navigation_interface.py   the ONE seam: set_goal/get_state/is_goal_reached/cancel
  skills/base.py                       @skill decorator + registry
  skills/navigation_skill.py           navigate_to_pose ✅  stop ✅  navigate_with_text ⏸
  spatial_memory/spatial_memory.py     tag/query_tagged ✅  query_by_text ⏸ (VLM, Phase 2)
  control/coordinator.py               goal-source priority (user > semantic > exploration)
  control/blueprints/mobile.py         build_mobile_nav(node) — composes the stack
  nodes/low_level_nav_test_node.py     Phase-1 acceptance harness (no VLM)
  test/test_agentic_nav.py             offline logic test
```

## Use

Build (in the Humble container / on the planner host):
```bash
colcon build --packages-select agentic_nav && source install/setup.bash
```

Offline logic test (no robot):
```bash
python3 src/agentic_nav/test/test_agentic_nav.py
```

Low-level nav acceptance (planner + robot up):
```bash
ros2 run agentic_nav low_level_nav_test --ros-args -p goals:="2.0,0.0,0.0; 0.0,0.0,3.14"
```

Compose the stack in your own node:
```python
from agentic_nav.control.blueprints.mobile import build_mobile_nav
stack = build_mobile_nav(node)
stack.skills.navigate_to_pose(2.0, 0.0, 0.0)
stack.memory.tag_location('kitchen', 4.0, 1.0)
```

**Deferred by design:** `navigate_with_text`, `query_by_text`, and the agent/LLM
node — added in Phase 2/3 once the low-level nav is proven.
