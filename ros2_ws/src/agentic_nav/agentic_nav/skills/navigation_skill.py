"""NavigationSkillContainer — the navigation capability, exposed as skills.

Phase 1 (this branch): the GEOMETRIC skills are real and testable —
``navigate_to_pose`` / ``stop_navigation`` drive the existing A*+MPC stack
through the NavigationInterface and block on `/navigation/state`.

Phase 2+ (deferred): ``navigate_with_text`` is the language entry point. Its
control flow is written out but the perception/memory calls are stubbed and
raise NotImplementedError, so the shape is fixed and reviewable, and nothing
pretends to work until the spatial memory + VLM land. Testing the geometric
skills first is exactly the point of this branch.
"""
from __future__ import annotations

import time
from typing import Optional

from ..interfaces.navigation_interface import NavigationInterface
from ..spatial_memory.spatial_memory import SpatialMemory
from .base import CAP_MEMORY, CAP_MOVEMENT, CAP_PERCEPTION, SkillContainer, skill


class NavigationSkillContainer(SkillContainer):
    def __init__(self, node, nav: NavigationInterface,
                 memory: Optional[SpatialMemory] = None):
        self._node = node
        self._nav = nav
        self._memory = memory

    # ── Phase 1: geometric skills (REAL, testable now) ─────────────────
    @skill('Navigate to an absolute metric goal (x, y[, yaw]) in the world frame. '
           'Blocks until the goal is reached or a timeout elapses.',
           uses=[CAP_MOVEMENT])
    def navigate_to_pose(self, x: float, y: float, yaw: float = 0.0,
                         timeout_s: float = 60.0, poll_s: float = 0.25) -> bool:
        self._nav.set_goal(x, y, yaw)
        deadline = time.time() + timeout_s
        # NAVIGATING is not observed instantly; give the planner a moment to latch.
        while time.time() < deadline:
            if self._nav.is_goal_reached():
                self._node.get_logger().info('[skill] navigate_to_pose: GOAL_REACHED')
                return True
            time.sleep(poll_s)
        self._node.get_logger().warning('[skill] navigate_to_pose: timed out')
        return False

    @skill('Stop navigation immediately (engages the software e-stop).',
           uses=[CAP_MOVEMENT])
    def stop_navigation(self) -> bool:
        self._nav.cancel()
        return True

    # ── Phase 2+: language/semantic skill (SCAFFOLDED, deferred) ────────
    @skill('Navigate to a natural-language target ("go to the kitchen", '
           '"find a chair"). Resolves the target against spatial memory / vision, '
           'then drives there. DEFERRED until the spatial memory + VLM land.',
           uses=[CAP_MOVEMENT, CAP_PERCEPTION, CAP_MEMORY])
    def navigate_with_text(self, query: str, timeout_s: float = 90.0) -> bool:
        if self._memory is None:
            raise NotImplementedError(
                'navigate_with_text needs SpatialMemory — not wired in Phase 1.')
        # The intended resolution pipeline (kept explicit so it is reviewable):
        #   1) tagged-location memory  -> exact pose
        pose = self._memory.query_tagged_location(query)
        #   2) live vision (VLM detect + localize) -> pose            [deferred]
        #   3) semantic map match (embedding query) -> pose
        if pose is None:
            pose = self._memory.query_by_text(query)
        if pose is None:
            # 4) unknown -> hand to exploration until found          [deferred]
            raise NotImplementedError(
                'target not in memory; exploration + VLM search is a later phase.')
        return self.navigate_to_pose(pose[0], pose[1], pose[2] if len(pose) > 2 else 0.0,
                                     timeout_s=timeout_s)
