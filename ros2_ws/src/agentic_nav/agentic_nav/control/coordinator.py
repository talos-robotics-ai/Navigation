"""NavigationCoordinator — arbitrates WHO sets the goal.

Multiple sources can want the robot to go somewhere: an explicit user/teleop
goal, a semantic/agent goal, an exploration goal. The coordinator resolves them
by priority so exactly one goal reaches the geometric planner at a time, and the
safety chain (e-stop -> MPC fail-safe) is never bypassed.

Phase 1 only needs the priority table + a `request_goal` seam; the exploration
and agent sources plug in later without touching the planner.
"""
from __future__ import annotations

from typing import Optional, Tuple

from ..interfaces.navigation_interface import NavigationInterface

# Higher number wins. Explicit human intent always beats autonomy.
PRIORITY = {
    'user': 100,        # teleop / operator / low-level test
    'semantic': 50,     # agent language goal (Phase 3)
    'exploration': 10,  # autonomous frontier search (Phase 4)
}


class NavigationCoordinator:
    def __init__(self, nav: NavigationInterface):
        self._nav = nav
        self._active_source: Optional[str] = None
        self._active_prio = -1

    def request_goal(self, source: str, x: float, y: float, yaw: float = 0.0) -> bool:
        """A source asks to drive to (x, y). Wins iff its priority >= the active
        source's (a source may always update its own goal)."""
        prio = PRIORITY.get(source, 0)
        if self._active_source is not None and source != self._active_source \
                and prio < self._active_prio:
            return False
        self._active_source, self._active_prio = source, prio
        self._nav.set_goal(x, y, yaw)
        return True

    def release(self, source: str) -> None:
        if source == self._active_source:
            self._active_source, self._active_prio = None, -1

    def stop(self) -> None:
        self._nav.cancel()
        self._active_source, self._active_prio = None, -1

    def active_source(self) -> Optional[str]:
        return self._active_source
