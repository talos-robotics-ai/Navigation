"""Offline logic test for the agentic-nav layer — no robot, no ROS runtime.

Uses duck-typed fakes for the node/nav so we exercise the skill polling, memory,
coordinator priority, and goal parsing deterministically. Run in the Humble
container (needs the ROS message types importable):
    python3 -m pytest src/agentic_nav/test/test_agentic_nav.py
or: python3 src/agentic_nav/test/test_agentic_nav.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from agentic_nav.spatial_memory.spatial_memory import SpatialMemory          # noqa: E402
from agentic_nav.skills.base import CAP_MOVEMENT, SkillContainer, skill       # noqa: E402
from agentic_nav.control.coordinator import NavigationCoordinator            # noqa: E402
from agentic_nav.nodes.low_level_nav_test_node import _parse_goals           # noqa: E402


class _FakeLogger:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass
    def error(self, *a, **k): pass


class _FakeNode:
    def get_logger(self): return _FakeLogger()


class _FakeNav:
    def __init__(self): self.goals = []; self.cancelled = False; self._reached = False
    def set_goal(self, x, y, yaw=0.0): self.goals.append((x, y, yaw))
    def cancel(self): self.cancelled = True
    def is_goal_reached(self): return self._reached


def test_spatial_memory():
    m = SpatialMemory()
    m.tag_location('Kitchen', 3.0, 1.0, 0.5)
    assert m.query_tagged_location('kitchen') == (3.0, 1.0, 0.5)
    assert m.query_tagged_location('go to the kitchen please') == (3.0, 1.0, 0.5)  # contains-match
    assert m.query_tagged_location('bathroom') is None
    assert m.query_by_text('a red chair') is None   # semantic deferred


def test_skill_registry():
    class C(SkillContainer):
        @skill('do a thing', uses=[CAP_MOVEMENT])
        def go(self): return 42
    c = C()
    sk = c.skills()
    assert 'go' in sk and sk['go'].uses == [CAP_MOVEMENT]
    assert c.skills()['go'].fn() == 42            # bound + callable
    assert c.describe()[0]['name'] == 'go'


def test_coordinator_priority():
    nav = _FakeNav()
    co = NavigationCoordinator(nav)
    assert co.request_goal('exploration', 1, 1)          # first taker wins
    assert co.active_source() == 'exploration'
    assert co.request_goal('user', 5, 5)                 # higher priority preempts
    assert co.active_source() == 'user'
    assert not co.request_goal('exploration', 9, 9)      # lower cannot preempt user
    assert nav.goals[-1] == (5.0, 5.0, 0.0)


def test_goal_parsing():
    g = _parse_goals('2.0,0.0,0.0; 1.5,-1.0; 0,0,3.14')
    assert g[0] == (2.0, 0.0, 0.0)
    assert g[1] == (1.5, -1.0, 0.0)   # yaw defaults to 0
    assert g[2] == (0.0, 0.0, 3.14)


def test_navigate_to_pose_polls_until_reached():
    from agentic_nav.skills.navigation_skill import NavigationSkillContainer
    nav = _FakeNav()
    skills = NavigationSkillContainer(_FakeNode(), nav)
    nav._reached = True                                  # already at goal
    assert skills.navigate_to_pose(1.0, 2.0, 0.0, timeout_s=1.0, poll_s=0.01) is True
    assert nav.goals[-1] == (1.0, 2.0, 0.0)
    nav2 = _FakeNav()
    skills2 = NavigationSkillContainer(_FakeNode(), nav2)
    assert skills2.navigate_to_pose(1.0, 2.0, timeout_s=0.05, poll_s=0.01) is False  # never reached


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'{name} ... OK')
    print('\nALL AGENTIC-NAV LOGIC TESTS PASSED')
