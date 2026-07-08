"""mobile blueprint — compose the agentic-nav stack for a mobile base.

A *blueprint* wires the layers together into a ready-to-use stack, so a node (or
the agent) constructs the whole thing with one call and never hand-wires topics.
This is the single composition point: swap the interface target, add the memory,
or attach the agent here without touching the layers themselves.

Phase 1 returns: NavigationInterface + SpatialMemory + NavigationSkillContainer +
NavigationCoordinator. The VLM detector + agent get added to this same blueprint
in later phases.
"""
from __future__ import annotations

from dataclasses import dataclass

from ...interfaces.navigation_interface import NavigationInterface
from ...skills.navigation_skill import NavigationSkillContainer
from ...spatial_memory.spatial_memory import SpatialMemory
from ..coordinator import NavigationCoordinator


@dataclass
class MobileNavStack:
    nav: NavigationInterface
    memory: SpatialMemory
    skills: NavigationSkillContainer
    coordinator: NavigationCoordinator


def build_mobile_nav(node, **interface_kwargs) -> MobileNavStack:
    """Construct + wire the Phase-1 agentic-nav stack on an existing rclpy node."""
    nav = NavigationInterface(node, **interface_kwargs)
    memory = SpatialMemory()
    skills = NavigationSkillContainer(node, nav, memory)
    coordinator = NavigationCoordinator(nav)
    return MobileNavStack(nav=nav, memory=memory, skills=skills, coordinator=coordinator)
