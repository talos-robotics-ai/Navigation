"""Skill primitives — the callable-capability layer.

A *skill* is a named, self-describing action an agent (or a test harness, or a
coordinator) can invoke. Skills declare the capabilities they *use* (e.g. robot
movement) so a scheduler can gate/arbitrate them. This is the same shape the
agent layer will call into later; for Phase 1 we invoke them directly from the
low-level test harness — no LLM required.

Deliberately tiny and framework-free: a decorator that tags a method as a skill
and records its metadata, plus a registry so a container can enumerate them.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List

# Capability tags — what a skill touches (for arbitration / safety gating).
CAP_MOVEMENT = 'movement'
CAP_PERCEPTION = 'perception'
CAP_MEMORY = 'memory'
CAP_SPEECH = 'speech'


@dataclass
class SkillSpec:
    name: str
    description: str
    uses: List[str] = field(default_factory=list)
    fn: Callable = None


def skill(description: str, uses: List[str] | None = None):
    """Decorator: mark a method as an agent/harness-callable skill."""
    def deco(fn: Callable):
        fn._skill = SkillSpec(name=fn.__name__, description=description,
                              uses=list(uses or []), fn=fn)
        return fn
    return deco


class SkillContainer:
    """Base for a class that exposes @skill-decorated methods."""

    def skills(self) -> Dict[str, SkillSpec]:
        out: Dict[str, SkillSpec] = {}
        for attr in dir(self):
            m = getattr(self, attr)
            spec = getattr(m, '_skill', None)
            if spec is not None:
                # bind the spec's fn to this instance
                bound = SkillSpec(spec.name, spec.description, spec.uses, m)
                out[spec.name] = bound
        return out

    def describe(self) -> List[dict]:
        """Machine-readable list of skills — the shape an agent tool-list needs."""
        return [{'name': s.name, 'description': s.description, 'uses': s.uses}
                for s in self.skills().values()]
