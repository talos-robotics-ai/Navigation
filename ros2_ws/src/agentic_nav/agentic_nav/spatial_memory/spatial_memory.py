"""SpatialMemory — the 'where things are' store the language layer queries.

Phase 1 (this branch): the TAGGED-location half is real and useful without any
vision — you can `tag_location('kitchen', x, y)` at runtime (e.g. drive there and
tag it) and later `query_tagged_location('kitchen')` resolves to a pose. This
already enables spoken known-goals once the agent is wired, and is testable now.

Phase 2+ (deferred): `query_by_text` is the SEMANTIC path — matching a free-text
query against object/landmark detections accumulated from a VLM detector. It
returns None until that perception + embedding store exists. Its design mirrors
the geometric GlobalCostmap memory: world-anchored, hit-counted for
anti-hallucination, decayed for permanence, re-anchored on odom jumps.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class TaggedLocation:
    name: str
    x: float
    y: float
    yaw: float = 0.0


@dataclass
class SemanticObject:
    """A detected object instance (Phase 2). Same memory model as GlobalCostmap:
    world-anchored, hit-counted, decayed. Populated by the VLM detector later."""
    label: str
    x: float
    y: float
    z: float = 0.0
    hit_count: int = 0
    last_seen: float = 0.0
    embedding: Optional[list] = None


class SpatialMemory:
    def __init__(self):
        self._tags: Dict[str, TaggedLocation] = {}
        self._objects: List[SemanticObject] = []   # Phase 2, populated by perception

    # ── Tagged locations (REAL now) ────────────────────────────────────
    def tag_location(self, name: str, x: float, y: float, yaw: float = 0.0) -> None:
        self._tags[name.strip().lower()] = TaggedLocation(name, float(x), float(y), float(yaw))

    def query_tagged_location(self, query: str) -> Optional[Tuple[float, float, float]]:
        loc = self._tags.get(query.strip().lower())
        if loc is None:
            # loose contains-match ("go to the kitchen" -> "kitchen")
            for key, loc2 in self._tags.items():
                if key in query.strip().lower():
                    loc = loc2
                    break
        return None if loc is None else (loc.x, loc.y, loc.yaw)

    def tagged_names(self) -> List[str]:
        return [t.name for t in self._tags.values()]

    # ── Semantic query (DEFERRED — needs the VLM detector) ─────────────
    def query_by_text(self, query: str) -> Optional[Tuple[float, float, float]]:
        """Match free text against accumulated object detections. Returns None
        until the Phase-2 VLM perception + embedding store is implemented."""
        if not self._objects:
            return None
        # TODO(phase2): embed `query`, nearest-object by embedding+distance, gate
        # on hit_count (anti-hallucination) + permanence. Not active in Phase 1.
        return None
