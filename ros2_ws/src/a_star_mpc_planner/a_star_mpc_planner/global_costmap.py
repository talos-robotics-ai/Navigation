"""
World-fixed global costmap for the global planner layer (long-horizon routing).

Why a SEPARATE map from the local FixedGaussianGridMap
------------------------------------------------------
The local costmap must stay clean and live so reactive avoidance and the MPCC
stay efficient. Long-horizon "plan the safe way home" memory therefore lives
here, in a coarse, world-fixed grid that is consumed ONLY by the global planner
to produce a route — never fused into the local costmap.

Two layers, both world-fixed in the odom frame:
  occupancy  — a hit-counted confidence per cell. A cell needs ``hit_threshold``
               separate observations before it counts as an obstacle, so a
               single-frame ghost (the failure mode that broke the naive DLIO
               fusion) does NOT block routing. Slow per-build decay lets a moved
               obstacle fade.
  free        — a "breadcrumb" of where the robot has actually driven (within
               ``free_radius``). The corridor the robot came through is known
               traversable, so the global planner can always find the safe way
               back along it. Free overrides occupancy in the global layer; the
               LIVE local costmap is the safety net for anything dynamic that
               later appears on that corridor.

The class deliberately mirrors the read interface of FixedGaussianGridMap
(``gmap``/``hmap``/``cells``/``reso``/``minx``/``miny``/``world_to_index``/
``index_to_world``) so the existing AStarPlanner runs on it unchanged.
"""

import numpy as np
from scipy.ndimage import distance_transform_edt


class GlobalCostmap:
    def __init__(
        self,
        reso: float = 0.20,
        half_width: float = 25.0,
        robot_radius: float = 0.35,
        inflation_radius: float = 0.70,
        soft_cost_max: float = 0.49,
        hit_threshold: float = 2.0,
        hit_cap: float = 6.0,
        decay: float = 0.997,
        free_radius: float = 0.35,
    ):
        self.reso = float(reso)
        self.half_width = float(half_width)
        self.robot_radius = float(robot_radius)
        self.inflation_radius = max(float(inflation_radius), float(robot_radius))
        self.soft_cost_max = float(np.clip(soft_cost_max, 0.0, 0.499))
        self.hit_threshold = float(hit_threshold)
        self.hit_cap = float(hit_cap)
        self.decay = float(decay)
        self.free_radius = float(free_radius)

        self.cells = int(round(2.0 * self.half_width / self.reso))

        # World-frame origin (bottom-left). Fixed at the first set_origin() call.
        self.minx = 0.0
        self.miny = 0.0
        self._origin_set = False

        # Layers (allocated on first set_origin)
        self._occ: np.ndarray | None = None    # confidence counts
        self._free: np.ndarray | None = None   # breadcrumb traversable

        # AStarPlanner-compatible read interface
        self.gmap: np.ndarray | None = None
        self.hmap = None                       # no 2.5D layer on the global map

    # ------------------------------------------------------------------
    # Origin / lifecycle
    # ------------------------------------------------------------------

    def set_origin(self, robot_xy) -> None:
        """Fix the world origin so the grid is centred on the first robot pose."""
        if self._origin_set:
            return
        self.minx = float(robot_xy[0]) - self.half_width
        self.miny = float(robot_xy[1]) - self.half_width
        self._occ = np.zeros((self.cells, self.cells), dtype=np.float32)
        self._free = np.zeros((self.cells, self.cells), dtype=bool)
        self.gmap = np.zeros((self.cells, self.cells), dtype=np.float32)
        self._origin_set = True

    @property
    def ready(self) -> bool:
        return self._origin_set

    # ------------------------------------------------------------------
    # Accumulation
    # ------------------------------------------------------------------

    def update(self, obstacle_points_world, robot_xy) -> None:
        """Fold one frame of obstacles + the robot footprint into the map.

        obstacle_points_world : (N, 2|3) world-frame obstacle points (the clean,
                                already-ground-removed local cloud).
        robot_xy              : current robot position (marks a free breadcrumb).
        """
        if not self._origin_set:
            self.set_origin(robot_xy)

        # Slow global decay so stale/removed obstacles fade over time.
        if self.decay < 1.0:
            self._occ *= self.decay

        # ── Breadcrumb free-space under/around the robot ──
        self._stamp_disk(self._free_set, robot_xy, self.free_radius)

        # ── Obstacle hits ──
        if obstacle_points_world is not None and len(obstacle_points_world) > 0:
            pts = np.asarray(obstacle_points_world, dtype=float)
            ix = ((pts[:, 0] - self.minx) / self.reso).astype(np.intp)
            iy = ((pts[:, 1] - self.miny) / self.reso).astype(np.intp)
            inb = (ix >= 0) & (ix < self.cells) & (iy >= 0) & (iy < self.cells)
            ix, iy = ix[inb], iy[inb]
            if len(ix) > 0:
                # +1 per observed cell this frame (dedupe so one frame = one hit).
                hit = np.zeros((self.cells, self.cells), dtype=np.float32)
                hit[ix, iy] = 1.0
                self._occ = np.minimum(self._occ + hit, self.hit_cap)

    def _free_set(self, ix, iy) -> None:
        self._free[ix, iy] = True

    def _stamp_disk(self, fn, center_xy, radius) -> None:
        """Apply fn(ix, iy) over the cells within `radius` of center_xy."""
        cx = (float(center_xy[0]) - self.minx) / self.reso
        cy = (float(center_xy[1]) - self.miny) / self.reso
        r = max(1, int(round(radius / self.reso)))
        ix0, ix1 = int(cx) - r, int(cx) + r + 1
        iy0, iy1 = int(cy) - r, int(cy) + r + 1
        xs = np.arange(max(0, ix0), min(self.cells, ix1))
        ys = np.arange(max(0, iy0), min(self.cells, iy1))
        if len(xs) == 0 or len(ys) == 0:
            return
        gx, gy = np.meshgrid(xs, ys, indexing='ij')
        mask = (gx - cx) ** 2 + (gy - cy) ** 2 <= float(r) ** 2
        fn(gx[mask], gy[mask])

    # ------------------------------------------------------------------
    # Costmap build (robot-radius inflation, identical model to the local map)
    # ------------------------------------------------------------------

    def build(self) -> None:
        """Rebuild self.gmap from the accumulated layers."""
        if not self._origin_set:
            return
        occupied = (self._occ >= self.hit_threshold) & (~self._free)
        gmap = np.zeros((self.cells, self.cells), dtype=np.float32)
        if not occupied.any():
            self.gmap = gmap
            return
        min_d = distance_transform_edt(~occupied) * self.reso
        lethal = min_d <= self.robot_radius
        gmap[lethal] = 1.0
        if self.inflation_radius > self.robot_radius:
            band = (~lethal) & (min_d <= self.inflation_radius)
            decay_len = max((self.inflation_radius - self.robot_radius) / 3.0, 1e-3)
            gmap[band] = (self.soft_cost_max *
                          np.exp(-(min_d[band] - self.robot_radius) / decay_len)).astype(np.float32)
        # Breadcrumb-free cells are always traversable in the global layer.
        gmap[self._free] = 0.0
        self.gmap = gmap

    def confirmed_hit_points(self) -> np.ndarray | None:
        """(K, 2) world xy of CONFIRMED obstacle cell centres — pre-inflation.

        Confirmed = hit-count ≥ hit_threshold and not breadcrumb-free: the same
        anti-ghost gate build() rasterises, but WITHOUT the robot-radius
        inflation. This is the layer the local planner fuses (global+local
        fusion mode): raw cells fuse cleanly because the local costmap applies
        its own inflation — fusing the inflated grid would double-inflate and
        close doorways.
        """
        if not self._origin_set:
            return None
        occupied = (self._occ >= self.hit_threshold) & (~self._free)
        idx = np.argwhere(occupied)
        if idx.size == 0:
            return None
        return (idx.astype(np.float64) + 0.5) * self.reso + np.array([self.minx, self.miny])

    # ------------------------------------------------------------------
    # Coordinate helpers (match FixedGaussianGridMap)
    # ------------------------------------------------------------------

    def world_to_index(self, x: float, y: float):
        ix = int((x - self.minx) / self.reso)
        iy = int((y - self.miny) / self.reso)
        if 0 <= ix < self.cells and 0 <= iy < self.cells:
            return ix, iy
        return None, None

    def index_to_world(self, ix: int, iy: int):
        return (ix * self.reso + self.minx, iy * self.reso + self.miny)
