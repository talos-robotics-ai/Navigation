"""
World-fixed global costmap for the global planner layer (long-horizon routing).

Why a SEPARATE map from the local FixedGaussianGridMap
------------------------------------------------------
The local costmap must stay clean and live so reactive avoidance and the MPCC
stay efficient. Long-horizon "plan the safe way home" memory therefore lives
here, in a coarse, world-fixed grid that is consumed ONLY by the global planner
to produce a route — never fused into the local costmap.

Layers, all world-fixed in the odom frame:
  occupancy  — a hit-counted confidence per cell. A cell needs ``hit_threshold``
               separate observations before it counts as an obstacle, so a
               single-frame ghost (the failure mode that broke the naive DLIO
               fusion) does NOT block routing. Slow per-build decay lets a moved
               obstacle fade.
  free        — a "breadcrumb" of where the robot has actually driven (within
               ``free_radius``). The corridor the robot came through is known
               traversable, so the global planner can always find the safe way
               back along it.
  penalty     — (T2) a decaying soft-cost layer of "explored but this way did
               NOT make progress" regions, stamped by the global planner when it
               detects the robot is stuck/oscillating. Raises route cost (never
               lethal) so A* prefers a fresh detour but can still traverse if
               there is no alternative. Decays so a temporary blockage is retried.

Memory model (the three upgrades)
---------------------------------
* T1 — the grid GROWS to contain the whole traversed scene instead of a fixed
  box pinned at the start pose (``_reframe`` reallocates + copies, cheap at the
  coarse 0.20 m resolution). It is capped at ``max_half_width``; once at the cap
  it ROLLS to re-centre on the robot (bounded-memory fallback, drops the far edge).
* T2 — the ``penalty`` layer + ``stamp_penalty`` give dead-end / stuck memory.
* T3 — ``shift`` re-anchors the accumulated layers after a DLIO odom
  discontinuity (loop closure) so a jump doesn't smear memory into ghost walls.

The class deliberately mirrors the read interface of FixedGaussianGridMap
(``gmap``/``cells``/``reso``/``minx``/``miny``/``world_to_index``/
``index_to_world``) so the existing AStarPlanner runs on it unchanged.
"""

import math

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
        max_half_width: float = 80.0,
        penalty_decay: float = 0.985,
        penalty_max: float = 4.0,
        penalty_cost_max: float = 0.60,
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
        # T1 growth cap; T2 penalty dynamics.
        self.max_half_width = max(float(max_half_width), self.half_width)
        self.penalty_decay = float(np.clip(penalty_decay, 0.0, 1.0))
        self.penalty_max = float(penalty_max)
        self.penalty_cost_max = float(np.clip(penalty_cost_max, 0.0, 0.99))

        self.cells = int(round(2.0 * self.half_width / self.reso))

        # World-frame origin (bottom-left). Set at first set_origin(); afterwards
        # it moves as the grid grows/rolls (T1) or is re-anchored (T3).
        self.minx = 0.0
        self.miny = 0.0
        self._origin_set = False

        # Layers (allocated on first set_origin)
        self._occ: np.ndarray | None = None      # confidence counts
        self._free: np.ndarray | None = None     # breadcrumb traversable
        self._penalty: np.ndarray | None = None  # dead-end / stuck soft cost

        # AStarPlanner-compatible read interface
        self.gmap: np.ndarray | None = None
        self.hmap = None                         # no 2.5D layer on the global map

    # ------------------------------------------------------------------
    # Origin / lifecycle
    # ------------------------------------------------------------------

    def set_origin(self, robot_xy) -> None:
        """Fix the world origin so the grid is initially centred on the robot."""
        if self._origin_set:
            return
        self.minx = float(robot_xy[0]) - self.half_width
        self.miny = float(robot_xy[1]) - self.half_width
        self._occ = np.zeros((self.cells, self.cells), dtype=np.float32)
        self._free = np.zeros((self.cells, self.cells), dtype=bool)
        self._penalty = np.zeros((self.cells, self.cells), dtype=np.float32)
        self.gmap = np.zeros((self.cells, self.cells), dtype=np.float32)
        self._origin_set = True

    @property
    def ready(self) -> bool:
        return self._origin_set

    # ------------------------------------------------------------------
    # T1 — grow / re-centre so the grid contains the whole traversed scene
    # ------------------------------------------------------------------

    def _place(self, old, new_cells, off_x, off_y, fill):
        """Copy `old` into a fresh (new_cells, new_cells) array at cell offset
        (off_x, off_y). Handles negative offsets (rolling drops the far edge)."""
        new = np.full((new_cells, new_cells), fill, dtype=old.dtype)
        sx, sy = max(0, off_x), max(0, off_y)          # dest start
        ox, oy = max(0, -off_x), max(0, -off_y)        # src start
        cx = min(old.shape[0] - ox, new_cells - sx)
        cy = min(old.shape[1] - oy, new_cells - sy)
        if cx > 0 and cy > 0:
            new[sx:sx + cx, sy:sy + cy] = old[ox:ox + cx, oy:oy + cy]
        return new

    def _regrid(self, new_minx, new_miny, new_cells) -> None:
        """Reallocate every layer onto a new origin/size, preserving world data.
        new_minx/new_miny MUST sit on the current cell lattice (integer offset)."""
        off_x = int(round((self.minx - new_minx) / self.reso))
        off_y = int(round((self.miny - new_miny) / self.reso))
        self._occ = self._place(self._occ, new_cells, off_x, off_y, 0.0)
        self._free = self._place(self._free, new_cells, off_x, off_y, False)
        self._penalty = self._place(self._penalty, new_cells, off_x, off_y, 0.0)
        self.gmap = np.zeros((new_cells, new_cells), dtype=np.float32)
        self.minx, self.miny, self.cells = new_minx, new_miny, new_cells

    def _reframe(self, robot_xy, pts_xy) -> None:
        """Ensure the grid comfortably contains the robot + this frame's points.
        Grows on the existing lattice up to max_half_width, then rolls to
        re-centre on the robot once at the cap."""
        if not self._origin_set:
            return
        stack = [np.asarray(robot_xy, dtype=float).reshape(1, 2)]
        if pts_xy is not None and len(pts_xy) > 0:
            stack.append(np.asarray(pts_xy, dtype=float).reshape(-1, 2)[:, :2])
        allp = np.vstack(stack)
        lo = allp.min(axis=0)
        hi = allp.max(axis=0)
        cur_maxx = self.minx + self.cells * self.reso
        cur_maxy = self.miny + self.cells * self.reso
        pad = self.reso * 6.0
        if (lo[0] - pad >= self.minx and lo[1] - pad >= self.miny and
                hi[0] + pad < cur_maxx and hi[1] + pad < cur_maxy):
            return  # comfortably inside — nothing to do

        max_cells = int(round(2.0 * self.max_half_width / self.reso))

        # Extend the origin down (staying on the lattice) and size up to fit
        # everything + pad. Decide GROW vs ROLL by the NEEDED size vs the cap.
        new_minx, new_miny = self.minx, self.miny
        if lo[0] - pad < self.minx:
            new_minx = self.minx - math.ceil((self.minx - (lo[0] - pad)) / self.reso) * self.reso
        if lo[1] - pad < self.miny:
            new_miny = self.miny - math.ceil((self.miny - (lo[1] - pad)) / self.reso) * self.reso
        need_maxx = max(cur_maxx, hi[0] + pad)
        need_maxy = max(cur_maxy, hi[1] + pad)
        span = max(need_maxx - new_minx, need_maxy - new_miny)
        need_cells = int(math.ceil(span / self.reso))

        if need_cells <= max_cells:
            self._regrid(new_minx, new_miny, need_cells)   # GROW to fit
            return

        # Exceeds the cap → ROLL a max-size grid re-centred on the robot, dropping
        # the far edge (bounded-memory fallback). Snap to the old lattice.
        want_minx = float(robot_xy[0]) - self.max_half_width
        want_miny = float(robot_xy[1]) - self.max_half_width
        off_x = int(round((self.minx - want_minx) / self.reso))
        off_y = int(round((self.miny - want_miny) / self.reso))
        self._regrid(self.minx - off_x * self.reso, self.miny - off_y * self.reso, max_cells)

    def shift(self, dx: float, dy: float) -> None:
        """T3 — re-anchor after an odom discontinuity (loop closure).

        A jump means the robot's reported position stepped by (dx, dy) without
        real motion, so every accumulated cell's odom coordinate is now stale by
        that amount. Moving the origin by the same delta re-labels the existing
        data into the new odom frame in O(1), keeping memory aligned instead of
        smearing it into ghost walls.
        """
        if not self._origin_set:
            return
        self.minx += float(dx)
        self.miny += float(dy)

    # ------------------------------------------------------------------
    # Accumulation
    # ------------------------------------------------------------------

    def update(self, obstacle_points_world, robot_xy) -> None:
        """Fold one frame of obstacles + the robot footprint into the map."""
        if not self._origin_set:
            self.set_origin(robot_xy)
        # T1: grow/roll so the robot and this frame's obstacles fit before we write.
        self._reframe(robot_xy, obstacle_points_world)

        # Slow global decay so stale/removed obstacles fade over time.
        if self.decay < 1.0:
            self._occ *= self.decay
        # T2: dead-end penalties fade so a temporary blockage is eventually retried.
        if self.penalty_decay < 1.0:
            self._penalty *= self.penalty_decay

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

    def stamp_penalty(self, center_xy, radius: float, amount: float) -> None:
        """T2 — mark a region as 'explored, did not make progress' (soft repulsion).
        Called by the global planner on a stuck/oscillation event."""
        if not self._origin_set:
            return

        def _add(ix, iy):
            self._penalty[ix, iy] = np.minimum(
                self._penalty[ix, iy] + float(amount), self.penalty_max)

        self._stamp_disk(_add, center_xy, radius)

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
        if occupied.any():
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
        # T2: dead-end/stuck penalty raises cost everywhere (soft, never lethal),
        # applied AFTER the breadcrumb override so a corridor the robot proved to
        # be a dead-end stays costly even though it's on the breadcrumb.
        if self._penalty is not None and self._penalty.max() > 0.0:
            pen = np.clip(self._penalty, 0.0, self.penalty_cost_max)
            gmap = np.maximum(gmap, pen)
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
