#!/usr/bin/env python3
"""
Gravity-aware ground segmentation by per-cell SVD/eigen plane fitting.

Pure-numpy (no ROS deps -> unit-testable) replacement for the old per-cell
lowest-point heuristic in ``local_voxel_map_node.segment_obstacles``. Operates on
the DLIO **accumulated odom-frame cloud** the local map already builds, using the
**gravity vector** DLIO gives us for free (odom is gravity-aligned at init, so
"up" = +Z = ``-g_hat``).

Pipeline (see docs/GROUND_REMOVAL_PLAN.md §2.3):

  1. Tile the cloud into XY cells of size ``cell``; drop cells with < ``min_pts``.
  2. Fit a plane per cell from the 3x3 covariance eigendecomposition:
     normal = smallest-eigenvalue eigenvector (oriented up); thickness =
     sqrt(lambda0); planarity = sqrt(lambda0/lambda1).
  3. A cell is a ground *candidate* if it is planar (thin) AND its normal is
     within ``slope_tol_deg`` of "up" (admits ramps/slopes, rejects walls).
  4. Region-grow the ground manifold from seed cells at the robot's foot height,
     across 8-neighbours whose planes stay continuous (height jump < ``step_tol``)
     -- so the surface may bend (ramp) but breaks at curbs/steps, and elevated
     horizontal slabs (shelf tops) stay obstacles because they don't connect.
  5. Label points: in a manifold cell, signed distance ``d`` to its plane decides
     ground (|d|<=band, drop) vs obstacle (band<d<=max_height, keep); points in
     non-manifold cells are kept (fail-open) up to ``max_height`` above the foot.

Fail-safe by construction: an empty/too-sparse cloud or a missing ground manifold
never blanks the obstacle cloud -- it passes geometry through (capped at
``max_height``). Better a cluttered costmap than a blind one.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GroundParams:
    """Tunables for :func:`segment_ground` (see docs/GROUND_REMOVAL_PLAN.md §3)."""

    cell: float = 0.40            # XY tile size for the per-cell plane fit (m)
    min_pts: int = 12             # min points to fit a cell plane
    planarity_max: float = 0.10   # sqrt(lambda0/lambda1) upper bound for "planar"
    flat_max: float = 0.05        # sqrt(lambda0) upper bound (m): absolute flatness
    slope_tol_deg: float = 30.0   # max plane<->gravity angle counted as ground (deg)
    step_tol: float = 0.08        # max height jump across a cell edge to keep growing (m)
    ground_band: float = 0.06     # |dist to ground plane| <= this => ground (m)
    seed_band: float = 0.15       # foot-height window for seed cells (m)
    leg_offset: float = 1.0       # robot_z (sensor, odom) -> foot height drop (m)
    max_height: float = 2.0       # ignore points this far above ground (m)
    min_total: int = 200          # below this many points, pass the cloud through


def segment_ground(xyz: np.ndarray, g_hat, robot_z: float,
                   params: GroundParams = GroundParams(),
                   return_info: bool = False):
    """Remove the (gravity-aware) ground from an accumulated odom-frame cloud.

    Args:
        xyz: (N, 3) float array of points in the gravity-aligned **odom** frame.
        g_hat: gravity unit vector in that frame (DLIO odom: ~(0, 0, -1) -> up=+Z).
        robot_z: robot/sensor Z in odom (from /dlio odom) -- sets the foot height
            (``robot_z - leg_offset``) used to seed the ground and cap fail-open cells.
        params: :class:`GroundParams`.
        return_info: also return a diagnostics dict (cell/seed/manifold counts).

    Returns:
        ``obstacle_xyz`` (M, 3), or ``(obstacle_xyz, info)`` if ``return_info``.
    """
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    n_pts = xyz.shape[0]
    foot = float(robot_z) - params.leg_offset

    # "up" = -gravity, normalised. Use it explicitly (not a hard-coded z index) so
    # a future per-scan gravity from the live odom orientation drops straight in.
    up = -np.asarray(g_hat, dtype=np.float64).reshape(3)
    up_norm = np.linalg.norm(up)
    up = up / up_norm if up_norm > 1e-9 else np.array([0.0, 0.0, 1.0])

    def _passthrough(status):
        keep = (xyz[:, 2] - foot) <= params.max_height if n_pts else np.zeros(0, bool)
        out = xyz[keep] if n_pts else xyz
        if return_info:
            return out, {"status": status, "n_pts": n_pts, "n_cells": 0,
                         "candidate_cells": 0, "seed_cells": 0, "ground_cells": 0,
                         "manifold_found": False}
        return out

    # Failsafe: too few points for a reliable per-cell fit -> pass through.
    if n_pts < params.min_total:
        return _passthrough("sparse")

    # 1. Tile into XY cells; build a compact per-cell id (inv) for vectorised stats.
    gx = np.floor(xyz[:, 0] / params.cell).astype(np.int64)
    gy = np.floor(xyz[:, 1] / params.cell).astype(np.int64)
    cell_ij, inv = np.unique(np.stack([gx, gy], axis=1), axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    n_cells = cell_ij.shape[0]
    counts = np.bincount(inv, minlength=n_cells)

    # 2. Per-cell mean + 3x3 covariance via binned moments (no Python loop / no SVD).
    def _csum(w):
        return np.bincount(inv, weights=w, minlength=n_cells)

    inv_c = 1.0 / np.maximum(counts, 1)
    mean = np.stack([_csum(xyz[:, 0]), _csum(xyz[:, 1]), _csum(xyz[:, 2])], axis=1) * inv_c[:, None]
    cov = np.empty((n_cells, 3, 3), dtype=np.float64)
    cov[:, 0, 0] = _csum(xyz[:, 0] * xyz[:, 0]) * inv_c - mean[:, 0] * mean[:, 0]
    cov[:, 1, 1] = _csum(xyz[:, 1] * xyz[:, 1]) * inv_c - mean[:, 1] * mean[:, 1]
    cov[:, 2, 2] = _csum(xyz[:, 2] * xyz[:, 2]) * inv_c - mean[:, 2] * mean[:, 2]
    cov[:, 0, 1] = cov[:, 1, 0] = _csum(xyz[:, 0] * xyz[:, 1]) * inv_c - mean[:, 0] * mean[:, 1]
    cov[:, 0, 2] = cov[:, 2, 0] = _csum(xyz[:, 0] * xyz[:, 2]) * inv_c - mean[:, 0] * mean[:, 2]
    cov[:, 1, 2] = cov[:, 2, 1] = _csum(xyz[:, 1] * xyz[:, 2]) * inv_c - mean[:, 1] * mean[:, 2]

    # eigh: ascending eigenvalues; eigvecs[..., :, 0] is the smallest -> plane normal.
    evals, evecs = np.linalg.eigh(cov)
    evals = np.clip(evals, 0.0, None)
    normal = evecs[:, :, 0].copy()
    ndotup = normal @ up
    flip = ndotup < 0.0          # orient every normal "up" (+ along -gravity)
    normal[flip] *= -1.0
    ndotup = np.abs(ndotup)

    thickness = np.sqrt(evals[:, 0])                       # RMS plane thickness (m)
    planarity = np.sqrt(evals[:, 0] / (evals[:, 1] + 1e-12))

    # 3. Ground-candidate cells: enough points, thin/planar, near-gravity-aligned.
    cos_tol = np.cos(np.deg2rad(params.slope_tol_deg))
    candidate = (
        (counts >= params.min_pts)
        & (thickness < params.flat_max)
        & (planarity < params.planarity_max)
        & (ndotup > cos_tol)
    )

    # 4. Region-grow the ground manifold from foot-height seeds across continuous
    #    candidate neighbours (height jump < step_tol). Lone slabs never connect.
    seed_mask = candidate & (np.abs(mean[:, 2] - foot) < params.seed_band)
    in_ground = np.zeros(n_cells, dtype=bool)

    if seed_mask.any():
        cellmap = {(int(cell_ij[k, 0]), int(cell_ij[k, 1])): k
                   for k in np.where(candidate)[0]}
        nz = np.where(np.abs(normal[:, 2]) > 1e-6, normal[:, 2], 1e-6)
        dq = deque()
        for s in np.where(seed_mask)[0]:
            if not in_ground[s]:
                in_ground[s] = True
                dq.append(int(s))
        neigh = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))
        while dq:
            a = dq.popleft()
            ai, aj = int(cell_ij[a, 0]), int(cell_ij[a, 1])
            for di, dj in neigh:
                b = cellmap.get((ai + di, aj + dj))
                if b is None or in_ground[b]:
                    continue
                # Height of A's plane at B's centroid vs B's own centroid height.
                dxy = mean[b, :2] - mean[a, :2]
                z_a = mean[a, 2] - (normal[a, 0] * dxy[0] + normal[a, 1] * dxy[1]) / nz[a]
                if abs(z_a - mean[b, 2]) < params.step_tol:
                    in_ground[b] = True
                    dq.append(b)

    # 5. Label every point by its cell.
    in_manifold = in_ground[inv]
    if in_manifold.any():
        # Signed distance to the owning cell's plane, along the up-oriented normal.
        d = np.einsum("ij,ij->i", xyz - mean[inv], normal[inv])
    else:
        d = np.zeros(n_pts)

    keep = np.empty(n_pts, dtype=bool)
    # Manifold cells: obstacle iff band < d <= max_height (drop ground & sub-ground).
    keep[in_manifold] = (d[in_manifold] > params.ground_band) & (d[in_manifold] <= params.max_height)
    # Non-manifold cells: fail-open -> keep geometry, only cap height above the foot.
    nm = ~in_manifold
    keep[nm] = (xyz[nm, 2] - foot) <= params.max_height

    obstacles = xyz[keep]
    if return_info:
        info = {
            "status": "ok" if in_ground.any() else "no_manifold",
            "n_pts": n_pts,
            "n_cells": n_cells,
            "candidate_cells": int(candidate.sum()),
            "seed_cells": int(seed_mask.sum()),
            "ground_cells": int(in_ground.sum()),
            "manifold_found": bool(in_ground.any()),
        }
        return obstacles, info
    return obstacles
