#!/usr/bin/env python3
"""Unit tests for the gravity-aware SVD ground segmentation (no ROS deps).

Mirrors the validation matrix in docs/GROUND_REMOVAL_PLAN.md §5. Pure numpy, so
runnable with plain `pytest` or `python3 test_ground_segmentation.py`.
"""

import numpy as np

from g1_local_map.ground_segmentation import GroundParams, segment_ground

G_HAT = np.array([0.0, 0.0, -1.0])   # odom: up = +Z
ROBOT_Z = 0.0                        # sensor at z=0; floor ~1 m below (leg_offset)
FLOOR_Z = ROBOT_Z - GroundParams().leg_offset   # = -1.0


def _grid(x0, x1, y0, y1, step, z_fn, jitter=0.0, rng=None):
    """Dense XY grid of points whose z = z_fn(x, y) (+ optional gaussian jitter)."""
    xs = np.arange(x0, x1, step)
    ys = np.arange(y0, y1, step)
    gx, gy = np.meshgrid(xs, ys)
    gx, gy = gx.ravel(), gy.ravel()
    z = z_fn(gx, gy)
    if jitter and rng is not None:
        z = z + rng.normal(0.0, jitter, size=z.shape)
    return np.stack([gx, gy, z], axis=1)


def _flat_floor(rng, jitter=0.005):
    return _grid(-4, 4, -4, 4, 0.05, lambda x, y: np.full_like(x, FLOOR_Z),
                 jitter=jitter, rng=rng)


def _frac_kept(obs, lo, hi):
    """Fraction of obstacle points with z in [lo, hi]."""
    if obs.shape[0] == 0:
        return 0.0
    return float(((obs[:, 2] >= lo) & (obs[:, 2] <= hi)).mean())


def test_flat_floor_removed_boxes_kept():
    rng = np.random.default_rng(0)
    floor = _flat_floor(rng)
    # Two solid boxes standing on the floor (0.3 m tall).
    box1 = _grid(1.0, 1.4, 1.0, 1.4, 0.03,
                 lambda x, y: np.full_like(x, FLOOR_Z), rng=rng)
    boxes = []
    for h in np.arange(FLOOR_Z, FLOOR_Z + 0.30, 0.03):
        boxes.append(_grid(1.0, 1.4, 1.0, 1.4, 0.03, lambda x, y: np.full_like(x, h), rng=rng))
        boxes.append(_grid(-2.0, -1.6, -1.5, -1.1, 0.03, lambda x, y: np.full_like(x, h), rng=rng))
    cloud = np.vstack([floor] + boxes + [box1])

    obs = segment_ground(cloud, G_HAT, ROBOT_Z)
    # Open floor (away from the box footprints, which legitimately fail open) is
    # fully removed. Box cells are at x,y in [1.0,1.4] and [-2.0,-1.6]x[-1.5,-1.1].
    near_box = (
        ((np.abs(obs[:, 0] - 1.2) < 0.6) & (np.abs(obs[:, 1] - 1.2) < 0.6))
        | ((np.abs(obs[:, 0] + 1.8) < 0.6) & (np.abs(obs[:, 1] + 1.3) < 0.6))
    )
    open_floor_left = np.sum((np.abs(obs[:, 2] - FLOOR_Z) < 0.05) & ~near_box)
    assert open_floor_left == 0, f"open floor not removed: {open_floor_left} pts"
    # Box tops survive.
    assert _frac_kept(obs, FLOOR_Z + 0.15, FLOOR_Z + 0.35) > 0.0
    assert obs.shape[0] > 100


def test_ramp_removed_object_on_ramp_kept():
    rng = np.random.default_rng(1)
    # 20 deg ramp: z rises with x. tan(20) ~= 0.364.
    slope = np.tan(np.deg2rad(20.0))
    ramp = _grid(-4, 4, -4, 4, 0.05, lambda x, y: FLOOR_Z + slope * x,
                 jitter=0.005, rng=rng)
    # A box sitting on the ramp at x~2 (so its base z ~= FLOOR_Z + slope*2).
    base = FLOOR_Z + slope * 2.0
    box = []
    for h in np.arange(base, base + 0.30, 0.03):
        box.append(_grid(1.9, 2.2, 0.0, 0.3, 0.03, lambda x, y: np.full_like(x, h), rng=rng))
    cloud = np.vstack([ramp] + box)

    params = GroundParams(slope_tol_deg=30.0)  # 20 deg ramp < tol -> counts as ground
    obs = segment_ground(cloud, G_HAT, ROBOT_Z, params)
    # The ramp surface, away from the box's cell footprint (the 0.40 m ground
    # cells the box occupies fail open), is removed.
    in_box_xy = (obs[:, 0] > 1.5) & (obs[:, 0] < 2.5) & (obs[:, 1] > -0.5) & (obs[:, 1] < 0.6)
    ramp_z = FLOOR_Z + slope * obs[:, 0]
    on_open_ramp = (np.abs(obs[:, 2] - ramp_z) < 0.06) & ~in_box_xy
    assert on_open_ramp.sum() == 0, f"ramp surface not removed: {on_open_ramp.sum()} pts"
    # The box on the ramp survives (points well above the ramp surface at its xy).
    above_ramp_box = in_box_xy & (obs[:, 2] - ramp_z > 0.10)
    assert above_ramp_box.sum() > 0, "object on the ramp was removed"


def test_curb_step_survives():
    rng = np.random.default_rng(2)
    # Lower floor for x<0, raised platform (+0.25 m, >> step_tol and outside the
    # foot-height seed_band) for x>=0: a step the robot must not walk off.
    step_h = 0.25
    def z_fn(x, y):
        return np.where(x < 0.0, FLOOR_Z, FLOOR_Z + step_h)
    cloud = _grid(-4, 4, -4, 4, 0.05, z_fn, jitter=0.004, rng=rng)

    obs = segment_ground(cloud, G_HAT, ROBOT_Z)
    # The raised slab (x>=0) is disconnected from the foot-level seed across the
    # step, so it is NOT removed -> survives as obstacle points.
    raised = obs[(obs[:, 0] >= 0.2) & (np.abs(obs[:, 2] - (FLOOR_Z + step_h)) < 0.05)]
    assert raised.shape[0] > 0, "curb step should survive as an obstacle"
    # The lower floor (x<0) IS the seeded ground -> removed.
    lower = obs[(obs[:, 0] <= -0.2) & (np.abs(obs[:, 2] - FLOOR_Z) < 0.05)]
    assert lower.shape[0] < 0.05 * np.sum(cloud[:, 0] < -0.2), "lower floor not removed"


def test_elevated_slab_not_removed():
    rng = np.random.default_rng(3)
    floor = _flat_floor(rng)
    # A flat shelf top 0.8 m above the floor, not connected to the ground.
    shelf = _grid(0.5, 1.5, 0.5, 1.5, 0.04,
                  lambda x, y: np.full_like(x, FLOOR_Z + 0.8), jitter=0.004, rng=rng)
    cloud = np.vstack([floor, shelf])

    obs = segment_ground(cloud, G_HAT, ROBOT_Z)
    kept_shelf = np.sum(np.abs(obs[:, 2] - (FLOOR_Z + 0.8)) < 0.05)
    assert kept_shelf > 0.5 * shelf.shape[0], "elevated slab wrongly removed"


def test_sparse_passthrough():
    rng = np.random.default_rng(4)
    cloud = rng.uniform(-1, 1, size=(50, 3))  # < min_total
    obs, info = segment_ground(cloud, G_HAT, ROBOT_Z, return_info=True)
    assert info["status"] == "sparse"
    # Pass-through (capped at max_height above foot, which all these satisfy).
    assert obs.shape[0] == 50


def test_empty_cloud_no_crash():
    obs = segment_ground(np.empty((0, 3)), G_HAT, ROBOT_Z)
    assert obs.shape == (0, 3)


def test_gravity_alignment_floor_normal():
    """A flat floor's fitted normal should be within ~2 deg of vertical."""
    rng = np.random.default_rng(5)
    floor = _flat_floor(rng, jitter=0.003)
    # Re-fit a single plane to confirm gravity alignment (sanity, mirrors §2.2).
    c = floor.mean(axis=0)
    _, _, vt = np.linalg.svd(floor - c, full_matrices=False)
    n = vt[2]
    n = n if n[2] > 0 else -n
    angle = np.degrees(np.arccos(np.clip(n[2], -1, 1)))
    assert angle < 2.0, f"floor normal {angle:.2f} deg off vertical"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"\nAll {len(fns)} tests passed.")
