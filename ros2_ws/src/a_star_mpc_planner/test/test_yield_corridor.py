"""Dynamic-obstacle YIELD corridor test + clustering-rewrite equivalence.

_dynamic_cluster_in_corridor is the pure predicate behind the MPC's YIELD
safety state (stop while something moves across the robot's path); these tests
pin its geometry. _cluster_points was rewritten from a per-point Python
union-find to scipy.ndimage.label; the partition must be unchanged.
"""

import math

import numpy as np

from a_star_mpc_planner.mpc_node import MPCNode

_corridor = MPCNode._dynamic_cluster_in_corridor
ROBOT = np.array([0.0, 0.0])
LEN, HALF, LOOK = 2.5, 0.7, 1.5


def test_no_clusters_is_clear():
    assert not _corridor(ROBOT, 0.0, None, None, LEN, HALF, LOOK)
    assert not _corridor(ROBOT, 0.0, np.empty((0, 2)), np.empty((0, 2)),
                         LEN, HALF, LOOK)


def test_static_cluster_ahead_does_not_yield():
    # Static clusters carry exactly-zero tracked velocity — A* / the barrier
    # handle them; yielding to a wall would freeze the robot forever.
    c = np.array([[1.0, 0.0]])
    v = np.array([[0.0, 0.0]])
    assert not _corridor(ROBOT, 0.0, c, v, LEN, HALF, LOOK)


def test_mover_in_corridor_yields():
    c = np.array([[1.5, 0.2]])
    v = np.array([[0.3, 0.0]])
    assert _corridor(ROBOT, 0.0, c, v, LEN, HALF, LOOK)


def test_mover_behind_robot_is_ignored():
    c = np.array([[-1.5, 0.0]])
    v = np.array([[-0.5, 0.0]])   # walking away behind us
    assert not _corridor(ROBOT, 0.0, c, v, LEN, HALF, LOOK)


def test_mover_beyond_corridor_length_is_ignored():
    c = np.array([[4.0, 0.0]])
    v = np.array([[0.1, 0.0]])    # slow drift, stays beyond 2.5 m within lookahead
    assert not _corridor(ROBOT, 0.0, c, v, LEN, HALF, LOOK)


def test_crossing_mover_triggers_via_prediction():
    # Person 1.5 m to the side, walking 1.2 m/s toward the path: not in the
    # corridor NOW, but inside it within the 1.5 s lookahead.
    c = np.array([[1.5, 1.5]])
    v = np.array([[0.0, -1.2]])
    assert _corridor(ROBOT, 0.0, c, v, LEN, HALF, LOOK)
    # Same person walking AWAY from the path: never enters.
    assert not _corridor(ROBOT, 0.0, c, np.array([[0.0, 1.2]]), LEN, HALF, LOOK)


def test_corridor_rotates_with_robot_yaw():
    yaw = math.pi / 2.0            # facing +y
    c = np.array([[0.0, 1.5]])     # ahead in body frame
    v = np.array([[0.0, 0.3]])
    assert _corridor(ROBOT, yaw, c, v, LEN, HALF, LOOK)
    c_side = np.array([[1.5, 0.0]])  # now lateral, outside halfwidth
    assert not _corridor(ROBOT, yaw, c_side, v, LEN, HALF, LOOK)


# ── clustering rewrite equivalence ──────────────────────────────────────────

def _reference_cluster(obs_2d: np.ndarray, cell: float):
    """Original per-point union-find implementation (oracle)."""
    n = len(obs_2d)
    cells = np.floor(obs_2d / cell).astype(np.int64)
    cell_of_pt, occupied = {}, {}
    for i in range(n):
        key = (int(cells[i, 0]), int(cells[i, 1]))
        cell_of_pt[i] = key
        occupied.setdefault(key, []).append(i)
    parent = {k: k for k in occupied}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for (cx, cy) in occupied:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if (dx or dy) and (cx + dx, cy + dy) in occupied:
                    ra, rb = find((cx, cy)), find((cx + dx, cy + dy))
                    if ra != rb:
                        parent[ra] = rb
    root_to_label, labels = {}, np.empty(n, dtype=np.int64)
    for i in range(n):
        root = find(cell_of_pt[i])
        labels[i] = root_to_label.setdefault(root, len(root_to_label))
    return labels, len(root_to_label)


def _partition(labels):
    groups = {}
    for i, lab in enumerate(labels):
        groups.setdefault(int(lab), set()).add(i)
    return {frozenset(g) for g in groups.values()}


def test_cluster_points_matches_union_find():
    rng = np.random.default_rng(5)
    wall = np.column_stack([rng.uniform(2.0, 2.3, 800), rng.uniform(-4, 4, 800)])
    blob1 = rng.normal([1.0, 1.0], 0.1, (200, 2))
    blob2 = rng.normal([-2.0, -0.5], 0.15, (200, 2))
    lone = np.array([[5.0, 5.0]])
    obs = np.vstack([wall, blob1, blob2, lone])
    l_new, n_new = MPCNode._cluster_points(obs, 0.30)
    l_ref, n_ref = _reference_cluster(obs, 0.30)
    assert n_new == n_ref
    assert _partition(l_new) == _partition(l_ref)


def test_cluster_points_empty():
    labels, n = MPCNode._cluster_points(np.empty((0, 2)), 0.30)
    assert n == 0 and labels.shape == (0,)
