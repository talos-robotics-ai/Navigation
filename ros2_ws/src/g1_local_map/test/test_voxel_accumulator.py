"""VoxelAccumulator (packed-key vectorised) — behavioural contract tests.

The accumulator was rewritten from a dict-of-tuples with a Python loop per
voxel to bit-packed int64 keys updated in bulk (12x faster per scan on the
Orin Nano's hot path). These tests pin the behaviour to the original
implementation: same voxel set after arbitrary update/prune sequences, same
decay eviction, same rolling-window eviction.
"""

import numpy as np
import pytest

from g1_local_map.local_voxel_map_node import VoxelAccumulator


class ReferenceAccumulator:
    """The original dict-of-tuples implementation, kept as the oracle."""

    def __init__(self, voxel_size: float, persistence_s: float):
        self.voxel_size = float(voxel_size)
        self.persistence_s = float(persistence_s)
        self._last_seen = {}

    def update(self, xyz, now_s):
        if xyz.shape[0]:
            idx = np.floor(xyz / self.voxel_size).astype(np.int64)
            for k in map(tuple, np.unique(idx, axis=0).tolist()):
                self._last_seen[k] = now_s

    def prune(self, now_s, center_xy, half_width):
        if not self._last_seen:
            return
        cx, cy = center_xy
        reach = (half_width + self.voxel_size) / self.voxel_size
        ix0, iy0 = cx / self.voxel_size, cy / self.voxel_size
        stale = now_s - self.persistence_s
        dead = [k for k, t in self._last_seen.items()
                if t < stale or abs(k[0] - ix0) > reach or abs(k[1] - iy0) > reach]
        for k in dead:
            del self._last_seen[k]

    def centers(self):
        if not self._last_seen:
            return np.empty((0, 3), dtype=np.float32)
        keys = np.asarray(list(self._last_seen.keys()), dtype=np.float64)
        return ((keys + 0.5) * self.voxel_size).astype(np.float32)


def _center_set(arr: np.ndarray) -> set:
    return {tuple(r) for r in np.round(arr, 5).tolist()}


def test_matches_reference_over_walk():
    rng = np.random.default_rng(11)
    new = VoxelAccumulator(0.10, 3.0)
    ref = ReferenceAccumulator(0.10, 3.0)
    for i in range(25):
        robot = np.array([0.4 * i, 0.15 * i])
        scan = np.column_stack([
            robot[0] + rng.uniform(-9, 9, 3000),
            robot[1] + rng.uniform(-9, 9, 3000),
            rng.uniform(-1.5, 1.5, 3000),
        ])
        now = 0.1 * i
        new.update(scan, now)
        ref.update(scan, now)
        new.prune(now, tuple(robot), 8.0)
        ref.prune(now, tuple(robot), 8.0)
        assert _center_set(new.centers()) == _center_set(ref.centers())


def test_temporal_decay_evicts_stale_voxels():
    acc = VoxelAccumulator(0.10, persistence_s=1.0)
    acc.update(np.array([[0.05, 0.05, 0.05]]), now_s=0.0)
    acc.update(np.array([[1.05, 1.05, 0.05]]), now_s=0.9)
    acc.prune(now_s=1.5, center_xy=(0.0, 0.0), half_width=8.0)
    centers = acc.centers()
    assert centers.shape == (1, 3)
    np.testing.assert_allclose(centers[0], [1.05, 1.05, 0.05], atol=1e-6)


def test_rehit_refreshes_timestamp():
    acc = VoxelAccumulator(0.10, persistence_s=1.0)
    acc.update(np.array([[0.05, 0.05, 0.05]]), now_s=0.0)
    acc.update(np.array([[0.06, 0.04, 0.09]]), now_s=0.9)  # same voxel
    acc.prune(now_s=1.5, center_xy=(0.0, 0.0), half_width=8.0)
    assert acc.centers().shape == (1, 3)


def test_window_prune_forgets_passed_obstacles():
    acc = VoxelAccumulator(0.10, persistence_s=100.0)
    acc.update(np.array([[0.05, 0.05, 0.0], [20.05, 0.05, 0.0]]), now_s=0.0)
    acc.prune(now_s=0.1, center_xy=(20.0, 0.0), half_width=8.0)
    centers = acc.centers()
    assert centers.shape == (1, 3)
    assert centers[0, 0] == pytest.approx(20.05, abs=1e-5)


def test_negative_coordinates_pack_correctly():
    acc = VoxelAccumulator(0.10, persistence_s=10.0)
    pts = np.array([[-5.23, -7.81, -1.31], [5.23, 7.81, 1.31]])
    acc.update(pts, now_s=0.0)
    got = _center_set(acc.centers())
    ref = ReferenceAccumulator(0.10, 10.0)
    ref.update(pts, now_s=0.0)
    assert got == _center_set(ref.centers())


def test_empty_update_and_prune_are_noops():
    acc = VoxelAccumulator(0.10, 3.0)
    acc.update(np.empty((0, 3)), now_s=0.0)
    acc.prune(0.0, (0.0, 0.0), 8.0)
    assert acc.centers().shape == (0, 3)
