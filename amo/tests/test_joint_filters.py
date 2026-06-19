"""
Unit tests for the AMO joint-smoothing filters (docs/amo_inference_plan.md).

Pure numpy — no robot, no robojudo. Run from the amo/ dir:

    cd Navigation/amo && python -m pytest tests/ -q
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from joint_filters import (  # noqa: E402
    CriticallyDampedFilter,
    EWMAFilter,
    JointSmoother,
    make_filter,
)

DT = 0.02  # 50 Hz


# ── layer-D filters ────────────────────────────────────────────────────────────
def test_ewma_starts_at_seed_and_converges():
    f = EWMAFilter(tau=0.08, dt=DT)
    f.reset(np.zeros(3))
    y0 = f.step(np.ones(3))
    # first step moves toward target but not all the way
    assert np.all(y0 > 0.0) and np.all(y0 < 1.0)
    for _ in range(500):
        y = f.step(np.ones(3))
    assert np.allclose(y, 1.0, atol=1e-3)


def test_ewma_zero_tau_is_passthrough():
    f = EWMAFilter(tau=0.0, dt=DT)
    f.reset(np.zeros(2))
    assert np.allclose(f.step(np.array([3.0, -2.0])), [3.0, -2.0])


def test_critdamp_no_overshoot_and_converges():
    f = CriticallyDampedFilter(wn=12.0, dt=DT)
    f.reset(np.zeros(1))
    target = np.array([1.0])
    ys = [f.step(target)[0] for _ in range(800)]
    # critically damped => monotone, never exceeds the target
    assert max(ys) <= 1.0 + 1e-4, f"overshoot detected: max={max(ys)}"
    assert ys == sorted(ys), "response should be monotonically increasing"
    assert abs(ys[-1] - 1.0) < 1e-3


def test_critdamp_first_command_is_near_seed():
    f = CriticallyDampedFilter(wn=12.0, dt=DT)
    f.reset(np.full(3, 0.5))
    first = f.step(np.full(3, 2.0))
    # a single tick moves only a small fraction of the 1.5 gap toward target
    # (raw filter; the JointSmoother further caps this via the layer-C clamp).
    assert np.all(np.abs(first - 0.5) < 0.15)


def test_make_filter_kinds():
    assert make_filter("none", DT) is None
    assert isinstance(make_filter("ewma", DT), EWMAFilter)
    assert isinstance(make_filter("critdamp", DT), CriticallyDampedFilter)
    with pytest.raises(ValueError):
        make_filter("bogus", DT)


# ── JointSmoother (full A→D→C stack) ────────────────────────────────────────────
def test_smoother_first_command_equals_measured():
    measured = np.array([0.1, -0.3, 0.7], dtype=np.float32)
    target = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    s = JointSmoother(DT, blend_s=5.0, clamp_delta=0.05, filter_kind="critdamp")
    s.reset(measured)
    cmd = s.step(target)
    # anti-snap guarantee: the first commanded delta is ~0
    assert np.max(np.abs(cmd - measured)) < 1e-2


def test_smoother_per_tick_delta_clamped():
    measured = np.zeros(4, dtype=np.float32)
    target = np.full(4, 10.0, dtype=np.float32)  # absurd reference
    s = JointSmoother(DT, blend_s=0.0, clamp_delta=0.05, filter_kind="none")
    s.reset(measured)
    prev = measured.copy()
    for _ in range(50):
        cmd = s.step(target)
        assert np.all(np.abs(cmd - prev) <= 0.05 + 1e-6)
        prev = cmd


def test_smoother_converges_to_reference():
    measured = np.zeros(3, dtype=np.float32)
    target = np.array([0.4, -0.2, 0.1], dtype=np.float32)
    s = JointSmoother(DT, blend_s=2.0, clamp_delta=0.05, filter_kind="critdamp", filter_wn=15.0)
    s.reset(measured)
    cmd = measured
    for _ in range(2000):
        cmd = s.step(target)
    assert np.allclose(cmd, target, atol=1e-2)


def test_smoother_gain_ramp_soft_to_full():
    s = JointSmoother(DT, gain_ramp_s=2.0, kp_scale_start=0.15, kd_scale_start=0.50)
    s.reset(np.zeros(2))
    skp0, skd0 = s.gain_scales()
    assert skp0 == pytest.approx(0.15, abs=1e-6)
    assert skd0 == pytest.approx(0.50, abs=1e-6)
    for _ in range(200):  # 4 s, past the 2 s ramp
        s.step(np.zeros(2))
    skp, skd = s.gain_scales()
    assert skp == pytest.approx(1.0, abs=1e-6)
    assert skd == pytest.approx(1.0, abs=1e-6)


def test_smoother_releases_to_passthrough_after_reaching():
    # While reaching the reference the clamp limits per-tick motion; once released
    # the raw policy target passes straight through (full reactivity for recovery).
    measured = np.zeros(2, dtype=np.float32)
    s = JointSmoother(
        DT, blend_s=0.0, clamp_delta=0.05, filter_kind="none",
        release_s=1.0, release_ramp_s=0.5,
    )
    s.reset(measured)
    # before release: a far target is rate-clamped
    early = s.step(np.full(2, 10.0, dtype=np.float32))
    assert np.all(np.abs(early - measured) <= 0.05 + 1e-6)
    # run past release_s + release_ramp_s
    for _ in range(int((1.0 + 0.5) / DT) + 5):
        cmd = s.step(np.full(2, 10.0, dtype=np.float32))
    assert s.release_factor == pytest.approx(1.0)
    assert not s.filtering
    # a fresh large step now lands on target in a single tick (no clamp)
    cmd = s.step(np.array([3.0, -2.0], dtype=np.float32))
    assert np.allclose(cmd, [3.0, -2.0], atol=1e-5)


def test_smoother_release_disabled_keeps_clamp_forever():
    measured = np.zeros(3, dtype=np.float32)
    target = np.full(3, 10.0, dtype=np.float32)
    s = JointSmoother(DT, blend_s=0.0, clamp_delta=0.05, filter_kind="none", release_s=0.0)
    s.reset(measured)
    prev = measured.copy()
    for _ in range(500):  # 10 s — far past any startup window
        cmd = s.step(target)
        assert np.all(np.abs(cmd - prev) <= 0.05 + 1e-6)
        prev = cmd
    assert s.filtering


def test_smoother_release_transition_is_continuous():
    # No command jump as filtering fades out around the release point.
    measured = np.zeros(4, dtype=np.float32)
    target = np.full(4, 0.6, dtype=np.float32)
    s = JointSmoother(
        DT, blend_s=2.0, clamp_delta=0.05, filter_kind="critdamp", filter_wn=15.0,
        release_s=2.0, release_ramp_s=1.0,
    )
    s.reset(measured)
    prev = s.step(target)
    max_jump = 0.0
    for _ in range(300):  # 6 s, spanning the release ramp
        cmd = s.step(target)
        max_jump = max(max_jump, float(np.max(np.abs(cmd - prev))))
        prev = cmd
    # joints have settled on the reference by release, so the hand-off is tiny
    assert max_jump <= 0.05 + 1e-3


def test_smoother_commanded_velocity_bounded():
    measured = np.zeros(2, dtype=np.float32)
    target = np.array([1.0, -1.0], dtype=np.float32)
    s = JointSmoother(DT, blend_s=5.0, clamp_delta=0.05, filter_kind="critdamp")
    s.reset(measured)
    prev = measured.copy()
    vmax = 0.0
    for _ in range(1000):
        cmd = s.step(target)
        vmax = max(vmax, float(np.max(np.abs(cmd - prev)) / DT))
        prev = cmd
    # clamp_delta 0.05 rad/tick at 50 Hz => <= 2.5 rad/s
    assert vmax <= 2.5 + 1e-6
