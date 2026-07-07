"""Offline regression test for the persistent GlobalCostmap.

Covers the world-memory upgrades without ROS or a robot — pure numpy/scipy:
  T1  growable map (grow preserves data; roll-at-cap re-centres)
  T2  dead-end / stuck penalty (soft cost that decays)
  T3  drift re-anchor (shift relabels accumulated data)
  2.5D height grading (tall structure lethal; low return soft; safe no-height fallback)

Run:  python3 test_global_costmap.py     (needs numpy + scipy)
"""
import importlib.util
import os

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_GC = os.path.join(_HERE, "..", "a_star_mpc_planner", "global_costmap.py")
_spec = importlib.util.spec_from_file_location("global_costmap", _GC)
_gm = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_gm)
GlobalCostmap = _gm.GlobalCostmap


def near(pts, target, tol):
    return pts is not None and np.any(np.linalg.norm(np.asarray(pts) - target, axis=1) < tol)


def main():
    # ── T1a: growth preserves accumulated obstacles at correct world coords ──
    gc = GlobalCostmap(reso=0.2, half_width=5.0, max_half_width=50.0, hit_threshold=2.0, decay=1.0)
    gc.update([], [0.0, 0.0])
    base_cells = gc.cells
    obs = np.array([[3.0, 3.0]])
    gc.update(obs, [0.0, 0.0]); gc.update(obs, [0.0, 0.0])
    assert near(gc.confirmed_hit_points(), [3.0, 3.0], 0.3), "T1a obstacle not confirmed at (3,3)"
    gc.update([], [40.0, 0.0])
    assert gc.cells > base_cells, "T1a grid did not grow"
    assert near(gc.confirmed_hit_points(), [3.0, 3.0], 0.3), "T1a obstacle LOST after growth"
    assert gc.world_to_index(40.0, 0.0)[0] is not None, "T1a robot not inside grown grid"
    print("T1a grow-preserves-data ...... OK  (cells %d -> %d)" % (base_cells, gc.cells))

    # ── T1b: at the cap, ROLL to re-centre on the robot ──
    gc4 = GlobalCostmap(reso=0.5, half_width=3.0, max_half_width=5.0, hit_threshold=2.0, decay=1.0)
    gc4.update([], [0.0, 0.0])
    maxc = int(round(2 * 5.0 / 0.5))
    gc4.update([], [30.0, 0.0])
    assert gc4.cells == maxc, "T1b grid should cap at %d, got %d" % (maxc, gc4.cells)
    assert gc4.world_to_index(30.0, 0.0)[0] is not None, "T1b robot not inside capped grid"
    assert gc4.world_to_index(0.0, 0.0)[0] is None, "T1b far origin should have rolled off"
    print("T1b roll-at-cap .............. OK  (capped at %d cells)" % gc4.cells)

    # ── T3: shift() re-anchors accumulated data by a world delta ──
    gc2 = GlobalCostmap(reso=0.2, half_width=5.0, hit_threshold=2.0, decay=1.0)
    o = np.array([[1.0, 1.0]]); gc2.update(o, [0, 0]); gc2.update(o, [0, 0])
    before = gc2.confirmed_hit_points()[0].copy()
    gc2.shift(2.0, -1.0)
    after = gc2.confirmed_hit_points()[0]
    assert np.linalg.norm(after - (before + [2.0, -1.0])) < 1e-6, "T3 shift did not relabel data"
    print("T3 drift-shift ............... OK  (%.1f,%.1f -> %.1f,%.1f)" %
          (before[0], before[1], after[0], after[1]))

    # ── T2: penalty raises cost in the build, and decays over updates ──
    gc3 = GlobalCostmap(reso=0.2, half_width=5.0, decay=1.0, penalty_decay=0.5)
    gc3.update([], [0.0, 0.0])
    gc3.stamp_penalty([2.0, 2.0], 0.5, 2.0)
    gc3.build()
    ix, iy = gc3.world_to_index(2.0, 2.0)
    assert gc3.gmap[ix, iy] > 0.1, "T2 penalty not reflected in gmap"
    p0 = float(gc3._penalty[ix, iy])
    gc3.update([], [0.0, 0.0])
    assert gc3._penalty[ix, iy] < p0, "T2 penalty did not decay"
    assert gc3.gmap[ix, iy] < 1.0, "T2 penalty must stay soft (never lethal)"
    print("T2 dead-end-penalty .......... OK  (cost %.2f, decays %.2f -> %.2f)" %
          (float(gc3.gmap[ix, iy]), p0, float(gc3._penalty[ix, iy])))

    # ── incremental growth keeps a fixed off-path obstacle put ──
    gc5 = GlobalCostmap(reso=0.2, half_width=4.0, max_half_width=60.0, hit_threshold=2.0, decay=1.0)
    gc5.update([], [0, 0]); w = np.array([[2.0, 3.0]]); gc5.update(w, [0, 0]); gc5.update(w, [0, 0])
    for x in np.linspace(0, 45, 90):
        gc5.update([], [float(x), 0.0])
    assert near(gc5.confirmed_hit_points(), [2.0, 3.0], 0.3), "obstacle drifted during incremental growth"
    print("incremental-growth-stable .... OK")

    # ── driving THROUGH a cell clears it (breadcrumb overrides occupancy) ──
    gc6 = GlobalCostmap(reso=0.2, half_width=4.0, hit_threshold=2.0, decay=1.0)
    gc6.update([], [0, 0]); t = np.array([[1.0, 0.0]]); gc6.update(t, [0, 0]); gc6.update(t, [0, 0])
    assert near(gc6.confirmed_hit_points(), [1.0, 0.0], 0.3), "pre: should be confirmed"
    gc6.update([], [1.0, 0.0])
    assert not near(gc6.confirmed_hit_points(), [1.0, 0.0], 0.3), "breadcrumb should clear a driven-through cell"
    print("breadcrumb-clears-path ....... OK")

    # ── 2.5D: tall structure lethal, measured LOW return soft ──
    gc7 = GlobalCostmap(reso=0.2, half_width=6.0, hit_threshold=2.0, decay=1.0,
                        use_height_cost=True, foot_offset=0.7, low_height=0.20, low_cost=0.45)
    gc7.update([], [0, 0], 0.7)                       # foot z = 0.0
    tall = np.array([[2.0, 0.0, 1.5]])                # 1.5 m -> lethal
    gc7.update(tall, [0, 0], 0.7); gc7.update(tall, [0, 0], 0.7)
    low = np.array([[4.0, 0.0, 0.1]])                 # 0.1 m -> soft
    gc7.update(low, [0, 0], 0.7); gc7.update(low, [0, 0], 0.7)
    gc7.build()
    ixt, iyt = gc7.world_to_index(2.0, 0.0)
    ixl, iyl = gc7.world_to_index(4.0, 0.0)
    assert gc7.gmap[ixt, iyt] >= 1.0, "2.5D tall structure must be lethal"
    assert 0.0 < gc7.gmap[ixl, iyl] < 1.0, "2.5D low return must be soft, not lethal"
    print("2.5D height grading .......... OK  (tall=%.2f lethal, low=%.2f soft)" %
          (float(gc7.gmap[ixt, iyt]), float(gc7.gmap[ixl, iyl])))

    # ── safe fallback: no robot_z → occupied stays lethal ──
    gc8 = GlobalCostmap(reso=0.2, half_width=6.0, hit_threshold=2.0, decay=1.0, use_height_cost=True)
    gc8.update([], [0, 0]); gc8.update(np.array([[4, 0, 0.1]]), [0, 0]); gc8.update(np.array([[4, 0, 0.1]]), [0, 0])
    gc8.build()
    ix, iy = gc8.world_to_index(4.0, 0.0)
    assert gc8.gmap[ix, iy] >= 1.0, "no height data must fall back to lethal (safe)"
    print("2.5D fallback (no z) ......... OK  (lethal without height data)")

    print("\nALL GLOBAL COSTMAP TESTS PASSED")


if __name__ == "__main__":
    main()
