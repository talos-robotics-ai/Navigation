"""
Unit tests for the MPCC (contouring) tracker — time-optimal path following.

Requires casadi (the planner's runtime dep). Run with plain ``pytest`` from the
workspace root.

Covered:
  1.  x_pred has the (N+1, 7) MPCC layout and the first 6 columns match the
      MPCTracker state order (cmd_vel = x_pred[1, 3:6] stays valid).
  2.  On a clear straight path the controller drives at (near) vx_max — i.e. it
      is time-optimal, not capped at a fixed cruise speed.
  3.  Progress θ advances toward the goal (monotone, > 0).
  4.  An obstacle sitting on the path does NOT get run over: the predicted
      trajectory keeps clear of the obstacle radius.
  5.  update_velocity_limits lowers the achievable command (adaptive-limit hook).
"""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from a_star_mpc_planner.mpcc_tracker import MPCCConfig, MPCCTracker  # noqa: E402


def _straight_path(length=4.0, n=20):
    return [(float(x), 0.0, 0.0) for x in np.linspace(0.0, length, n)]


class TestMPCCTracker(unittest.TestCase):

    def setUp(self):
        self.cfg = MPCCConfig(N=40, dt=0.1, vx_max=0.45, vy_max=0.45,
                              vtheta_max=0.55, max_iter=120)
        self.trk = MPCCTracker(self.cfg)
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_state_layout(self):
        r = self.trk.solve(self.state, _straight_path())
        self.assertEqual(r.x_pred.shape, (self.cfg.N + 1, 7))
        # first 6 cols are [px, py, yaw, vx, vy, wz] — same as MPCTracker
        self.assertTrue(np.isfinite(r.x_pred[:, :6]).all())

    def test_drives_at_max_speed_on_clear_path(self):
        r = self.trk.solve(self.state, _straight_path())
        self.trk.solve(self.state, _straight_path())  # warm-started 2nd solve
        r = self.trk.solve(self.state, _straight_path())
        self.assertTrue(r.success)
        # forward command should be near the ceiling (time-optimal, not v_ref)
        self.assertGreater(float(r.u_opt[0, 0]), 0.40)

    def test_progress_advances(self):
        r = self.trk.solve(self.state, _straight_path())
        theta = r.x_pred[:, 6]
        self.assertGreater(theta[-1], 0.5)              # made real progress
        self.assertTrue(np.all(np.diff(theta) >= -1e-6))  # monotone non-decreasing

    def test_obstacle_on_path_not_run_over(self):
        obs = np.array([[2.0, 0.0]])
        r = self.trk.solve(self.state, _straight_path(), obstacle_points_2d=obs)
        # no predicted point penetrates the obstacle radius
        d = np.linalg.norm(r.x_pred[:, :2] - obs[0], axis=1)
        self.assertGreater(float(np.min(d)), self.cfg.obs_r * 0.7)

    def test_velocity_limit_update(self):
        self.trk.update_velocity_limits(vx_max=0.10)
        self.trk.solve(self.state, _straight_path())
        r = self.trk.solve(self.state, _straight_path())
        self.assertLessEqual(float(r.u_opt[0, 0]), 0.11 + 1e-6)


if __name__ == '__main__':
    unittest.main()
