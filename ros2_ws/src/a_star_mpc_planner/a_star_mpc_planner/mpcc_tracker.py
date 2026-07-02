"""
Model Predictive Contouring Control (MPCC) tracker — time-optimal path following.

Why MPCC instead of the reference-tracking MPCTracker
-----------------------------------------------------
The tracking MPC samples a reference along the A* path at a FIXED cruise speed
(v_ref) and tries to be at that moving point each step. It therefore drives at
v_ref regardless of how much speed budget is left — it is NOT time-optimal.

MPCC reframes the problem: the path is parameterised by arc length θ, a virtual
"progress" the optimiser advances as fast as the dynamics/limits allow. The cost
- penalises CONTOURING error  e_c (lateral distance from the path)  → stay on path
- penalises LAG error         e_l (longitudinal mismatch vs θ)      → θ tracks the robot
- REWARDS progress            (−q·θ_N, −q·v_θ)                      → go as fast as possible
- keeps the obstacle barrier  (sigmoid + quadratic)                → safety
subject to velocity limits. Maximising progress under those limits ⇒ the robot
reaches the goal in minimum time while staying on the (collision-free) A* path
and vetoing any point that gets too close.

The local path is fit with low-degree polynomials in normalised arc length so
p(θ) and its tangent are smooth/differentiable inside the NLP (classic MPCC),
and the coefficients are NLP PARAMETERS refreshed every solve — so the single
cached NLP handles an arbitrary, replanned path without a rebuild.

State    x = [px, py, yaw, vx, vy, wz, θ]            NX = 7
Control  u = [vx_cmd, vy_cmd, wz_cmd, v_θ]           NU = 4

The first six state columns match MPCTracker exactly, so mpc_node consumes the
result (cmd_vel = x_pred[1, 3:6], predicted path = x_pred[:, :3]) unchanged.

author: Lorenzo Ortolani
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import casadi as ca


@dataclass
class MPCCConfig:
    """Tunable MPCC parameters."""

    # Horizon
    N: int = 50
    dt: float = 0.1

    # Actuator lag [s]
    tau_v: float = 0.15
    tau_w: float = 0.12

    # Command limits
    vx_max: float = 0.45
    vy_max: float = 0.45      # holonomic / crab-walk
    omega_max: float = 0.40
    vtheta_max: float = 0.55  # max progress speed (≈ top translational speed)

    # Contouring / lag / progress weights
    w_contour: float = 300.0     # lateral path-deviation penalty
    w_lag: float = 60.0          # longitudinal θ-mismatch penalty
    q_progress: float = 2.0      # per-step reward on v_θ (push speed)
    q_progress_terminal: float = 60.0   # terminal reward on θ_N (push to goal)
    Q_yaw_align: float = 40.0    # soft "face the path tangent" (low → crab freely)

    # Control effort / smoothness
    R_vx: float = 1.0
    R_vy: float = 1.0
    R_omega: float = 1.0
    R_vtheta: float = 0.1
    R_jerk: float = 0.2

    # Obstacle barrier (same shape as MPCTracker)
    W_obs_sigmoid: float = 80.0
    obs_alpha: float = 12.0
    obs_r: float = 0.55
    max_obs_constraints: int = 12
    obs_check_radius: float = 3.0

    # Local path polynomial fit
    poly_degree: int = 3

    # IPOPT
    max_iter: int = 80
    warm_start: bool = True
    print_level: int = 0


@dataclass
class MPCCResult:
    success: bool
    x_pred: np.ndarray          # (N+1, 7) [px, py, yaw, vx, vy, wz, θ]
    u_opt: np.ndarray           # (N,   4) [vx_cmd, vy_cmd, wz_cmd, v_θ]
    cost: float
    solve_time_ms: float
    progress_m: float = 0.0     # θ_N − θ_0 (planned arc-length advance)
    security_mode: bool = False


class MPCCTracker:
    NX = 7
    NU = 4
    _OBS_SENTINEL = 1e3
    _MAX_CONSEC_FAILURES = 3

    def __init__(self, config: Optional[MPCCConfig] = None):
        self.cfg = config or MPCCConfig()
        self._D = self.cfg.poly_degree + 1   # number of polynomial coefficients

        # Warm start
        self._prev_u: Optional[np.ndarray] = None
        self._prev_x: Optional[np.ndarray] = None

        # Cached parametric NLP
        self._nlp_built = False
        self._opti = None
        self._X = None
        self._U = None
        self._p_x0 = None
        self._p_obs = None
        self._p_cx = None
        self._p_cy = None
        self._p_Ln = None
        self._p_smax = None
        self._p_vx_max = None
        self._p_vy_max = None
        self._p_omega_max = None
        self._p_vtheta_max = None
        self._p_goal_yaw = None       # terminal goal-heading target (parameter)
        self._p_goal_yaw_w = None     # its weight (0 = off this solve)

        # Runtime-adaptive limits (mpc_node lowers these on failure bursts)
        self._vx_max_eff = self.cfg.vx_max
        self._vy_max_eff = self.cfg.vy_max
        self._omega_max_eff = self.cfg.omega_max
        self._vtheta_max_eff = self.cfg.vtheta_max

        self._consecutive_failures = 0
        self._last_valid_x0: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    # API compatibility with MPCTracker (used by mpc_node)
    # ------------------------------------------------------------------

    def update_grid(self, grid_map) -> None:
        pass

    def update_velocity_limits(self, vx_max=None, vy_max=None, omega_max=None) -> None:
        if vx_max is not None:
            self._vx_max_eff = float(vx_max)
            # Keep progress speed in step with the forward-speed ceiling.
            self._vtheta_max_eff = max(float(vx_max), 0.05)
        if vy_max is not None:
            self._vy_max_eff = float(vy_max)
        if omega_max is not None:
            self._omega_max_eff = float(omega_max)

    # ------------------------------------------------------------------
    # Obstacle selection (identical policy to MPCTracker)
    # ------------------------------------------------------------------

    def _select_obs_points(self, pts_2d: np.ndarray, robot_xy: np.ndarray) -> np.ndarray:
        """Pick the obstacle points the barrier watches, with ANGULAR coverage.

        Issue #3: the old policy took the ``n_target`` globally-nearest points.
        On an extended obstacle (a wall, a row of clutter) those all bunch on the
        single closest segment, so the barrier only "saw" one patch and the robot
        stayed blind to obstacles to its side/other-front. Here we instead keep
        the nearest point in each of ``n_target`` angular sectors around the
        robot, so coverage is spread around it; any leftover slots are filled with
        the next-nearest unused points. Result: the same constraint budget now
        represents obstacles on every side rather than one clump.
        """
        n_target = self.cfg.max_obs_constraints
        if len(pts_2d) > 0:
            pts_2d = pts_2d[np.isfinite(pts_2d).all(axis=1)]
        if len(pts_2d) == 0:
            return np.full((n_target, 2), self._OBS_SENTINEL)

        rel = pts_2d - robot_xy
        dists = np.hypot(rel[:, 0], rel[:, 1])
        mask = dists < self.cfg.obs_check_radius
        if not np.any(mask):
            return np.full((n_target, 2), self._OBS_SENTINEL)

        close = pts_2d[mask]
        cdist = dists[mask]
        crel = rel[mask]

        # Nearest point per angular sector (spread coverage around the robot).
        ang = np.arctan2(crel[:, 1], crel[:, 0])            # [-pi, pi)
        sector = np.floor((ang + np.pi) / (2.0 * np.pi) * n_target).astype(int)
        sector = np.clip(sector, 0, n_target - 1)
        chosen = []
        chosen_set = set()
        order = np.argsort(cdist)
        seen_sectors = set()
        for i in order:                                     # nearest-first
            s = int(sector[i])
            if s not in seen_sectors:
                seen_sectors.add(s)
                chosen.append(i)
                chosen_set.add(int(i))
                if len(chosen) >= n_target:
                    break
        # Fill any remaining slots with the next-nearest not-yet-chosen points.
        if len(chosen) < n_target:
            for i in order:
                if int(i) not in chosen_set:
                    chosen.append(i)
                    chosen_set.add(int(i))
                    if len(chosen) >= n_target:
                        break

        selected = close[np.asarray(chosen, dtype=int)]
        n_found = len(selected)
        if n_found < n_target:
            sentinel = np.full((n_target - n_found, 2), self._OBS_SENTINEL)
            selected = np.vstack([selected, sentinel])
        return selected

    # ------------------------------------------------------------------
    # Local path → normalised-arc-length polynomial fit
    # ------------------------------------------------------------------

    def _fit_path(self, robot_xy: np.ndarray, path_world: list):
        """Return (cx, cy, Ln, s_max) for the path AHEAD of the robot.

        Fits px(τ), py(τ) with degree poly_degree polynomials where τ = s / Ln is
        the normalised arc length (Ln = total remaining length) — normalisation
        keeps the Vandermonde fit well-conditioned. s_max caps how far θ may
        advance this horizon (a bit beyond the max reachable distance).
        """
        D = self._D
        path = np.asarray(path_world, dtype=float)[:, :2]
        # Start at the closest point so θ measures progress from "here".
        i0 = int(np.argmin(np.linalg.norm(path - robot_xy, axis=1)))
        seg = path[i0:]
        if len(seg) < 2:
            seg = np.vstack([robot_xy, path[-1]]) if len(path) else None
        if seg is None or len(seg) < 2:
            return None

        diffs = np.diff(seg, axis=0)
        seglen = np.hypot(diffs[:, 0], diffs[:, 1])
        s = np.concatenate([[0.0], np.cumsum(seglen)])
        L = float(s[-1])
        if L < 1e-6:
            return None

        deg = min(self.cfg.poly_degree, len(seg) - 1)
        tau = s / L
        try:
            # polyfit returns HIGHEST-power-first; flip to lowest-first and pad.
            cx_hi = np.polyfit(tau, seg[:, 0], deg)
            cy_hi = np.polyfit(tau, seg[:, 1], deg)
        except Exception:
            return None
        cx = np.zeros(D)
        cy = np.zeros(D)
        cx[: deg + 1] = cx_hi[::-1]
        cy[: deg + 1] = cy_hi[::-1]

        # Allow θ to reach the path end, capped to what is reachable this horizon
        # (plus 30 % headroom so the progress reward is never artificially clipped).
        reach = self._vtheta_max_eff * self.cfg.N * self.cfg.dt * 1.3
        s_max = min(L, reach)
        return cx, cy, float(L), float(s_max)

    # ------------------------------------------------------------------
    # Parametric NLP — built once
    # ------------------------------------------------------------------

    def _poly(self, coeffs, tau):
        """Σ coeffs[j] * tau**j  (coeffs lowest-power-first)."""
        val = coeffs[0]
        tpow = 1.0
        for j in range(1, self._D):
            tpow = tpow * tau
            val = val + coeffs[j] * tpow
        return val

    def _dpoly(self, coeffs, tau):
        """d/dtau Σ coeffs[j] tau**j = Σ j coeffs[j] tau**(j-1)."""
        val = 0.0
        tpow = 1.0  # tau**0 for j=1
        for j in range(1, self._D):
            val = val + j * coeffs[j] * tpow
            tpow = tpow * tau
        return val

    def _build_nlp(self) -> None:
        cfg = self.cfg
        N, dt = cfg.N, cfg.dt
        NX, NU, D = self.NX, self.NU, self._D
        n_obs = cfg.max_obs_constraints

        lag_v = float(1.0 - np.exp(-dt / max(cfg.tau_v, 1e-6)))
        lag_w = float(1.0 - np.exp(-dt / max(cfg.tau_w, 1e-6)))

        opti = ca.Opti()
        X = opti.variable(NX, N + 1)
        U = opti.variable(NU, N)
        p_x0 = opti.parameter(NX)
        p_obs = opti.parameter(2, n_obs)
        p_cx = opti.parameter(D)
        p_cy = opti.parameter(D)
        p_Ln = opti.parameter()
        p_smax = opti.parameter()
        p_vx_max = opti.parameter()
        p_vy_max = opti.parameter()
        p_omega_max = opti.parameter()
        p_vtheta_max = opti.parameter()
        p_goal_yaw = opti.parameter()       # terminal goal-heading target
        p_goal_yaw_w = opti.parameter()     # its weight (0 = disabled this solve)

        R = np.diag([cfg.R_vx, cfg.R_vy, cfg.R_omega, cfg.R_vtheta])
        cost = 0.0

        for k in range(N):
            theta = X[6, k]
            tau = theta / p_Ln                      # normalised arc length
            xr = self._poly(p_cx, tau)
            yr = self._poly(p_cy, tau)
            dxr = self._dpoly(p_cx, tau) / p_Ln     # d/dθ via chain rule
            dyr = self._dpoly(p_cy, tau) / p_Ln
            tnorm = ca.sqrt(dxr ** 2 + dyr ** 2 + 1e-9)
            cphi = dxr / tnorm
            sphi = dyr / tnorm

            ex = X[0, k] - xr
            ey = X[1, k] - yr
            e_c = sphi * ex - cphi * ey             # contouring (lateral)
            e_l = cphi * ex + sphi * ey             # lag (longitudinal)
            cost += cfg.w_contour * e_c ** 2 + cfg.w_lag * e_l ** 2

            # Soft yaw alignment to the path tangent (wrap-safe via 1−cos).
            phi = ca.atan2(dyr, dxr)
            cost += cfg.Q_yaw_align * (1.0 - ca.cos(X[2, k] - phi))

            # Progress reward (drive v_θ up → minimum time)
            cost += -cfg.q_progress * U[3, k] * dt

            # Control effort + jerk
            u_k = U[:, k]
            cost += ca.mtimes([u_k.T, R, u_k])
            if k > 0:
                du = U[:, k] - U[:, k - 1]
                cost += cfg.R_jerk * ca.dot(du, du)

            # Obstacle barrier (sigmoid soft zone + quadratic penetration)
            for j in range(n_obs):
                dist_k = ca.sqrt((X[0, k] - p_obs[0, j]) ** 2 +
                                 (X[1, k] - p_obs[1, j]) ** 2 + 1e-6)
                s_arg = cfg.obs_alpha * (dist_k - cfg.obs_r)
                cost += cfg.W_obs_sigmoid * 0.5 * (1.0 - ca.tanh(0.5 * s_arg))
                pen = ca.fmax(0.0, cfg.obs_r - dist_k)
                cost += cfg.W_obs_sigmoid * 2.0 * pen ** 2

        # Terminal: contour + progress-to-goal reward
        theta_N = X[6, N]
        tau_N = theta_N / p_Ln
        xr_N = self._poly(p_cx, tau_N)
        yr_N = self._poly(p_cy, tau_N)
        dxr_N = self._dpoly(p_cx, tau_N) / p_Ln
        dyr_N = self._dpoly(p_cy, tau_N) / p_Ln
        tnorm_N = ca.sqrt(dxr_N ** 2 + dyr_N ** 2 + 1e-9)
        ex_N = X[0, N] - xr_N
        ey_N = X[1, N] - yr_N
        e_c_N = (dyr_N * ex_N - dxr_N * ey_N) / tnorm_N
        e_l_N = (dxr_N * ex_N + dyr_N * ey_N) / tnorm_N
        cost += cfg.w_contour * 2.0 * e_c_N ** 2 + cfg.w_lag * e_l_N ** 2
        cost += -cfg.q_progress_terminal * theta_N    # push θ_N toward s_max (goal)

        # Terminal GOAL-heading alignment (parameter-gated). Drives the horizon-end
        # yaw toward the imposed goal heading. p_goal_yaw_w is 0 unless mpc_node
        # ramps it in near the goal (require_goal_heading) — so in transit the
        # path-tangent term above owns yaw and this contributes nothing (no fight,
        # no mid-path spin); near the goal it blends the robot into goal_yaw so it
        # arrives already aligned instead of rotating in place afterwards.
        cost += p_goal_yaw_w * (1.0 - ca.cos(X[2, N] - p_goal_yaw))

        for j in range(n_obs):
            dist_T = ca.sqrt((X[0, N] - p_obs[0, j]) ** 2 +
                             (X[1, N] - p_obs[1, j]) ** 2 + 1e-6)
            s_argT = cfg.obs_alpha * (dist_T - cfg.obs_r)
            cost += cfg.W_obs_sigmoid * 0.5 * (1.0 - ca.tanh(0.5 * s_argT))
            penT = ca.fmax(0.0, cfg.obs_r - dist_T)
            cost += cfg.W_obs_sigmoid * 2.0 * penT ** 2

        opti.minimize(cost)

        # ── Dynamics ──
        for k in range(N):
            yaw_k = X[2, k]
            vx_k, vy_k, wz_k = X[3, k], X[4, k], X[5, k]
            vx_cmd, vy_cmd, wz_cmd, vth = U[0, k], U[1, k], U[2, k], U[3, k]

            vx_n = (1.0 - lag_v) * vx_k + lag_v * vx_cmd
            vy_n = (1.0 - lag_v) * vy_k + lag_v * vy_cmd
            wz_n = (1.0 - lag_w) * wz_k + lag_w * wz_cmd

            cy_, sy_ = ca.cos(yaw_k), ca.sin(yaw_k)
            opti.subject_to(X[0, k + 1] == X[0, k] + (vx_n * cy_ - vy_n * sy_) * dt)
            opti.subject_to(X[1, k + 1] == X[1, k] + (vx_n * sy_ + vy_n * cy_) * dt)
            opti.subject_to(X[2, k + 1] == yaw_k + wz_n * dt)
            opti.subject_to(X[3, k + 1] == vx_n)
            opti.subject_to(X[4, k + 1] == vy_n)
            opti.subject_to(X[5, k + 1] == wz_n)
            opti.subject_to(X[6, k + 1] == X[6, k] + vth * dt)

        opti.subject_to(X[:, 0] == p_x0)

        # ── Box constraints ──
        for k in range(N):
            opti.subject_to(opti.bounded(0.0, U[0, k], p_vx_max))
            opti.subject_to(opti.bounded(-p_vy_max, U[1, k], p_vy_max))
            opti.subject_to(opti.bounded(-p_omega_max, U[2, k], p_omega_max))
            opti.subject_to(opti.bounded(0.0, U[3, k], p_vtheta_max))   # progress ≥ 0
            opti.subject_to(opti.bounded(0.0, X[6, k + 1], p_smax))     # θ ∈ [0, s_max]

        p_opts = {'expand': True, 'print_time': False}
        s_opts = {
            'max_iter': cfg.max_iter,
            'print_level': cfg.print_level,
            'sb': 'yes',
            'warm_start_init_point': 'yes' if cfg.warm_start else 'no',
        }
        opti.solver('ipopt', p_opts, s_opts)

        self._opti = opti
        self._X, self._U = X, U
        self._p_x0, self._p_obs = p_x0, p_obs
        self._p_cx, self._p_cy = p_cx, p_cy
        self._p_Ln, self._p_smax = p_Ln, p_smax
        self._p_vx_max, self._p_vy_max = p_vx_max, p_vy_max
        self._p_omega_max, self._p_vtheta_max = p_omega_max, p_vtheta_max
        self._p_goal_yaw, self._p_goal_yaw_w = p_goal_yaw, p_goal_yaw_w
        self._nlp_built = True
        self._prev_u = None
        self._prev_x = None

    # ------------------------------------------------------------------
    # Solve
    # ------------------------------------------------------------------

    def solve(self, robot_state, path_world, obstacle_points_2d=None,
              goal_yaw=None, goal_yaw_weight=0.0) -> MPCCResult:
        t0 = time.perf_counter()
        cfg = self.cfg
        N, NX, NU = cfg.N, self.NX, self.NU

        x6 = np.asarray(robot_state, dtype=float)
        if len(x6) == 3:
            x6 = np.concatenate([x6, [0.0, 0.0, 0.0]])
        elif len(x6) != 6:
            raise ValueError(f"Expected state length 3 or 6, got {len(x6)}")
        if not np.isfinite(x6).all():
            x6 = self._last_valid_x0.copy() if self._last_valid_x0 is not None else np.zeros(6)
        self._last_valid_x0 = x6.copy()

        fit = self._fit_path(x6[:2], path_world) if path_world else None
        if fit is None:
            # No usable path → hold in place (mpc_node will publish ~zero vel).
            x_hold = np.tile(np.concatenate([x6, [0.0]]), (N + 1, 1))
            return MPCCResult(False, x_hold, np.zeros((N, NU)), float('inf'),
                              (time.perf_counter() - t0) * 1e3)
        cx, cy, Ln, s_max = fit

        x0 = np.concatenate([x6, [0.0]])   # θ_0 = 0 (progress measured from here)

        robot_xy = x6[:2]
        if obstacle_points_2d is not None and len(obstacle_points_2d) > 0:
            obs_pts = self._select_obs_points(obstacle_points_2d, robot_xy)
        else:
            obs_pts = np.full((cfg.max_obs_constraints, 2), self._OBS_SENTINEL)
        if not np.isfinite(obs_pts).all():
            obs_pts = np.full((cfg.max_obs_constraints, 2), self._OBS_SENTINEL)

        if not self._nlp_built:
            self._build_nlp()
        opti = self._opti

        opti.set_value(self._p_x0, x0)
        opti.set_value(self._p_obs, obs_pts.T)
        opti.set_value(self._p_cx, cx)
        opti.set_value(self._p_cy, cy)
        opti.set_value(self._p_Ln, max(Ln, 1e-6))
        opti.set_value(self._p_smax, max(s_max, 1e-3))
        opti.set_value(self._p_vx_max, max(self._vx_max_eff, 0.05))
        opti.set_value(self._p_vy_max, max(self._vy_max_eff, 0.01))
        opti.set_value(self._p_omega_max, max(self._omega_max_eff, 0.05))
        opti.set_value(self._p_vtheta_max, max(self._vtheta_max_eff, 0.05))
        # Terminal goal-heading target + weight (0 → tangent alone owns yaw).
        opti.set_value(self._p_goal_yaw, float(goal_yaw) if goal_yaw is not None else 0.0)
        opti.set_value(self._p_goal_yaw_w, max(float(goal_yaw_weight), 0.0))

        # Warm start (or a forward-rolling cold guess)
        if cfg.warm_start and self._prev_u is not None and self._prev_x is not None \
                and self._consecutive_failures < self._MAX_CONSEC_FAILURES:
            try:
                opti.set_initial(self._U, self._prev_u.T)
                opti.set_initial(self._X, self._prev_x.T)
            except Exception:
                self._cold_guess(opti, x0, s_max)
        else:
            self._cold_guess(opti, x0, s_max)
            self._consecutive_failures = 0

        try:
            sol = opti.solve()
            success = True
            cost_val = float(sol.value(opti.f))
            U_opt = np.array(sol.value(self._U), dtype=float)
            X_opt = np.array(sol.value(self._X), dtype=float)
            if np.any(np.isnan(U_opt)) or np.any(np.isnan(X_opt)):
                raise ValueError('NaN in solution')
            u_seq = U_opt.T
            x_pred = X_opt.T
            self._consecutive_failures = 0
            self._prev_u = np.vstack([u_seq[1:], u_seq[-1:]])
            self._prev_x = np.vstack([x_pred[1:], x_pred[-1:]])
        except Exception:
            success = False
            self._consecutive_failures += 1
            # Issue #1 (stop/go): do NOT discard the last good warm start on a
            # transient failure. Zeroing _prev_u/_prev_x forced a COLD start next
            # cycle, which on this heavy 7-state NLP is more likely to fail again
            # → a failure cascade the caller turns into a sustained stop. Keeping
            # the previous (successful) trajectory lets IPOPT resume from a good
            # guess; the _MAX_CONSEC_FAILURES gate above still falls back to a
            # cold start once failures actually persist.
            if self._consecutive_failures >= self._MAX_CONSEC_FAILURES:
                self._prev_u = None
                self._prev_x = None
            try:
                dbg = opti.debug
                X_opt = np.array(dbg.value(self._X), dtype=float)
                U_opt = np.array(dbg.value(self._U), dtype=float)
                x_pred = X_opt.T
                u_seq = U_opt.T
                cost_val = float(dbg.value(opti.f))
                if not np.isfinite(x_pred).all():
                    raise ValueError
            except Exception:
                x_pred = np.tile(x0, (N + 1, 1))
                u_seq = np.zeros((N, NU))
                cost_val = float('inf')

        progress = float(x_pred[-1, 6] - x_pred[0, 6])
        return MPCCResult(success, x_pred, u_seq, cost_val,
                          (time.perf_counter() - t0) * 1e3, progress_m=progress)

    def _cold_guess(self, opti, x0, s_max):
        """Straight forward-progress initial guess."""
        N, NU = self.cfg.N, self.NU
        X_init = np.tile(x0, (N + 1, 1))
        # ramp θ linearly toward s_max so IPOPT starts with forward progress
        X_init[:, 6] = np.linspace(0.0, min(s_max, self._vtheta_max_eff * N * self.cfg.dt), N + 1)
        U_init = np.zeros((NU, N))
        U_init[3, :] = self._vtheta_max_eff   # seed progress speed
        try:
            opti.set_initial(self._X, X_init.T)
            opti.set_initial(self._U, U_init)
        except Exception:
            pass
