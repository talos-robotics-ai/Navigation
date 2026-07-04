# MPCC — Model Predictive Contouring Control (time-optimal tracker)

The `a_star_mpc_planner` MPC node can run in one of two modes (`mpc_mode`):

- **`tracking`** — the original reference-tracking MPC ([`mpc_tracker.py`](../ros2_ws/src/a_star_mpc_planner/a_star_mpc_planner/mpc_tracker.py)): follows the A\* path at a *fixed* cruise speed `mpc_v_ref`.
- **`contouring`** — the **MPCC** ([`mpcc_tracker.py`](../ros2_ws/src/a_star_mpc_planner/a_star_mpc_planner/mpcc_tracker.py)) described here: reaches the goal in **minimum time** while staying on the (collision-free) A\* path and respecting the obstacle barrier.

This document specifies the MPCC formulation, maps it to the code, and gives the tuning/operational notes.

---

## 1. Why contouring instead of tracking

A reference-tracking MPC samples a target point that moves along the path at a **fixed** speed `v_ref` and tries to sit on that point each step. Consequences:

- It drives at `v_ref` regardless of remaining speed budget → **not time-optimal** (e.g. `v_ref = 0.30` while `vx_max = 0.45` wastes a third of the speed).
- Speed and path-following are coupled through one reference, so "go faster" and "stay on the line" cannot be traded independently.

MPCC decouples them. The path is parameterised by **arc length `θ`**, which the optimiser is free to advance as a *virtual progress state*. The cost then expresses three independent goals:

| goal | term | effect |
|---|---|---|
| stay on the path | contouring error `e_c²` | lateral deviation → 0 |
| keep `θ` honest | lag error `e_l²` | `θ` tracks the robot's true progress |
| reach the goal fast | progress reward `−q·θ` | advance `θ` as fast as limits allow |

Maximising progress **subject to the velocity limits and the obstacle barrier** yields minimum-time motion along a collision-free path — that is the time-optimal behaviour requested.

---

## 2. State, control, model

```
state    x = [ px, py, yaw, vx, vy, wz, θ ]      NX = 7
control  u = [ vx_cmd, vy_cmd, wz_cmd, vθ ]       NU = 4
```

The first six entries are identical to the tracking MPC (so `mpc_node` consumes the result — `cmd_vel = x_pred[1, 3:6]`, predicted path = `x_pred[:, :3]` — unchanged). The seventh, `θ`, is the **progress** (arc length travelled along the local path); `vθ ≥ 0` is its rate.

**Discrete dynamics** (first-order actuator lag, exact ZOH; holonomic body velocity):

```
lag_v = 1 − exp(−dt/τ_v)         lag_w = 1 − exp(−dt/τ_w)

vx⁺ = (1−lag_v)·vx + lag_v·vx_cmd
vy⁺ = (1−lag_v)·vy + lag_v·vy_cmd        # vy kept (holonomic / crab-walk)
wz⁺ = (1−lag_w)·wz + lag_w·wz_cmd

px⁺  = px + (vx⁺·cos yaw − vy⁺·sin yaw)·dt
py⁺  = py + (vx⁺·sin yaw + vy⁺·cos yaw)·dt
yaw⁺ = yaw + wz⁺·dt
θ⁺   = θ  + vθ·dt
```

Position update uses the **post-lag** velocity, so the predicted trajectory matches the identified actuator response. (Code: dynamics constraints at [`mpcc_tracker.py:345-364`](../ros2_ws/src/a_star_mpc_planner/a_star_mpc_planner/mpcc_tracker.py#L345-L364).)

---

## 3. Path parameterisation `p(θ)`

MPCC needs the reference path as a smooth, differentiable function of `θ`. Each solve, the **local A\* path ahead of the robot** is fit with low-degree polynomials in **normalised** arc length `τ = θ / L` (`L` = remaining path length; normalisation keeps the Vandermonde fit well-conditioned):

```
xr(θ) = Σ_{j=0}^{d} cx_j · τ^j           yr(θ) = Σ_{j=0}^{d} cy_j · τ^j
                                          d = mpcc_poly_degree
```

The polynomial **coefficients `cx, cy` (plus `L`, `s_max`) are NLP parameters**, refreshed every solve — so the single cached NLP handles an arbitrary, continuously replanned path without a rebuild. The path tangent is the analytic derivative:

```
dxr/dθ = (1/L)·Σ j·cx_j·τ^{j−1}          dyr/dθ = (1/L)·Σ j·cy_j·τ^{j−1}
φ(θ)   = atan2(dyr/dθ, dxr/dθ)            (unit tangent t = (cosφ, sinφ))
```

(Code: fit at [`_fit_path`, mpcc_tracker.py:188-234`](../ros2_ws/src/a_star_mpc_planner/a_star_mpc_planner/mpcc_tracker.py#L188-L234); evaluation helpers `_poly`/`_dpoly` at [236-252](../ros2_ws/src/a_star_mpc_planner/a_star_mpc_planner/mpcc_tracker.py#L236-L252).)

`θ` starts at `0` (measured from the robot's closest point on the path) and is bounded `θ ∈ [0, s_max]`, where `s_max = min(path length, reachable distance this horizon)`.

---

## 4. Cost function

Let `e = p − p_ref(θ) = (px − xr, py − yr)`. Project it onto the path frame at `θ`:

```
contouring error   e_c =  sinφ·e_x − cosφ·e_y      (lateral, ⟂ to path)
lag error          e_l =  cosφ·e_x + sinφ·e_y      (longitudinal, ∥ to path)
```

Stage cost, summed over `k = 0 … N−1`:

```
J_k =  w_c · e_c²                       # stay on the path
     + w_l · e_l²                       # θ stays consistent with the robot
     + Q_yaw_align · (1 − cos(yaw − φ)) # soft "face the tangent" (wrap-safe)
     − q_progress · vθ · dt             # REWARD progress  → minimum time
     + uᵀ R u  +  R_jerk · ‖Δu‖²        # effort + smoothness
     + Σ_obs [ W·½(1 − tanh(½α(d−r))) + W·2·max(0, r−d)² ]   # obstacle barrier
```

Terminal cost (`k = N`):

```
J_N =  2·w_c · e_c,N²  +  w_l · e_l,N²
     − q_progress_terminal · θ_N        # pull θ_N toward s_max (the goal)
     + w_gψ · (1 − cos(yaw_N − yaw_goal))# terminal: face the GOAL heading (param-gated)
     + Σ_obs [ barrier ]
```

The terminal goal-heading term is **off by default** (`w_gψ = 0`): its weight is a per-solve NLP parameter that `mpc_node` sets to `β · mpcc_Q_goal_yaw` — non-zero only when `require_goal_heading` is true, ramping `β: 0→1` over the last `goal_align_start_dist` metres. So in transit the path-tangent term owns yaw; near the goal the robot blends into `yaw_goal` and arrives aligned (no in-place spin). See `_goal_yaw` handling in `mpc_node` and [`mpcc_tracker.py`](../ros2_ws/src/a_star_mpc_planner/a_star_mpc_planner/mpcc_tracker.py) (`p_goal_yaw`, `p_goal_yaw_w`).

The two progress terms (`−q_progress·vθ` per step and `−q_progress_terminal·θ_N` at the end) are what drive the robot **as fast as the constraints allow**; the contour/lag terms keep it on the A\* path; the barrier is the per-step collision veto.

(Code: objective at [`mpcc_tracker.py:286-343`](../ros2_ws/src/a_star_mpc_planner/a_star_mpc_planner/mpcc_tracker.py#L286-L343); the contour/lag/progress lines are [293-302](../ros2_ws/src/a_star_mpc_planner/a_star_mpc_planner/mpcc_tracker.py#L293-L302).)

---

## 5. Constraints

```
x_0 = [robot state, θ=0]                       # initial state
dynamics (Section 2)                           # equality, k = 0 … N−1
0       ≤ vx_cmd ≤ vx_max                       # forward-only translational
−vy_max ≤ vy_cmd ≤ vy_max                       # holonomic lateral (crab-walk)
−ω_max  ≤ wz_cmd ≤ ω_max
0       ≤ vθ     ≤ vθ_max                        # progress never goes backward
0       ≤ θ_k    ≤ s_max                         # stay within the local path
```

The velocity limits are **NLP parameters** (`p_vx_max`, …), so `update_velocity_limits()` (the adaptive-limit hook in `mpc_node`) lowers them on failure bursts without rebuilding the NLP. (Code: box constraints [`mpcc_tracker.py:366-374`](../ros2_ws/src/a_star_mpc_planner/a_star_mpc_planner/mpcc_tracker.py#L366-L374).)

---

## 6. Safety: three layers, defence in depth

1. **A\* path** — already collision-free against the inflated costmap (lethal core = `robot_radius`), so the *reference geometry* avoids obstacles.
2. **Contour + lag cost** — keeps the robot ON that safe path; it does not freely cut corners.
3. **Obstacle barrier** — a last-resort per-step veto on the live obstacle points; if anything intrudes within `obs_r`, progress is penalised so the robot slows/stops rather than colliding.

In testing, an obstacle placed on the path makes MPCC **halt short of it** (progress drops, no penetration) instead of running it over — confirmed by `test_obstacle_on_path_not_run_over` in [`test/test_mpcc_tracker.py`](../ros2_ws/src/a_star_mpc_planner/test/test_mpcc_tracker.py).

---

## 7. Solver

CasADi `Opti` + IPOPT, built once and cached; all per-solve data (state, obstacles, path coefficients, velocity limits) enters as **parameters**. Warm-started from the previous solution (shifted one step); a forward-progress cold guess seeds `θ` linearly and `vθ = vθ_max` after failures. (Code: `solve()` [`mpcc_tracker.py:398-491`](../ros2_ws/src/a_star_mpc_planner/a_star_mpc_planner/mpcc_tracker.py#L398-L491); `_cold_guess` [493-511](../ros2_ws/src/a_star_mpc_planner/a_star_mpc_planner/mpcc_tracker.py#L493-L511).)

Measured: **p50 ≈23 ms / solve at `N=50`** on the dev machine — within the 10 Hz (`mpc_rate_hz`) budget. The MPCC NLP is heavier than the tracking MPC (7 states / 4 controls vs 6 / 3), so watch `solve_ms` in `/mpc/diagnostics` and reduce `mpc_N` or `mpcc_poly_degree` if it approaches `1000/mpc_rate_hz` ms.

> **Control-grade termination (2026-07-04).** IPOPT's default KKT tolerance is
> `1e-8` — far finer than a 10 Hz velocity command can use. Setting `tol=1e-3`
> plus early-accept (`acceptable_tol`/`acceptable_iter`) returns as soon as the
> solution is control-accurate: **p50 32 → 23 ms, p95 halved, 100 % solve
> success** on the deployed problem. `max_cpu_time` (0.3 s) is a **hang guard,
> not a per-cycle budget** — it must stay well above a cold solve on the Orin
> Nano (~150 ms); an 80 ms wall was benchmarked and cascaded into permanent
> failure. Warm-start `mu_init`/bound-push overrides were also tried and
> rejected (slower recovery solves). See
> [DISTRIBUTED_NAV_PLAN.md](DISTRIBUTED_NAV_PLAN.md) §9.

---

## 8. Parameters

Set in [`config/planner_params_default.yaml`](../ros2_ws/src/a_star_mpc_planner/config/planner_params_default.yaml).

| param | meaning | default |
|---|---|---|
| `mpc_mode` | `tracking` or `contouring` | `contouring` |
| `mpcc_vtheta_max` | max progress (arc-length) speed [m/s] | `0.55` |
| `mpcc_w_contour` | lateral path-deviation weight (stay on path) | `300.0` |
| `mpcc_w_lag` | longitudinal θ-mismatch weight | `60.0` |
| `mpcc_q_progress` | per-step progress-speed reward (push speed) | `2.0` |
| `mpcc_q_progress_terminal` | terminal θ reward (push toward goal) | `60.0` |
| `mpcc_Q_yaw_align` | soft face-tangent weight (low → crab freely) | `40.0` |
| `mpcc_Q_goal_yaw` | terminal goal-heading weight (0 in transit; ramps in near goal, only if `require_goal_heading`) | `40.0` |
| `mpcc_R_vtheta` | progress-rate effort | `0.1` |
| `mpcc_poly_degree` | local path polynomial order | `3` |

Shared with the tracking MPC: `mpc_N`, `mpc_dt`, `mpc_tau_v/w`, `mpc_vx_max/mpc_vy_max/mpc_omega_max`, `mpc_R_vx/vy/omega/jerk`, the `mpc_W_obs_*`/`mpc_obs_*` barrier set, `mpc_max_iter`, `mpc_warm_start`.

---

## 9. Tuning guide

- **Cuts corners / clips obstacles** → raise `mpcc_w_contour`, or lower `mpcc_q_progress` / `mpcc_q_progress_terminal`. The A\* path is the primary avoider; MPCC should hug it.
- **Too slow / hesitant** → raise `mpcc_q_progress_terminal` and/or `mpcc_vtheta_max` (up to `vx_max`); lower `mpcc_w_lag` slightly.
- **Oscillates along the path (θ hunts)** → raise `mpcc_w_lag` and `mpcc_R_vtheta`.
- **Crabs sideways too much / looks unnatural** → raise `mpcc_Q_yaw_align` (faces the travel direction more) or lower `mpc_vy_max`.
- **Solve too slow** → lower `mpc_N`, lower `mpcc_poly_degree` (3 → 2 for gentle paths), or lower `mpc_max_iter`.

---

## 10. Enable / fall back

```yaml
# planner_params_default.yaml
mpc_mode: contouring     # time-optimal MPCC
# mpc_mode: tracking     # instant fallback to the proven reference MPC
```

Switching modes needs only a node relaunch (no rebuild for the param itself). The result layout is identical across modes, so the state machine, `/mpc/cmd_vel`, the e-stop, and the RViz markers all work regardless. If anything misbehaves on the robot, set `mpc_mode: tracking` to revert instantly.

See [A_STAR_MPC_PLANNER.md](A_STAR_MPC_PLANNER.md) §4b for the operational summary.

---

## Appendix A — Full formulation (LaTeX)

The equations below are the exact MPCC implemented in [`mpcc_tracker.py`](../ros2_ws/src/a_star_mpc_planner/a_star_mpc_planner/mpcc_tracker.py) (GitHub renders the math).

### A.1 State and control

$$
\mathbf{x} = \big[\,p_x,\; p_y,\; \psi,\; v_x,\; v_y,\; \omega,\; \theta\,\big]^\top \in \mathbb{R}^{7},
\qquad
\mathbf{u} = \big[\,v_x^{c},\; v_y^{c},\; \omega^{c},\; v_\theta\,\big]^\top \in \mathbb{R}^{4}
$$

where $\psi$ is yaw, $\theta$ the path progress (arc length), and $v_\theta \ge 0$ its rate. Superscript $c$ denotes a commanded input.

### A.2 Actuator lag and discrete dynamics

$$
\lambda_v = 1 - e^{-\Delta t/\tau_v}, \qquad \lambda_w = 1 - e^{-\Delta t/\tau_w}
$$

$$
\begin{aligned}
v_x^{+} &= (1-\lambda_v)\,v_x + \lambda_v\,v_x^{c}, &
v_y^{+} &= (1-\lambda_v)\,v_y + \lambda_v\,v_y^{c}, &
\omega^{+} &= (1-\lambda_w)\,\omega + \lambda_w\,\omega^{c},\\
p_x^{+} &= p_x + \big(v_x^{+}\cos\psi - v_y^{+}\sin\psi\big)\,\Delta t, &
p_y^{+} &= p_y + \big(v_x^{+}\sin\psi + v_y^{+}\cos\psi\big)\,\Delta t, &
\psi^{+} &= \psi + \omega^{+}\,\Delta t,\\
\theta^{+} &= \theta + v_\theta\,\Delta t. & & & &
\end{aligned}
$$

The position update uses the **post-lag** velocity $v^{+}$ (ZOH), so the prediction matches the identified actuator response.

### A.3 Path parameterisation and tangent

With normalised arc length $\tau = \theta / L$ (remaining length $L$) and degree $d = $ `mpcc_poly_degree`:

$$
x_r(\theta) = \sum_{j=0}^{d} c^{x}_{j}\,\tau^{j}, \qquad
y_r(\theta) = \sum_{j=0}^{d} c^{y}_{j}\,\tau^{j},
$$

$$
x_r'(\theta) = \frac{1}{L}\sum_{j=1}^{d} j\,c^{x}_{j}\,\tau^{\,j-1}, \qquad
y_r'(\theta) = \frac{1}{L}\sum_{j=1}^{d} j\,c^{y}_{j}\,\tau^{\,j-1}.
$$

Unit tangent and heading of the path at $\theta$:

$$
\lVert \mathbf{t} \rVert = \sqrt{x_r'^{\,2} + y_r'^{\,2} + \epsilon},\qquad
\cos\phi = \frac{x_r'}{\lVert \mathbf{t}\rVert},\quad
\sin\phi = \frac{y_r'}{\lVert \mathbf{t}\rVert},\qquad
\phi = \operatorname{atan2}\!\big(y_r',\, x_r'\big).
$$

The coefficients $c^{x}, c^{y}$ and $L,\, s_{\max}$ are **NLP parameters**, refreshed every solve.

### A.4 Contouring and lag errors

$$
e_x = p_x - x_r(\theta), \qquad e_y = p_y - y_r(\theta),
$$

$$
\underbrace{e_c = \sin\phi\,e_x - \cos\phi\,e_y}_{\text{contouring (lateral, }\perp\text{ path)}},
\qquad
\underbrace{e_l = \cos\phi\,e_x + \sin\phi\,e_y}_{\text{lag (longitudinal, }\parallel\text{ path)}}.
$$

### A.5 Obstacle barrier

For each of the $M$ selected obstacle points $\mathbf{o}_i = (o^x_i, o^y_i)$, at stage $k$:

$$
d_{k,i} = \sqrt{(p_{x,k} - o^x_i)^2 + (p_{y,k} - o^y_i)^2 + \epsilon},
$$

$$
B_{k,i} = \underbrace{\tfrac{1}{2}\,W_{\text{obs}}\Big(1 - \tanh\!\big(\tfrac{\alpha}{2}(d_{k,i} - r)\big)\Big)}_{\text{sigmoid soft zone}}
\;+\;
\underbrace{2\,W_{\text{obs}}\,\big[\max(0,\; r - d_{k,i})\big]^{2}}_{\text{quadratic penetration}},
$$

with $W_{\text{obs}} = $ `mpc_W_obs_sigmoid`, $\alpha = $ `mpc_obs_alpha`, $r = $ `mpc_obs_r`.

### A.6 Stage and terminal cost

Control-weight matrix $R = \operatorname{diag}(R_{v_x}, R_{v_y}, R_{\omega}, R_{v_\theta})$. For $k = 0,\dots,N-1$:

$$
\ell_k = w_c\,e_c^2 + w_l\,e_l^2
+ Q_\psi\big(1 - \cos(\psi - \phi)\big)
- q\,v_\theta\,\Delta t
+ \mathbf{u}_k^\top R\,\mathbf{u}_k
+ R_j\,\lVert \mathbf{u}_k - \mathbf{u}_{k-1}\rVert^2
+ \sum_{i=1}^{M} B_{k,i},
$$

(the jerk term applies for $k \ge 1$). Terminal cost at $k = N$:

$$
\ell_N = 2\,w_c\,e_{c,N}^2 + w_l\,e_{l,N}^2 - q_T\,\theta_N
+ w_{g\psi}\big(1 - \cos(\psi_N - \psi_{\text{goal}})\big)
+ \sum_{i=1}^{M} B_{N,i}.
$$

The terminal goal-heading weight is a per-solve parameter $w_{g\psi} = \beta\,Q_{g\psi}$, with $\beta \in [0,1]$ ramped as the robot nears the goal and $\beta = 0$ (term off) unless `require_goal_heading` is set — so the path tangent alone governs yaw in transit.

### A.7 The optimal control problem

$$
\min_{\substack{\mathbf{x}_{0:N}\\ \mathbf{u}_{0:N-1}}}\;
\sum_{k=0}^{N-1} \ell_k \;+\; \ell_N
$$

subject to, for $k = 0,\dots,N-1$:

$$
\mathbf{x}_0 = \big[\mathbf{x}_{\text{robot}},\, \theta{=}0\big], \qquad
\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k) \ \ (\text{A.2}),
$$

$$
0 \le v_{x,k}^{c} \le v_x^{\max}, \quad
\lvert v_{y,k}^{c}\rvert \le v_y^{\max}, \quad
\lvert \omega_{k}^{c}\rvert \le \omega^{\max}, \quad
0 \le v_{\theta,k} \le v_\theta^{\max}, \quad
0 \le \theta_{k+1} \le s_{\max}.
$$

### A.8 Symbol → parameter map

| symbol | meaning | parameter |
|---|---|---|
| $N,\ \Delta t$ | horizon length, step | `mpc_N`, `mpc_dt` |
| $\tau_v,\ \tau_w$ | actuator lag constants | `mpc_tau_v`, `mpc_tau_w` |
| $w_c,\ w_l$ | contouring, lag weights | `mpcc_w_contour`, `mpcc_w_lag` |
| $q,\ q_T$ | progress reward (stage, terminal) | `mpcc_q_progress`, `mpcc_q_progress_terminal` |
| $Q_\psi$ | face-tangent weight | `mpcc_Q_yaw_align` |
| $Q_{g\psi},\ w_{g\psi}=\beta Q_{g\psi}$ | terminal goal-heading weight (max, ramped) | `mpcc_Q_goal_yaw` |
| $\psi_{\text{goal}}$ | imposed goal heading | `_goal_yaw` (RViz goal quaternion) |
| $R_{v_x},R_{v_y},R_{\omega},R_{v_\theta}$ | control effort | `mpc_R_vx/vy/omega`, `mpcc_R_vtheta` |
| $R_j$ | jerk (Δu) weight | `mpc_R_jerk` |
| $W_{\text{obs}},\ \alpha,\ r$ | barrier weight, sharpness, radius | `mpc_W_obs_sigmoid`, `mpc_obs_alpha`, `mpc_obs_r` |
| $v_x^{\max},v_y^{\max},\omega^{\max}$ | velocity limits | `mpc_vx_max/vy_max/omega_max` |
| $v_\theta^{\max}$ | max progress speed | `mpcc_vtheta_max` |
| $d$ | path polynomial degree | `mpcc_poly_degree` |
| $M$ | obstacle points in the barrier | `mpc_max_obs_constraints` |
