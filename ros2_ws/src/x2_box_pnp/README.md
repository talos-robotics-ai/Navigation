# x2_box_pnp

Box pick-and-place FSM. Pure Python: `geometry.py`, `keyframes.py`, `fsm.py` (one `TRANSITIONS` table, `PLANS`
for manipulation); `pnp_fsm_node.py` is the rclpy wrapper. Tests: `python3 -m pytest test`.

States: WAIT_FOR_BOX -> NAV_TO_PREGRASP -> ALIGN -> SETTLE -> REACH -> GRASP -> LIFT -> NAV_TO_PLACE -> LOWER ->
RELEASE -> RETRACT -> BACK_OFF -> DONE; plus FAILED, ESTOP. Each transition is logged `PNP: A -> B (reason)`;
`/pnp/state` (String) is published. `/x2/cmd_vel_out` is published at 10 Hz in EVERY state (zero unless
NAV_TO_PREGRASP / ALIGN / NAV_TO_PLACE).

* Box never/no longer seen: stand still (zero velocity). In NAV_TO_PREGRASP, ALIGN, SETTLE a box unseen for
  `lost_timeout` (1.5 s) -> WAIT_FOR_BOX (zero velocity already after 0.5 s). No search or wandering.
  After REACH the box may be occluded by the arms, so it is no longer required.
* Start: `/pnp/start` (Trigger service or Bool topic) AND crate seen `start_frames` consecutive frames
  (or `auto_start`). Start is consumed; after a box loss or reset a new start is needed.
* NAV_TO_PREGRASP: goal = crate xy minus `standoff` along the robot->crate line (`pregrasp_mode: axis` uses the crate
  axis), yaw toward the crate; re-published if the crate moves > 0.15 m. Arrival: within 0.15 m / 0.15 rad, or the
  planner's `/navigation/state == GOAL_REACHED` and < 0.35 m (planner does not align heading; ALIGN does).
* ALIGN: P control, vx in [0, 0.15], walks only when bearing < 0.4 rad; too close (cannot back up) -> FAILED.
* Arms: `config/arm_keyframes.yaml` (default = null = policy default; the others are TODO-calibrate placeholders equal
  to the default). Smoothstep interpolation over `arm_move_s`; advance only when |q_arm - target| < `arm_tol`;
  timeout -> FAILED. GRASP sends `hand_closed` (TODO-calibrate, placeholder 1.0); RELEASE sends default (open).
* BACK_OFF: vx >= 0 only, so no backing up: stand `backoff_time`, then DONE.
* `/pnp/reset` -> WAIT_FOR_BOX (ignored while `/estop` is true). ESTOP/FAILED hold the arms, zero velocity.
