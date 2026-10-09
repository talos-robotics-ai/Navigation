"""Box pick-and-place state machine (pure Python; no rclpy). One transition table, explicit enums.

Driven by PnpFsm.update(Inputs) at ~10 Hz; returns Outputs. Every transition is logged as
    PNP: A -> B (reason)
Safety contract: the robot only moves (non-zero vx/wz) in NAV_TO_PREGRASP, ALIGN and
NAV_TO_PLACE. Everywhere else the output velocity is exactly zero (the robot stands balancing).

Bring-up (Params.bringup, for the on-robot RL walker): the same hanging-to-walking handover as the
teleop stack (deployment_from_mac teleop/robot_control/x2_lower.py), driven by the walker's own
phases (Inputs.walker_phase: ramp|glide|stance|blend|live|damped|stopped):
    BRINGUP      walker ramping stiffness / gliding the hanging legs onto the gait's phase-0 stance
    STANCE_WAIT  stance held open-ended; the operator lowers the gantry until the feet are flat and
                 the robot is upright, then requests the engage
    ENGAGE       engage sent; the walker's pose-parity gate (legs+waist within 0.15 rad of the
                 stance) accepts -> blend, or refuses -> back to STANCE_WAIT with the joint named
    BLEND        the policy takes the legs over (1 s)
    READY        the policy is balancing; the hook is still on. The operator releases the hook and
                 confirms ("released") -> WAIT_FOR_BOX
The walker ignores vx/wz until it is live; this machine outputs zero velocity in all of these
states anyway. A walker that drops to damped/stopped anywhere -> FAILED (walker down).
"""
import enum
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from . import geometry as g
from .keyframes import ArmMotion, Keyframes


class State(enum.Enum):
    BRINGUP = 'BRINGUP'
    STANCE_WAIT = 'STANCE_WAIT'
    ENGAGE = 'ENGAGE'
    BLEND = 'BLEND'
    READY = 'READY'
    WAIT_FOR_BOX = 'WAIT_FOR_BOX'
    GOTO = 'GOTO'                       # debug: drive to an operator goal (Foxglove), turn to its heading, stand
    MANUAL = 'MANUAL'                   # the PS5 pad took over (L1 held): navigation paused until reset
    NAV_TO_PREGRASP = 'NAV_TO_PREGRASP'
    ALIGN = 'ALIGN'
    SETTLE = 'SETTLE'
    REACH = 'REACH'
    GRASP = 'GRASP'
    LIFT = 'LIFT'
    NAV_TO_PLACE = 'NAV_TO_PLACE'
    LOWER = 'LOWER'
    RELEASE = 'RELEASE'
    RETRACT = 'RETRACT'
    BACK_OFF = 'BACK_OFF'
    DONE = 'DONE'
    FAILED = 'FAILED'
    ESTOP = 'ESTOP'


class Event(enum.Enum):
    START_OK = 'start'
    BOX_LOST = 'box lost'
    ARRIVED = 'arrived'
    ALIGNED = 'aligned'
    SETTLED = 'settled'
    STEP_DONE = 'step done'
    TIMEOUT = 'timeout'
    FAIL = 'failure'
    ESTOP = 'estop'
    RESET = 'reset'
    IN_STANCE = 'walker in stance'
    ENGAGE_REQ = 'engage requested'
    GATE_OK = 'gate accepted'
    GATE_REFUSED = 'gate refused'
    LIVE = 'walker live'
    RELEASED = 'hook released'
    WALKER_DOWN = 'walker down'
    GOAL = 'goal received'
    MANUAL = 'pad override'


S, E = State, Event
# The ONE transition table: state -> {event: next state}.
TRANSITIONS = {
    S.BRINGUP: {E.IN_STANCE: S.STANCE_WAIT},
    S.STANCE_WAIT: {E.ENGAGE_REQ: S.ENGAGE},
    S.ENGAGE: {E.GATE_OK: S.BLEND, E.GATE_REFUSED: S.STANCE_WAIT, E.LIVE: S.READY},
    S.BLEND: {E.LIVE: S.READY},
    S.READY: {E.RELEASED: S.WAIT_FOR_BOX},
    S.WAIT_FOR_BOX: {E.START_OK: S.NAV_TO_PREGRASP, E.GOAL: S.GOTO},
    S.GOTO: {E.ARRIVED: S.DONE, E.GOAL: S.GOTO},
    S.NAV_TO_PREGRASP: {E.ARRIVED: S.ALIGN, E.BOX_LOST: S.WAIT_FOR_BOX},
    S.ALIGN: {E.ALIGNED: S.SETTLE, E.BOX_LOST: S.WAIT_FOR_BOX},
    S.SETTLE: {E.SETTLED: S.REACH, E.BOX_LOST: S.WAIT_FOR_BOX},
    S.REACH: {E.STEP_DONE: S.GRASP},
    S.GRASP: {E.STEP_DONE: S.LIFT},
    S.LIFT: {E.STEP_DONE: S.NAV_TO_PLACE},
    S.NAV_TO_PLACE: {E.ARRIVED: S.LOWER},
    S.LOWER: {E.STEP_DONE: S.RELEASE},
    S.RELEASE: {E.STEP_DONE: S.RETRACT},
    S.RETRACT: {E.STEP_DONE: S.BACK_OFF},
    S.BACK_OFF: {E.STEP_DONE: S.DONE},
    S.DONE: {E.GOAL: S.GOTO}, S.FAILED: {}, S.ESTOP: {}, S.MANUAL: {},
}
# Valid from any state (looked up when the state's own row has no entry).
GLOBAL = {E.ESTOP: S.ESTOP, E.RESET: S.WAIT_FOR_BOX, E.TIMEOUT: S.FAILED, E.FAIL: S.FAILED,
          E.WALKER_DOWN: S.FAILED, E.MANUAL: S.MANUAL}
BRINGUP_STATES = (S.BRINGUP, S.STANCE_WAIT, S.ENGAGE, S.BLEND, S.READY)
WALKER_DOWN_PHASES = ('damped', 'stopped')

MOVING_STATES = (S.NAV_TO_PREGRASP, S.ALIGN, S.NAV_TO_PLACE, S.GOTO)

# Manipulation plans: state -> ordered segments. ('arm', keyframe) | ('hands', 'closed'|'open') | ('wait', 's')
PLANS = {
    S.REACH: [('arm', 'pregrasp'), ('arm', 'grasp')],
    S.GRASP: [('hands', 'closed')],
    S.LIFT: [('arm', 'lift')],
    S.LOWER: [('arm', 'place')],
    S.RELEASE: [('hands', 'open')],
    S.RETRACT: [('arm', 'default')],
    # vx >= 0 only (no backwards gait): "back off" is standing still for backoff_time, then DONE.
    S.BACK_OFF: [('wait', 'backoff_time')],
}


@dataclass
class Params:
    # The crate does not move: its pose is kept in odom and only refreshed by occasional detections (the
    # detector may run at ~1-2 Hz). Navigation and ALIGN run on that world-frame estimate plus odometry;
    # they need fresh detections only to correct the heading, not to keep moving.
    start_frames: int = 2               # crate detections (consecutive, within visible_timeout) before a start
    auto_start: bool = False
    standoff: float = 0.55              # pelvis-to-crate-centre distance for the pre-grasp [m]
    # 'axis': stand on the side whose normal is the crate's `pregrasp_axis` (y = the 0.449 m long-side
    # face, for a two-handed grasp), choosing the sign facing the robot (yaw is only known mod pi).
    # 'line': stand on the robot->crate line instead.
    pregrasp_mode: str = 'axis'
    pregrasp_axis: str = 'y'
    crate_avg_window: float = 5.0       # s; crate estimate = mean of the DETECTIONS in this window (latency/noise)
    goal_update_dist: float = 0.15      # re-publish goal if the crate moved more than this
    visible_timeout: float = 3.0        # no detection for this long = not currently seen (no goal refresh)
    lost_timeout: float = 10.0          # ... for this long = lost -> stand (WAIT_FOR_BOX)
    arrive_xy: float = 0.15
    arrive_yaw: float = 0.15
    planner_arrive_xy: float = 0.35     # accept the planner's GOAL_REACHED only this close
    nav_timeout: float = 120.0
    planner_cmd_timeout: float = 0.5
    # ALIGN (direct P control, no planner)
    align_kp_w: float = 1.0
    align_wz_max: float = 0.4
    align_wz_min: float = 0.0           # some gaits ignore tiny yaw commands
    align_kp_v: float = 0.5
    align_vx_max: float = 0.15
    align_walk_bearing: float = 0.4     # only walk forward if |bearing| below this [rad]
    align_yaw_tol: float = 0.08
    align_dist_tol: float = 0.05
    align_hold: float = 0.5             # within tolerance for this long
    align_too_close: float = 0.15       # closer than standoff - this: cannot back up -> FAILED
    align_timeout: float = 30.0
    settle_time: float = 1.5
    # False: no arm/hand stage -- after SETTLE the task is DONE, standing (the vendor `mc` walker owns the arms)
    manipulation: bool = True
    # True: start in BRINGUP and run the teleop's hanging-to-walking handover through the walker's
    # phases (on-robot RL walker). False: start in WAIT_FOR_BOX (vendor `mc` walker, already standing).
    bringup: bool = False
    engage_timeout: float = 5.0
    # GOTO (operator goal): planner to the position, then turn in place to the goal heading
    goto_xy_tol: float = 0.15
    goto_yaw_tol: float = 0.10
    goto_timeout: float = 120.0         # ENGAGE: no gate verdict / blend within this -> back to STANCE_WAIT
    # manipulation
    arm_default: List[float] = field(default_factory=lambda: [
        0.3, 0.2, 0.0, -0.8, 0.0, 0.0, 0.0, 0.3, -0.2, 0.0, -0.8, 0.0, 0.0, 0.0])
    arm_move_s: float = 3.0
    arm_move_s_by_key: dict = field(default_factory=dict)
    arm_tol: float = 0.08               # rad
    arm_settle_timeout: float = 4.0     # allowed beyond the move duration
    hand_closed: List[float] = field(default_factory=lambda: [1.0] * 20)   # TODO-calibrate
    hand_time: float = 1.5
    backoff_time: float = 1.0
    place_pose: Tuple[float, float, float] = (1.0, -1.0, 0.0)
    place_xy_tol: float = 0.25


@dataclass
class Inputs:
    t: float
    robot: Optional[Tuple[float, float, float]] = None     # base (x, y, yaw) in odom
    crate: Optional[Tuple[float, float, float]] = None     # latest crate (x, y, yaw) in odom
    crate_age: float = math.inf                            # s since the last crate message
    crate_frames: int = 0                                  # monotonic count of crate messages
    q_arm: Optional[List[float]] = None
    planner_cmd: Optional[Tuple[float, float]] = None      # (vx, wz) from /mpc/cmd_vel
    planner_age: float = math.inf
    nav_state: str = ''
    estop: bool = False
    manual: bool = False                                   # the pad holds the walker (L1): pause navigation
    walker_phase: Optional[str] = None                     # ramp|glide|stance|blend|live|damped|stopped
    engage_result: Optional[dict] = None                   # walker's last gate verdict {verdict, reason, seq?}


@dataclass
class Outputs:
    state: State
    vx: float = 0.0
    wz: float = 0.0
    arm: Optional[List[float]] = None       # None = policy default (empty arm_cmd)
    hand: Optional[List[float]] = None      # None = policy default (open)
    goal: Optional[Tuple[float, float, float]] = None   # publish on /global_goal when not None
    engage: bool = False                    # one-shot: ask the walker to engage (only in ENGAGE)


class PnpFsm:
    def __init__(self, params: Params, keyframes: Keyframes,
                 log: Callable[[str], None] = print):
        self.p, self.kf, self.log = params, keyframes, log
        self.state = State.BRINGUP if params.bringup else State.WAIT_FOR_BOX
        self.start_pending = params.auto_start
        self._engage_pending = False
        self._released_pending = False
        self._goal_req = None               # operator goal (x, y, yaw) in odom, consumed by update()
        self._goto_turning = False
        self._engage_sent = False
        self._gate_seen = None              # engage_result object at the time ENGAGE was entered
        self._enter_t = 0.0
        self._reset_flag = False
        self._seen = 0                      # consecutive crate frames
        self._last_frames = 0
        self._goal = None                   # current published goal
        self._goal_crate = None             # crate xy the goal was computed from
        self._crate_est = None
        self._samples = deque()
        self._aligned_since = None
        self._seg = 0
        self._motion: Optional[ArmMotion] = None
        self._seg_t0 = 0.0
        self._arm_hold: Optional[List[float]] = None
        self._hand_hold: Optional[List[float]] = None
        self._last_arm_cmd: Optional[List[float]] = None
        self._pending_goal = None
        self._align_cmd = (0.0, 0.0)

    def _average(self, t, crate):
        """Mean of the crate samples in the last crate_avg_window s (yaw: circular mean mod pi).
        TODO: crate pose has ~0.3 s latency; while walking/turning it biases the estimate, so the
        goal is best computed from the stationary WAIT/SETTLE windows."""
        self._samples.append((t, crate))
        while self._samples and t - self._samples[0][0] > self.p.crate_avg_window:
            self._samples.popleft()
        n = len(self._samples)
        x = sum(c[0] for _, c in self._samples) / n
        y = sum(c[1] for _, c in self._samples) / n
        sx = sum(math.sin(2 * c[2]) for _, c in self._samples)
        cx = sum(math.cos(2 * c[2]) for _, c in self._samples)
        return (x, y, math.atan2(sx, cx) / 2.0)

    # ---------------- external requests
    def request_start(self):
        self.start_pending = True

    def request_reset(self):
        self._reset_flag = True

    def request_goal(self, goal_odom):
        """Operator goal (x, y, yaw) in odom (debug; e.g. a pre-grasp pose clicked in the crate frame)."""
        self._goal_req = tuple(float(v) for v in goal_odom)

    def request_engage(self):
        """Operator: feet flat, robot upright under the gantry -> hand the legs to the policy."""
        self._engage_pending = True

    def request_released(self):
        """Operator: the hook is released, the robot stands on its own under the policy."""
        self._released_pending = True

    # ---------------- transition machinery
    def _next(self, event: Event) -> Optional[State]:
        row = TRANSITIONS[self.state]
        return row.get(event, GLOBAL.get(event))

    def _fire(self, event: Event, reason: str, inp: Inputs) -> bool:
        nxt = self._next(event)
        if nxt is None or (nxt == self.state and event != Event.RESET):
            return False
        if nxt == S.REACH and not self.p.manipulation:
            nxt, reason = S.DONE, reason + '; manipulation off'
        if event == Event.RESET and self.p.bringup and inp.walker_phase != 'live':
            nxt, reason = S.BRINGUP, reason + f'; walker {inp.walker_phase or "absent"}, bring-up first'
        self.log(f'PNP: {self.state.name} -> {nxt.name} ({reason})')
        prev, self.state = self.state, nxt
        self._enter(prev, inp)
        return True

    def _enter(self, prev: State, inp: Inputs):
        self._enter_t = inp.t
        self._aligned_since = None
        self._seg = 0
        self._motion = None
        self._pending_goal = None
        self._align_cmd = (0.0, 0.0)
        if self.state == State.GOTO:
            self._goto_turning = False
            self._pending_goal = self._goal
        if self.state in (State.BRINGUP, State.STANCE_WAIT):
            self._engage_pending = False
        elif self.state == State.ENGAGE:
            self._engage_sent = False
            self._gate_seen = inp.engage_result
        elif self.state == State.READY:
            self._released_pending = False
        if self.state == State.WAIT_FOR_BOX:
            self._arm_hold = self._hand_hold = self._last_arm_cmd = None
            self._seen = 0
            self._samples.clear()
            self._goal = self._goal_crate = None
            self.start_pending = self.p.auto_start      # a new /pnp/start is needed after a loss/reset
        elif self.state == State.NAV_TO_PREGRASP:
            self.start_pending = False
            self._goal = self._goal_crate = None
        elif self.state == State.NAV_TO_PLACE:
            self._goal = tuple(self.p.place_pose)
            self._pending_goal = self._goal
        elif self.state in PLANS:
            self._start_segment(inp)

    # ---------------- manipulation segments
    def _start_segment(self, inp: Inputs):
        kind, arg = PLANS[self.state][self._seg]
        self._seg_t0 = inp.t
        self._motion = None
        if kind == 'arm':
            dur = self.p.arm_move_s_by_key.get(arg, self.p.arm_move_s)
            q0 = self._last_arm_cmd or inp.q_arm or self.kf.default
            self._motion = ArmMotion(q0, self.kf.get(arg), dur, inp.t)
        elif kind == 'hands':
            self._hand_hold = list(self.p.hand_closed) if arg == 'closed' else None

    def _segment_tick(self, inp: Inputs) -> Optional[Tuple[Event, str]]:
        kind, arg = PLANS[self.state][self._seg]
        done, why = False, ''
        if kind == 'arm':
            m = self._motion
            self._arm_hold = self._last_arm_cmd = m.command(inp.t)
            if m.finished(inp.t) and m.reached(inp.q_arm, self.p.arm_tol):
                done, why = True, f'arm keyframe {arg!r} reached'
            elif m.elapsed(inp.t) > m.duration + self.p.arm_settle_timeout:
                return Event.TIMEOUT, f'arm keyframe {arg!r} not reached within tolerance {self.p.arm_tol} rad'
        else:
            dur = self.p.hand_time if kind == 'hands' else getattr(self.p, arg)
            if inp.t - self._seg_t0 >= dur:
                done, why = True, f'{kind} {arg} done'
        if not done:
            return None
        self._seg += 1
        if self._seg >= len(PLANS[self.state]):
            return Event.STEP_DONE, why
        self._start_segment(inp)
        return None

    # ---------------- main update
    def update(self, inp: Inputs) -> Outputs:
        visible = inp.crate is not None and inp.crate_age < self.p.visible_timeout
        new = inp.crate_frames > self._last_frames
        if visible:
            self._seen += max(0, inp.crate_frames - self._last_frames)
            if new or self._crate_est is None:          # average detections, not ticks
                self._crate_est = self._average(inp.t, inp.crate)
        else:
            self._seen = 0
        self._last_frames = inp.crate_frames
        lost = (inp.crate is None) or inp.crate_age > self.p.lost_timeout

        # global events first
        if inp.manual and self.state not in (State.MANUAL, State.ESTOP) and self.state not in BRINGUP_STATES:
            self._fire(Event.MANUAL, 'PS5 pad holds the walker (L1); navigation paused until reset', inp)
        if inp.estop and self.state != State.ESTOP:
            self._fire(Event.ESTOP, 'estop asserted', inp)
        elif self._reset_flag:
            self._reset_flag = False
            if inp.estop:
                self.log('PNP: reset ignored (estop still asserted)')
            else:
                self._fire(Event.RESET, 'reset requested', inp)

        if (self.p.bringup and inp.walker_phase in WALKER_DOWN_PHASES
                and self.state not in (State.FAILED, State.ESTOP, State.BRINGUP)):
            self._fire(Event.WALKER_DOWN, f'walker {inp.walker_phase}', inp)

        if self._goal_req is not None:
            goal, self._goal_req = self._goal_req, None
            if self.state == State.GOTO:
                self._goal, self._pending_goal, self._goto_turning = goal, goal, False
                self._enter_t, self._aligned_since = inp.t, None
                self.log(f'PNP: GOTO goal replaced ({goal[0]:.2f}, {goal[1]:.2f}, yaw {math.degrees(goal[2]):.0f} deg)')
            elif self._next(Event.GOAL) == State.GOTO:
                self._goal = goal
                self._fire(Event.GOAL, f'operator goal ({goal[0]:.2f}, {goal[1]:.2f}, '
                                       f'yaw {math.degrees(goal[2]):.0f} deg)', inp)
                self._goal = goal
                self._pending_goal = goal
            else:
                self.log(f'PNP: goal ignored in {self.state.name} (only from WAIT_FOR_BOX / DONE / GOTO)')

        st = self.state
        if st == State.GOTO:
            self._tick_goto(inp)
        elif st in BRINGUP_STATES:
            self._tick_bringup(inp)
        elif st == State.WAIT_FOR_BOX:
            if self.start_pending and self._seen >= self.p.start_frames and inp.robot is not None:
                self._fire(Event.START_OK, f'box seen {self._seen} frames, start requested', inp)
        elif st == State.NAV_TO_PREGRASP:
            self._tick_nav_pregrasp(inp, visible, lost)
        elif st == State.ALIGN:
            self._tick_align(inp, visible, lost)
        elif st == State.SETTLE:
            if lost:
                self._fire(Event.BOX_LOST, f'box not seen for {self.p.lost_timeout}s', inp)
            elif inp.t - self._enter_t >= self.p.settle_time:
                self._fire(Event.SETTLED, f'stood still {self.p.settle_time}s', inp)
        elif st == State.NAV_TO_PLACE:
            self._tick_nav_place(inp)
        elif st in PLANS:
            r = self._segment_tick(inp)
            if r:
                self._fire(r[0], r[1], inp)

        return self._outputs(inp)

    def _tick_bringup(self, inp: Inputs):
        ph, st = inp.walker_phase, self.state
        if st == State.BRINGUP:
            self._engage_pending = False
            if ph == 'stance':
                self._fire(Event.IN_STANCE, 'walker holds the stance; lower the gantry until the feet are flat', inp)
            elif ph in ('blend', 'live'):
                # already handed over (e.g. this machine restarted): follow the walker
                self._fire(Event.IN_STANCE, f'walker already {ph}', inp)
                self._fire(Event.ENGAGE_REQ, 'walker already engaged', inp)
        elif st == State.STANCE_WAIT:
            if ph in ('blend', 'live'):
                self._fire(Event.ENGAGE_REQ, f'walker already {ph}', inp)
            elif self._engage_pending:
                self._engage_pending = False
                if ph == 'stance':
                    self._fire(Event.ENGAGE_REQ, 'operator: feet flat, engage', inp)
                else:
                    self.log(f'PNP: engage ignored (walker phase {ph})')
        elif st == State.ENGAGE:
            res = inp.engage_result
            fresh = res is not None and res is not self._gate_seen and res != self._gate_seen
            if ph == 'live':
                self._fire(Event.LIVE, 'walker live', inp)
            elif ph == 'blend':
                self._fire(Event.GATE_OK, 'pose parity OK, blending the policy in', inp)
            elif fresh and str(res.get('verdict', '')).startswith('refused'):
                self._fire(Event.GATE_REFUSED, f"gate refused: {res.get('reason', '')}", inp)
            elif inp.t - self._enter_t > self.p.engage_timeout:
                self._fire(Event.GATE_REFUSED, f'no gate verdict within {self.p.engage_timeout}s', inp)
        elif st == State.BLEND:
            if ph == 'live':
                self._fire(Event.LIVE, 'policy has the legs; release the hook, then confirm', inp)
        elif st == State.READY:
            if self._released_pending:
                self._released_pending = False
                self._fire(Event.RELEASED, 'operator: hook released', inp)

    def _outputs(self, inp: Inputs) -> Outputs:
        st = self.state
        out = Outputs(st, arm=self._arm_hold, hand=self._hand_hold)
        if st == State.ENGAGE and not self._engage_sent:
            out.engage, self._engage_sent = True, True
        if st == State.GOTO:
            out.goal, self._pending_goal = self._pending_goal, None
            if self._goto_turning:
                out.vx, out.wz = self._align_cmd
            elif inp.planner_cmd is not None and inp.planner_age < self.p.planner_cmd_timeout:
                out.vx, out.wz = max(0.0, inp.planner_cmd[0]), inp.planner_cmd[1]
        elif st in (State.NAV_TO_PREGRASP, State.NAV_TO_PLACE):
            out.goal, self._pending_goal = self._pending_goal, None
            # the crate is static and kept in odom: walking only needs it not LOST, not freshly seen
            visible_ok = st == State.NAV_TO_PLACE or (
                inp.crate is not None and inp.crate_age < self.p.lost_timeout)
            if (visible_ok and inp.planner_cmd is not None
                    and inp.planner_age < self.p.planner_cmd_timeout):
                out.vx, out.wz = max(0.0, inp.planner_cmd[0]), inp.planner_cmd[1]
        elif st == State.ALIGN:
            out.vx, out.wz = self._align_cmd
        return out

    # ---------------- per-state logic
    def _tick_nav_pregrasp(self, inp, visible, lost):
        if lost:
            self._fire(Event.BOX_LOST, f'box not seen for {self.p.lost_timeout}s', inp)
            return
        if inp.robot is None or self._crate_est is None:
            return
        cxy = self._crate_est[:2]
        if self._goal is None or (visible and math.hypot(cxy[0] - self._goal_crate[0],
                                                          cxy[1] - self._goal_crate[1]) > self.p.goal_update_dist):
            self._goal = g.pregrasp_pose(inp.robot[:2], cxy, self.p.standoff, self.p.pregrasp_mode,
                                         self._crate_est[2], self.p.pregrasp_axis)
            self._goal_crate = cxy
            self._pending_goal = self._goal
            self.log(f'PNP: pre-grasp goal ({self._goal[0]:.2f}, {self._goal[1]:.2f}, '
                     f'yaw {math.degrees(self._goal[2]):.0f} deg) for crate ({cxy[0]:.2f}, {cxy[1]:.2f})')
        d, dyaw = g.pose_error(inp.robot, self._goal)
        if d < self.p.arrive_xy and dyaw < self.p.arrive_yaw:
            self._fire(Event.ARRIVED, f'within {d:.2f} m / {dyaw:.2f} rad of pre-grasp', inp)
        elif inp.nav_state == 'GOAL_REACHED' and d < self.p.planner_arrive_xy:
            self._fire(Event.ARRIVED, f'planner reports goal reached ({d:.2f} m)', inp)
        elif inp.t - self._enter_t > self.p.nav_timeout:
            self._fire(Event.TIMEOUT, 'navigation timeout', inp)

    def _tick_goto(self, inp):
        """Planner to the goal position, then turn in place to its heading (the planner does not)."""
        self._align_cmd = (0.0, 0.0)
        if inp.robot is None or self._goal is None:
            return
        p = self.p
        d = math.hypot(self._goal[0] - inp.robot[0], self._goal[1] - inp.robot[1])
        if not self._goto_turning and (d < p.goto_xy_tol or
                                       (inp.nav_state == 'GOAL_REACHED' and d < p.planner_arrive_xy)):
            self._goto_turning = True
            self.log(f'PNP: goal position reached ({d:.2f} m), turning to the goal heading')
        if self._goto_turning:
            err = math.atan2(math.sin(self._goal[2] - inp.robot[2]), math.cos(self._goal[2] - inp.robot[2]))
            if abs(err) < p.goto_yaw_tol:
                if self._aligned_since is None:
                    self._aligned_since = inp.t
                elif inp.t - self._aligned_since >= p.align_hold:
                    self._fire(Event.ARRIVED, f'at goal ({d:.2f} m, {err:+.2f} rad)', inp)
                return
            self._aligned_since = None
            wz = max(-p.align_wz_max, min(p.align_wz_max, p.align_kp_w * err))
            if 0 < abs(wz) < p.align_wz_min:
                wz = math.copysign(p.align_wz_min, err)
            self._align_cmd = (0.0, wz)
        if inp.t - self._enter_t > p.goto_timeout:
            self._fire(Event.TIMEOUT, 'goal timeout', inp)

    def _tick_nav_place(self, inp):
        if inp.robot is None:
            return
        pp = self.p.place_pose
        d = math.hypot(pp[0] - inp.robot[0], pp[1] - inp.robot[1])
        if d < self.p.place_xy_tol or (inp.nav_state == 'GOAL_REACHED' and d < self.p.planner_arrive_xy):
            self._fire(Event.ARRIVED, f'at place pose ({d:.2f} m)', inp)
        elif inp.t - self._enter_t > self.p.nav_timeout:
            self._fire(Event.TIMEOUT, 'navigation timeout', inp)

    def _tick_align(self, inp, visible, lost):
        self._align_cmd = (0.0, 0.0)
        if lost:
            self._fire(Event.BOX_LOST, f'box not seen for {self.p.lost_timeout}s', inp)
            return
        if inp.robot is None or self._crate_est is None:
            self._aligned_since = None
            return
        p = self.p
        f, l, bearing, dist = g.crate_in_base(inp.robot, self._crate_est[:2])
        err = dist - p.standoff
        if err < -p.align_too_close:
            self._fire(Event.FAIL, f'too close to the crate ({dist:.2f} m < standoff {p.standoff} m; cannot back up)', inp)
            return
        wz = max(-p.align_wz_max, min(p.align_wz_max, p.align_kp_w * bearing))
        if abs(bearing) > p.align_yaw_tol and 0 < abs(wz) < p.align_wz_min:
            wz = math.copysign(p.align_wz_min, bearing)
        vx = 0.0
        if abs(bearing) < p.align_walk_bearing and err > p.align_dist_tol:
            vx = max(0.0, min(p.align_vx_max, p.align_kp_v * err))
        ok = abs(bearing) < p.align_yaw_tol and err < p.align_dist_tol
        if ok:
            self._align_cmd = (0.0, 0.0)
            if self._aligned_since is None:
                self._aligned_since = inp.t
            elif inp.t - self._aligned_since >= p.align_hold:
                self._fire(Event.ALIGNED, f'bearing {bearing:.2f} rad, dist {dist:.2f} m', inp)
            return
        self._aligned_since = None
        self._align_cmd = (vx, wz)
        if inp.t - self._enter_t > p.align_timeout:
            self._fire(Event.TIMEOUT, 'align timeout', inp)
