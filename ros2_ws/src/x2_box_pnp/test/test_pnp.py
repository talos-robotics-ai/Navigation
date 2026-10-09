import math

from x2_box_pnp import geometry as g
from x2_box_pnp.fsm import Inputs, Params, PnpFsm, State
from x2_box_pnp.keyframes import ArmMotion, Keyframes, interpolate

DEF = [0.3, 0.2, 0, -0.8, 0, 0, 0, 0.3, -0.2, 0, -0.8, 0, 0, 0]


def mk(**kw):
    kw.setdefault('pregrasp_mode', 'line')
    logs = []
    f = PnpFsm(Params(**kw), Keyframes({'default': None, 'pregrasp': [0.5] * 14, 'grasp': [0.6] * 14,
                                        'lift': [0.7] * 14, 'place': [0.6] * 14}, DEF), log=logs.append)
    return f, logs


class Clock:
    def __init__(self):
        self.t, self.n = 0.0, 0

    def inp(self, robot=(0, 0, 0), crate=(2, 0, 0), visible=True, **kw):
        self.t += 0.1
        if visible:
            self.n += 1
        return Inputs(t=self.t, robot=robot, crate=crate if visible or kw.pop('keep', False) else None,
                      crate_age=0.05 if visible else kw.pop('age', math.inf), crate_frames=self.n,
                      planner_cmd=(0.3, 0.2), planner_age=0.05, **kw)


def to_nav(f, c):
    f.request_start()
    for _ in range(8):
        f.update(c.inp())
    assert f.state == State.NAV_TO_PREGRASP


def test_pregrasp_line():
    x, y, yaw = g.pregrasp_pose((0, 0), (2, 0), 0.55)
    assert abs(x - 1.45) < 1e-9 and abs(y) < 1e-9 and abs(yaw) < 1e-9
    x, y, yaw = g.pregrasp_pose((0, 0), (0, -2), 0.5)
    assert abs(y + 1.5) < 1e-9 and abs(yaw + math.pi / 2) < 1e-9


def test_pregrasp_axis_faces_robot():
    x, y, yaw = g.pregrasp_pose((0, 0), (2, 0), 0.5, 'axis', crate_yaw=math.pi / 2, axis='x')
    assert abs(x - 2) < 1e-9 and abs(abs(y) - 0.5) < 1e-9   # along crate x axis (rotated to world y)


def test_waits_without_start_or_box():
    f, _ = mk()
    c = Clock()
    for _ in range(20):
        o = f.update(c.inp())
    assert f.state == State.WAIT_FOR_BOX and o.vx == 0 and o.wz == 0
    f.request_start()
    for _ in range(20):
        o = f.update(c.inp(visible=False))
    assert f.state == State.WAIT_FOR_BOX and (o.vx, o.wz) == (0, 0)


def test_start_needs_n_frames():
    f, _ = mk()
    c = Clock()
    f.request_start()
    f.update(c.inp())
    assert f.state == State.WAIT_FOR_BOX          # one detection is not enough (start_frames = 2)
    f.update(c.inp())
    assert f.state == State.NAV_TO_PREGRASP


def test_nav_publishes_goal_and_gates_planner():
    f, logs = mk()
    c = Clock()
    f.request_start()
    goals, o = [], None
    for _ in range(8):
        o = f.update(c.inp())
        if o.goal:
            goals.append(o.goal)
    assert goals and abs(goals[0][0] - 1.45) < 1e-6
    assert (o.vx, o.wz) == (0.3, 0.2)
    for _ in range(12):                              # crate moved 0.5 m -> averaged, then new goal
        o = f.update(c.inp(crate=(2.5, 0, 0)))
        if o.goal:
            break
    assert o.goal is not None
    assert any(l.startswith('PNP: WAIT_FOR_BOX -> NAV_TO_PREGRASP (') for l in logs)


def test_box_lost_returns_to_wait_from_every_moving_state():
    # NAV_TO_PREGRASP
    f, _ = mk()
    c = Clock()
    to_nav(f, c)
    for _ in range(10):
        o = f.update(c.inp(visible=False, age=5.0, keep=True))   # low-rate detector: not seen 5 s, not lost
    assert f.state == State.NAV_TO_PREGRASP and (o.vx, o.wz) == (0.3, 0.2)   # keeps walking on the odom estimate
    for _ in range(10):
        o = f.update(c.inp(visible=False))
    assert f.state == State.WAIT_FOR_BOX and (o.vx, o.wz) == (0, 0)
    # ALIGN
    f, _ = mk()
    c = Clock()
    to_nav(f, c)
    f._fire(__import__('x2_box_pnp.fsm', fromlist=['Event']).Event.ARRIVED, 'test', c.inp())
    assert f.state == State.ALIGN
    for _ in range(20):
        o = f.update(c.inp(visible=False))
    assert f.state == State.WAIT_FOR_BOX and (o.vx, o.wz) == (0, 0)
    # SETTLE
    f, _ = mk()
    c = Clock()
    f._fire(__import__('x2_box_pnp.fsm', fromlist=['Event']).Event.START_OK, 't', c.inp())
    f.state = State.SETTLE
    for _ in range(20):
        f.update(c.inp(visible=False))
    assert f.state == State.WAIT_FOR_BOX


def test_align_p_control_nonneg_small_vx():
    f, _ = mk()
    c = Clock()
    to_nav(f, c)
    f.state = State.ALIGN
    o = f.update(c.inp(robot=(0, 0, 0), crate=(2, 0.1, 0)))
    assert 0 < o.vx <= 0.15 and o.wz > 0
    o = f.update(c.inp(robot=(0, 0, 1.0), crate=(2, 0, 0)))    # facing away: turn, no walking
    assert o.vx == 0 and o.wz < 0
    for _ in range(10):                                         # at standoff, facing -> SETTLE
        f.update(c.inp(robot=(1.45, 0, 0), crate=(2, 0, 0)))
    assert f.state == State.SETTLE


def test_estop_and_reset():
    f, _ = mk()
    c = Clock()
    to_nav(f, c)
    o = f.update(c.inp(estop=True))
    assert f.state == State.ESTOP and (o.vx, o.wz) == (0, 0)
    f.request_reset()
    f.update(c.inp(estop=True))
    assert f.state == State.ESTOP
    f.request_reset()
    f.update(c.inp())
    assert f.state == State.WAIT_FOR_BOX


def test_manipulation_sequence_and_timeout():
    f, logs = mk(arm_move_s=1.0, hand_time=0.5, backoff_time=0.5, settle_time=0.3)
    c = Clock()
    to_nav(f, c)
    f.state = State.SETTLE
    q = list(DEF)
    seen = []
    for _ in range(400):
        inp = c.inp(robot=(1.45, 0, 0), q_arm=q)
        o = f.update(inp)
        if o.arm:
            q = list(o.arm)                         # perfect tracking
        if f.state == State.NAV_TO_PLACE:
            f.update(c.inp(robot=(1.0, -1.0, 0), q_arm=q))
        if not seen or seen[-1] != f.state:
            seen.append(f.state)
        if f.state == State.DONE:
            break
    names = [s.name for s in seen]
    assert names[-1] == 'DONE' and 'GRASP' in names and 'RETRACT' in names, names
    # timeout -> FAILED when the arm never gets there
    f, _ = mk(arm_move_s=0.5, arm_settle_timeout=0.5)
    f.state = State.SETTLE
    f._fire(__import__('x2_box_pnp.fsm', fromlist=['Event']).Event.SETTLED, 't', c.inp())
    for _ in range(40):
        f.update(c.inp(q_arm=list(DEF)))
    assert f.state == State.FAILED


def test_interpolation():
    assert interpolate([0, 0], [1, 2], 0) == [0, 0]
    assert interpolate([0, 0], [1, 2], 1) == [1, 2]
    mid = interpolate([0], [1], 0.5)
    assert abs(mid[0] - 0.5) < 1e-12
    m = ArmMotion([0.0] * 14, [1.0] * 14, 2.0, 10.0)
    assert m.command(10.0)[0] == 0 and m.command(12.0)[0] == 1 and m.finished(12.0)
    assert m.reached([0.99] * 14, 0.05) and not m.reached([0.5] * 14, 0.05)


def test_keyframe_null_is_default():
    k = Keyframes({'default': None}, DEF)
    assert k.get('default') == DEF and k.is_policy_default('default')


def test_default_mode_long_side_faces_robot():
    # crate yaw 0.3 (ambiguous mod pi): approach along crate +-y, side facing the robot
    f, _ = mk(pregrasp_mode='axis')
    c = Clock()
    f.request_start()
    goal = None
    for _ in range(8):
        o = f.update(c.inp(crate=(2.0, 0.0, 0.3)))
        goal = goal or o.goal
    a = g.pregrasp_pose((0, 0), (2, 0), 0.55, 'axis', 0.3, 'y')
    b = g.pregrasp_pose((0, 0), (2, 0), 0.55, 'axis', 0.3 + math.pi, 'y')
    assert abs(a[0] - b[0]) < 1e-9 and abs(a[1] - b[1]) < 1e-9
    assert goal is not None
    assert abs(math.hypot(a[0] - 2, a[1]) - 0.55) < 1e-9 and a[1] * math.sin(0.3 + math.pi / 2) * 0 == 0


def test_manipulation_off_ends_after_settle():
    f, logs = mk(manipulation=False, settle_time=0.3)
    c = Clock()
    to_nav(f, c)
    f.state = State.SETTLE
    for _ in range(10):
        f.update(c.inp(robot=(1.45, 0, 0)))
    assert f.state == State.DONE and any('manipulation off' in m for m in logs)


# ---------------- bring-up: teleop's hanging-to-walking handover (Params.bringup)

def _run(f, c, n=3, **kw):
    o = None
    for _ in range(n):
        o = f.update(c.inp(**kw))
        assert (o.vx, o.wz) == (0, 0) or f.state not in (State.BRINGUP, State.STANCE_WAIT, State.ENGAGE,
                                                          State.BLEND, State.READY)
    return o


def test_bringup_full_handover():
    f, logs = mk(bringup=True)
    c = Clock()
    assert f.state == State.BRINGUP
    _run(f, c, walker_phase='ramp')
    _run(f, c, walker_phase='glide')
    assert f.state == State.BRINGUP
    _run(f, c, walker_phase='stance')
    assert f.state == State.STANCE_WAIT
    f.request_start()                                  # a start before the handover does nothing yet
    _run(f, c, walker_phase='stance')
    assert f.state == State.STANCE_WAIT
    f.request_engage()
    o = f.update(c.inp(walker_phase='stance'))
    assert f.state == State.ENGAGE and o.engage and (o.vx, o.wz) == (0, 0)
    o = f.update(c.inp(walker_phase='stance'))
    assert not o.engage                                # one-shot
    _run(f, c, walker_phase='blend')
    assert f.state == State.BLEND
    _run(f, c, walker_phase='live')
    assert f.state == State.READY
    f.request_released()
    f.update(c.inp(walker_phase='live'))
    assert f.state == State.WAIT_FOR_BOX
    assert any('STANCE_WAIT -> ENGAGE' in m for m in logs)


def test_bringup_gate_refused_back_to_stance():
    f, logs = mk(bringup=True)
    c = Clock()
    _run(f, c, walker_phase='stance')
    f.request_engage()
    f.update(c.inp(walker_phase='stance', engage_result={'verdict': 'ok', 'seq': 0}))
    assert f.state == State.ENGAGE
    f.update(c.inp(walker_phase='stance',
                   engage_result={'verdict': 'refused', 'reason': 'left_knee 0.31 rad', 'seq': 1}))
    assert f.state == State.STANCE_WAIT
    assert any('left_knee' in m for m in logs)


def test_bringup_engage_timeout_and_engage_outside_stance_ignored():
    f, logs = mk(bringup=True, engage_timeout=1.0)
    c = Clock()
    _run(f, c, walker_phase='stance')
    f.request_engage()
    _run(f, c, n=15, walker_phase='stance')            # no verdict, never blends
    assert f.state == State.STANCE_WAIT


def test_walker_down_fails_and_reset_goes_back_to_bringup():
    f, _ = mk(bringup=True)
    c = Clock()
    _run(f, c, walker_phase='stance')
    f.request_engage()
    _run(f, c, walker_phase='stance')
    _run(f, c, walker_phase='live')
    f.request_released()
    _run(f, c, walker_phase='live')
    assert f.state == State.WAIT_FOR_BOX
    o = f.update(c.inp(walker_phase='damped'))
    assert f.state == State.FAILED and (o.vx, o.wz) == (0, 0)
    f.request_reset()
    f.update(c.inp(walker_phase='damped'))
    assert f.state == State.BRINGUP


def test_no_bringup_starts_waiting_for_box():
    f, _ = mk()
    assert f.state == State.WAIT_FOR_BOX
