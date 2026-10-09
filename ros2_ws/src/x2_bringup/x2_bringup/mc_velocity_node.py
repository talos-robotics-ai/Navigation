"""Feed the X2's vendor walking controller (`mc`, on PC1) a velocity from a ROS Twist topic.

Adapted from tools/x2_teleop/mc_velocity_bridge.py (the PS5 bridge, which keeps running unchanged next
to this): same mc interface -- SetMcInputSource ADD (else MODIFY) then ENABLE, McLocomotionVelocity on
/aima/mc/locomotion/velocity at 50 Hz, success = header.code == 0, caps vx 0.5 / vy 0.3 / wz 0.5.

Differences from the bridge:
* input is a Twist on `input_topic` (/x2/cmd_vel_out: the pnp FSM gate's output, never the raw planner)
  and /estop (Bool, latched while true) instead of TCP + a dead-man;
* input source `talos_nav`, priority 64 -- STRICTLY BELOW the PS5 bridge's `talos_gamepad` (65): the pad
  (L1 dead-man held) always overrides navigation; the priority is clamped to 64 whatever the param says;
* NO mode changes: no SetMcAction, no GetMcAction. The user puts the robot in STAND_DEFAULT with the pad;
* stale input (> stale_s = 0.3 s) -> zeros for 0.3 s, then silence (mc hands control back);
* deadband_mode (mc_gate.py): "lift" raises small nonzero commands to mc's start threshold;
* shutdown sends INPUT_DELETE while the context is still valid (the container installs its own signal
  handlers; rclpy's default SIGINT handler invalidates the context first -- the original's crash).

aimdk_msgs is optional at import: without it the node only logs (nothing can be sent).
`dry_run:=true` registers nothing and publishes nothing; it logs what would be sent.
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from .mc_gate import VelocityGate

try:
    from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader
    from aimdk_msgs.srv import SetMcInputSource
    _AIMDK_ERR = None
except ImportError as _e:   # pragma: no cover
    McLocomotionVelocity = MessageHeader = SetMcInputSource = None
    _AIMDK_ERR = _e

INPUT_ADD, INPUT_MODIFY, INPUT_DELETE = 1001, 1002, 1003
INPUT_ENABLE, INPUT_DISABLE = 2001, 2002
VERB = {INPUT_ADD: 'ADD', INPUT_MODIFY: 'MODIFY', INPUT_DELETE: 'DELETE', INPUT_ENABLE: 'ENABLE', INPUT_DISABLE: 'DISABLE'}
#: the PS5 bridge's talos_gamepad priority is 65; this source must stay below it
MAX_PRIORITY = 64
PUBLISH_HZ = 50.0


class McVelocity(Node):
    def __init__(self, **kw):
        super().__init__('mc_velocity', **kw)
        P = self.declare_parameter
        P('input_topic', '/x2/cmd_vel_out')
        P('estop_topic', '/estop')
        P('source', 'talos_nav')
        P('priority', MAX_PRIORITY)
        P('timeout_ms', 500)
        P('stale_s', 0.3)
        P('max_vx', 0.5)
        P('max_vy', 0.3)
        P('max_wz', 0.5)
        P('deadband_mode', 'lift')
        P('dry_run', False)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self._source, self._dry = str(g('source')), bool(g('dry_run'))
        self._timeout = int(g('timeout_ms'))
        self._priority = int(g('priority'))
        if self._priority > MAX_PRIORITY:
            self.get_logger().warn(f'priority {self._priority} clamped to {MAX_PRIORITY}: must stay below talos_gamepad (65)')
            self._priority = MAX_PRIORITY
        mode = str(g('deadband_mode'))
        if mode not in ('lift', 'zero', 'none'):
            raise ValueError(f"deadband_mode {mode!r}: lift | zero | none")
        self._gate = VelocityGate(float(g('max_vx')), float(g('max_vy')), float(g('max_wz')), float(g('stale_s')), mode)
        self._cmd = None
        self._estop = False
        self._sent = 0
        self._t_log = 0.0
        self._registered = False
        self._pub = self._cli = None

        self.create_subscription(Twist, g('input_topic'), self._on_twist, 10)
        self.create_subscription(Bool, g('estop_topic'), self._on_estop, 10)
        if self._dry:
            self.get_logger().warn('DRY RUN: nothing registered, nothing published')
        elif SetMcInputSource is None:
            self.get_logger().error(f'aimdk_msgs not importable ({_AIMDK_ERR}): cannot command mc; logging only')
        else:
            self._pub = self.create_publisher(McLocomotionVelocity, '/aima/mc/locomotion/velocity', 10)
            self._cli = self.create_client(SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource')
            # An earlier run that died without DELETE leaves the name registered, and a second ADD is
            # refused (code 1): MODIFY updates that registration. Blocking here is fine: the executor
            # that runs this node does not exist yet.
            ok = self._input_source(INPUT_ADD) or self._input_source(INPUT_MODIFY)
            if ok and not self._input_source(INPUT_ENABLE):
                self.get_logger().error('registered but ENABLE failed: mc will DROP the velocity commands')
            self._registered = ok
            if not ok:
                self.get_logger().error('could not register the input source: NOTHING will reach mc')
        self.get_logger().warn(
            f'MC WALKER {"DRY-RUN" if self._dry else "LIVE"}: {g("input_topic")} -> /aima/mc/locomotion/velocity as '
            f'source {self._source!r} priority {self._priority} (< talos_gamepad 65: the PS5 pad overrides). '
            f'caps vx {g("max_vx")} vy {g("max_vy")} wz {g("max_wz")}, deadband {mode}, stale {g("stale_s")} s. '
            f'No mode changes are ever requested.')
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)

    # ------------------------------------------------------------------ mc input source
    def _input_source(self, action):
        if not self._cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('SetMcInputSource service not available')
            return False
        req = SetMcInputSource.Request()
        req.action.value = action
        req.input_source.name = self._source
        req.input_source.priority = self._priority
        req.input_source.timeout = self._timeout
        for _ in range(8):   # cross-board services are flaky; the SDK examples retry too
            req.request.header.stamp = self.get_clock().now().to_msg()
            fut = self._cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=0.25)
            if fut.done():
                resp = fut.result().response
                code, state = int(resp.header.code), int(resp.state.value)
                self.get_logger().info(f'input source {self._source!r} {VERB[action]}: code={code} state={state} '
                                       f'({"OK" if code == 0 else "FAILED"})')
                return code == 0
        self.get_logger().error('SetMcInputSource: no answer')
        return False

    def on_shutdown(self):
        """Call BEFORE rclpy shutdown, with the context valid."""
        if self._registered and rclpy.ok():
            self._registered = False
            self._input_source(INPUT_DELETE)

    # ------------------------------------------------------------------ io
    def _on_twist(self, m):
        self._cmd = (m.linear.x, m.linear.y, m.angular.z, time.monotonic())

    def _on_estop(self, m):
        if bool(m.data) != self._estop:
            self.get_logger().warn(f'ESTOP {"LATCHED: zeros then silence" if m.data else "released"}')
        self._estop = bool(m.data)

    def _tick(self):
        now = time.monotonic()
        was_active = self._gate.active
        out = self._gate.step(now, self._cmd, self._estop)
        if was_active and not self._gate.active:
            self.get_logger().info('released: silent (mc returns to the next input source)')
        if out is not None:
            self._sent += 1
            if self._pub is not None:
                msg = McLocomotionVelocity()
                msg.header = MessageHeader()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.source = self._source
                msg.forward_velocity, msg.lateral_velocity, msg.angular_velocity = out
                self._pub.publish(msg)
        if now - self._t_log >= 2.0 and (out is not None or self._sent):
            self._t_log = now
            shown = 'silent' if out is None else f'vx {out[0]:+.2f} vy {out[1]:+.2f} wz {out[2]:+.2f}'
            self.get_logger().info(f'{"[dry] " if self._dry else ""}{shown} | sent {self._sent}')


def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)   # Ctrl-C -> KeyboardInterrupt, context intact
    n = McVelocity()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.on_shutdown()
        n.destroy_node()
        rclpy.try_shutdown()
