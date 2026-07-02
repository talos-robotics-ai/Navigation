#!/usr/bin/env python3
"""Standalone tester for the Unitree G1 NATIVE (factory) walking policy.

Brings the robot up to walking control and sends velocity commands — no ROS, no
navigation stack, no AMO. Use it to confirm the native gait walks (standing +
velocity) before wiring it into the nav stack via cmd_vel_to_unitree_loco_node.

Velocity commands go through the SAME VelocitySmoother (ramp-in + slew + low-pass)
the bridge uses — the high-level analog of AMO's joint filtering (the native gait
is high-level, so there are no joint targets to filter; we smooth the command).

Examples (inside the `unitree` container, robot NIC = enp3s0):
    # stand up ready, then hold (Ctrl-C to damp & exit)
    python3 unitree_gait_test.py --net_if enp3s0 --bring-up

    # bring up, walk forward 0.3 m/s for 5 s, ramp down, stand
    python3 unitree_gait_test.py --net_if enp3s0 --vx 0.3 --duration 5

    # keyboard teleop:  w/s=fwd/back  a/d=left/right  q/e=turn  space=stop  z=quit
    python3 unitree_gait_test.py --net_if enp3s0 --teleop

    # just damp the robot and exit
    python3 unitree_gait_test.py --net_if enp3s0 --damp-only

SAFETY: the native gait and AMO both drive the motors — run only ONE. Suspend or
clear the robot for FSM transitions and keep the hardware e-stop in hand.
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from select import select

from g1_sim_bridge.unitree_loco import (
    FSM_DEFAULT_WALK,
    LocoDriver,
    SmootherConfig,
    VelocitySmoother,
    fsm_name,
)


class Pacer:
    def __init__(self, dt: float):
        self.dt = dt
        self._next = time.perf_counter()

    def wait(self) -> None:
        self._next += self.dt
        s = self._next - time.perf_counter()
        if s > 0:
            time.sleep(s)
        else:
            self._next = time.perf_counter()


def _run_constant(driver, smoother, target, duration, rate_hz, stop):
    """Hold a smoothed constant velocity for `duration`, then ramp down to 0."""
    dt = 1.0 / rate_hz
    hold = max(0.5, 2.0 * dt)
    pacer = Pacer(dt)
    t0 = time.monotonic()
    print(f">> walking cmd={target} for {duration:.1f}s (smoothed) ...")
    while not stop["flag"]:
        elapsed = time.monotonic() - t0
        tgt = target if elapsed < duration else (0.0, 0.0, 0.0)
        vx, vy, wz = smoother.step(*tgt, dt=dt)
        driver.set_velocity(vx, vy, wz, hold)
        # done once we've passed duration AND the ramp-down has settled to ~0
        if elapsed >= duration and abs(vx) < 0.01 and abs(vy) < 0.01 and abs(wz) < 0.01:
            break
        pacer.wait()
    driver.stop_move()
    print(">> stopped (balance-standing).")


def _run_teleop(driver, smoother, caps, rate_hz, stop):
    """Single-key teleop (cbreak). Falls back to line mode if not a TTY."""
    dt = 1.0 / rate_hz
    hold = max(0.5, 2.0 * dt)
    vmax, vymax, wmax = caps
    step_v, step_w = 0.1, 0.15
    tgt = [0.0, 0.0, 0.0]

    import termios
    import tty
    is_tty = sys.stdin.isatty()
    old = termios.tcgetattr(sys.stdin) if is_tty else None
    if is_tty:
        tty.setcbreak(sys.stdin.fileno())

    def clamp(v, lim):
        return max(-lim, min(lim, v))

    print(">> teleop: w/s=fwd/back  a/d=left/right  q/e=turn  space=stop  z=quit")
    pacer = Pacer(dt)
    try:
        while not stop["flag"]:
            r, _, _ = select([sys.stdin], [], [], 0.0)
            if r:
                c = sys.stdin.read(1).lower()
                if c == "w":   tgt[0] = clamp(tgt[0] + step_v, vmax)
                elif c == "s": tgt[0] = clamp(tgt[0] - step_v, vmax)
                elif c == "a": tgt[1] = clamp(tgt[1] + step_v, vymax)
                elif c == "d": tgt[1] = clamp(tgt[1] - step_v, vymax)
                elif c == "q": tgt[2] = clamp(tgt[2] + step_w, wmax)
                elif c == "e": tgt[2] = clamp(tgt[2] - step_w, wmax)
                elif c in (" ", "x"): tgt = [0.0, 0.0, 0.0]
                elif c == "z": break
                sys.stdout.write(f"\r   target vx={tgt[0]:+.2f} vy={tgt[1]:+.2f} "
                                 f"yaw={tgt[2]:+.2f}    ")
                sys.stdout.flush()
            vx, vy, wz = smoother.step(*tgt, dt=dt)
            driver.set_velocity(vx, vy, wz, hold)
            pacer.wait()
    finally:
        if is_tty and old is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    driver.stop_move()
    print("\n>> teleop stopped (balance-standing).")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Unitree G1 native-gait tester.",
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--net_if", default=os.environ.get("UNITREE_NET_IFACE", "eth0"),
                   help="robot network interface (Unitree DDS).")
    p.add_argument("--domain", type=int, default=0, help="Unitree DDS domain (default 0).")
    p.add_argument("--control-fsm", type=int,
                   default=int(os.environ.get("UNITREE_LOCO_FSM", FSM_DEFAULT_WALK)),
                   help="walking FSM to enter/verify; default 501 (Walk Motion-3Dof-waist).")
    p.add_argument("--bring-up", action="store_true",
                   help="damp->stand_up->control-fsm before commanding.")
    p.add_argument("--no-bring-up", action="store_true",
                   help="skip bring-up (robot already in the requested walking FSM).")
    p.add_argument("--damp-only", action="store_true", help="damp the robot and exit.")
    p.add_argument("--vx", type=float, default=0.0)
    p.add_argument("--vy", type=float, default=0.0)
    p.add_argument("--yaw", type=float, default=0.0)
    p.add_argument("--duration", type=float, default=4.0, help="seconds for constant cmd.")
    p.add_argument("--teleop", action="store_true", help="keyboard teleop mode.")
    p.add_argument("--rate_hz", type=float, default=20.0)
    p.add_argument("--max_forward_vel", type=float, default=0.4)
    p.add_argument("--max_lateral_vel", type=float, default=0.12)
    p.add_argument("--max_yaw_rate", type=float, default=0.5)
    p.add_argument("--damp-on-exit", action="store_true",
                   help="damp (release) on exit instead of leaving it standing.")
    a = p.parse_args(argv)

    driver = LocoDriver(a.net_if, domain_id=a.domain)
    try:
        driver.connect()
    except ImportError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print(f">> connected (net_if={a.net_if}, DDS domain={a.domain})")

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))

    try:
        if a.damp_only:
            print(">> Damp."); driver.damp(); return 0

        if not a.no_bring_up:
            try:
                driver.bring_up(settle_s=3.0, target_fsm=a.control_fsm)
            except RuntimeError as exc:
                print(f"\n[unitree_gait_test] {exc}", file=sys.stderr)
                return 1
        elif a.teleop or abs(a.vx) + abs(a.vy) + abs(a.yaw) > 0.0:
            ok, code, fsm = driver.probe()
            if not ok or int(fsm) != int(a.control_fsm):
                print(
                    "[unitree_gait_test] --no-bring-up was requested, but the "
                    "robot is not in the requested walking mode "
                    f"{a.control_fsm} ({fsm_name(a.control_fsm)}); current "
                    f"fsm={fsm}, GetFsmId code={code}. Put the robot into that "
                    "mode and verify the remote left stick walks it before "
                    "streaming velocity.",
                    file=sys.stderr,
                )
                return 1
        if stop["flag"]:
            return 0

        smoother = VelocitySmoother(SmootherConfig(
            startup_ramp_s=2.0, lin_accel_max=0.6, yaw_accel_max=1.2, lowpass_alpha=0.35))
        caps = (a.max_forward_vel, a.max_lateral_vel, a.max_yaw_rate)

        if a.teleop:
            _run_teleop(driver, smoother, caps, a.rate_hz, stop)
        elif abs(a.vx) + abs(a.vy) + abs(a.yaw) > 0.0:
            tgt = (max(-caps[0], min(caps[0], a.vx)),
                   max(-caps[1], min(caps[1], a.vy)),
                   max(-caps[2], min(caps[2], a.yaw)))
            _run_constant(driver, smoother, tgt, a.duration, a.rate_hz, stop)
        else:
            print(">> standing (no velocity given). Ctrl-C to exit.")
            while not stop["flag"]:
                time.sleep(0.2)
    finally:
        try:
            driver.stop_move()
            if a.damp_on_exit:
                print(">> Damp on exit."); driver.damp()
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
