#!/usr/bin/env python3
"""Shared helpers for driving the Unitree G1 NATIVE (factory) walking policy.

The G1's on-board locomotion is a HIGH-LEVEL controller: you send it a velocity
(vx, vy, yaw-rate) over the Unitree SDK's DDS and the robot's firmware produces
the whole-body joint motion. This is fundamentally different from the AMO
(RoboJuDo) policy, which is a LOW-LEVEL joint policy whose per-joint position
targets we filter/smooth (amo/joint_filters.py) before writing rt/lowcmd.

Consequence for "the same joint filtering as AMO": there are **no joint targets
to filter** on the native path — the firmware owns the joints. The correct
analog at this interface is **velocity-command smoothing**: ramp the command in
from zero on activation, slew-rate-limit it, and low-pass it, so the gait never
receives a step change. That is exactly what :class:`VelocitySmoother` does, and
it is used by BOTH the ``cmd_vel_to_unitree_loco`` bridge and the standalone
``unitree_gait_test`` tool — mirroring how AMO shares joint_filters.py.

This module has NO ROS 2 dependency, so the test tool can run without rclpy.

LocoClient API targeted (from unitree_sdk2 g1_loco_client.hpp, mirrored by
unitree_sdk2py): SetVelocity(vx, vy, omega, duration), Move, Damp()=FSM 1,
StandUp()=FSM 4, Start()=FSM 500, expert walk=FSM 501, BalanceStand(),
StopMove(), ZeroTorque()=FSM 0.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional


# ── FSM ids (from g1_loco_client.hpp) ───────────────────────────────────────
FSM_ZERO_TORQUE = 0
FSM_DAMP = 1
FSM_SQUAT = 2
FSM_SIT = 3
FSM_STAND_UP = 4
FSM_WALK_MOTION = 500
FSM_EXPERT_WALK_3DOF_WAIST = 501
FSM_DEFAULT_WALK = FSM_EXPERT_WALK_3DOF_WAIST


def fsm_name(fsm_id: int) -> str:
    names = {
        FSM_ZERO_TORQUE: "Zero Torque",
        FSM_DAMP: "Damping",
        FSM_SQUAT: "Position Control Squat",
        FSM_SIT: "Position Control Sit Down",
        FSM_STAND_UP: "Lock Standing",
        702: "Lie Down / Stand Up",
        706: "Balance Squat / Squat Stand",
        FSM_WALK_MOTION: "Walk Motion",
        FSM_EXPERT_WALK_3DOF_WAIST: "Walk Motion-3Dof-waist",
        801: "Run",
    }
    return names.get(int(fsm_id), f"FSM {int(fsm_id)}")


@dataclass
class SmootherConfig:
    """Velocity-command smoothing (high-level analog of AMO joint filtering)."""
    startup_ramp_s: float = 3.0     # ease the command 0→full on activation
    lin_accel_max: float = 0.6      # slew cap on vx, vy   [m/s^2]
    yaw_accel_max: float = 1.2      # slew cap on yaw-rate [rad/s^2]
    lowpass_alpha: float = 0.35     # EMA factor per step (1 = no low-pass)


class VelocitySmoother:
    """Ramp-in + slew-limit + EMA low-pass on the (vx, vy, yaw) command.

    step() returns the smoothed command to actually send to the gait. The first
    command after reset() is ~0 (ramp) and can only move by accel_max*dt per
    tick, so the robot never gets a velocity step change — the high-level twin of
    the AMO JointSmoother's snap-free blend + slew clamp.
    """

    def __init__(self, cfg: Optional[SmootherConfig] = None):
        self.cfg = cfg or SmootherConfig()
        self.reset()

    def reset(self) -> None:
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._t0 = time.monotonic()

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self._t0

    @staticmethod
    def _approach(prev: float, target: float, alpha: float, rate: float, dt: float) -> float:
        # EMA toward target, but the per-tick change is slew-limited to rate*dt.
        step = alpha * (target - prev)
        lim = rate * dt
        if step > lim:
            step = lim
        elif step < -lim:
            step = -lim
        return prev + step

    def step(self, vx: float, vy: float, wz: float, dt: float) -> tuple[float, float, float]:
        if dt <= 0.0:
            dt = 1e-3
        c = self.cfg
        ramp = 1.0 if c.startup_ramp_s <= 0.0 else min(1.0, self.elapsed / c.startup_ramp_s)
        tvx, tvy, twz = vx * ramp, vy * ramp, wz * ramp
        self._vx = self._approach(self._vx, tvx, c.lowpass_alpha, c.lin_accel_max, dt)
        self._vy = self._approach(self._vy, tvy, c.lowpass_alpha, c.lin_accel_max, dt)
        self._wz = self._approach(self._wz, twz, c.lowpass_alpha, c.yaw_accel_max, dt)
        return self._vx, self._vy, self._wz


class LocoDriver:
    """Thin wrapper over the Unitree G1 LocoClient (native high-level gait).

    Lazy-imports unitree_sdk2py so this module (and the ROS package) load even
    where the SDK is absent; connect() raises a clear, actionable error if it is.
    """

    def __init__(self, net_if: str, domain_id: int = 0, timeout: float = 10.0):
        self.net_if = net_if
        self.domain_id = int(domain_id)
        self.timeout = float(timeout)
        self._client = None

    def connect(self):
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        except ImportError as exc:  # noqa: BLE001
            raise ImportError(
                "unitree_sdk2py is required for the Unitree native gait but is not "
                "installed. Install it in this container, e.g.:\n"
                "  pip3 install --no-cache-dir "
                "git+https://github.com/unitreerobotics/unitree_sdk2_python.git\n"
                f"(original import error: {exc})"
            ) from exc
        # Unitree DDS uses domain 0 on the robot NIC — SEPARATE from the ROS 2
        # RMW participant (domain 42). They coexist in one process.
        ChannelFactoryInitialize(self.domain_id, self.net_if)
        self._client = LocoClient()
        self._client.SetTimeout(self.timeout)
        self._client.Init()
        return self

    @property
    def ready(self) -> bool:
        return self._client is not None

    def probe(self):
        """Liveness check of the robot's high-level loco service via GetFsmId.

        Returns (ok, code, fsm_id). ok=True only if the service replied (code 0).
        A non-zero code (e.g. 3102) or an exception means the RPC got NO reply —
        typically the robot is NOT in factory high-level control mode (it's in the
        low-level/SDK mode AMO uses), so the loco service isn't serving. Ping can
        still succeed in that state; this is the check that actually matters.
        """
        if self._client is None:
            return False, -1, None
        try:
            code, fsm = self._client.GetFsmId()
        except Exception:  # noqa: BLE001
            return False, -2, None
        return (code == 0 and fsm is not None), int(code), fsm

    _SERVICE_HINT = (
        "Unitree high-level loco service not responding (robot reachable but the "
        "RPC times out). Put the G1 in FACTORY HIGH-LEVEL control mode: boot it "
        "normally, let it reach the balance-ready state, and make sure NO low-level "
        "program (AMO / rt/lowcmd) is running — those two modes are mutually "
        "exclusive. If the Unitree remote can walk the robot, the service is up."
    )

    # ── high-level commands ────────────────────────────────────────────────
    def _require_client(self):
        if self._client is None:
            raise RuntimeError("Unitree LocoClient is not connected")
        return self._client

    @staticmethod
    def _require_ok(command: str, code: Optional[int]) -> int:
        code = -1 if code is None else int(code)
        if code != 0:
            raise RuntimeError(f"{command} failed (SDK code={code})")
        return code

    def _set_fsm_id(self, fsm_id: int) -> int:
        return int(self._require_client().SetFsmId(int(fsm_id)))

    def _wait_for_fsm(
        self,
        expected_fsm: int,
        timeout_s: float,
        poll_s: float = 0.2,
    ) -> tuple[bool, int, Optional[int]]:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        last_code = -1
        last_fsm = None
        while True:
            ok, code, fsm = self.probe()
            last_code, last_fsm = code, fsm
            if ok and int(fsm) == int(expected_fsm):
                return True, code, int(fsm)
            if time.monotonic() >= deadline:
                return False, last_code, last_fsm
            time.sleep(poll_s)

    def set_velocity(self, vx: float, vy: float, omega: float, duration: float = 1.0) -> int:
        return int(self._require_client().SetVelocity(
            float(vx), float(vy), float(omega), float(duration)))

    def stop_move(self) -> int:
        return self.set_velocity(0.0, 0.0, 0.0)

    def damp(self) -> int:
        return self._set_fsm_id(FSM_DAMP)

    def zero_torque(self) -> int:
        return self._set_fsm_id(FSM_ZERO_TORQUE)

    def stand_up(self) -> int:
        # The Python SDK's LocoClient exposes no StandUp() wrapper (the C++ header
        # does); SetFsmId(FSM_STAND_UP=4) is the same firmware "stand up" FSM it
        # maps to. Guaranteed present (SetFsmId is in the Python API).
        return self._set_fsm_id(FSM_STAND_UP)

    def start_main_control(self, target_fsm: int = FSM_DEFAULT_WALK) -> int:
        # LocoClient.Start() only calls SetFsmId(500) and discards the SDK code.
        # Call SetFsmId directly so bring_up() can target the documented expert
        # interface (501) and fail closed if the transition is refused.
        return self._set_fsm_id(int(target_fsm))

    def balance_stand(self, balance_mode: int = 0) -> int:
        # Python signature requires balance_mode (0 = normal balance stand,
        # 1 = continuous gait); the C++ BalanceStand() defaulted to 0.
        return int(self._require_client().SetBalanceMode(int(balance_mode)))

    def bring_up(
        self,
        logger=None,
        settle_s: float = 3.0,
        target_fsm: int = FSM_DEFAULT_WALK,
    ) -> None:
        """Damp → StandUp → target walking FSM, with settle pauses.

        Defaults to FSM 501 (Walk Motion-3Dof-waist), Unitree's documented expert
        walking interface. SetVelocity walks the robot directly in this mode.
        BalanceStand is intentionally NOT called; it froze stepping.

        SAFETY: transitions the robot between FSM states — keep it suspended or
        clear of obstacles and be ready on the hardware e-stop. Only call this
        when the AMO policy is NOT running (single controller on the motors).
        """
        def _log(m):
            (logger.info if logger else print)(m)
        # Fail fast with a clear reason if the loco service isn't answering,
        # instead of firing Damp/StandUp/… into the void (each printing an
        # opaque "[ClientStub] send request error").
        ok, code, _ = self.probe()
        if not ok:
            raise RuntimeError(f"{self._SERVICE_HINT} (GetFsmId code={code})")
        target_fsm = int(target_fsm)
        target_label = f"{target_fsm} {fsm_name(target_fsm)}"
        # Sequence: damp → stand_up → documented walking FSM → velocity. NO
        # BalanceStand: calling SetBalanceMode(0) after Start put the G1 in a
        # static balance sub-mode where SetVelocity produced NO stepping.
        _log("[loco] Damp ...")
        self._require_ok("Damp", self.damp())
        time.sleep(settle_s)

        _log("[loco] StandUp ...")
        self._require_ok("StandUp", self.stand_up())
        time.sleep(settle_s)

        last_code = -1
        last_fsm = None
        for attempt in range(1, 4):
            suffix = "" if attempt == 1 else f" (retry {attempt}/3)"
            _log(f"[loco] SetFsmId({target_label}){suffix} ...")
            self._require_ok(f"SetFsmId({target_fsm})", self.start_main_control(target_fsm))
            ok, last_code, last_fsm = self._wait_for_fsm(target_fsm, settle_s)
            _log(
                f"[loco] FSM after SetFsmId = {last_fsm} "
                f"(expect {target_fsm} for walking; code={last_code})")
            if ok:
                _log("[loco] ready to walk — streaming velocity now.")
                return

        raise RuntimeError(
            "Walking FSM transition did not enter "
            f"{target_label}; current FSM is {last_fsm} "
            f"(GetFsmId code={last_code}). The loco service is reachable, but "
            "the robot stayed out of the requested walking control mode. Put the "
            "G1 into the matching built-in walking mode, verify the remote left "
            "stick walks it, then rerun with --no-bring-up or retry bring-up. "
            f"Refusing to stream velocity while FSM != {target_fsm}.")
