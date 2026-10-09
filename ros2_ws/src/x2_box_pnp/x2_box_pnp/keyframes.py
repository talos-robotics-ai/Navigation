"""Joint-space arm keyframes and interpolation (pure Python)."""
import math

N_ARM = 14


def smoothstep(s):
    s = min(1.0, max(0.0, s))
    return s * s * (3.0 - 2.0 * s)


def interpolate(q0, q1, s):
    """Smoothstep blend q0 -> q1, s in [0, 1]."""
    k = smoothstep(s)
    return [a + (b - a) * k for a, b in zip(q0, q1)]


def max_abs_error(q, target):
    return max(abs(a - b) for a, b in zip(q, target))


class Keyframes:
    """Named poses; a value of None means 'policy default' and resolves to `default`."""

    def __init__(self, frames, default):
        if len(default) != N_ARM:
            raise ValueError(f'default arm pose needs {N_ARM} values')
        self.default = [float(v) for v in default]
        self.frames = {}
        for name, q in (frames or {}).items():
            if q is not None and len(q) != N_ARM:
                raise ValueError(f'keyframe {name!r} needs {N_ARM} values, got {len(q)}')
            self.frames[name] = None if q is None else [float(v) for v in q]

    def get(self, name):
        q = self.frames.get(name)
        return list(self.default) if q is None else list(q)

    def is_policy_default(self, name):
        return self.frames.get(name) is None


class ArmMotion:
    """Time-parameterised move from q_start to q_target over `duration` seconds."""

    def __init__(self, q_start, q_target, duration, t0):
        self.q0, self.q1 = list(q_start), list(q_target)
        self.duration, self.t0 = max(1e-3, float(duration)), t0

    def command(self, t):
        return interpolate(self.q0, self.q1, (t - self.t0) / self.duration)

    def finished(self, t):
        return t - self.t0 >= self.duration

    def reached(self, q_meas, tol):
        return q_meas is not None and len(q_meas) == len(self.q1) and max_abs_error(q_meas, self.q1) < tol

    def elapsed(self, t):
        return t - self.t0
