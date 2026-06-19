#!/usr/bin/env python3
"""Generate an RTX-Lidar config approximating a Livox MID-360.

Real MID-360:
    * Horizontal FOV : 360 deg
    * Vertical FOV   : -7 deg .. +52 deg  (59 deg span, asymmetric)
    * Range          : ~0.1 m .. 70 m
    * Point rate     : ~200 000 pts/s, 10 Hz frames
    * Pattern        : non-repetitive rosette (proprietary)

We model it as an RTX **rotary** multi-beam lidar: a vertical fan of beams
spanning -7..+52 deg, spun 360 deg at 10 Hz. This is NOT the proprietary
non-repetitive rosette, but it is a 360deg x (-7..+52) cloud at a comparable
point rate that DLIO consumes exactly like the real sensor's cloud -- and,
unlike a hand-authored `solidState` config, the `rotary` scan type is the
well-tested Isaac path that reliably produces returns. (An earlier solidState
version produced EMPTY clouds: width=0 -- the emitter-state schema requires
numLines/channelId/bank wiring that is easy to get subtly wrong.)

Tuning knobs (below): N_BEAMS sets vertical density; TARGET_POINTS_PER_SEC sets
horizontal density via reportRateBaseHz = TARGET_POINTS_PER_SEC / N_BEAMS.

Run:  python3 gen_mid360_config.py
Writes: lidar_configs/Livox_Mid360.json
"""

from __future__ import annotations

import json
import os

# --- MID-360 geometry ------------------------------------------------------
EL_MIN_DEG, EL_MAX_DEG = -7.0, 52.0        # asymmetric vertical FOV
NEAR_RANGE_M = 0.1
FAR_RANGE_M = 70.0
SCAN_RATE_HZ = 10.0                         # rotations per second (frame rate)

# --- density knobs ---------------------------------------------------------
N_BEAMS = 64                               # vertical beams across the FOV
TARGET_POINTS_PER_SEC = 200_000            # ~real MID-360 throughput
# reportRateBaseHz = azimuth columns per second; pts/s = columns/s * N_BEAMS.
REPORT_RATE_HZ = max(1, round(TARGET_POINTS_PER_SEC / N_BEAMS))


def _linspace(a: float, b: float, n: int) -> list:
    if n == 1:
        return [0.5 * (a + b)]
    step = (b - a) / (n - 1)
    return [round(a + i * step, 4) for i in range(n)]


def build_config() -> dict:
    elevations = _linspace(EL_MIN_DEG, EL_MAX_DEG, N_BEAMS)
    # All beams share the head azimuth (0); the rotation sweeps the full 360.
    azimuths = [0.0] * N_BEAMS
    vert_offsets = [0.0] * N_BEAMS
    # Spread firing across one azimuth column so per-ray motion comp has a
    # monotonic sub-column timeline (matches the shipped rotary configs).
    col_dt_ns = int(1e9 / (REPORT_RATE_HZ * max(1, N_BEAMS)))
    fire_times = [i * col_dt_ns for i in range(N_BEAMS)]

    return {
        "class": "sensor",
        "type": "lidar",
        "name": "Livox MID-360 (rotary approx)",
        "driveWorksId": "GENERIC",
        "profile": {
            "scanType": "rotary",
            "intensityProcessing": "normalization",
            "rotationDirection": "CW",
            "rayType": "IDEALIZED",
            "nearRangeM": NEAR_RANGE_M,
            "farRangeM": FAR_RANGE_M,
            "rangeResolutionM": 0.004,
            "rangeAccuracyM": 0.02,
            "avgPowerW": 0.002,
            "minReflectance": 0.1,
            "minReflectanceRange": 40.0,
            "wavelengthNm": 905.0,
            "pulseTimeNs": 6,
            "maxReturns": 1,
            "scanRateBaseHz": SCAN_RATE_HZ,
            "reportRateBaseHz": REPORT_RATE_HZ,
            "numberOfEmitters": N_BEAMS,
            "emitterStateCount": 1,
            "emitterStates": [
                {
                    "azimuthDeg": azimuths,
                    "elevationDeg": elevations,
                    "vertOffsetM": vert_offsets,
                    "fireTimeNs": fire_times,
                }
            ],
            "intensityMappingType": "LINEAR",
        },
    }


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(here, "lidar_configs")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "Livox_Mid360.json")
    with open(out_path, "w") as fh:
        json.dump(build_config(), fh, indent=1)
    print(f"wrote {out_path}")
    print(f"  vertical FOV  : {EL_MIN_DEG}..{EL_MAX_DEG} deg over {N_BEAMS} beams")
    print(f"  rotation      : {SCAN_RATE_HZ} Hz, 360 deg horizontal")
    print(f"  reportRateHz  : {REPORT_RATE_HZ} (~{REPORT_RATE_HZ * N_BEAMS} pts/s)")


if __name__ == "__main__":
    main()
