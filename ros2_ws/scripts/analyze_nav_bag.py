#!/usr/bin/env python3
"""
analyze_nav_bag.py — offline troubleshooting plots for the A*+MPC(C) nav stack.

Reads a bag recorded with scripts/record_nav_bag.sh and produces a figure that
overlays what was PLANNED against what the robot ACTUALLY did:

  Left  : XY overlay — actual DLIO trajectory, the global route, the local A*
          plans, and the MPC predicted trajectories (so you can see where the
          executed path diverged from the plan, and where A*/MPC disagreed).
  Right : time series —
            (top)    commanded gait velocity /mpc/cmd_vel [vx, vy, wz] plus the
                     actual speed differentiated from the DLIO odometry, so you
                     can spot the stop/go stutter (issue #1) directly;
            (middle) MPC solve time + success flag from /mpc/diagnostics;
            (bottom) navigation state timeline (/navigation/state).

Usage:
    python3 scripts/analyze_nav_bag.py <bag_dir> [--out plot.png] [--no-show]

Depends on rosbag2_py + rclpy (ROS 2 env sourced) and matplotlib/numpy.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg") if "--no-show" in sys.argv else None
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib is required: pip install matplotlib")

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit("ROS 2 (rosbag2_py, rclpy) not found — source your ROS 2 workspace first.")


# ── Topics we care about ────────────────────────────────────────────────────
ODOM_TOPIC = "/dlio/odom_node/odom"
CMD_TOPIC = "/mpc/cmd_vel"
ASTAR_TOPIC = "/a_star/path"
GLOBAL_TOPIC = "/global_path"
PRED_TOPIC = "/mpc/predicted_path"
GOAL_TOPIC = "/global_goal"
DIAG_TOPIC = "/mpc/diagnostics"
STATE_TOPIC = "/navigation/state"


def _detect_storage_id(bag_dir: str) -> str:
    """Guess the storage backend so StorageOptions is happy on any distro."""
    for f in os.listdir(bag_dir):
        if f.endswith(".mcap"):
            return "mcap"
        if f.endswith(".db3"):
            return "sqlite3"
    return "sqlite3"


def _open_reader(bag_dir: str):
    storage_id = _detect_storage_id(bag_dir)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, type_map


def _path_xy(msg) -> np.ndarray:
    return np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=float)


def load(bag_dir: str) -> dict:
    reader, type_map = _open_reader(bag_dir)
    msg_classes = {name: get_message(t) for name, t in type_map.items()}

    odom = []          # (t, x, y)
    cmd = []           # (t, vx, vy, wz)
    diag = []          # (t, success, cost, solve_ms, avg_ms, fails, security, vx_eff)
    states = []        # (t, state_str)
    astar_paths = []   # (t, (N,2))
    global_paths = []  # (t, (N,2))
    pred_paths = []    # (t, (N,2))
    goals = []         # (t, x, y)

    t0 = None
    while reader.has_next():
        topic, data, tns = reader.read_next()
        if topic not in msg_classes:
            continue
        t = tns * 1e-9
        if t0 is None:
            t0 = t
        t -= t0
        try:
            msg = deserialize_message(data, msg_classes[topic])
        except Exception:
            continue

        if topic == ODOM_TOPIC:
            odom.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))
        elif topic == CMD_TOPIC:
            cmd.append((t, msg.linear.x, msg.linear.y, msg.angular.z))
        elif topic == DIAG_TOPIC:
            d = list(msg.data) + [0.0] * 7
            diag.append((t, *d[:7]))
        elif topic == STATE_TOPIC:
            states.append((t, msg.data))
        elif topic == ASTAR_TOPIC and msg.poses:
            astar_paths.append((t, _path_xy(msg)))
        elif topic == GLOBAL_TOPIC and msg.poses:
            global_paths.append((t, _path_xy(msg)))
        elif topic == PRED_TOPIC and msg.poses:
            pred_paths.append((t, _path_xy(msg)))
        elif topic == GOAL_TOPIC:
            goals.append((t, msg.pose.position.x, msg.pose.position.y))

    return dict(odom=np.array(odom), cmd=np.array(cmd), diag=np.array(diag),
                states=states, astar=astar_paths, glob=global_paths,
                pred=pred_paths, goals=np.array(goals) if goals else np.empty((0, 3)))


def _speed_from_odom(odom: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Differentiate the odom XY into a speed [m/s] time series."""
    if len(odom) < 2:
        return np.array([]), np.array([])
    t = odom[:, 0]
    dxy = np.diff(odom[:, 1:3], axis=0)
    dt = np.diff(t)
    dt[dt <= 1e-6] = 1e-6
    spd = np.hypot(dxy[:, 0], dxy[:, 1]) / dt
    return t[1:], spd


def plot(data: dict, out: str, show: bool) -> None:
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.2, 1.0])

    # ── Left: XY overlay ────────────────────────────────────────────────────
    axm = fig.add_subplot(gs[:, 0])
    axm.set_title("Trajectories (odom frame)")
    axm.set_xlabel("x [m]"); axm.set_ylabel("y [m]")
    axm.set_aspect("equal", adjustable="datalim")
    axm.grid(True, alpha=0.3)

    # planned paths (faint so the many snapshots don't dominate)
    for _, xy in data["glob"]:
        axm.plot(xy[:, 0], xy[:, 1], color="tab:blue", alpha=0.10, lw=1)
    for _, xy in data["astar"]:
        axm.plot(xy[:, 0], xy[:, 1], color="tab:green", alpha=0.12, lw=1)
    for _, xy in data["pred"]:
        axm.plot(xy[:, 0], xy[:, 1], color="tab:orange", alpha=0.10, lw=1)
    # legend proxies + the most-recent of each (bold)
    if data["glob"]:
        xy = data["glob"][-1][1]; axm.plot(xy[:, 0], xy[:, 1], color="tab:blue", lw=2, label="global path (last)")
    if data["astar"]:
        xy = data["astar"][-1][1]; axm.plot(xy[:, 0], xy[:, 1], color="tab:green", lw=2, label="A* path (last)")
    if data["pred"]:
        xy = data["pred"][-1][1]; axm.plot(xy[:, 0], xy[:, 1], color="tab:orange", lw=2, label="MPC predicted (last)")

    odom = data["odom"]
    if len(odom):
        axm.plot(odom[:, 1], odom[:, 2], color="k", lw=2.2, label="actual (DLIO)")
        axm.plot(odom[0, 1], odom[0, 2], "o", color="lime", ms=10, label="start")
        axm.plot(odom[-1, 1], odom[-1, 2], "s", color="red", ms=9, label="end")
    if len(data["goals"]):
        g = data["goals"]
        axm.plot(g[:, 1], g[:, 2], "*", color="magenta", ms=16, label="goal(s)")
    axm.legend(loc="best", fontsize=8)

    # ── Right-top: velocities ───────────────────────────────────────────────
    axv = fig.add_subplot(gs[0, 1])
    axv.set_title("Gait velocity command (/mpc/cmd_vel) + actual speed")
    cmd = data["cmd"]
    if len(cmd):
        axv.plot(cmd[:, 0], cmd[:, 1], label="cmd vx [m/s]", color="tab:red")
        axv.plot(cmd[:, 0], cmd[:, 2], label="cmd vy [m/s]", color="tab:purple")
        axv.plot(cmd[:, 0], cmd[:, 3], label="cmd wz [rad/s]", color="tab:brown")
    ts, spd = _speed_from_odom(odom)
    if len(ts):
        axv.plot(ts, spd, label="actual speed [m/s]", color="k", alpha=0.6, lw=1.2)
    axv.axhline(0, color="gray", lw=0.6)
    axv.grid(True, alpha=0.3); axv.legend(fontsize=8, ncol=2); axv.set_xlabel("t [s]")

    # ── Right-mid: solver health + MPC cost (issue #1) ──────────────────────
    axd = fig.add_subplot(gs[1, 1])
    axd.set_title("MPC solver: solve time, cost + failures (/mpc/diagnostics)")
    diag = data["diag"]
    if len(diag):
        axd.plot(diag[:, 0], diag[:, 3], label="solve_ms", color="tab:blue")
        axd.plot(diag[:, 0], diag[:, 4], label="avg_ms", color="tab:cyan", alpha=0.7)
        axd.set_ylabel("ms"); axd.set_xlabel("t [s]")
        # MPC cost function on a twin axis (log scale — the objective spans orders
        # of magnitude and spikes on hard/near-infeasible solves). |J| so a rare
        # negative (progress-reward-dominated MPCC) objective still plots on log;
        # non-finite entries (inf on a failed solve) are dropped.
        axc = axd.twinx()
        cost = diag[:, 2]
        finite = np.isfinite(cost)
        if finite.any():
            axc.plot(diag[finite, 0], np.abs(cost[finite]) + 1e-6,
                     label="MPC cost |J|", color="tab:green", alpha=0.85, lw=1.3)
            try:
                axc.set_yscale("log")
            except Exception:
                pass
            axc.set_ylabel("MPC cost |J| (log)", color="tab:green")
            axc.tick_params(axis="y", labelcolor="tab:green")
            axc.legend(fontsize=8, loc="upper right")
        # mark failed solves (success flag == 0) and security engagements
        fail_t = diag[diag[:, 1] < 0.5, 0]
        for ft in fail_t:
            axd.axvline(ft, color="red", alpha=0.25, lw=0.8)
        sec_t = diag[diag[:, 6] > 0.5, 0]
        for st_ in sec_t:
            axd.axvline(st_, color="orange", alpha=0.15, lw=0.8)
        axd.plot([], [], color="red", alpha=0.5, label="solve failed")
        axd.plot([], [], color="orange", alpha=0.5, label="security mode")
    axd.grid(True, alpha=0.3); axd.legend(fontsize=8, loc="upper left")

    # ── Right-bottom: state timeline ────────────────────────────────────────
    axs = fig.add_subplot(gs[2, 1])
    axs.set_title("Navigation state (/navigation/state)")
    states = data["states"]
    if states:
        labels = sorted({s for _, s in states})
        lut = {s: i for i, s in enumerate(labels)}
        st = np.array([lut[s] for _, s in states])
        tt = np.array([t for t, _ in states])
        axs.step(tt, st, where="post", color="tab:green")
        axs.set_yticks(range(len(labels))); axs.set_yticklabels(labels, fontsize=8)
    axs.grid(True, alpha=0.3); axs.set_xlabel("t [s]")

    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(f"[analyze_nav_bag] wrote {out}")
    _print_summary(data)
    if show:
        plt.show()


def _print_summary(data: dict) -> None:
    odom, cmd, diag = data["odom"], data["cmd"], data["diag"]
    print("── summary " + "─" * 50)
    if len(odom):
        dur = odom[-1, 0] - odom[0, 0]
        dist = float(np.sum(np.hypot(*np.diff(odom[:, 1:3], axis=0).T)))
        print(f"  duration        : {dur:6.1f} s")
        print(f"  path length     : {dist:6.2f} m")
    if len(cmd):
        ts, spd = _speed_from_odom(odom)
        if len(spd):
            near_zero = float(np.mean(spd < 0.03))
            print(f"  time near-still : {near_zero*100:5.1f} %  (candidate stop/go — issue #1)")
    if len(diag):
        fails = float(np.mean(diag[:, 1] < 0.5))
        sec = float(np.mean(diag[:, 6] > 0.5))
        print(f"  solve fail rate : {fails*100:5.1f} %")
        print(f"  security rate   : {sec*100:5.1f} %")
        print(f"  mean solve time : {np.mean(diag[:, 3]):5.1f} ms   max {np.max(diag[:, 3]):5.1f} ms")
        cost = diag[:, 2]; fc = cost[np.isfinite(cost)]
        if len(fc):
            print(f"  MPC cost (J)    : mean {np.mean(fc):9.1f}  max {np.max(fc):9.1f}  last {fc[-1]:9.1f}")
    print("─" * 60)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag_dir", help="path to the recorded bag directory")
    ap.add_argument("--out", default=None, help="output PNG (default: <bag>/nav_analysis.png)")
    ap.add_argument("--no-show", action="store_true", help="save only, do not open a window")
    args = ap.parse_args()

    if not os.path.isdir(args.bag_dir):
        sys.exit(f"not a directory: {args.bag_dir}")
    out = args.out or os.path.join(args.bag_dir, "nav_analysis.png")

    data = load(args.bag_dir)
    if not len(data["odom"]):
        print("[warn] no odometry in bag — is /dlio/odom_node/odom recorded?")
    plot(data, out, show=not args.no_show)


if __name__ == "__main__":
    main()
