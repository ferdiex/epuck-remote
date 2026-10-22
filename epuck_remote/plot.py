#!/usr/bin/env python3
"""
plot.py - figures from epuck-remote logs, one panel per figure, English
labels, file names by content.

  python3 -m epuck_remote.plot run --log runs/x.csv --out runs/x
      -> x_proximity.png, x_wheel_commands.png, x_ground_sensors.png
         (runs with --floor), x_trajectory.png (mock/sim)
  python3 -m epuck_remote.plot approach --csv calib/real.csv calib/sim.csv \
          --labels real sim --out calib/ir_approach
      -> ir_approach_front_ir_vs_distance.png
"""
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")


def load_meta(log):
    import json
    import os
    path = log[:-4] + "_meta.json" if log.endswith(".csv") else log + ".meta.json"
    if os.path.exists(path):
        with open(path) as f:
            return json.load(f)
    return {}


PS_LABELS = ["ps0 front R", "ps1 diag R", "ps2 side R", "ps3 back R",
             "ps4 back L", "ps5 side L", "ps6 diag L", "ps7 front L"]


def plot_run(a):
    d = load(a.log)
    meta = load_meta(a.log)
    t = d["t"]

    has_floor = bool(np.any(np.column_stack([d["gs0"], d["gs1"], d["gs2"]]) > 0))
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.1
    extent = [t[0] - dt / 2, t[-1] + dt / 2]

    # normalized proximity as a heatmap: one row per sensor, 0 (white) .. 1
    prox = np.vstack([d[f"prox{i}"] for i in range(8)])
    fig, ax = plt.subplots(figsize=(7, 3.5))
    im = ax.imshow(prox, aspect="auto", origin="upper", cmap="Reds", vmin=0, vmax=1,
                   interpolation="nearest", extent=extent + [7.5, -0.5])
    ax.set_yticks(range(8))
    ax.set_yticklabels(PS_LABELS, fontsize=8)
    ax.set_xlabel("time [s]")
    fig.colorbar(im, ax=ax, label="normalized proximity", pad=0.02)
    fig.tight_layout()
    fig.savefig(f"{a.out}_proximity.png", dpi=a.dpi)

    # wheel commands: one panel per wheel (shared time axis), fixed scale with
    # margin, dotted lines at the +/-1000 steps/s limit of the robot
    fig, axes = plt.subplots(2, 1, figsize=(7, 4.2), sharex=True)
    for ax, col, name, color in ((axes[0], "cmd_l", "left", "tab:blue"),
                                 (axes[1], "cmd_r", "right", "tab:orange")):
        ax.axhline(0, color="0.75", lw=0.6)
        for lim in (-1000, 1000):
            ax.axhline(lim, color="0.5", lw=0.8, ls=":")
        ax.plot(t, d[col], lw=1.5, color=color)
        ax.set_ylim(-1200, 1200)
        ax.set_yticks([-1000, -500, 0, 500, 1000])
        ax.set_ylabel(f"{name} [steps/s]")
    axes[1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(f"{a.out}_wheel_commands.png", dpi=a.dpi)

    # ground sensors as a heatmap in grey levels: dark floor reads dark
    if has_floor:
        gs = np.vstack([d["gs0"], d["gs1"], d["gs2"]])
        fig, ax = plt.subplots(figsize=(7, 2.4))
        im = ax.imshow(gs, aspect="auto", origin="upper", cmap="gray", vmin=0, vmax=1000,
                       interpolation="nearest", extent=extent + [2.5, -0.5])
        ax.set_yticks(range(3))
        ax.set_yticklabels(["gs0 left", "gs1 center", "gs2 right"], fontsize=8)
        ax.set_xlabel("time [s]")
        fig.colorbar(im, ax=ax, label="ground raw value", pad=0.02)
        fig.tight_layout()
        fig.savefig(f"{a.out}_ground_sensors.png", dpi=a.dpi)

    # trajectory in the arena (mock/sim)
    if "x" in d.dtype.names:
        from matplotlib.patches import Circle, Rectangle
        lay = meta.get("layout") or {}
        fig, ax = plt.subplots(figsize=(5.6 if has_floor else 5, 5))
        if lay.get("food"):
            fx, fy, fr = lay["food"]
            ax.add_patch(Circle((fx, fy), fr, color="0.25", alpha=0.35, lw=0, label="dark floor patch"))
        for k, (ox, oy, r) in enumerate(lay.get("obstacles") or []):
            ax.add_patch(Circle((ox, oy), r, color="tab:red", alpha=0.8, lw=0,
                                label="obstacle" if k == 0 else None))
        if has_floor:
            ax.plot(d["x"], d["y"], lw=0.6, color="0.6", zorder=2)
            g = (d["gs0"] + d["gs1"] + d["gs2"]) / 3
            # plot each reading where the ground sensors are: 3 cm ahead of the
            # centre (GS_POS x = 0.03 in sim/sensors.py)
            yaw = np.radians(d["yaw_deg"])
            gx, gy = d["x"] + 0.03 * np.cos(yaw), d["y"] + 0.03 * np.sin(yaw)
            sc = ax.scatter(gx, gy, c=g, cmap="viridis", vmin=0, vmax=1000,
                            s=9, zorder=3, label="ground reading")
            fig.colorbar(sc, ax=ax, label="mean ground raw value", shrink=0.8, pad=0.02)
        else:
            ax.plot(d["x"], d["y"], lw=1.0, color="tab:blue", label="path")
        ax.plot(d["x"][0], d["y"][0], "o", ms=8, mfc="white", mec="black", mew=1.5, zorder=4, label="start")
        if "state" in d.dtype.names:                       # forager: where food was found
            st = np.asarray(d["state"]).astype(str)
            idx = [i for i in range(1, len(st)) if st[i] == "spin" and st[i - 1] != "spin"]
            if idx:
                ax.plot(d["x"][idx], d["y"][idx], "*", ms=13, color="gold", mec="black",
                        zorder=5, label=f"food found ({len(idx)})")
        ax.plot(d["x"][-1], d["y"][-1], "s", ms=6, color="black", zorder=4, label="end")
        if lay.get("arena"):
            h = lay["arena"] / 2
            ax.add_patch(Rectangle((-h, -h), 2 * h, 2 * h, fill=False, lw=2, color="tab:red"))
            lim = 1.04 * h
            ax.set_xlim(-lim, lim)
            ax.set_ylim(-lim, lim)
        else:
            cx = 0.5 * (d["x"].min() + d["x"].max())
            cy = 0.5 * (d["y"].min() + d["y"].max())
            half = 0.55 * max(np.ptp(d["x"]), np.ptp(d["y"]), 0.2)
            ax.set_xlim(cx - half, cx + half)
            ax.set_ylim(cy - half, cy + half)
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.legend(fontsize=7, loc="upper center", bbox_to_anchor=(0.5, 1.13), ncol=3, frameon=False)
        fig.tight_layout()
        fig.savefig(f"{a.out}_trajectory.png", dpi=a.dpi)
    print("saved figures with prefix", a.out)


def model_front_raw(front_dist):
    """Front IR value the model gives for a flat wall `front_dist` m from the
    robot's front (body radius 37 mm), from ps0/ps7 positions and bearings
    and the Webots lookup table (same geometry as the mock robot)."""
    from .model import BODY_RADIUS, IR_ANGLES, IR_POS, ir_raw_from_distance
    out = []
    for D in np.atleast_1d(front_dist):
        wall_x = BODY_RADIUS + D
        vals = []
        for k in (0, 7):
            ang = np.radians(IR_ANGLES[k])
            ray = (wall_x - IR_POS[k, 0]) / np.cos(ang)
            vals.append(ir_raw_from_distance(ray) if ray < 0.07 else 67.19)
        out.append(max(vals))
    return np.array(out)


def true_distance(d, start, measured_end):
    """est_dist comes from the wheel step counters; if the final distance
    was measured with a ruler, rescale the travelled distance to match it."""
    est = d["est_dist"]
    if measured_end is None:
        return est
    travelled_est = start - est[-1]
    k = (start - measured_end) / travelled_est if travelled_est > 0 else 1.0
    return start - k * (start - est)


def estimate_shift(dist, raw, lo=150, hi=1400):
    """How much closer than the model the robot reads the wall: for each
    sample with lo < raw < hi, the model distance giving the same raw value,
    subtracted from the true distance. Returns the median, or None."""
    grid = np.linspace(0.0, 0.07, 701)
    model = model_front_raw(grid)
    order = np.argsort(model)
    m = (raw > lo) & (raw < hi)
    if m.sum() < 3:
        return None
    eq = np.interp(raw[m], model[order], grid[order])
    return float(np.median(dist[m] - eq))


def plot_approach(a):
    labels = a.labels or a.csv
    ends = a.measured_end or [None] * len(a.csv)
    if len(ends) != len(a.csv):
        raise SystemExit("--measured-end needs one value (cm) per --csv file")
    fig, ax = plt.subplots(figsize=(6.5, 4.2))
    for path, lab, end in zip(a.csv, labels, ends):
        d = load(path)
        dist = true_distance(d, a.start_dist / 100, None if end is None else end / 100)
        front = np.maximum(d["raw0"], d["raw7"])
        ax.plot(100 * dist, front, ".-", ms=3, lw=0.8, label=lab)
        sh = estimate_shift(dist, front)
        note = "too few samples in the 150-1400 range" if sh is None else f"{100 * sh:+.1f} cm"
        print(f"{lab}: estimated ir_shift {note}"
              + ("" if end is not None else " (distance from wheel counters only)"))
    if a.model:
        g = np.linspace(0.0, 0.10, 201)
        ax.plot(100 * g, model_front_raw(g), "k--", lw=1.2, label="model (twin)")
    ax.set_xlabel("distance robot front to wall [cm]")
    ax.set_ylabel("front IR raw value, max(ps0, ps7)")
    ax.set_yscale(a.yscale)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(f"{a.out}_front_ir_vs_distance.png", dpi=a.dpi)
    print("saved", f"{a.out}_front_ir_vs_distance.png")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--log", required=True)
    r.add_argument("--out", required=True)
    r.add_argument("--dpi", type=int, default=150)
    q = sub.add_parser("approach")
    q.add_argument("--csv", nargs="+", required=True)
    q.add_argument("--labels", nargs="*")
    q.add_argument("--out", required=True)
    q.add_argument("--yscale", choices=["linear", "log"], default="log")
    q.add_argument("--model", action="store_true", help="overlay the model curve")
    q.add_argument("--measured-end", type=float, nargs="*",
                   help="final distance measured with a ruler [cm], one per --csv")
    q.add_argument("--start-dist", type=float, default=10.0, help="start distance [cm]")
    q.add_argument("--dpi", type=int, default=150)
    a = p.parse_args()
    {"run": plot_run, "approach": plot_approach}[a.cmd](a)


if __name__ == "__main__":
    main()
