#!/usr/bin/env python3
"""
epuck-remote command line.  Run from the repo root:

  python3 -m epuck_remote.cli <command> --backend {mock,sim,real} [options]

Commands
  probe     connect, firmware version, time sensor reads (link check)
  baseline  average IR in open space -> calibration JSON
  run       closed loop: sensors -> controller -> wheel speeds, CSV log
  drive     fixed wheel command for N seconds (speed / turn calibration)
  approach  drive slowly toward a wall, log raw IR vs distance (IR calibration)
"""
import argparse
import csv
import json
import math
import os
import signal
import sys
import time

import numpy as np

from .adapter import SensorAdapter
from .controllers import load_controller
from .model import AXLE, BODY_RADIUS, STEP_LEN, steps_from_mps
from .protocol import AsercomRobot, BackendClosed, LinkError


# ------------------------------------------------------------------ backends
def parse_xyr(text, n):
    vals = [float(v) for v in text.split(",")]
    if len(vals) != n:
        raise SystemExit(f"error: expected {n} comma-separated numbers, got '{text}'")
    return vals


def make_transport(a, start=None):
    obstacles = json.loads(a.obstacles) if a.obstacles else []
    if start is None and a.start:
        x, y, yaw_deg = parse_xyr(a.start, 3)
        start = (x, y, math.radians(yaw_deg))
    dt = 1.0 / a.hz
    if a.backend == "mock":
        from .transport_mock import MockTransport
        return MockTransport(arena=a.arena or 0.5, obstacles=obstacles, dt=dt,
                             noise=a.mock_noise, latency=a.wall_latency,
                             seed=a.seed, start=start,
                             food=parse_xyr(a.food, 3) if a.food else None)
    if a.backend == "sim":
        from .transport_sim import SimTransport
        food = parse_xyr(a.food, 3) if a.food else None
        return SimTransport(config=a.sim_config or None, gui=a.gui, dt=dt,
                            latency=a.sim_latency, seed=a.seed,
                            noise=False if a.no_sim_noise else None,
                            n_rays=a.n_rays, start=start, food=food,
                            obstacles=obstacles, arena=a.arena,
                            wall_latency=a.wall_latency,
                            video=getattr(a, "video", "") or None,
                            video_fps=getattr(a, "video_fps", 25),
                            video_size=tuple(int(v) for v in getattr(a, "video_size", "800x600").split("x")),
                            video_yaw=getattr(a, "video_yaw", 0.0),
                            video_top=getattr(a, "video_top", False))
    if not a.port:
        raise SystemExit("error: --port is required with --backend real")
    from .transport_serial import SerialTransport
    eol = {"cr": "\r", "lf": "\n", "crlf": "\r\n"}[a.eol]
    return SerialTransport(a.port, timeout=a.timeout, eol=eol)


def open_robot(t, a):
    """Protocol on top of a transport; on the real robot, wait until the
    Bluetooth link answers before the first command."""
    robot = AsercomRobot(t, retries=a.retries)
    if a.backend == "real" and a.connect_timeout > 0:
        print("waiting for the robot...", end=" ", flush=True)
        try:
            print(robot.wait_ready(a.connect_timeout))
        except LinkError:
            print("no answer.\nIf the robot is still moving from an earlier run, switch it off.",
                  file=sys.stderr)
            raise
        safe_stop(robot)          # clear wheel speeds left by an earlier run
    return robot


def step_diff(start, end):
    """Difference of two motor step counters, which are 16-bit on the
    robot and wrap around (a raw difference of -62980 meant ~2500 steps)."""
    d = (end - start) % 65536
    return d - 65536 if d >= 32768 else d


def confirm_start(robot, a, what):
    """Real robot: after connecting (which can jolt the wheels, see
    docs/bluetooth.md), wait until the robot is placed again and Enter is
    pressed, then stop the wheels once more and drop any pending reply."""
    if a.backend != "real" or a.no_confirm:
        return
    safe_stop(robot)
    input(f"Connected. Place the robot for the {what}, then press Enter to start "
          f"(Ctrl-C to cancel)... ")
    safe_stop(robot)
    while robot.t.read_line():
        pass


def realtime(a):
    return a.backend == "real" or a.realtime or getattr(a, "gui", False)


def apply_motion(left, right, a):
    """Motion calibration knobs, applied to wheel commands in steps/s:
    speed_scale scales the whole command; turn_scale also scales the
    differential part (the robot may turn more or less than the model even
    when it drives straight right); trim balances the wheels (positive trim
    slows the left wheel and speeds the right one: use it when the robot
    drifts right on a straight command)."""
    m, d = 0.5 * (left + right), 0.5 * (right - left) * a.turn_scale
    l, r = a.speed_scale * (m - d), a.speed_scale * (m + d)
    return l * (1.0 - a.trim), r * (1.0 + a.trim)


STOP_WARNING = ("\n*** COULD NOT STOP THE ROBOT: the Bluetooth link is lost and the robot "
                "keeps its last wheel speeds. Switch it off. ***\n")


def safe_stop(robot, tries=10):
    """Stop the wheels, retrying for ~2 s. asercom has no watchdog: if the
    link is lost the robot keeps its last speeds, so say it loudly."""
    for _ in range(tries):
        try:
            robot.stop()
            return True
        except LinkError:
            time.sleep(0.2)
        except BackendClosed:
            return True
    print(STOP_WARNING, file=sys.stderr)
    return False


# ------------------------------------------------------------------ commands
def cmd_probe(a):
    t = make_transport(a)
    robot = open_robot(t, a)
    try:
        print("version:", robot.version())
        lat = []
        for _ in range(a.n):
            t0 = time.perf_counter()
            prox = robot.get_proximity()
            lat.append(time.perf_counter() - t0)
        print("proximity ps0..ps7:", prox)
        print("floor gs0..gs2 (left, center, right):", robot.get_floor())
        print("step counters:", robot.get_steps())
        t0 = time.perf_counter()
        robot.stop()
        print(f"set_speed round trip: {1e3 * (time.perf_counter() - t0):.1f} ms")
        lat = np.array(lat) * 1e3
        print(f"N round trip over {a.n}: mean {lat.mean():.1f} ms, "
              f"p95 {np.percentile(lat, 95):.1f} ms, max {lat.max():.1f} ms")
        print(f"link errors: {robot.errors}")
    finally:
        robot.close()


def cmd_baseline(a):
    t = make_transport(a)
    robot = open_robot(t, a)
    try:
        robot.stop()
        if a.calibrate:
            print(robot.calibrate())
            time.sleep(1.0)
        samples = np.array([robot.get_proximity() for _ in range(a.n)], float)
        base, std = samples.mean(axis=0), samples.std(axis=0)
        print("mean:", np.round(base, 1).tolist())
        print("std :", np.round(std, 1).tolist())
        out = {"backend": a.backend, "samples": a.n, "baseline": base.tolist(),
               "std": std.tolist(), "date": time.strftime("%Y-%m-%d %H:%M:%S")}
        with open(a.out, "w") as f:
            json.dump(out, f, indent=2)
        print("saved", a.out)
    finally:
        robot.close()


def load_calib(path):
    if not path:
        return None, None
    with open(path) as f:
        c = json.load(f)
    return c["baseline"], c.get("std")


def cmd_run(a):
    t_launch = time.perf_counter()
    if a.log == "auto":
        tag = a.backend if a.backend != "real" else (a.port or "real")
        tag = "".join(ch if ch.isalnum() else "_" for ch in tag).strip("_")
        a.log = f"runs/{time.strftime('%Y%m%d_%H%M%S')}_{tag}.csv"
        os.makedirs("runs", exist_ok=True)
    t = make_transport(a)
    robot = open_robot(t, a)
    base, std = load_calib(a.calib)
    if a.auto_baseline > 0:                  # robot must be in open space
        safe_stop(robot)
        samples = np.array([robot.get_proximity() for _ in range(a.auto_baseline)], float)
        base, std = samples.mean(axis=0).tolist(), samples.std(axis=0).tolist()
        print("auto baseline:", np.round(base, 1).tolist())
    adapter = SensorAdapter(mode=a.norm, ir_range=a.ir_range, order=a.sensor_order,
                            baseline=base, baseline_std=std, ir_gain=a.ir_gain,
                            ir_shift=a.ir_shift, nsigma=a.ir_nsigma, margin=a.ir_margin)
    kw, fkw = {}, None
    if a.controller == "forager":
        a.floor = True                       # the forager needs the ground sensors
        fkw = dict(food_threshold=a.food_threshold, spins=a.spins, spin_speed=a.spin_speed,
                   advance_time=a.advance_time, leave_time=a.leave_time,
                   ignore_time=a.ignore_time, wander_interval=a.wander_interval,
                   wander_time=a.wander_time, seed=a.seed)
    if a.controller in ("braitenberg", "forager"):
        steps = lambda sec: max(1, int(round(sec * a.hz)))        # noqa: E731
        kw = dict(mode=a.brait_mode, escape=not a.no_escape, stuck_thr=a.stuck_thr,
                  patience=steps(a.stuck_time), back_steps=steps(a.back_time),
                  turn_steps=steps(a.turn_time))
    ctrl = load_controller(a.controller, a.ctrl_arg, max_speed=a.max_speed,
                           forager_kw=fkw, **kw)
    ctrl.reset()

    stop_flag = {"stop": False}
    signal.signal(signal.SIGINT, lambda *_: stop_flag.update(stop=True))
    period = 1.0 / a.hz
    max_steps = a.steps if a.steps else int(a.duration * a.hz)
    pose_ok = t.pose() is not None
    log_f = open(a.log, "w", newline="") if a.log else None
    writer = None
    if log_f:
        cols = (["step", "t", "loop_ms"] + [f"raw{i}" for i in range(8)]
                + [f"prox{i}" for i in range(8)] + ["gs0", "gs1", "gs2"]
                + ["v_l", "v_r", "cmd_l", "cmd_r", "link_errors"])
        if pose_ok:
            cols += ["x", "y", "yaw_deg"]
        if hasattr(ctrl, "state"):
            cols += ["state"]
        writer = csv.writer(log_f)
        writer.writerow(cols)

    if a.log:
        meta = {"backend": a.backend, "hz": a.hz, "layout": t.layout(),
                "date": time.strftime("%Y-%m-%d %H:%M:%S"), "args": vars(a)}
        with open(a.log[:-4] + "_meta.json" if a.log.endswith(".csv") else a.log + ".meta.json", "w") as f:
            json.dump(meta, f, indent=2)

    loop_ms, overruns, consecutive, step = [], 0, 0, 0
    t_start = time.perf_counter()
    rt = realtime(a)
    try:
        # real time: --duration is wall-clock seconds (a slow link must not
        # stretch the run); simulated time: a number of steps
        def more():
            if a.steps or not rt:
                return step < max_steps
            return time.perf_counter() - t_start < a.duration
        while more() and not stop_flag["stop"]:
            t0 = time.perf_counter()
            # time axis: wall clock when running in real time, else simulated time
            t_now = (t0 - t_start) if rt else step * period
            try:
                raw = robot.get_proximity()
                floor = robot.get_floor() if a.floor else [0, 0, 0]
                pose = t.pose() if pose_ok else None   # same instant as the sensors
                prox = adapter.normalize(raw)
                obs = {"prox": prox, "prox_native": adapter.last_native, "prox_raw": raw,
                       "floor": floor, "t": t_now, "step": step}
                vl, vr = ctrl.step(obs)
                cl, cr = robot.set_speed(*apply_motion(steps_from_mps(vl),
                                                       steps_from_mps(vr), a))
                consecutive = 0
            except BackendClosed as e:
                print(f"[step {step}] {e}: stopping, the log is kept", file=sys.stderr)
                break
            except LinkError as e:
                consecutive += 1
                print(f"[step {step}] link error: {e}", file=sys.stderr)
                if consecutive >= a.max_link_errors:
                    print("too many consecutive link errors, stopping", file=sys.stderr)
                    break
                continue
            dt = time.perf_counter() - t0
            loop_ms.append(dt * 1e3)
            if writer:
                row = ([step, f"{t_now:.3f}", f"{dt * 1e3:.1f}"] + list(raw)
                       + [f"{v:.3f}" for v in prox] + list(floor)
                       + [f"{vl:.4f}", f"{vr:.4f}", cl, cr, robot.errors])
                if pose_ok:
                    row += [f"{v:.4f}" for v in pose]
                if hasattr(ctrl, "state"):
                    row += [ctrl.state]
                writer.writerow(row)
            if a.verbose and step % a.print_every == 0:
                print(f"{step:5d} prox {np.round(prox, 2)} floor {floor} -> cmd {cl:5d} {cr:5d}")
            step += 1
            if rt:
                rest = period - (time.perf_counter() - t0)
                if rest > 0:
                    time.sleep(rest)
                elif a.backend == "real":            # the smooth GUI uses the whole period on purpose
                    overruns += 1
    finally:
        safe_stop(robot)
        robot.close()
        if log_f:
            log_f.close()

    t_loop_end = time.perf_counter()
    lm = np.array(loop_ms) if loop_ms else np.zeros(1)
    print(f"backend {a.backend}: steps {step}, loop mean {lm.mean():.1f} ms, "
          f"p95 {np.percentile(lm, 95):.1f} ms, overruns {overruns}, link errors {robot.errors}")
    if hasattr(ctrl, "escapes"):
        print(f"escapes: {ctrl.escapes}")
    if hasattr(ctrl, "summary"):
        summ = ctrl.summary()
        print("food:", summ)
        if a.log:
            with open(a.log[:-4] + "_summary.json", "w") as f:
                json.dump(summ, f, indent=2)
    if a.backend != "real":
        print(f"steps touching an obstacle: {t.contacts()}")
    if a.log:
        print("log:", a.log)
        if a.plot and step > 1:
            from types import SimpleNamespace
            from .plot import plot_run
            plot_run(SimpleNamespace(log=a.log, out=a.log[:-4], dpi=150))
    if rt:
        print(f"time: control loop {t_loop_end - t_start:.1f} s; whole command "
              f"{time.perf_counter() - t_launch:.1f} s (includes connecting, stopping and figures)")
    return step


def cmd_drive(a):
    t = make_transport(a)
    robot = open_robot(t, a)
    n = int(round(a.seconds * a.hz))
    try:
        confirm_start(robot, a, "drive")
        s0 = robot.get_steps()
        p0 = t.pose()
        yaw_acc, prev = 0.0, p0
        t_start = time.perf_counter()
        for _ in range(n):
            t0 = time.perf_counter()
            robot.set_speed(*apply_motion(a.left, a.right, a))
            if p0 is not None:                       # unwrap: several turns possible
                cur = t.pose()
                yaw_acc += (cur[2] - prev[2] + 180) % 360 - 180
                prev = cur
            if realtime(a):
                time.sleep(max(0.0, 1.0 / a.hz - (time.perf_counter() - t0)))
        elapsed = time.perf_counter() - t_start
        safe_stop(robot)
        s1 = robot.get_steps()
        p1 = t.pose()
    except (KeyboardInterrupt, LinkError):
        safe_stop(robot)
        robot.close()
        raise
    else:
        robot.close()
    T = n / a.hz if not realtime(a) else elapsed
    vl, vr = a.left * STEP_LEN, a.right * STEP_LEN
    print(f"command {a.left} / {a.right} steps/s for {T:.2f} s")
    if (a.speed_scale, a.turn_scale, a.trim) != (1.0, 1.0, 0.0):
        l, r = apply_motion(a.left, a.right, a)
        print(f"sent after motion knobs: {l:.0f} / {r:.0f} steps/s "
              f"(speed_scale {a.speed_scale}, turn_scale {a.turn_scale}, trim {a.trim})")
    print(f"expected from kinematics: distance {100 * 0.5 * (vl + vr) * T:.1f} cm, "
          f"rotation {math.degrees((vr - vl) / AXLE * T):.1f} deg (CCW +)")
    print(f"step counters: left {step_diff(s0[0], s1[0])}, right {step_diff(s0[1], s1[1])}")
    if p0 is not None:
        d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        print(f"{a.backend} pose: distance {100 * d:.1f} cm, rotation {yaw_acc:.1f} deg")
    else:
        print("measure the real distance / rotation and compare with the expected values;"
              " see docs/calibration.md for speed_scale, turn_scale and trim")


def cmd_approach(a):
    start = None
    if a.backend != "real" and not a.start:
        half = (a.arena or (0.5 if a.backend == "mock" else 1.0)) / 2
        start = (half - BODY_RADIUS - a.start_dist, 0.0, 0.0)   # facing the +x wall
    t = make_transport(a, start=start)
    robot = open_robot(t, a)
    rows = []
    try:
        confirm_start(robot, a, "approach (front 10 cm from the wall, square to it)")
        s0 = robot.get_steps()
        t_start = time.perf_counter()
        for k in range(int(a.timeout_s * a.hz)):
            t0 = time.perf_counter()
            raw = robot.get_proximity()
            s = robot.get_steps()
            travelled = 0.5 * (step_diff(s0[0], s[0]) + step_diff(s0[1], s[1])) * STEP_LEN
            est = a.start_dist - travelled
            rows.append([k, f"{t0 - t_start:.3f}", f"{est:.4f}"] + list(raw))
            if max(raw[0], raw[7]) > a.stop_raw or est < 0.002:
                break
            robot.set_speed(*apply_motion(a.speed, a.speed, a))
            if realtime(a):
                time.sleep(max(0.0, 1.0 / a.hz - (time.perf_counter() - t0)))
    finally:
        safe_stop(robot)
        robot.close()
    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step", "t", "est_dist"] + [f"raw{i}" for i in range(8)])
        w.writerows(rows)
    print(f"{len(rows)} samples, final estimated distance {rows[-1][2]} m, saved {a.out}")


# ------------------------------------------------------------------ parser
def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    common = argparse.ArgumentParser(add_help=False)
    g = common.add_argument_group("backend")
    g.add_argument("--backend", choices=["mock", "sim", "real"], default="mock")
    g.add_argument("--hz", type=float, default=10.0, help="control rate")
    g.add_argument("--seed", type=int, default=0)
    g.add_argument("--start", default="", help="x,y,yaw_deg (mock/sim; default random)")
    g.add_argument("--arena", type=float, default=None, help="arena side [m] (mock 0.5, sim from config)")
    g.add_argument("--obstacles", default="", help='mock/sim cylinders, JSON [[x,y,r],...]')
    g.add_argument("--wall-latency", type=float, default=0.0,
                   help="mock/sim: real seconds slept per command (timing realism)")
    g.add_argument("--realtime", action="store_true", help="mock/sim: keep --hz in wall-clock time")
    r = common.add_argument_group("real robot")
    r.add_argument("--port", help="serial port, e.g. /dev/rfcomm0, /dev/tty.e-puck_1234, COM5")
    r.add_argument("--timeout", type=float, default=0.5)
    r.add_argument("--retries", type=int, default=2)
    r.add_argument("--eol", choices=["cr", "lf", "crlf"], default="cr")
    r.add_argument("--no-confirm", action="store_true",
                   help="drive/approach: start right after connecting, without waiting for Enter")
    r.add_argument("--connect-timeout", type=float, default=10.0,
                   help="seconds to wait for the Bluetooth link to answer (0 = do not wait)")
    s = common.add_argument_group("sim (PyBullet twin)")
    s.add_argument("--sim-config", default="", help="default sim/config.json")
    s.add_argument("--gui", action="store_true")
    s.add_argument("--sim-latency", type=float, default=0.0,
                   help="seconds between sensor read and new wheel command taking effect")
    s.add_argument("--no-sim-noise", action="store_true")
    s.add_argument("--n-rays", type=int, default=None, help="IR rays per sensor (config default)")
    s.add_argument("--food", default="", help="dark floor patch x,y,r (ground sensors)")
    m = common.add_argument_group("mock")
    m.add_argument("--mock-noise", type=float, default=0.03)
    mv = common.add_argument_group("motion calibration (all backends)")
    mv.add_argument("--speed-scale", type=float, default=1.0)
    mv.add_argument("--turn-scale", type=float, default=1.0)
    mv.add_argument("--trim", type=float, default=0.0,
                    help="wheel balance; positive slows left / speeds right")
    common.add_argument("--profile", default="",
                        help="JSON of option defaults (e.g. calib/profile_2473_real.json); "
                             "options given on the command line win")

    pr = sub.add_parser("probe", parents=[common])
    pr.add_argument("--n", type=int, default=50)

    pb = sub.add_parser("baseline", parents=[common])
    pb.add_argument("--n", type=int, default=100)
    pb.add_argument("--calibrate", action="store_true", help="send K first (real robot)")
    pb.add_argument("--out", default="calib/baseline.json")

    pn = sub.add_parser("run", parents=[common])
    c = pn.add_argument_group("controller")
    c.add_argument("--controller", default="braitenberg",
                   help="'braitenberg', 'forager' (food-finding demo) or 'file.py:Class'")
    c.add_argument("--ctrl-arg", default="controllers/braitenberg_avoidance.json")
    c.add_argument("--max-speed", type=float, default=None, help="m/s (default: from JSON)")
    c.add_argument("--brait-mode", choices=["relative", "absolute"], default="relative")
    c.add_argument("--no-escape", action="store_true")
    c.add_argument("--stuck-thr", type=float, default=0.6)
    c.add_argument("--stuck-time", type=float, default=1.5,
                   help="seconds near an obstacle before escaping (any --hz)")
    c.add_argument("--back-time", type=float, default=0.6, help="escape: seconds backing off")
    c.add_argument("--turn-time", type=float, default=0.8, help="escape: seconds turning")
    fg = pn.add_argument_group("forager (food-finding demo)")
    fg.add_argument("--food-threshold", type=float, default=500.0,
                    help="raw ground value below which the floor counts as food (twin ~500, 2292 ~330)")
    fg.add_argument("--spins", type=float, default=5, help="full turns on the food")
    fg.add_argument("--spin-speed", type=float, default=1.0, help="fraction of max speed while spinning")
    fg.add_argument("--advance-time", type=float, default=1.5,
                    help="s at half speed moving onto the patch once seen (the ground sensors are "
                         "3 cm ahead of the body centre)")
    fg.add_argument("--leave-time", type=float, default=2.0, help="s driving straight off the patch")
    fg.add_argument("--ignore-time", type=float, default=4.0, help="s ignoring food after leaving")
    fg.add_argument("--wander-interval", type=float, default=4.0, help="mean s between random turns")
    fg.add_argument("--wander-time", type=float, default=0.6, help="s of each random turn")
    k = pn.add_argument_group("sensor calibration")
    k.add_argument("--norm", choices=["lookup", "linear"], default="lookup")
    k.add_argument("--ir-range", type=float, default=0.07)
    k.add_argument("--ir-gain", type=float, default=1.0)
    k.add_argument("--ir-shift", type=float, default=0.0,
                   help="m added to the IR distance (white walls read closer: ~0.035)")
    k.add_argument("--ir-nsigma", type=float, default=3.0)
    k.add_argument("--ir-margin", type=float, default=0.0,
                   help="raw counts above the background needed to detect")
    k.add_argument("--auto-baseline", type=int, default=0,
                   help="N reads in open space at start, instead of --calib")
    k.add_argument("--sensor-order", choices=["native", "ring"], default="native")
    k.add_argument("--calib", default="", help="JSON written by 'baseline'")
    o = pn.add_argument_group("run")
    o.add_argument("--duration", type=float, default=30.0)
    o.add_argument("--steps", type=int, default=0)
    o.add_argument("--floor", action="store_true", help="read ground sensors every step")
    o.add_argument("--max-link-errors", type=int, default=5)
    o.add_argument("--log", default="", help="CSV path, or 'auto' for runs/<date>_<target>.csv")
    o.add_argument("--plot", action="store_true", help="write the figures after the run")
    o.add_argument("--video", default="", help="sim: record an MP4 (no window needed), e.g. runs/demo.mp4")
    o.add_argument("--video-fps", type=int, default=25)
    o.add_argument("--video-size", default="800x600", help="WIDTHxHEIGHT")
    o.add_argument("--video-yaw", type=float, default=0.0,
                   help="degrees: turn the camera around the arena, counter-clockwise seen from above")
    o.add_argument("--video-top", action="store_true", help="camera straight above the arena")
    o.add_argument("--verbose", action="store_true")
    o.add_argument("--print-every", type=int, default=10)

    pd = sub.add_parser("drive", parents=[common])
    pd.add_argument("--left", type=int, default=500)
    pd.add_argument("--right", type=int, default=500)
    pd.add_argument("--seconds", type=float, default=5.0)

    pa = sub.add_parser("approach", parents=[common])
    pa.add_argument("--start-dist", type=float, default=0.10,
                    help="front of the robot to the wall at start [m]")
    pa.add_argument("--speed", type=int, default=100, help="steps/s (100 = 1.3 cm/s)")
    pa.add_argument("--stop-raw", type=int, default=1500)
    pa.add_argument("--timeout-s", type=float, default=20.0)
    pa.add_argument("--out", default="calib/approach.csv")
    return p


VALUE_OPTS = ("--start", "--food", "--obstacles")


def join_negative_values(argv):
    """argparse reads '--start -0.35,0,0' as two options; join such pairs
    into '--start=-0.35,0,0' so negative coordinates work either way."""
    out, i = [], 0
    while i < len(argv):
        if argv[i] in VALUE_OPTS and i + 1 < len(argv) and argv[i + 1][:2].lstrip("-")[:1] in "0123456789.":
            out.append(f"{argv[i]}={argv[i + 1]}")
            i += 2
        else:
            out.append(argv[i])
            i += 1
    return out


def load_profile(path):
    """Profile JSON: option names with - or _ as keys; keys starting with _
    are comments. Returns a dict of argparse dests."""
    with open(path) as f:
        raw = json.load(f)
    return {k.replace("-", "_"): v for k, v in raw.items() if not k.startswith("_")}


def parse_args(argv):
    argv = join_negative_values(list(argv))
    parser = build_parser()
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--profile", default="")
    known, _ = pre.parse_known_args(argv)
    if known.profile:
        prof = load_profile(known.profile)
        subs = parser._subparsers._group_actions[0].choices.values()
        dests = {act.dest for sp in subs for act in sp._actions}
        unknown = sorted(set(prof) - dests)
        if unknown:
            raise SystemExit(f"error: unknown options in profile {known.profile}: {unknown}")
        for sp in subs:
            mine = {act.dest for act in sp._actions}
            sp.set_defaults(**{k: v for k, v in prof.items() if k in mine})
    return parser.parse_args(argv)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    a = parse_args(argv)
    try:
        return _dispatch(a)
    except BackendClosed as e:
        print(f"{e}: stopped", file=sys.stderr)
        return None
    except LinkError as e:
        print(f"link error: {e}", file=sys.stderr)
        sys.exit(2)
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        sys.exit(130)


def _dispatch(a):
    for attr in ("out", "log"):
        path = getattr(a, attr, "")
        if path and "/" in path:
            import os
            os.makedirs(os.path.dirname(path), exist_ok=True)
    return {"probe": cmd_probe, "baseline": cmd_baseline, "run": cmd_run,
            "drive": cmd_drive, "approach": cmd_approach}[a.cmd](a)


if __name__ == "__main__":
    main()
