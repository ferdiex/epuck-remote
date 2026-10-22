#!/usr/bin/env python3
"""
Smoke tests for epuck-remote, no robot needed. Sim tests run only if
PyBullet is installed. Run from the repo root: python3 tests/test_smoke.py
"""
import csv
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
os.chdir(ROOT)

from epuck_remote import cli                                    # noqa: E402
from epuck_remote.adapter import SensorAdapter                  # noqa: E402
from epuck_remote.controllers import BraitenbergJSON            # noqa: E402
from epuck_remote.model import (STEP_LEN, IR_FAR_RAW, ir_distance_from_raw,
                                ir_raw_from_distance)          # noqa: E402
from epuck_remote.protocol import AsercomRobot, LinkError       # noqa: E402
from epuck_remote.transport_mock import MockTransport           # noqa: E402

BRAIT = "controllers/braitenberg_avoidance.json"
TMP = os.environ.get("TMPDIR", "/tmp")
try:
    import pybullet  # noqa: F401
    HAVE_PB = True
except ImportError:
    HAVE_PB = False
results = []


def check(name, fn, needs_pb=False):
    if needs_pb and not HAVE_PB:
        results.append((name, None, 0.0, "skipped: PyBullet not installed"))
        return
    t0 = time.perf_counter()
    try:
        fn()
        results.append((name, True, time.perf_counter() - t0, ""))
    except Exception as e:
        results.append((name, False, time.perf_counter() - t0, repr(e)))


def run_cli(args):
    return cli.main(args)


def path_stats(log):
    rows = list(csv.DictReader(open(log)))
    xs = np.array([float(r["x"]) for r in rows])
    ys = np.array([float(r["y"]) for r in rows])
    return float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))


# ------------------------------------------------------------- pure python
def t_lookup_roundtrip():
    for d in [0.002, 0.008, 0.02, 0.035, 0.055, 0.069]:
        assert abs(ir_distance_from_raw(ir_raw_from_distance(d)) - d) < 1e-6, d


def t_normalize():
    ad = SensorAdapter()
    assert np.allclose(ad.normalize([30] * 8), 0.0)
    assert np.allclose(ad.normalize([IR_FAR_RAW] * 8), 0.0)      # sim far reading
    near = ad.normalize([4095] + [30] * 7)
    assert near[0] > 0.99 and np.allclose(near[1:], 0.0)
    mid = ad.normalize([ir_raw_from_distance(0.035)] * 8)
    assert np.allclose(mid, 0.5, atol=1e-6), mid


def t_baseline_offset():
    ad = SensorAdapter(baseline=[20] * 8, baseline_std=[3] * 8)
    assert np.allclose(ad.normalize([25] * 8), 0.0)                  # background
    raw = ir_raw_from_distance(0.035) - IR_FAR_RAW + 20              # shifted real reading
    assert np.allclose(ad.normalize([raw] * 8), 0.5, atol=1e-6)


def t_ring_order():
    ad = SensorAdapter(order="ring")
    r = ad.reorder(np.arange(8, dtype=float) / 10)
    assert r.tolist() == [0.7, 0.6, 0.5, 0.4, 0.4, 0.3, 0.2, 0.1], r


def t_protocol_mock():
    rb = AsercomRobot(MockTransport())
    assert rb.version().startswith("v,")
    assert len(rb.get_proximity()) == 8 and len(rb.get_floor()) == 3
    assert rb.set_speed(5000, -5000) == (1000, -1000)
    assert rb.get_speed() == [1000, -1000]


def t_link_error():
    class Silent(MockTransport):
        def read_line(self):
            return ""
    rb = AsercomRobot(Silent(), retries=1)
    try:
        rb.get_proximity()
    except LinkError:
        assert rb.errors == 2
        return
    raise AssertionError("expected LinkError")


def t_braitenberg_turns_away():
    c = BraitenbergJSON(BRAIT)
    right = np.zeros(8); right[0] = right[1] = 0.8                   # obstacle front-right
    left = np.zeros(8); left[6] = left[7] = 0.8                      # obstacle front-left
    vl, vr = c.step({"prox": right})
    assert vl < vr, (vl, vr)
    vl, vr = c.step({"prox": left})
    assert vl > vr, (vl, vr)


def t_mock_kinematics():
    t = MockTransport(dt=0.1, start=(0.0, 0.0, 0.0))
    rb = AsercomRobot(t)
    for _ in range(10):
        rb.set_speed(500, 500)
    assert abs(t.x - 10 * 0.1 * 500 * STEP_LEN) < 1e-9


def t_mock_closed_loop():
    log = f"{TMP}/epuck_remote_mock.csv"
    run_cli(["run", "--backend", "mock", "--steps", "1500", "--log", log,
             "--obstacles", "[[0.08,0.08,0.04],[-0.08,-0.06,0.04]]"])
    assert path_stats(log) > 3.0


# ------------------------------------------------------------- PyBullet twin
def t_model_matches_sim():
    from sim import sensors as S
    from epuck_remote import model as M
    for name in ("IR_POS", "IR_DIR", "IR_LOOKUP", "GS_LOOKUP"):
        assert np.allclose(getattr(S, name), getattr(M, name)), name


def sim(**kw):
    from epuck_remote.transport_sim import SimTransport
    return SimTransport(**kw)


def t_sim_protocol():
    t = sim(start=(0.0, 0.0, 0.0))
    rb = AsercomRobot(t)
    assert rb.version().startswith("v,")
    prox = rb.get_proximity()
    assert len(prox) == 8 and max(prox) < IR_FAR_RAW * 1.3, prox     # open space
    assert len(rb.get_floor()) == 3
    rb.close()


def t_sim_kinematics():
    t = sim(start=(0.0, 0.0, 0.0), dt=0.1)
    rb = AsercomRobot(t)
    for _ in range(20):                                   # 2 s at 500 steps/s
        rb.set_speed(500, 500)
    x, y, yaw = t.pose()
    expected = 2.0 * 500 * STEP_LEN
    assert abs(x - expected) / expected < 0.10, (x, expected)
    assert abs(y) < 0.01 and abs(yaw) < 5, (y, yaw)
    rb.close()


def t_sim_sees_wall():
    half = 0.5
    t = sim(start=(half - 0.037 - 0.02, 0.0, 0.0), noise=False)
    rb = AsercomRobot(t)
    prox = rb.get_proximity()
    assert min(prox[0], prox[7]) > 200 and max(prox[2:6]) < IR_FAR_RAW * 1.3, prox
    rb.close()


def t_sim_floor_patch():
    t = sim(start=(0.0, 0.0, 0.0), food=(0.03, 0.0, 0.05), noise=False)
    rb = AsercomRobot(t)
    on = rb.get_floor()
    rb.close()
    t = sim(start=(0.0, 0.0, 0.0), noise=False)
    rb = AsercomRobot(t)
    off = rb.get_floor()
    rb.close()
    assert np.mean(on) < 500 < np.mean(off), (on, off)


def t_sim_closed_loop():
    log = f"{TMP}/epuck_remote_sim.csv"
    run_cli(["run", "--backend", "sim", "--steps", "600", "--log", log, "--seed", "1",
             "--sim-latency", "0.03", "--obstacles", "[[0.2,0.2,0.05],[-0.2,-0.1,0.05]]"])
    assert path_stats(log) > 3.0


def t_sim_window_closed():
    """Closing the GUI window mid-run (simulated with p.disconnect from a
    controller) must stop cleanly and keep the log."""
    ctrl = f"{TMP}/epuck_remote_closer.py"
    with open(ctrl, "w") as f:
        f.write("import pybullet\n"
                "class Closer:\n"
                "    def reset(self): pass\n"
                "    def step(self, obs):\n"
                "        if obs['step'] == 20: pybullet.disconnect()\n"
                "        return 0.05, 0.05\n")
    log = f"{TMP}/epuck_remote_closed.csv"
    run_cli(["run", "--backend", "sim", "--steps", "100", "--log", log,
             "--controller", f"{ctrl}:Closer", "--ctrl-arg", ""])
    rows = list(csv.DictReader(open(log)))
    assert 15 <= len(rows) <= 21, len(rows)


def t_serial_interface():
    """SerialTransport must offer every method the CLI calls on a transport
    (it cannot be opened without a robot, so check the class itself)."""
    from epuck_remote.transport_serial import SerialTransport
    for name in ("write_line", "read_line", "close", "pose", "layout", "contacts"):
        assert callable(getattr(SerialTransport, name, None)), name


def t_ir_shift():
    ad = SensorAdapter(ir_shift=0.035)
    n = ad.normalize([ir_raw_from_distance(0.02)] * 8)
    assert np.allclose(n, 1 - 0.055 / 0.07, atol=1e-6), n          # 2 cm + 3.5 cm
    assert np.allclose(ad.normalize([ir_raw_from_distance(0.04)] * 8), 0.0)


def t_ir_margin():
    """e-puck 2473: ps5 baseline 60, read 85 in another place with nothing
    near. Without margin that is a detection; with margin 30 it is not."""
    import json
    c = json.load(open("calib/baseline_2473.json"))
    raw = [30, 25, 45, 55, 75, 85, 46, 60]
    n0 = SensorAdapter(baseline=c["baseline"], baseline_std=c["std"]).normalize(raw)
    n30 = SensorAdapter(baseline=c["baseline"], baseline_std=c["std"], margin=30).normalize(raw)
    assert n0[5] > 0 and np.allclose(n30, 0.0), (n0, n30)


def t_apply_motion():
    from types import SimpleNamespace as NS
    a = NS(speed_scale=0.93, turn_scale=1.10, trim=0.0)
    l, r = cli.apply_motion(500, 500, a)
    assert np.isclose(l, 465) and np.isclose(r, 465)               # straight: speed only
    l, r = cli.apply_motion(-300, 300, a)
    assert np.isclose(r, 300 * 0.93 * 1.10) and np.isclose(l, -r)  # spin: speed x turn
    l, r = cli.apply_motion(500, 500, NS(speed_scale=1.0, turn_scale=1.0, trim=0.02))
    assert np.isclose(l, 490) and np.isclose(r, 510)


def t_profile():
    a = cli.parse_args(["run", "--profile", "calib/profile_2473_real.json"])
    assert (a.backend, a.port, a.hz, a.speed_scale, a.ir_shift) == ("real", "COM5", 5, 0.93, 0.035)
    a = cli.parse_args(["run", "--profile", "calib/profile_2473_real.json", "--hz", "4"])
    assert a.hz == 4.0                                               # command line wins
    bad = f"{TMP}/epuck_remote_bad_profile.json"
    open(bad, "w").write('{"hz": 5, "no_such_option": 1}')
    try:
        cli.parse_args(["run", "--profile", bad])
    except SystemExit:
        return
    raise AssertionError("unknown profile key accepted")


def t_auto_log_plot_baseline():
    import glob
    import shutil
    existed = os.path.isdir("runs")
    before = set(glob.glob("runs/*"))
    run_cli(["run", "--backend", "mock", "--hz", "5", "--steps", "60", "--log", "auto",
             "--plot", "--auto-baseline", "10", "--stuck-time", "1.6"])
    new = set(glob.glob("runs/*")) - before
    try:
        assert any(f.endswith(".csv") for f in new), new
        assert any(f.endswith("_trajectory.png") for f in new), new
        assert any(f.endswith("_meta.json") for f in new), new
    finally:
        for f in new:
            os.remove(f)
        if not existed:
            shutil.rmtree("runs", ignore_errors=True)


def t_estimate_shift():
    """A synthetic wall that reads exactly like the model 3 cm closer must
    give an estimated ir_shift of 3 cm."""
    from epuck_remote.plot import estimate_shift, model_front_raw
    dist = np.linspace(0.031, 0.09, 60)
    raw = model_front_raw(dist - 0.03)
    sh = estimate_shift(dist, raw)
    assert sh is not None and abs(sh - 0.03) < 0.002, sh


def t_trim_formula():
    """docs/calibration.md: trim = 5.2 * y / D^2 (cm). A mock robot whose left
    wheel runs 2% fast drifts right; the trim from the formula cancels it."""
    from types import SimpleNamespace as NS

    def drive(trim):
        t = MockTransport(arena=3.0, dt=0.1, start=(0.0, 0.0, 0.0))
        a = NS(speed_scale=1.0, turn_scale=1.0, trim=trim)
        for _ in range(50):
            l, r = cli.apply_motion(500, 500, a)
            t.apply_speed(l * 1.02, r * 0.98)
        return 100 * t.x, -100 * t.y
    D, y = drive(0.0)
    D2, y2 = drive(5.2 * y / D ** 2)
    assert y > 3 and abs(y2) < 0.3, (y, y2)


def t_wait_ready_slow_link():
    """What a MacBook showed with robot 2473: no answer for ~2.5 s after the
    port opens, one garbled reply, then normal answers (V has two lines)."""
    class SlowLink(MockTransport):
        writes = 0

        def write_line(self, text):
            self.writes += 1
            if self.writes <= 4:
                return                                   # link not up yet
            if self.writes == 5:
                self._out.append("z,Command not found")  # first command garbled
                return
            super().write_line(text)
    rb = AsercomRobot(SlowLink())
    line = rb.wait_ready(timeout=10, period=0.01)
    assert line.startswith("v,"), line
    assert len(rb.get_proximity()) == 8            # nothing left over from the V reply
    assert rb.set_speed(100, 100) == (100, 100)


def t_lost_link_warning():
    """If the link dies while the robot moves, safe_stop must say so."""
    import io
    import contextlib

    class Dead(MockTransport):
        def read_line(self):
            return ""
    rb = AsercomRobot(Dead(), retries=0)
    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        ok = cli.safe_stop(rb, tries=2)
    assert ok is False and "Switch it off" in err.getvalue()


def t_step_diff():
    assert cli.step_diff(10, 2510) == 2500
    assert cli.step_diff(65000, 1964) == 2500               # wrapped forwards
    assert cli.step_diff(2000, 65036) == -2500              # wrapped backwards


def t_confirm_start():
    """Real robot: wait for Enter, stopping the wheels before and after;
    mock/sim and --no-confirm: no prompt."""
    import builtins
    from types import SimpleNamespace as NS
    t = MockTransport(start=(0.0, 0.0, 0.0))
    rb = AsercomRobot(t)
    rb.set_speed(300, -300)                                  # a jolt at connection
    asked = []
    real_input = builtins.input
    builtins.input = lambda prompt="": asked.append(prompt) or ""
    try:
        cli.confirm_start(rb, NS(backend="real", no_confirm=False), "approach")
        assert len(asked) == 1 and t.cmd_l == 0 and t.cmd_r == 0
        cli.confirm_start(rb, NS(backend="real", no_confirm=True), "approach")
        cli.confirm_start(rb, NS(backend="mock", no_confirm=False), "approach")
        assert len(asked) == 1
    finally:
        builtins.input = real_input


def t_forager_mock():
    """The food demo finds the patch in the mock arena, spins, leaves and
    logs its state; the summary is written next to the log."""
    import json
    log = f"{TMP}/epuck_remote_forager.csv"
    run_cli(["run", "--backend", "mock", "--controller", "forager", "--arena", "0.63",
             "--food", "0,0,0.075", "--hz", "5", "--max-speed", "0.08", "--duration", "120",
             "--start", "-0.2,-0.2,45", "--seed", "1", "--log", log])
    rows = list(csv.DictReader(open(log)))
    states = {r["state"] for r in rows}
    assert {"search", "spin", "leave"} <= states, states
    summ = json.load(open(log[:-4] + "_summary.json"))
    assert summ["finds"] >= 1, summ
    assert all(r["gs0"] != "0" for r in rows)              # --floor switched on by itself


def t_sim_video():
    """--video writes an MP4 of the twin (skipped without imageio)."""
    try:
        import imageio  # noqa: F401
        import imageio_ffmpeg  # noqa: F401
    except ImportError:
        return
    out = f"{TMP}/epuck_remote_test.mp4"
    if os.path.exists(out):
        os.remove(out)
    run_cli(["run", "--backend", "sim", "--steps", "10", "--video", out,
             "--video-size", "320x240", "--log", f"{TMP}/epuck_remote_video.csv"])
    import imageio.v3 as iio
    frames = iio.imread(out, index=None)
    assert frames.shape[0] >= 20 and frames.shape[1:3] == (240, 320), frames.shape


TESTS = [
    ("lookup roundtrip", t_lookup_roundtrip, False),
    ("normalize", t_normalize, False),
    ("baseline offset", t_baseline_offset, False),
    ("ring order", t_ring_order, False),
    ("protocol on mock", t_protocol_mock, False),
    ("link error + retries", t_link_error, False),
    ("serial transport interface", t_serial_interface, False),
    ("wait for a slow Bluetooth link", t_wait_ready_slow_link, False),
    ("warn when the robot cannot be stopped", t_lost_link_warning, False),
    ("16-bit step counters wrap", t_step_diff, False),
    ("forager finds food (mock)", t_forager_mock, False),
    ("pause before drive/approach (real)", t_confirm_start, False),
    ("braitenberg turns away", t_braitenberg_turns_away, False),
    ("ir_shift (white walls)", t_ir_shift, False),
    ("ir_margin (ps5 drift, 2473)", t_ir_margin, False),
    ("motion knobs", t_apply_motion, False),
    ("trim formula", t_trim_formula, False),
    ("profiles", t_profile, False),
    ("ir_shift estimate from approach", t_estimate_shift, False),
    ("log auto + plot + auto baseline", t_auto_log_plot_baseline, False),
    ("mock kinematics", t_mock_kinematics, False),
    ("mock closed loop 1500 steps", t_mock_closed_loop, False),
    ("model constants = sim/sensors", t_model_matches_sim, True),
    ("protocol on sim", t_sim_protocol, True),
    ("sim kinematics 2 s", t_sim_kinematics, True),
    ("sim IR sees wall at 2 cm", t_sim_sees_wall, True),
    ("sim ground sensors on patch", t_sim_floor_patch, True),
    ("sim closed loop 600 steps", t_sim_closed_loop, True),
    ("sim window closed mid-run", t_sim_window_closed, True),
    ("sim video (MP4)", t_sim_video, True),
]

if __name__ == "__main__":
    for name, fn, pb in TESTS:
        check(name, fn, pb)
    print()
    for name, ok, dt, err in results:
        tag = "SKIP" if ok is None else ("PASS" if ok else "FAIL")
        print(f"{tag}  {name:32s} {dt * 1e3:9.1f} ms  {err}")
    sys.exit(0 if all(r[1] is not False for r in results) else 1)
