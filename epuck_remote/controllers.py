"""
controllers.py - controllers return wheel speeds in m/s from an observation
dict: {"prox": normalized IR (chosen order), "prox_native": normalized IR
ps0..ps7, "prox_raw": raw ps0..ps7, "floor": raw gs0..gs2, "t", "step"}.

Any class with reset() and step(obs) -> (v_l, v_r) can be loaded with
--controller path/to/file.py:ClassName (--ctrl-arg is passed to __init__).
"""
import importlib.util
import json

import numpy as np


class BraitenbergJSON:
    """Continuous Braitenberg from the project JSON format
    (left_weights/right_weights/left_bias/right_bias/max_speed).

    mode "absolute": v = clip(w.s + b, -max, max), the literal JSON reading.
      With bias > max_speed both wheels saturate and steering is lost.
    mode "relative": v = max * clip((w.s + b) / |b|, -1, 1), so free space
      gives full speed and the weights keep their relative effect.

    Escape (as controllers.BraitenbergController in the essim project): if
    the strongest sensor stays above stuck_thr for `patience` steps, back
    off and turn, alternating the turn side each time."""

    def __init__(self, path, max_speed=None, mode="relative", escape=True,
                 stuck_thr=0.6, patience=15, back_steps=6, turn_steps=8,
                 escape_speed=0.6):
        with open(path) as f:
            cfg = json.load(f)
        self.wl = np.asarray(cfg["left_weights"], float)
        self.wr = np.asarray(cfg["right_weights"], float)
        self.bl = float(cfg.get("left_bias", 0.3))
        self.br = float(cfg.get("right_bias", 0.3))
        self.max_speed = float(max_speed if max_speed is not None else cfg.get("max_speed", 0.12))
        self.mode = mode
        self.ref = max(abs(self.bl), abs(self.br)) or 1.0
        self.escape = escape
        self.stuck_thr = stuck_thr
        self.patience = patience
        self.back_steps = back_steps
        self.turn_steps = turn_steps
        self.escape_speed = escape_speed
        self.reset()

    def reset(self):
        self.stuck_counter = 0
        self.escape_timer = 0
        self.turn_side = 1
        self.escapes = 0

    def step(self, obs):
        m = self.max_speed
        if self.escape:
            e = self.escape_speed * m
            if self.escape_timer > 0:
                self.escape_timer -= 1
                if self.escape_timer >= self.turn_steps:
                    return -e, -e
                return -e * self.turn_side, e * self.turn_side
            if float(np.max(obs["prox"])) > self.stuck_thr:
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0
            if self.stuck_counter > self.patience:
                self.stuck_counter = 0
                self.turn_side *= -1
                self.escape_timer = self.back_steps + self.turn_steps
                self.escapes += 1
                return -e, -e
        s = obs["prox"]
        vl = float(self.wl @ s) + self.bl
        vr = float(self.wr @ s) + self.br
        if self.mode == "relative":
            return (m * float(np.clip(vl / self.ref, -1, 1)),
                    m * float(np.clip(vr / self.ref, -1, 1)))
        return float(np.clip(vl, -m, m)), float(np.clip(vr, -m, m))


def load_controller(spec, ctrl_arg=None, max_speed=None, forager_kw=None, **kw):
    """spec: 'braitenberg' or 'forager' (ctrl_arg = Braitenberg JSON path),
    or 'file.py:Class'."""
    if spec == "braitenberg":
        return BraitenbergJSON(ctrl_arg, max_speed=max_speed, **kw)
    if spec == "forager":
        return Forager(BraitenbergJSON(ctrl_arg, max_speed=max_speed, **kw), **(forager_kw or {}))
    path, cls = spec.split(":")
    mod_spec = importlib.util.spec_from_file_location("user_controller", path)
    mod = importlib.util.module_from_spec(mod_spec)
    mod_spec.loader.exec_module(mod)
    return getattr(mod, cls)(ctrl_arg) if ctrl_arg else getattr(mod, cls)()


class Forager:
    """Food-finding demo: explore with the Braitenberg (plus a random turn
    now and then, so the robot also crosses the middle of the arena); when
    the ground sensors see the dark patch, move onto it, spin `spins` full
    turns, leave it, ignore food for a while, and search again.

    States: search -> approach (patch seen, move onto it) -> spin -> leave
    -> search. Times are in seconds of obs["t"], so they do not depend on
    the control rate. Needs the ground sensors (obs["floor"], run with
    --floor). food_threshold is in raw ground units of the robot (2292:
    ~330; twin: ~500)."""

    AXLE = 0.052

    def __init__(self, brait, food_threshold=500.0, spins=5, spin_speed=1.0,
                 advance_time=1.5, leave_time=2.0, ignore_time=4.0,
                 wander_interval=4.0, wander_time=0.6, seed=0):
        self.brait = brait
        self.max_speed = brait.max_speed
        self.food_threshold = food_threshold
        self.spins = spins
        self.spin_speed = spin_speed
        self.advance_time = advance_time
        self.leave_time = leave_time
        self.ignore_time = ignore_time
        self.wander_interval = wander_interval
        self.wander_time = wander_time
        self.rng = np.random.default_rng(seed)
        self.reset()

    def reset(self):
        self.brait.reset()
        self.state = "search"
        self.t_state = 0.0
        self.ignore_until = 0.0
        self.next_wander = self.wander_interval
        self.wander_until = -1.0
        self.wander_side = 1
        self.finds = []                       # times at which food was found
        self.escapes = 0

    def _go(self, state, t):
        self.state, self.t_state = state, t

    def spin_duration(self):
        w = 2 * self.spin_speed * self.max_speed / self.AXLE      # rad/s spinning in place
        return self.spins * 2 * np.pi / w

    def step(self, obs):
        t = float(obs["t"])
        m = self.max_speed
        below = int(np.sum(np.asarray(obs["floor"][:3], float) < self.food_threshold))

        if self.state == "search":
            if t >= self.ignore_until and below >= 2:
                self._go("approach", t)
            else:
                v = self.brait.step(obs)
                self.escapes = self.brait.escapes
                if self.brait.escape_timer > 0 or max(obs["prox"]) > 0.2:
                    return v                                       # obstacles first
                if t >= self.next_wander:                          # occasional random turn
                    self.wander_until = t + self.wander_time
                    self.wander_side = 1 if self.rng.random() < 0.5 else -1
                    self.next_wander = t + self.wander_interval * (0.5 + self.rng.random())
                if t < self.wander_until:
                    return -0.6 * m * self.wander_side, 0.6 * m * self.wander_side
                return v

        if self.state == "approach":                               # move the body onto the patch
            if t - self.t_state < self.advance_time:
                return 0.5 * m, 0.5 * m
            self.finds.append(t)
            self._go("spin", t)

        if self.state == "spin":
            if t - self.t_state < self.spin_duration():
                v = self.spin_speed * m
                return -v, v
            self._go("leave", t)

        if self.state == "leave":
            if t - self.t_state < self.leave_time:
                return m, m
            self.ignore_until = t + self.ignore_time
            self._go("search", t)
            return self.step(obs)
        return 0.0, 0.0

    def summary(self):
        f = self.finds
        gaps = np.diff([0.0] + f) if f else []
        return {"finds": len(f), "find_times_s": [round(x, 1) for x in f],
                "time_to_first_find_s": round(f[0], 1) if f else None,
                "mean_time_between_finds_s": round(float(np.mean(gaps)), 1) if f else None}
