"""
transport_mock.py - dependency-free fake e-puck: differential-drive
kinematics in a square arena with round obstacles, IR from the Webots lookup
table at the real sensor bearings. For tests and quick checks, not physics.
"""
import math
import random

from .model import (BODY_RADIUS, AXLE, GS_POS, IR_ANGLES, IR_POS, IR_RANGE,
                    STEP_LEN, ir_raw_from_distance)
from .transport_base import AsercomServer


class MockTransport(AsercomServer):
    kind = "mock"

    def __init__(self, arena=0.5, obstacles=None, dt=0.1, noise=0.03,
                 latency=0.0, seed=0, start=None, food=None):
        super().__init__(latency)
        self.rng = random.Random(seed)
        self.half = arena / 2.0
        self.obstacles = obstacles or []          # [(x, y, r)]
        self.food = list(food) if food else None  # (x, y, r) dark floor patch
        self.dt = dt
        self.noise = noise
        if start is None:
            start = (0.0, 0.0, self.rng.uniform(-math.pi, math.pi))
        self.x, self.y, self.yaw = start
        self.steps_l = self.steps_r = 0.0
        self.blocked = 0

    def _ray(self, x0, y0, ang):
        c, s = math.cos(ang), math.sin(ang)
        best = float("inf")
        for t in ((self.half - x0) / c if c > 1e-9 else None,
                  (-self.half - x0) / c if c < -1e-9 else None,
                  (self.half - y0) / s if s > 1e-9 else None,
                  (-self.half - y0) / s if s < -1e-9 else None):
            if t is not None and t >= 0:
                best = min(best, t)
        for ox, oy, r in self.obstacles:
            dx, dy = x0 - ox, y0 - oy
            b = dx * c + dy * s
            disc = b * b - (dx * dx + dy * dy - r * r)
            if disc >= 0:
                t = -b - math.sqrt(disc)
                if t >= 0:
                    best = min(best, t)
        return best

    def _collides(self, x, y):
        if abs(x) > self.half - BODY_RADIUS or abs(y) > self.half - BODY_RADIUS:
            return True
        return any(math.hypot(x - ox, y - oy) < r + BODY_RADIUS for ox, oy, r in self.obstacles)

    def proximity(self):
        vals = []
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        for (px, py, _), a in zip(IR_POS, IR_ANGLES):
            sx = self.x + c * px - s * py
            sy = self.y + s * px + c * py
            d = self._ray(sx, sy, self.yaw + math.radians(a))
            if d >= IR_RANGE:
                raw = self.rng.uniform(10, 50)        # background, as the real robot
            else:
                raw = ir_raw_from_distance(d) * (1.0 + self.rng.gauss(0, self.noise))
            vals.append(int(max(0, min(4095, raw))))
        return vals

    def floor(self):
        """~800 on the floor, ~170 on the dark patch (like the twin)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        out = []
        for gx, gy, _ in GS_POS:
            x = self.x + c * gx - s * gy
            y = self.y + s * gx + c * gy
            on = self.food and math.hypot(x - self.food[0], y - self.food[1]) < self.food[2]
            out.append(int((170 if on else 800) * (1 + self.rng.gauss(0, 0.02))))
        return out

    def step_counters(self):
        return int(self.steps_l), int(self.steps_r)

    def apply_speed(self, left, right):
        vl, vr = left * STEP_LEN, right * STEP_LEN
        v, w = 0.5 * (vl + vr), (vr - vl) / AXLE
        nyaw = self.yaw + w * self.dt
        nx = self.x + v * math.cos(nyaw) * self.dt
        ny = self.y + v * math.sin(nyaw) * self.dt
        self.yaw = (nyaw + math.pi) % (2 * math.pi) - math.pi
        if self._collides(nx, ny):
            self.blocked += 1                     # rotate in place only
        else:
            self.x, self.y = nx, ny
        self.steps_l += left * self.dt
        self.steps_r += right * self.dt

    def pose(self):
        return self.x, self.y, math.degrees(self.yaw)

    def contacts(self):
        return self.blocked

    def layout(self):
        return {"arena": 2 * self.half, "obstacles": [list(o) for o in self.obstacles],
                "food": self.food}
