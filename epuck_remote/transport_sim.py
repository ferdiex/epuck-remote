"""
transport_sim.py - the PyBullet twin: the e-puck model and sensors of
sim/ (from easabrain), answering asercom commands like the real robot.

Each D command advances the physics one control period (1/hz s). With
--sim-latency L the new wheel command is applied L seconds into that
period (the old command keeps running before it), which reproduces the
delay between reading the sensors and the robot reacting over Bluetooth.
Wheel speeds: steps/s -> rad/s (1000 steps = 1 revolution).
"""
import math
import os
import sys
import time

import numpy as np

from .model import RAD_PER_STEP
from .protocol import BackendClosed
from .transport_base import AsercomServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


class SimTransport(AsercomServer):
    kind = "sim"

    def __init__(self, config=None, gui=False, dt=0.1, latency=0.0, seed=0,
                 noise=None, n_rays=None, start=None, food=None, obstacles=None,
                 arena=None, wall_latency=0.0, video=None, video_fps=25,
                 video_size=(800, 600), smooth=True, video_yaw=0.0, video_top=False):
        super().__init__(wall_latency)
        import pybullet as p
        from sim import world as W
        from sim.sensors import EpuckSensors, FloorMap
        self.p, self.W = p, W
        self.gui = gui
        self.smooth = smooth and gui
        self.video = video
        visual = gui or bool(video)            # decorated model and patch drawn
        self.cfg = W.load_config(config) if config else W.load_config()
        if arena:
            self.cfg["world"]["arena_size"] = arena
        ts = self.cfg["world"]["timestep"]
        self.n_ctrl = max(1, int(round(dt / ts)))
        self.n_lat = min(self.n_ctrl, int(round(latency / ts)))
        self.rng = np.random.default_rng(seed)

        W.setup_physics(self.cfg, gui=gui)
        self.arena = W.build_arena(self.cfg)
        self.obstacle_ids = [self._add_cylinder(*o) for o in (obstacles or [])]
        self.obstacles = obstacles or []

        self.floor_map = FloorMap(self.cfg["floor"]["reflectance"])
        self.food = list(food) if food else None
        if food:
            fx, fy, fr = food
            self.floor_map.add_patch(fx, fy, fr, self.cfg["task"]["food_reflectance"])
            if visual:
                W.add_patch_visual(fx, fy, fr)

        self.robot = W.load_epuck(self.cfg, lean=not visual)
        if start is None:
            start = self._random_start()
        W.place_robot(self.robot, self.cfg, *start)
        for _ in range(self.n_ctrl):                      # settle
            p.stepSimulation()
        er = self.cfg["epuck"]["ir_rays"]
        self.sensors = EpuckSensors(
            self.floor_map,
            noise=self.cfg["sensors"]["noise"] if noise is None else noise,
            rng=np.random.default_rng(self.rng.integers(1 << 31)),
            n_rays=er["n_rays"] if n_rays is None else n_rays,
            aperture=er["aperture"], gaussian_width=er["gaussian_width"])
        self.touching = 0
        self.prev = (0.0, 0.0)
        if gui:
            W.set_camera(distance=1.1 * self.cfg["world"]["arena_size"] + 0.3)
        self.ts = ts
        self.sim_t = 0.0
        self._writer = None
        if video:
            self._open_video(video, video_fps, video_size, video_yaw, video_top)

    def write_line(self, text):
        # closing the GUI window disconnects PyBullet; report it once, clearly
        if not self.p.isConnected():
            raise BackendClosed("simulation window closed")
        try:
            super().write_line(text)
        except self.p.error as e:
            # on macOS the GUI can go away while isConnected() still says True
            if self.gui or not self.p.isConnected():
                raise BackendClosed("simulation window closed (or PyBullet stopped)") from e
            raise

    def pose(self):
        if not self.p.isConnected():
            raise BackendClosed("simulation window closed")
        x, y, yaw = self.W.get_pose(self.robot)[:3]
        return x, y, math.degrees(yaw)

    def _add_cylinder(self, x, y, r, h=0.06):
        p = self.p
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=r, height=h)
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=h,
                                  rgbaColor=[0.9, 0.2, 0.27, 1.0])
        return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                                 baseVisualShapeIndex=vis, basePosition=[x, y, h / 2])

    def _random_start(self):
        half = self.cfg["world"]["arena_size"] / 2 - 0.08
        for _ in range(1000):
            x, y = self.rng.uniform(-half, half, 2)
            if all(math.hypot(x - ox, y - oy) > r + 0.08 for ox, oy, r in self.obstacles):
                return float(x), float(y), float(self.rng.uniform(-math.pi, math.pi))
        return 0.0, 0.0, 0.0

    def _wheels(self, l, r):
        self.W.set_wheels(self.robot, l * RAD_PER_STEP, r * RAD_PER_STEP)

    # ------------------------------------------------------------ video
    def _open_video(self, path, fps, size, yaw_deg=0.0, top=False):
        try:
            import imageio
        except ImportError as e:
            raise SystemExit("--video needs imageio and imageio-ffmpeg: "
                             "pip install imageio imageio-ffmpeg") from e
        self._writer = imageio.get_writer(path, fps=fps, codec="libx264", quality=8,
                                          macro_block_size=8)
        self._frame_dt = 1.0 / fps
        self._next_frame = 0.0
        self._vw, self._vh = size
        a = self.cfg["world"]["arena_size"]
        # camera on a circle around the arena; yaw rotates it counter-clockwise
        # seen from above (0 = looking from -y, the default view)
        c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        if top:                                   # straight down; yaw turns the picture
            eye, up = [0.0, 0.0, 1.25 * a], [-s, c, 0]
        else:
            ex, ey = 0.0, -1.05 * a
            eye, up = [c * ex - s * ey, s * ex + c * ey, 1.15 * a], [0, 0, 1]
        self._view = self.p.computeViewMatrix(eye, [0, 0, 0], up)
        self._proj = self.p.computeProjectionMatrixFOV(50, self._vw / self._vh, 0.01, 10)
        self._capture()

    def _capture(self):
        img = self.p.getCameraImage(self._vw, self._vh, self._view, self._proj,
                                    renderer=self.p.ER_TINY_RENDERER, shadow=0,
                                    lightDirection=[0.5, -0.6, 1.0])
        rgb = np.reshape(np.array(img[2], dtype=np.uint8), (self._vh, self._vw, 4))[:, :, :3]
        self._writer.append_data(rgb)
        self._next_frame += self._frame_dt

    def apply_speed(self, left, right):
        p = self.p
        for k in range(self.n_ctrl):
            if k == self.n_lat:
                self._wheels(left, right)
            p.stepSimulation()
            self.sim_t += self.ts
            if self._writer is not None and self.sim_t >= self._next_frame:
                self._capture()
            if self.smooth:                      # GUI: spread the physics over the period
                time.sleep(self.ts)
        if self.n_lat >= self.n_ctrl:
            self._wheels(left, right)
        others = set(self.arena["walls"]) | set(self.obstacle_ids)
        if any(c[2] in others for c in p.getContactPoints(bodyA=self.robot["id"])):
            self.touching += 1

    def proximity(self):
        self._last = self.sensors.read(self.robot)
        return np.round(self._last["ir"]).astype(int).tolist()

    def floor(self):
        r = self.sensors.read(self.robot)
        return np.round(r["ground"]).astype(int).tolist()

    def step_counters(self):
        p = self.p
        a_l = p.getJointState(self.robot["id"], self.robot["left"])[0]
        a_r = p.getJointState(self.robot["id"], self.robot["right"])[0]
        return int(a_l / RAD_PER_STEP), int(a_r / RAD_PER_STEP)

    def contacts(self):
        return self.touching

    def layout(self):
        return {"arena": self.cfg["world"]["arena_size"],
                "obstacles": [list(o) for o in self.obstacles], "food": self.food}

    def close(self):
        if self._writer is not None:
            try:
                self._writer.close()
                print("video saved:", self.video)
            except Exception:
                pass
            self._writer = None
        try:
            self.p.disconnect()
        except Exception:
            pass
