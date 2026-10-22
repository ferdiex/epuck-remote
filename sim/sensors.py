"""
sim/sensors.py - e-puck 1 sensors for the PyBullet twin (epuck-remote).

Provenance: the e-puck part of easabrain_sensors.py, copied verbatim; only
the Khepera II section (KheperaSensors) and this paragraph were removed or
changed. easabrain took it from easagru (easagru_sensors.py, commit 2c3191d).

EpuckSensors can cast several rays per IR sensor (Webots numberOfRays),
see "Multiple rays" below; with n_rays=1 (the default) it reads exactly as
in easagru.

Multiple rays (easabrain): as Webots DistanceSensor does for "infra-red"
sensors with numberOfRays > 1 (Webots reference manual, DistanceSensor):
the rays fill a cone of opening angle `aperture` around the sensor axis
(E-puckDistanceSensor.proto: aperture 0.1 rad), laid out as in the Webots
figure (1: centre; 2-3: on the cone edge; 4 or more: centre plus the rest
on the edge, starting at the top), each ray weighted by
exp(-(t / (aperture * gaussianWidth))^2), t = its angle to the axis, the
weights normalised; the sensor distance is the weighted average of the
rays' distances (a ray that hits nothing counts as the maximum range),
then mapped through the lookup table. Reason: with a single ray the
e-puck's IR (33 mm high, tilted up 9 deg) passes over the Khepera II.

Source: webots_ref/E-puck.proto, E-puckDistanceSensor.proto,
E-puckGroundSensors.proto (e-puck v1).

IR proximity sensors ps0..ps7
  - Positions: copied from E-puck.proto (z = 0.033 m).
  - Directions: the sensor axis (local +x) after composing each sensor's
    rotation in E-puck.proto with the inner rotation in
    E-puckDistanceSensor.proto. Result: horizontal angles
    -17.2, -45.9, -90, -151.5, 151.2, 90, 45.8, 17.1 deg, and every ray
    tilted ~9.2 deg upwards (z component 0.1593), as in the Webots model.
  - Response: Webots lookupTable (distance -> raw value, relative noise),
    linear interpolation; beyond 7 cm the reading stays at the last entry.
  - One ray per sensor (Webots default numberOfRays = 1).
  - The ray starts where it leaves the robot's own collision cylinder
    (r = 37 mm), so it never hits its own body; the reported distance is
    still measured from the sensor position. As in Webots, this means a wall
    can never be closer than ~5.5 mm to a sensor.

Ground sensors gs0..gs2
  - Positions: ground slot (0.03, 0, 0.0003) + sensor offsets
    y = +0.01 / 0 / -0.01, z = 0.003, pointing straight down.
  - Response: Webots lookupTable 0 m -> 1000, 0.016 m -> 300.
  - Our addition: the reading is scaled by the floor reflectance at the hit
    point (1.0 = plain floor, lower = dark patch). This is how a food patch
    on the floor becomes visible to the robot.

Simplification (documented): obstacles (walls, the other robot) are treated
as reflectance 1.0 for the IR proximity sensors.
"""
import numpy as np
import pybullet as p

# --- IR proximity: poses (robot frame: x forward, y left, z up) -------------
IR_POS = np.array([
    [0.030, -0.010, 0.033], [0.022, -0.025, 0.033], [0.000, -0.031, 0.033],
    [-0.030, -0.015, 0.033], [-0.030, 0.015, 0.033], [0.000, 0.031, 0.033],
    [0.022, 0.025, 0.033], [0.030, 0.010, 0.033],
])
IR_DIR = np.array([
    [0.9429, -0.2925, 0.1593], [0.6872, -0.7087, 0.1593], [0.0000, -0.9872, 0.1593],
    [-0.8675, -0.4712, 0.1593], [-0.8652, 0.4754, 0.1593], [0.0000, 0.9872, 0.1593],
    [0.6884, 0.7076, 0.1593], [0.9434, 0.2910, 0.1593],
])
IR_DIR = IR_DIR / np.linalg.norm(IR_DIR, axis=1, keepdims=True)

# Webots lookupTable: distance [m], raw value, relative noise (std / value)
IR_LOOKUP = np.array([
    [0.000, 4095.00, 0.002], [0.005, 2133.33, 0.003], [0.010, 1465.73, 0.007],
    [0.015, 601.46, 0.0406], [0.020, 383.84, 0.01472], [0.030, 234.93, 0.0241],
    [0.040, 158.03, 0.0287], [0.050, 120.00, 0.04225], [0.060, 104.09, 0.03065],
    [0.070, 67.19, 0.04897],
])
IR_RANGE = IR_LOOKUP[-1, 0]

# --- Ground sensors ----------------------------------------------------------
GS_POS = np.array([[0.03, 0.01, 0.0033], [0.03, 0.00, 0.0033], [0.03, -0.01, 0.0033]])
GS_LOOKUP = np.array([[0.000, 1000.0, 0.002], [0.016, 300.0, 0.004]])
GS_RANGE = GS_LOOKUP[-1, 0]
GS_RAY_START_Z = 0.0020  # just below the body cylinder (bottom at 2.5 mm)

BODY_RADIUS = 0.037       # collision cylinder, E-puck.proto boundingObject
RAY_CLEARANCE = 0.0005    # start IR rays 0.5 mm outside the body


def lookup(distance, table):
    """Webots-style lookup: interpolated value and noise std (relative * value)."""
    d = np.asarray(distance, dtype=float)
    value = np.interp(d, table[:, 0], table[:, 1])
    rel_noise = np.interp(d, table[:, 0], table[:, 2])
    return value, rel_noise * value


def _exit_offsets():
    """Distance along each IR ray from the sensor to the outside of the body."""
    r = BODY_RADIUS + RAY_CLEARANCE
    out = np.zeros(len(IR_POS))
    for i, (pos, d) in enumerate(zip(IR_POS[:, :2], IR_DIR[:, :2])):
        a = d @ d
        b = 2 * pos @ d
        c = pos @ pos - r * r
        out[i] = (-b + np.sqrt(b * b - 4 * a * c)) / (2 * a)
    return out


IR_EXIT = _exit_offsets()


class FloorMap:
    """Floor reflectance: 1.0 everywhere except circular patches (e.g. food)."""

    def __init__(self, base_reflectance=1.0):
        self.base = base_reflectance
        self.patches = []  # list of (x, y, radius, reflectance)

    def add_patch(self, x, y, radius, reflectance):
        self.patches.append((x, y, radius, reflectance))

    def reflectance(self, x, y):
        for px, py, pr, refl in self.patches:
            if (x - px) ** 2 + (y - py) ** 2 <= pr * pr:
                return refl
        return self.base


def ray_layout(axis, n_rays, aperture, gaussian_width):
    """Webots-style ray directions around a unit sensor axis, and their
    normalised weights (see "Multiple rays" in the module docstring)."""
    axis = axis / np.linalg.norm(axis)
    if n_rays == 1:
        return axis[None, :], np.ones(1)
    up = np.array([0.0, 0.0, 1.0]) - axis[2] * axis
    up /= np.linalg.norm(up)
    side = np.cross(axis, up)
    n_edge = n_rays if n_rays <= 3 else n_rays - 1
    start = 0.0 if n_edge == 2 else np.pi / 2       # 2 rays: left/right; else from the top
    half = aperture / 2
    dirs, angles = [], []
    if n_rays >= 4:
        dirs.append(axis)
        angles.append(0.0)
    for k in range(n_edge):
        phi = start + 2 * np.pi * k / n_edge
        off = np.cos(phi) * side + np.sin(phi) * up
        d = np.cos(half) * axis + np.sin(half) * off
        dirs.append(d / np.linalg.norm(d))
        angles.append(half)
    v = np.exp(-(np.array(angles) / (aperture * gaussian_width)) ** 2)
    return np.array(dirs), v / v.sum()


def _exit_offset(pos_xy, dir_xy, r):
    a, b, c = dir_xy @ dir_xy, 2 * pos_xy @ dir_xy, pos_xy @ pos_xy - r * r
    return (-b + np.sqrt(b * b - 4 * a * c)) / (2 * a)


class EpuckSensors:
    """Reads all 11 sensors of one e-puck with a single rayTestBatch call.
    n_rays, aperture, gaussian_width: rays per IR sensor (easabrain; 1 =
    easagru behaviour)."""

    def __init__(self, floor, noise=True, rng=None, n_rays=1, aperture=0.1, gaussian_width=1.0):
        self.floor = floor
        self.noise = noise
        self.rng = rng if rng is not None else np.random.default_rng()
        self.last_rays = None  # (starts, ends, hit_flags) for GUI drawing
        self.noise_scale = 1.0  # robustness tests: multiplies the Webots noise levels
        self.n_rays = int(n_rays)
        if self.n_rays > 1:
            dirs, weights = zip(*(ray_layout(d, self.n_rays, aperture, gaussian_width) for d in IR_DIR))
            self.ray_dir = np.vstack(dirs)                      # (8 * n_rays, 3)
            self.ray_w = np.concatenate(weights)
            self.ray_pos = np.repeat(IR_POS, self.n_rays, axis=0)
            r = BODY_RADIUS + RAY_CLEARANCE
            self.ray_exit = np.array([_exit_offset(pp[:2], dd[:2], r)
                                      for pp, dd in zip(self.ray_pos, self.ray_dir)])

    def read(self, handle):
        pos, orn = p.getBasePositionAndOrientation(handle["id"])
        # getBasePositionAndOrientation returns the centre of mass; convert to
        # the base link frame origin (the URDF origin, where poses are defined).
        rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        com_offset = np.array(p.getDynamicsInfo(handle["id"], -1)[3])
        origin = np.array(pos) - rot @ com_offset

        if self.n_rays > 1:
            return self._read_multi(handle, origin, rot)
        ir_pos_w = origin + IR_POS @ rot.T
        ir_dir_w = IR_DIR @ rot.T
        ir_start = ir_pos_w + ir_dir_w * IR_EXIT[:, None]
        ir_end = ir_pos_w + ir_dir_w * IR_RANGE

        gs_pos_w = origin + GS_POS @ rot.T
        down = -rot[:, 2]
        gs_start = origin + np.column_stack([GS_POS[:, :2], np.full(3, GS_RAY_START_Z)]) @ rot.T
        gs_end = gs_pos_w + down * GS_RANGE

        starts = np.vstack([ir_start, gs_start])
        ends = np.vstack([ir_end, gs_end])
        hits = p.rayTestBatch(starts.tolist(), ends.tolist())

        # IR proximity
        ir_dist = np.full(8, IR_RANGE)
        ir_hit = np.zeros(8, dtype=bool)
        for i in range(8):
            body, _, frac = hits[i][0], hits[i][1], hits[i][2]
            if body != -1 and body != handle["id"]:
                ir_dist[i] = IR_EXIT[i] + frac * (IR_RANGE - IR_EXIT[i])
                ir_hit[i] = True
        ir_val, ir_std = lookup(ir_dist, IR_LOOKUP)

        # Ground
        gs_dist = np.full(3, GS_RANGE)
        gs_refl = np.ones(3)
        for k in range(3):
            h = hits[8 + k]
            if h[0] != -1 and h[0] != handle["id"]:
                hit_pt = np.array(h[3])
                gs_dist[k] = np.linalg.norm(hit_pt - gs_pos_w[k])
                gs_refl[k] = self.floor.reflectance(hit_pt[0], hit_pt[1])
        gs_val, gs_std = lookup(gs_dist, GS_LOOKUP)
        gs_val = gs_val * gs_refl
        gs_std = gs_std * gs_refl

        if self.noise:
            ir_val = ir_val + self.rng.normal(0.0, 1.0, 8) * ir_std * self.noise_scale
            gs_val = gs_val + self.rng.normal(0.0, 1.0, 3) * gs_std * self.noise_scale

        self.last_rays = (ir_start, ir_end, ir_hit, ir_dist, gs_start, gs_end)
        return {
            "ir": np.clip(ir_val, 0.0, 4095.0),
            "ground": np.clip(gs_val, 0.0, 1000.0),
            "ir_dist": ir_dist,
            "ir_hit": ir_hit,
        }

    def _read_multi(self, handle, origin, rot):
        """Same as read() with several rays per IR sensor (easabrain)."""
        n, nr = len(IR_POS), self.n_rays
        pos_w = origin + self.ray_pos @ rot.T
        dir_w = self.ray_dir @ rot.T
        ir_start = pos_w + dir_w * self.ray_exit[:, None]
        ir_end = pos_w + dir_w * IR_RANGE
        gs_pos_w = origin + GS_POS @ rot.T
        down = -rot[:, 2]
        gs_start = origin + np.column_stack([GS_POS[:, :2], np.full(3, GS_RAY_START_Z)]) @ rot.T
        gs_end = gs_pos_w + down * GS_RANGE
        hits = p.rayTestBatch(np.vstack([ir_start, gs_start]).tolist(), np.vstack([ir_end, gs_end]).tolist())

        ray_dist = np.full(n * nr, IR_RANGE)
        ray_hit = np.zeros(n * nr, dtype=bool)
        for i in range(n * nr):
            if hits[i][0] != -1 and hits[i][0] != handle["id"]:
                ray_dist[i] = self.ray_exit[i] + hits[i][2] * (IR_RANGE - self.ray_exit[i])
                ray_hit[i] = True
        ir_dist = (self.ray_w * ray_dist).reshape(n, nr).sum(axis=1) / self.ray_w.reshape(n, nr).sum(axis=1)
        ir_hit = ray_hit.reshape(n, nr).any(axis=1)
        ir_val, ir_std = lookup(ir_dist, IR_LOOKUP)

        gs_dist = np.full(3, GS_RANGE)
        gs_refl = np.ones(3)
        for k in range(3):
            h = hits[n * nr + k]
            if h[0] != -1 and h[0] != handle["id"]:
                hit_pt = np.array(h[3])
                gs_dist[k] = np.linalg.norm(hit_pt - gs_pos_w[k])
                gs_refl[k] = self.floor.reflectance(hit_pt[0], hit_pt[1])
        gs_val, gs_std = lookup(gs_dist, GS_LOOKUP)
        gs_val = gs_val * gs_refl
        gs_std = gs_std * gs_refl
        if self.noise:
            ir_val = ir_val + self.rng.normal(0.0, 1.0, n) * ir_std * self.noise_scale
            gs_val = gs_val + self.rng.normal(0.0, 1.0, 3) * gs_std * self.noise_scale
        self.last_rays = (ir_start, ir_end, ray_hit, ray_dist, gs_start, gs_end)
        return {
            "ir": np.clip(ir_val, 0.0, 4095.0),
            "ground": np.clip(gs_val, 0.0, 1000.0),
            "ir_dist": ir_dist,
            "ir_hit": ir_hit,
        }
