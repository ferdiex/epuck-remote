"""
sim/world.py - PyBullet world for the e-puck 1 twin (epuck-remote).

Provenance: the e-puck functions of easabrain_world.py, copied verbatim
(easabrain derived them unchanged from easagru_world.py, easagru commit
2c3191d). Changes: Khepera loading removed; paths resolved from the repo
root (robots/ and sim/config.json).

Robot handles are plain dicts: {"id", "left", "right", "max_angvel",
"force", "kind"}.
"""
import json
import os

import numpy as np
import pybullet as p
import pybullet_data

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEFAULT_CONFIG = os.path.join(HERE, "config.json")

WALL_COLOR = (0.9, 0.2, 0.27, 1.0)  # same red as the easa / easagru arenas


def load_config(path=DEFAULT_CONFIG):
    with open(path) as f:
        return json.load(f)


def setup_physics(cfg, gui=False):
    """Connect to PyBullet and set global physics parameters. Returns client id."""
    client = p.connect(p.GUI if gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, cfg["world"]["gravity"])
    p.setTimeStep(cfg["world"]["timestep"])
    p.setPhysicsEngineParameter(numSolverIterations=cfg["world"]["solver_iterations"])
    # Process contact pairs in a fixed order. Without it the order depends on
    # the history of the simulation, so the same episode (seed and actions)
    # after a different one differed in the 7th decimal and then diverged.
    # Added in easabrain after the runs gru_s1 and easa_ga_s1 had started.
    p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    return client


def set_camera(distance=1.2, yaw=0.0, pitch=-75.0, target=(0.0, 0.0, 0.0)):
    """Place the GUI camera. Call AFTER the world and robots are loaded."""
    p.resetDebugVisualizerCamera(cameraDistance=distance, cameraYaw=yaw, cameraPitch=pitch,
                                 cameraTargetPosition=list(target))


def _make_wall(center, half_extents, color=WALL_COLOR):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
    wall = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                             baseVisualShapeIndex=vis, basePosition=center)
    p.changeDynamics(wall, -1, lateralFriction=0.5)
    return wall


def build_arena(cfg):
    """Floor plane plus four walls enclosing a square arena centred at the origin."""
    w = cfg["world"]
    plane = p.loadURDF("plane.urdf")
    p.changeDynamics(plane, -1, lateralFriction=w["floor_friction"])
    half = w["arena_size"] / 2.0
    t = w["wall_thickness"] / 2.0
    h = w["wall_height"] / 2.0
    walls = [
        _make_wall([half + t, 0, h], [t, half + 2 * t, h]),
        _make_wall([-half - t, 0, h], [t, half + 2 * t, h]),
        _make_wall([0, half + t, h], [half, t, h]),
        _make_wall([0, -half - t, h], [half, t, h]),
    ]
    return {"plane": plane, "walls": walls}


def add_patch_visual(x, y, radius, rgba=(0.15, 0.15, 0.15, 1.0)):
    """Draw a floor patch (visual only, no collision: robots drive over it and
    rays pass through it; its effect on the ground sensors comes from FloorMap).
    Copied from easagru_world.py: drawn 5 mm tall on purpose, thinner patches
    showed z-fighting stripes in the OpenGL view. Physics and sensors never see it.
    """
    height = 0.005
    vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=list(rgba))
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis, basePosition=[x, y, height / 2])


def library_versions():
    """Versions to store with every run (results depend slightly on PyBullet)."""
    import platform
    import sys
    from importlib import metadata
    out = {"python": sys.version.split()[0], "platform": platform.platform(),
           "numpy": np.__version__, "pybullet_api": p.getAPIVersion()}
    try:
        out["pybullet"] = metadata.version("pybullet")
    except metadata.PackageNotFoundError:
        out["pybullet"] = "unknown"
    return out


def _link_index_by_name(body_id):
    mapping = {}
    for j in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, j)
        mapping[info[12].decode("utf-8")] = j
    return mapping


def _resolve(path):
    return path if os.path.isabs(path) else os.path.join(ROOT, path)


def load_epuck(cfg, pos_xy=(0.0, 0.0), yaw=0.0, lean=False):
    """Load the e-puck exactly as easagru does; return a handle dict."""
    r = cfg["epuck"]
    urdf = _resolve(r["urdf_lean_path"] if lean else r["urdf_path"])
    orn = p.getQuaternionFromEuler([0, 0, yaw])
    body = p.loadURDF(urdf, [pos_xy[0], pos_xy[1], r["spawn_z"]], orn, useFixedBase=False)

    links = _link_index_by_name(body)
    fr = r["friction"]
    p.changeDynamics(body, -1, lateralFriction=fr["chassis"],
                     linearDamping=r["linear_damping"], angularDamping=r["angular_damping"])
    for name in ("left_wheel", "right_wheel"):
        p.changeDynamics(body, links[name], lateralFriction=fr["wheel"])
    for name in ("caster_front", "caster_rear"):
        p.changeDynamics(body, links[name], lateralFriction=fr["caster"])
    if not lean:
        _apply_epuck_colors(body, links)

    handle = {"id": body, "kind": "epuck", "left": links["left_wheel"], "right": links["right_wheel"],
              "max_angvel": r["max_wheel_angvel"], "force": r["wheel_motor_force"],
              "body_led": links.get("body_led"), "led_on": False}
    set_wheels(handle, 0.0, 0.0)
    return handle


def set_body_led(handle, on):
    """Switch the e-puck body LED (the visual signal). Records the state in
    the handle (handle["led_on"]); recolours the LED only in the GUI model,
    which is the only one that draws it. Perception must read handle["led_on"],
    never the colour."""
    handle["led_on"] = bool(on)
    if handle.get("body_led") is not None:
        import sys
        sys.path.insert(0, os.path.join(ROOT, "robots"))
        from make_epuck_urdf import COLORS, BODY_LED_ON_COLOR
        color = BODY_LED_ON_COLOR if on else COLORS["body_led"]
        p.changeVisualShape(handle["id"], handle["body_led"], rgbaColor=list(color))


def _apply_epuck_colors(body, links):
    """Re-apply every link colour explicitly (the macOS GUI may ignore URDF materials)."""
    import sys
    sys.path.insert(0, os.path.join(ROOT, "robots"))
    from make_epuck_urdf import COLORS, color_for_link
    p.changeVisualShape(body, -1, rgbaColor=list(COLORS["base"]))
    for name, idx in links.items():
        color = color_for_link(name)
        if color is not None:
            p.changeVisualShape(body, idx, rgbaColor=list(color))


def set_wheels(handle, left, right):
    """Wheel commands in rad/s, clipped to the robot's own limit."""
    m = handle["max_angvel"]
    left = float(np.clip(left, -m, m))
    right = float(np.clip(right, -m, m))
    p.setJointMotorControl2(handle["id"], handle["left"], p.VELOCITY_CONTROL,
                            targetVelocity=left, force=handle["force"])
    p.setJointMotorControl2(handle["id"], handle["right"], p.VELOCITY_CONTROL,
                            targetVelocity=right, force=handle["force"])


def place_robot(handle, cfg, x, y, yaw):
    """Put a robot at (x, y, yaw), resting on the floor, motors stopped.

    resetBasePositionAndOrientation moves the base CENTRE OF MASS, which is
    above the URDF base frame (15 mm on the e-puck);
    passing the base-frame height directly would sink the robot into the
    floor. This places the base frame at the robot's spawn height instead.
    """
    set_wheels(handle, 0.0, 0.0)
    orn = p.getQuaternionFromEuler([0, 0, yaw])
    local_pos = p.getDynamicsInfo(handle["id"], -1)[3]
    com, _ = p.multiplyTransforms([x, y, cfg[handle["kind"]]["spawn_z"]], orn, local_pos, [0, 0, 0, 1])
    p.resetBasePositionAndOrientation(handle["id"], com, orn)
    p.resetBaseVelocity(handle["id"], [0, 0, 0], [0, 0, 0])


def get_pose(handle):
    """Return (x, y, yaw, roll, pitch, z) of the robot base."""
    pos, orn = p.getBasePositionAndOrientation(handle["id"])
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)
    return pos[0], pos[1], yaw, roll, pitch, pos[2]
