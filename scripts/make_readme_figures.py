#!/usr/bin/env python3
"""
make_readme_figures.py - regenerate every figure in docs/img used by the
README (except the photos and screenshots added by hand, and the figures of
the first real run, which come from its log on the Windows PC).

  python scripts/make_readme_figures.py            (needs pybullet)

Writes:
  docs/img/twin_arena.png          offscreen render of the twin arena
  docs/img/twin_robot.png          close-up of the e-puck model
  docs/img/twin_run_*.png          a 60 s Braitenberg run of the twin timed
                                   like robot 2473 (5 Hz, 85 ms latency)
  docs/img/twin_forager_*.png      the food-finding demo in the twin of the
                                   63 cm arena (3 min)
  docs/img/ir_approach_real_vs_model_front_ir_vs_distance.png
                                   IR approach of robot 2473 vs the model
"""
import math
import os
import sys
from types import SimpleNamespace

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, ROOT)

import matplotlib                                     # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                       # noqa: E402

OUT = "docs/img"
OBSTACLES = [[0.2, 0.2, 0.05], [-0.2, -0.1, 0.05]]
FOOD = [0.25, -0.25, 0.1]


def render(path, eye, target, width, height, fov, robot_pose):
    import pybullet as p
    from sim import world as W
    cfg = W.load_config()
    W.setup_physics(cfg, gui=False)
    W.build_arena(cfg)
    for x, y, r in OBSTACLES:
        h = 0.06
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=r, height=h)
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=h, rgbaColor=[0.9, 0.2, 0.27, 1])
        p.createMultiBody(0, col, vis, [x, y, h / 2])
    W.add_patch_visual(*FOOD)
    robot = W.load_epuck(cfg, lean=False)
    W.place_robot(robot, cfg, *robot_pose)
    for _ in range(30):
        p.stepSimulation()
    view = p.computeViewMatrix(eye, target, [0, 0, 1])
    proj = p.computeProjectionMatrixFOV(fov, width / height, 0.01, 10)
    img = p.getCameraImage(width, height, view, proj, renderer=p.ER_TINY_RENDERER,
                           lightDirection=[0.5, -0.6, 1.0], shadow=1)
    rgb = np.reshape(np.array(img[2], dtype=np.uint8), (height, width, 4))[:, :, :3]
    plt.imsave(path, rgb)
    p.disconnect()
    print("wrote", path)


def main():
    os.makedirs(OUT, exist_ok=True)
    pose = (-0.1, 0.05, math.radians(30))
    render(f"{OUT}/twin_arena.png", eye=[0.0, -1.05, 1.15], target=[0, 0.02, 0],
           width=900, height=640, fov=50, robot_pose=pose)
    render(f"{OUT}/twin_robot.png", eye=[0.05, -0.08, 0.10], target=[-0.1, 0.05, 0.02],
           width=640, height=480, fov=40, robot_pose=pose)

    from epuck_remote import cli
    from epuck_remote.plot import plot_approach, plot_run
    import json
    log = "/tmp/epuck_remote_readme_run.csv" if os.name != "nt" else "readme_run.csv"
    cli.main(["run", "--profile", "calib/profile_twin_2473.json", "--seed", "2",
              "--duration", "60", "--floor", "--log", log,
              "--obstacles", json.dumps(OBSTACLES), "--food", ",".join(map(str, FOOD))])
    plot_run(SimpleNamespace(log=log, out=f"{OUT}/twin_run", dpi=130))

    log2 = log.replace(".csv", "_forager.csv")
    cli.main(["run", "--profile", "calib/profile_twin_arena63.json", "--controller", "forager",
              "--seed", "1", "--duration", "180", "--log", log2])
    plot_run(SimpleNamespace(log=log2, out=f"{OUT}/twin_forager", dpi=130))

    plot_approach(SimpleNamespace(
        csv=["docs/data/approach_2473_white_run2.csv", "docs/data/approach_2473_white_run1.csv",
             "docs/data/approach_2473_mahogany.csv"],
        labels=["white wall, run 2 (end measured)", "white wall, run 1", "mahogany-painted board"],
        measured_end=[1.0, None, None], start_dist=10.0, model=True, yscale="log",
        out=f"{OUT}/ir_approach_real_vs_model", dpi=130))


if __name__ == "__main__":
    main()
