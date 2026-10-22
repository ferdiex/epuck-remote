"""
model.py - e-puck 1 constants shared by every backend (pure numpy, no
PyBullet). Values copied from sim/sensors.py and sim/config.json, which take
them from the Webots model (E-puck.proto, E-puckDistanceSensor.proto,
E-puckGroundSensors.proto); tests/test_smoke.py checks they stay identical.
"""
import math

import numpy as np

# IR proximity ps0..ps7, robot frame (x forward, y left, z up). ps0 is
# front-right and the order runs clockwise, as the asercom N command reports.
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
IR_ANGLES = np.degrees(np.arctan2(IR_DIR[:, 1], IR_DIR[:, 0]))   # horizontal bearings

# Webots lookupTable: distance [m], raw value, relative noise
IR_LOOKUP = np.array([
    [0.000, 4095.00, 0.002], [0.005, 2133.33, 0.003], [0.010, 1465.73, 0.007],
    [0.015, 601.46, 0.0406], [0.020, 383.84, 0.01472], [0.030, 234.93, 0.0241],
    [0.040, 158.03, 0.0287], [0.050, 120.00, 0.04225], [0.060, 104.09, 0.03065],
    [0.070, 67.19, 0.04897],
])
IR_RANGE = float(IR_LOOKUP[-1, 0])
IR_FAR_RAW = float(IR_LOOKUP[-1, 1])          # reading at the end of the range
GS_LOOKUP = np.array([[0.000, 1000.0, 0.002], [0.016, 300.0, 0.004]])
# ground sensors gs0 (left), gs1 (centre), gs2 (right), robot frame [m]
GS_POS = np.array([[0.03, 0.01, 0.0033], [0.03, 0.00, 0.0033], [0.03, -0.01, 0.0033]])

BODY_RADIUS = 0.037          # m, collision cylinder
WHEEL_RADIUS = 0.020         # m (sim/config.json epuck.wheel_radius)
AXLE = 0.052                 # m (sim/config.json epuck.axle_length)
STEPS_PER_REV = 1000         # stepper motors
MAX_STEPS = 1000             # asercom speed limit, steps/s (= 1 rev/s)
STEP_LEN = 2 * math.pi * WHEEL_RADIUS / STEPS_PER_REV   # m per step
RAD_PER_STEP = 2 * math.pi / STEPS_PER_REV


def ir_raw_from_distance(d):
    """Lookup-table value for distance d [m] (Webots style, clamped)."""
    return float(np.interp(d, IR_LOOKUP[:, 0], IR_LOOKUP[:, 1]))


def ir_distance_from_raw(raw):
    """Inverse lookup: raw value -> distance [m] (np.interp needs ascending x)."""
    return float(np.interp(raw, IR_LOOKUP[::-1, 1], IR_LOOKUP[::-1, 0]))


def steps_from_mps(v):
    return v / STEP_LEN


def mps_from_steps(s):
    return s * STEP_LEN
