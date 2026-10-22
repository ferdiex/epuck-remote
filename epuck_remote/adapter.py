"""
adapter.py - raw IR readings -> the normalized values controllers expect.

"lookup" mode (default): raw -> distance with the inverse Webots lookup
table, then n = clip(1 - d / ir_range, 0, 1), the same form the simulators
use (1 - dist / sensor_range). Readings at or below the far-range threshold
count as "nothing detected" (n = 0).

Calibration (from the 'baseline' command, robot in open space):
  offset_i = IR_FAR_RAW - baseline_i, so sensor i reads the table's far value
  in open space whatever its own background is; detection threshold
  IR_FAR_RAW + max(nsigma * std_i, margin). Without calibration: offset 0
  and std_i = the table noise at the far end. std_i never goes below that
  table noise. margin (raw counts) covers backgrounds that drift between
  places or lighting, which the std of one baseline cannot see.
ir_gain multiplies (raw + offset - IR_FAR_RAW) before the lookup, for
robots whose sensors are more or less sensitive than the model.
ir_shift [m] is added to the looked-up distance: a white wall reflects more
IR than the model assumes and reads as if it were closer (measured on
e-puck 2473: +3.3 to +4 cm for white paper, +1 cm for mahogany-painted
board), so the shift restores the true distance.

Sensor order:
  native : ps0..ps7 (ps0 front-right, clockwise), as the robot reports them
  ring   : 8 bearings 0, 45, ..., 315 deg CCW from the heading, as in
           essim3d.get_sensors / foraging_env (max of the nearest ps).
"""
import numpy as np

from .model import IR_FAR_RAW, IR_LOOKUP, ir_distance_from_raw

FAR_REL_NOISE = float(IR_LOOKUP[-1, 2])


class SensorAdapter:
    def __init__(self, mode="lookup", ir_range=0.07, order="native",
                 baseline=None, baseline_std=None, nsigma=3.0, ir_gain=1.0,
                 linear_sat=4095.0, ir_shift=0.0, margin=0.0):
        self.mode = mode
        self.ir_range = ir_range
        self.order = order
        self.ir_gain = ir_gain
        self.ir_shift = ir_shift
        self.linear_sat = linear_sat
        floor_std = IR_FAR_RAW * FAR_REL_NOISE
        if baseline is None:
            self.offset = np.zeros(8)
            std = np.full(8, floor_std)
        else:
            self.offset = IR_FAR_RAW - np.asarray(baseline, float)
            std = np.zeros(8) if baseline_std is None else np.asarray(baseline_std, float)
            std = np.maximum(std, floor_std)
        self.threshold = IR_FAR_RAW + np.maximum(nsigma * std, margin)
        self.last_native = np.zeros(8)

    def normalize(self, raw):
        raw = np.asarray(raw, float) + self.offset
        out = np.zeros(8)
        for i, r in enumerate(raw):
            if self.mode == "linear":
                out[i] = np.clip((r - self.threshold[i]) / (self.linear_sat - self.threshold[i]), 0, 1)
            elif r > self.threshold[i]:
                r = IR_FAR_RAW + self.ir_gain * (r - IR_FAR_RAW)
                d = ir_distance_from_raw(r) + self.ir_shift
                out[i] = np.clip(1.0 - d / self.ir_range, 0.0, 1.0)
        self.last_native = out
        return self.reorder(out)

    def reorder(self, v):
        if self.order == "native":
            return v
        p = v
        return np.array([max(p[7], p[0]), p[6], p[5], p[4],
                         max(p[3], p[4]), p[3], p[2], p[1]])
