# Calibrating a robot against the twin

Goal: the same controller with the same parameters behaves the same on the
twin (`sim`) and on the robot (`real`). The knobs below correct the real
robot towards the model, so the twin keeps its defaults (only its timing is
matched). Results go into a profile, `calib/profile_<robot>_real.json`,
and a line in `docs/hardware_log.md`.

**Calibrate on the floor and against the walls of the experiments.** Wheel
slip depends on the floor (glass and wood gave different results on robot
2473) and the IR reading depends a lot on the wall colour.

Commands below use the launcher, `python scripts/epuck.py <command> PORT`
(PORT = `COM5`, `/dev/rfcomm0`, ...). Work through the steps in order.

On the real robot, `drive` and `approach` connect, stop the wheels and then
wait for Enter: opening the Bluetooth link can jolt the wheels (robot 2292
turned ~45 deg before an approach), so place the robot after connecting
(`--no-confirm` skips the pause).

## 1. Link and control rate

    python scripts/epuck.py probe PORT --n 200

Look at the N round trip. A control step costs one N plus one D (plus one M
with `--floor`), so `--hz` must stay below 1 / (2 x round trip). Robot 2473
from Windows: ~85 ms per command -> **`--hz 5`**. In the twin use the same
`--hz` and `--sim-latency` about one round trip (0.085).

At 5 Hz the robot moves 2.6 cm per step at full speed and reacts late; in
the twin that gave 2-7% of steps touching obstacles, and **`--max-speed
0.08`** brought it to ~0-1%. Use the same value on the twin and the robot.

## 2. IR background

Robot in open space, nothing within ~15 cm, where the experiments will run:

    python scripts/epuck.py baseline PORT --out calib/baseline_<robot>.json

Use it with `--calib`. If the background changes from place to place (on
2473, ps5 read 60 in one place and 85 in another with nothing near),
either run `baseline` again just before the experiment, use
`--auto-baseline 20` (reads it at the start of every run: the robot must be
in open space then), or add **`--ir-margin`** (raw counts the reading must
exceed above the background; 30 on 2473). A noisy sensor (ps0 on 2473, std
20) automatically gets a higher threshold and a shorter range.

## 3. Speed and turning

On the experiment floor, without motion knobs (no profile):

    python scripts/epuck.py drive PORT --left 500 --right 500 --seconds 5
    python scripts/epuck.py drive PORT --left -300 --right 300 --seconds 5

The command prints what the model expects (distance, rotation). Measure the
real distance with a ruler and the real rotation (marks on the floor). Then

    speed_scale = expected distance / measured distance
    turn_scale  = (expected rotation / measured rotation) / speed_scale

2473 on wood: 31.7 cm expected, 34 measured -> 0.93; ~417 deg expected,
~405 measured -> (417/405)/0.93 = 1.10.

**Trim.** In the straight run, also measure how far the front ended to the
side of the starting line (y, cm, positive to the right) over the distance
travelled (D, cm). With the 5.2 cm axle,

    trim = 5.2 * y / D^2          (positive when it drifts right)

e.g. 2 cm to the right over 34 cm -> 0.009. Check: run the same `drive`
with the new knobs (`--speed-scale ... --turn-scale ... --trim ...`): it
prints the commands actually sent, and the robot should now match the
expected values.

## 4. IR response to the walls

Robot facing the wall material of the arena, its front 10 cm away
(measured), nothing else within 15 cm:

    python scripts/epuck.py approach PORT

It drives at 1.3 cm/s and stops when the front reads more than 1500. When
it stops, measure the distance from its front to the wall. Then

    python -m epuck_remote.plot approach --csv calib/approach_<...>.csv --model --measured-end <cm>

(or `scripts/compare_approach.sh` / `scripts\win\compare_approach.bat`)
overlays the curve on the model and prints the estimated **`--ir-shift`**:
how much closer than the model the robot reads that wall. On 2473: white
paper +3.1 to +3.8 cm (two runs; **0.035**), mahogany-painted board +0.9 cm.
Repeat it two or three times; keep the robot square to the wall (it drifts
on a long approach, which shows as a dip in the curve).

`--ir-range` (0.07) is the model's range and stays as is when `--ir-shift`
is used. `--ir-gain` scales the reading instead of shifting it; on 2473 the
curves were shifted, not scaled, so it stays at 1.

## 5. Profile and behaviour check

Write the values into `calib/profile_<robot>_real.json` (see the 2473 one;
keys starting with `_` are comments) and run the same controller on both:

    python scripts/epuck.py run calib/profile_twin_2473.json --gui --duration 60
    python scripts/epuck.py run calib/profile_2473_real.json --duration 60

in a similar arena; compare the logs and figures (time near walls,
escapes, turning).
