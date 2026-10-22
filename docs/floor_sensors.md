# Testing the ground (floor) sensors

The e-puck 1 ground-sensor extension has three IR sensors looking down at
the front of the robot: gs0 left, gs1 center, gs2 right. They are read with
the asercom `M` command. Bright floor gives high values, a dark surface low
values. In the twin a dark disc (`--food x,y,r`) reflects 20% of the light
(`task.food_reflectance` in sim/config.json).

## 1. Twin, robot standing still
On a dark patch and off it:

    scripts/probe.sh sim --start 0,0,0 --food 0.03,0,0.05
    scripts/probe.sh sim --start 0,0,0

The `floor gs0..gs2` line should read about 170 on the patch and about 860
off it.

## 2. Twin, driving over the patch
The robot starts at the left heading +x and crosses a 10 cm patch at the
centre (open floor, so the Braitenberg goes straight):

    scripts/run.sh sim-gui --start -0.35,0,0 --food 0,0,0.1 --floor --duration 5 --verbose

The terminal prints the floor values every 10 steps; the run leaves
`runs/<date>_sim-gui_ground_sensors.png`, where the three curves drop
while the robot is on the patch.

## 3. Real robot, standing still

Note from robot 2473: its ground sensors read 0 on every surface (white
paper, black neoprene, in the air), i.e. the firmware does not get readings
from the module; the module's cable runs under the motors and was not
checked. The e-puck Monitor's three "micro amplitude" values are the
microphones, not the ground sensors. Try `M` on another robot before
blaming the firmware.
Selector at 3, robot on the floor of the arena:

    scripts/probe.sh /dev/rfcomm0

- `floor gs0..gs2` all 0: no ground-sensor extension on this robot (or the
  firmware does not read it). Note it in docs/hardware_log.md.
- non-zero values: slide a sheet of black paper or tape under the front of
  the robot and probe again; the values must drop. Some black materials
  reflect infrared (black neoprene did not change anything on 2473, but
  that robot read 0 anyway): prefer laser-printed black, black electrical
  tape or black cardboard. Try it under each side
  to check which sensor is which (gs0 left, gs2 right).

Write the white-floor and black-paper values in docs/hardware_log.md: they
give the threshold for "on food" on the real floor (the twin uses 500).

## 4. Real robot, driving over a dark patch
Put a black paper disc in the arena, the robot ~30 cm before it:

    scripts/run.sh /dev/rfcomm0 --floor --duration 10 --calib calib/<baseline>.json

and compare runs/<date>_..._ground_sensors.png with the twin's.
