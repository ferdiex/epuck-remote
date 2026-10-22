# TODO / ideas (not planned yet)

## Real robot
- Build a small arena (white foam board on a desk) and redo the short
  calibration inside it: baseline, approach against the foam board, the two
  drives on the desk surface (docs/calibration.md).
- Measure the trim (sideways drift on a straight drive).
- macOS bridge: only the first session after pairing passes data (both
  robots, both firmware builds). Add `scripts/epuck.py bridge <port>`: open
  the Bluetooth port once, forward it to localhost (TCP), and let the CLI
  use `--port socket://localhost:5555` (SerialTransport via
  serial.serial_for_url). Then macOS never reconnects.
- Alternative to try if a cheap USB Bluetooth adapter is at hand: pass it
  through to a Linux virtual machine on the Mac (docs/bluetooth.md).
- Ground sensors: 2292 with rev123 reads them (white ~450-540, black
  ~125-190); 2473 (older build, `m,0,0,0`) does not. Kept as is on purpose,
  for comparison; reflash it with rev123 to know whether it is the build
  or its module.
- A ground-sensor scaling knob (robot white/black -> twin scale) for
  controllers that use the floor.
- 2292: a small turn when the wheels start at low speed (mechanical?).
- Rebuild the battery packs.

## Vision on board (idea)
The camera is useless over Bluetooth for control (images take about a
second). The dsPIC (8 KB RAM) cannot hold a full image, but it can grab a
small window or one line and find a colour blob on board. Idea: a modified
asercom compiled with MPLAB/XC16, uploaded with the bootloader, with a new
command returning only "blob column, blob size"; a matching sensor in the
twin. Would allow colour-seeking behaviours (e.g. the basal ganglia action
selection with seek-object / wall / corner, and "push to the corner"
instead of pick-up: the e-puck has no gripper). Develop on 2292.

## Simulation
- Check how an evolved controller (e.g. the GRU) changes between the
  easabrain control rate (~15 Hz) and the 5 Hz of the real robot over
  Bluetooth; evaluate, or evolve, at 5 Hz with calib/profile_twin_2473.json.
