# Connecting an e-puck 1 over Bluetooth

## The robot

1. Charge the battery. Turn the selector to **3** and switch the robot on
   (or press the blue reset button: the firmware reads the selector only at
   start-up). With the original GCtronic firmware the robot then stays
   **still and quiet, only the green LED on**: it is waiting for commands.
   Other positions of that firmware: 0 reacts to acceleration and free
   fall, 1 turns towards a hand clap, 2 follows walls, 4-15 other demos
   (obstacle avoidance, following...).
2. If at 3 the robot moves, plays music or blinks, it runs another program
   (robot 2292 carried GCtronic's 2008 Christmas demo: E = obstacle
   avoidance, every other position = the Christmas demo). Reload the
   original firmware: `docs/firmware.md`.
3. Each e-puck has a 4-digit number on its sticker; its Bluetooth name is
   `e-puck_NNNN` and the pairing PIN is that number.

## Windows 10/11 (works)

1. Settings > Devices > Bluetooth > Add device > `e-puck_NNNN`, PIN NNNN.
2. More Bluetooth options > COM Ports: use the **Outgoing** port (2473:
   incoming COM4, outgoing COM5). "Paired" only means the two know each
   other: the link opens when a program opens the port.
3. Quick checks, in order:
   - e-puck Monitor (GCtronic, https://www.gctronic.com/e-puck_links.php):
     shows the sensors. Windows SmartScreen may block it: "More info" >
     "Run anyway", or Properties > Unblock. Continuous camera images hang
     over Bluetooth; we do not use the camera.
   - PuTTY: Serial, COM5, 115200; type `V` and Enter, it answers
     `v,Version ...` (PuTTY does not echo what you type).
   - `python scripts/epuck.py probe COM5`
4. Only one program can use the COM port at a time: close the Monitor,
   PuTTY or Tiny Bootloader before running epuck-remote.

## macOS: one session per pairing

What we found (MacBook Air and Mac mini, robots 2473 and 2292, both
firmware builds):

- after forgetting the device and pairing it again, the **first** time a
  program opens `/dev/cu.e-puck_NNNN` the link comes up in ~2.5 s, the
  first reply is garbled (`z,Command not found`) and then every command is
  answered;
- once that program closes the port, later sessions fail: no data at all,
  even when the robot's Bluetooth LED shows the link is up, and even after
  resetting the robot during the session;
- forgetting and pairing again gives one more good session.

So on a Mac: pair, then do everything you need in one session, or pair
again before the next one. Confirmed on 2026-10-21 with robot 2292: the
food demo ran 2 minutes right after pairing (3 finds); the next command got
no answer. The link is also slower than from Windows (~225 ms per control
step with the ground sensors, against ~135 ms): use `--hz 4` or less. epuck-remote waits for the link to answer
(`--connect-timeout`). Windows does not have this problem.

Steps: System Settings > Bluetooth, forget `e-puck_NNNN`, pair it again
(PIN NNNN), close the window (it keeps trying to "connect" on its own), use
`/dev/cu.e-puck_NNNN` (not `tty`).

Planned fix (`TODO.md`): a bridge process that opens the Bluetooth port
once and shares it on localhost (pyserial `socket://` URLs), so that
probe/run/drive never reopen the Bluetooth port.

Untested alternative: a virtual machine (Parallels, UTM, VMware) using the
Mac's own Bluetooth goes through macOS and should have the same problem. A
cheap **USB Bluetooth adapter passed through to the virtual machine** (USB
passthrough) bypasses macOS: the guest drives it with its own stack, as on
a normal PC. On Apple-silicon Macs, a Linux guest (`scripts/bt_bind.sh`) is
likely easier than Windows on ARM, whose drivers for such adapters vary.

## Jolt when the link opens

While the link comes up, the first bytes can be garbled (`z,Command not
found`). Once, robot 2292 turned ~45 deg before the first real command, most
likely a garbled byte read as a binary speed command (asercom accepts
binary commands, bytes with the high bit set). The programs stop the
wheels as soon as the robot answers, and `drive`/`approach` wait for Enter
so the robot can be placed again.

## Linux

`bluetoothctl` -> `scan on`, `pair <MAC>` (PIN NNNN), `quit`; then
`scripts/bt_bind.sh <MAC>` creates `/dev/rfcomm0` (sudo; join the
`dialout` group to open it without sudo). Not yet tried with our robots.

## Protocol

GCtronic wiki, "Advanced sercom protocol"
(https://www.gctronic.com/doc/index.php/Advanced_sercom_protocol).
Commands used here: D, E, K, M, N, O, Q, V (ASCII, ended by CR). By hand,
for a quick test in any serial terminal: `V` version, `N` proximity,
`M` ground sensors, `D,200,200` both wheels forward (wheels off the floor
first), `D,0,0` stop.
