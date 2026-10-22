# Reloading the original firmware (no MPLAB needed)

The e-puck 1 has a Bluetooth bootloader: a new program (.hex) is sent over
the same Bluetooth serial port, right after a reset. MPLAB is only needed to
compile your own C code, and MPLAB with an ICD 2 programmer only to repair a
damaged bootloader.

## Files

- Original firmware (selector 3 = asercom): `DemoGCtronic-complete-rev123.hex`,
  linked from https://www.gctronic.com/e-puck_links.php ("Software example").
- Uploader on Windows: Tiny Bootloader (tinybld). Its original site
  (etc.ugal.ro) no longer answers; keep your local copy. The GCtronic page
  also links an August 2008 snapshot of the e-puck software
  (e-puck-gna-svn-rev113.zip) with the bootloader tools. Tiny Multi
  Bootloader+ (SourceForge) is derived from it and lists dsPIC30, but its
  compatibility with the e-puck bootloader is not verified.
- For reference, the 2008 Christmas demo found on robot 2292:
  https://www.gctronic.com/files/demoGCtronicSound_jingle.hex

## Procedure (Windows, Tiny Bootloader)

1. Battery charged (a cut in the middle of the upload is the main risk).
   Robot with its wheels off the floor. Selector: any position except one
   where the current program drives around.
2. Pair the robot and note its **outgoing** COM port (`docs/bluetooth.md`).
3. Tiny Bootloader: COM port, 115200.
4. First a harmless check: **Check PIC**, then press the robot's reset
   within ~5 s. It must report the dsPIC (robot 2473: "found ds6014/6012").
   This writes nothing. If it cannot find the PIC, stop.
5. Browse to the .hex, click **Write Flash**, press reset within ~5 s.
   Wait until it finishes; do not switch anything off.
6. Selector at 3, reset: the robot must stay still and quiet.
   `python scripts/epuck.py probe COMx` must answer `v,Version ...`.

Try a new uploader or a new .hex on one robot first.

## Bluetooth module menu (selector 11)

In the standard firmware (main.c of the GCtronic demo), selector 11 is a
text menu for the Bluetooth module, used from a serial terminal (PuTTY,
COM port, 115200): `P` read the PIN, `N` read the name, `R` soft reset of
the module, `O,#` write the PIN, `M,name` write the name, `S,#` write the
PIN and the name e-puck_#, `F` factory reset (then restore with `S,NNNN`).
Note: the selector positions of the 2008 firmware on our robots differ from
the current main.c beyond 0-3 (on 2473, position 8 is a tilt test).
