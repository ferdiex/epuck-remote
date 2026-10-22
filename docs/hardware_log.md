# Hardware log

One section per robot (number on the sticker), newest notes last.

## e-puck 2473

| date | test | result |
|------|------|--------|
| 2026-10-15 | firmware / selector | original GCtronic firmware: 1 = turns to a clap, 3 = still and quiet (asercom) |
| 2026-10-15 | Bluetooth, Windows 10 | works; outgoing COM5 (incoming COM4); e-puck Monitor shows the sensors |
| 2026-10-15 | Bluetooth, macOS (Mac mini, laptop) | pairs and creates /dev/cu.e-puck_2473, but the link never opens (no LED, no reply) |
| 2026-10-16 | firmware version (V) | asercom 1.2.2, August 2008, GCtronic; HW version FFF9 |
| 2026-10-16 | Bluetooth, MacBook Air (after pairing again) | first session: link up ~2.5 s after opening /dev/cu.e-puck_2473, first reply garbled, then stable; later sessions: no answer, Bluetooth LED never lights |
| 2026-10-16 | Bluetooth, Mac mini (Bluetooth keyboard and mouse) | one reply, then the link drops |
| 2026-10-16 | macOS diagnosis (with 2292 too) | one good session per pairing; later sessions: link up (LED) but no data, robot reset does not help -> macOS, not the robot |
| 2026-10-16 | selector 8 | tilt test (red LED on the lowered side), not the ground-sensor behaviour of the current firmware |
| 2026-10-15 | Tiny Bootloader, Check PIC over COM5 | "found ds6014/6012" (nothing written) |
| 2026-10-15 | link timing | ~85 ms per asercom command; run loop (N + D) ~185 ms -> 5 Hz |
| 2026-10-15 | microphones | the three "micro amplitude" values respond to sound |
| 2026-10-15 | ground sensors (M) | 0 on every surface; module cable runs under the motors, not checked |
| 2026-10-15 | IR baseline (open floor) | ps0..ps7 mean 41, 21, 42, 52, 74, 60, 45, 59; ps0 noisy (std 20), ps7 std 5, others < 2 |
| 2026-10-15 | IR background drift | ps5 read ~85 (baseline 60) and ps2 ~105 (baseline 42) in other places -> ir_margin 30 |
| 2026-10-15 | drive 500/500 5 s, glass desk | 33 cm (model 31.7) |
| 2026-10-15 | drive 500/500 5 s, wood floor | 34 cm (model 31.7) -> speed_scale 0.93 |
| 2026-10-15 | drive -300/300 5 s, glass and wood | ~405 deg (model ~417) -> turn_scale 1.10 |
| 2026-10-15 | approach, mahogany-painted board, wood floor | close to the model; ir_shift ~+0.9 cm |
| 2026-10-15 | approach, white paper wall, wood floor | reads 3.1-3.8 cm closer than the model (2 runs) -> ir_shift 0.035; drifts right on a long straight approach |
| 2026-10-15 | first Braitenberg run (5 Hz) | avoids the wall correctly; false weak detection on ps5 (fixed by ir_margin) |
| 2026-10-15 | battery | one robot with good cells ran over an hour (earlier test) |

Profile: `calib/profile_2473_real.json`.

## e-puck 2292

| date | test | result |
|------|------|--------|
| 2026-10-15 | firmware | GCtronic December 2008 Christmas demo (demoGCtronicSound_jingle.hex): E = obstacle avoidance, every other position = Christmas demo; no asercom |
| 2026-10-15 | Bluetooth, macOS | pairs and keeps the link (LED on) |
| 2026-10-16 | reload original firmware | DemoGCtronic-complete-rev123.hex with Tiny Bootloader over COM7: Check PIC "found ds6014/6012", then WRITE OK in 19 s (the reset has to come ~1-2 s after clicking, once the Bluetooth link is up) |
| 2026-10-16 | after reload | selector 3 = asercom; V answers "1.2.2 August 2008" too, but M returns five numbers (2473: three) -> a newer build |
| 2026-10-16 | ground sensors (M) | work: white 446, 511, 539; black 161, 124, 192 (gs0, gs1, gs2); threshold ~330; a thin marker line gives intermediate values |
| 2026-10-16 | macOS | same as 2473: one good session per pairing |
| 2026-10-21 | probe (Windows, COM7) | N round trip 56 ms mean (2473: 85 ms) |
| 2026-10-21 | IR baseline, arena centre | 101, 59, 125, 120, 155, 261, 86, 74; ps5 high but steady (std 1.1), ps0 noisy (std 16) as on 2473 |
| 2026-10-21 | drive 500/500 5 s, arena paper floor | 32.5 cm, straight (no sideways drift) |
| 2026-10-21 | drive -300/300 5 s, arena | ~405 deg (model 415) |
| 2026-10-21 | motion knobs (arena paper floor) | speed_scale 0.975 (32.5 cm vs 31.7), turn_scale 1.05, trim 0 |
| 2026-10-21 | approach x2, white wall | invalid: the robot turned ~45 deg while the link opened, arrived at an angle (ps0 >> ps7), stopped at 2-3 cm; front IR looks weaker than 2473's. Repeat with the Enter pause |
| 2026-10-21 | approach x2 with the Enter pause, white wall | repeatable; ended ~2 cm (measured); ir_shift +2.1 / +2.2 cm -> 0.02 (2473: 0.035); a small turn remains when the wheels start (mechanical) |
| 2026-10-21 | profile | `calib/profile_2292_real.json`; twin of the arena: `calib/profile_twin_arena63.json` |
| 2026-10-21 | Braitenberg 60 s, arena (Windows) | 300 steps at 5 Hz, loop 133 ms mean, 0 overruns, 0 link errors; follows the walls (the "diamond"); whole command 66 s (PowerShell Measure-Command) |
| 2026-10-21 | food demo (Windows) | found the patch at 23.4 s and 45.4 s, spinning 10 s each time; stopped by hand at 57 s |
| 2026-10-21 | food demo (MacBook Air, right after pairing) | 3 finds in 2 min; loop 226 ms mean, p95 402 ms (overruns at 5 Hz); next command: no answer (macOS, one session per pairing) |
| 2026-10-21 | step counters | 16-bit, wrap around (drive printed -62980); fixed in the program |
| 2026-10-21 | lost link during a drive | the robot kept moving after Ctrl-C: asercom has no watchdog; switch it off. Programs now warn when they cannot stop it |

## Template

| date | test | result |
|------|------|--------|
|      | firmware / selector 3 | |
|      | Bluetooth (OS, port) | |
|      | probe: version, latency | |
|      | ground sensors (M) | |
|      | baseline | |
|      | drive straight / spin | |
|      | approach (wall material) | |
