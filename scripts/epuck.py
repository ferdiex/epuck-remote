#!/usr/bin/env python3
"""
epuck.py - cross-platform launcher (Windows, macOS, Linux).

  python scripts/epuck.py <command> <target> [options]

command: probe | baseline | run | drive | approach
target:  mock | sim | sim-gui | a serial port (COM5, /dev/rfcomm0,
         /dev/cu.e-puck_2473) | a profile JSON (calib/profile_2473_real.json)
options: any option of python -m epuck_remote.cli <command> --help

Adds sensible file names: run -> runs/<date>_<target>.csv plus figures;
baseline -> calib/baseline_<target>.json; approach ->
calib/approach_<target>_<date>.csv (unless --log / --out is given).

  python scripts/epuck.py test        runs the smoke tests
"""
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, ROOT)


def target_options(target):
    if target.endswith(".json"):
        return ["--profile", target], os.path.splitext(os.path.basename(target))[0]
    if target == "mock":
        return ["--backend", "mock"], "mock"
    if target == "sim":
        return ["--backend", "sim"], "sim"
    if target == "sim-gui":
        return ["--backend", "sim", "--gui"], "sim-gui"
    tag = "".join(ch if ch.isalnum() else "_" for ch in target).strip("_")
    return ["--backend", "real", "--port", target], tag


def main(argv):
    if argv and argv[0] == "test":
        return subprocess.call([sys.executable, os.path.join("tests", "test_smoke.py")])
    if len(argv) < 2:
        print(__doc__)
        return 1
    cmd, target, rest = argv[0], argv[1], argv[2:]
    opts, tag = target_options(target)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    if cmd == "run" and "--log" not in rest:
        opts += ["--log", f"runs/{stamp}_{tag}.csv", "--plot"]
    elif cmd == "baseline" and "--out" not in rest:
        opts += ["--out", f"calib/baseline_{tag}.json"]
    elif cmd == "approach" and "--out" not in rest:
        opts += ["--out", f"calib/approach_{tag}_{stamp}.csv"]
    from epuck_remote import cli
    cli.main([cmd] + opts + rest)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
