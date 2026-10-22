Calibration results and robot profiles.

- `baseline_<robot>.json`: IR background in open space (`baseline` command).
- `profile_<robot>_real.json`: every calibration knob of one real robot,
  used as `--profile` (keys starting with `_` are comments; options on the
  command line win).
- `profile_twin_<robot>.json`: the twin timed like that robot.
- `approach_*.csv` and figures are working files, ignored by git.

How they were measured: docs/calibration.md; history: docs/hardware_log.md.
