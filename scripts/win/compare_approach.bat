@echo off
REM Overlay approach logs with the model and print the estimated --ir-shift.
REM Usage: scripts\win\compare_approach.bat calib\approach_a.csv [more.csv] [--measured-end 1.0 ...]
cd /d "%~dp0\..\.."
python -m epuck_remote.plot approach --model --out calib\ir_approach_comparison --csv %*
