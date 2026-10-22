@echo off
REM Smoke tests; the sim ones are skipped without pybullet.
python "%~dp0..\epuck.py" test
