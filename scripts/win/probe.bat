@echo off
REM Usage: scripts\win\probe.bat TARGET [options]
REM TARGET is a COM port (COM5), mock, or a profile such as calib\profile_2473_real.json
REM Run it from a terminal where the Python environment is active.
python "%~dp0..\epuck.py" probe %*
