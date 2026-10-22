@echo off
REM Create .venv and install what the mock and the real robot need.
REM (The PyBullet twin is easier on macOS/Linux with conda: scripts/setup.sh.)
cd /d "%~dp0\..\.."
python -m venv .venv
if errorlevel 1 (echo Python not found: install it from python.org with "Add python.exe to PATH" & exit /b 1)
.venv\Scripts\python -m pip install -U pip
.venv\Scripts\python -m pip install numpy matplotlib pyserial
.venv\Scripts\python tests\test_smoke.py
echo.
echo Ready. In every new terminal: cd to this folder, then .venv\Scripts\activate
