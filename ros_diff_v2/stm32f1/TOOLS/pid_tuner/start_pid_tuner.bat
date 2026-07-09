@echo off
setlocal

set "WEB_ROOT=%~dp0"
set "PORT=8765"
set "URL=http://127.0.0.1:%PORT%/"

call :find_python
if not defined PYTHON_CMD (
    echo Python was not found.
    echo Install Python or add it to PATH, then run this file again.
    pause
    exit /b 1
)

echo Starting chassis PID tuner server...
echo Web root: %WEB_ROOT%
echo URL: %URL%
echo.
echo Keep this window open while tuning. Close it to stop the server.

start "" powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -Command "Start-Sleep -Milliseconds 800; Start-Process '%URL%'"
%PYTHON_CMD% -m http.server %PORT% --bind 127.0.0.1 --directory "%WEB_ROOT%"

echo.
echo Server stopped.
pause
exit /b %errorlevel%

:find_python
where py >nul 2>nul
if not errorlevel 1 (
    set "PYTHON_CMD=py -3"
    exit /b 0
)

where python >nul 2>nul
if not errorlevel 1 (
    set "PYTHON_CMD=python"
    exit /b 0
)

if exist "D:\Python_env\python.exe" (
    set "PYTHON_CMD=D:\Python_env\python.exe"
    exit /b 0
)

exit /b 1
