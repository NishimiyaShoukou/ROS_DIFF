@echo off
setlocal

set "WEB_ROOT=%~dp0"
set "PORT=8766"
set "URL=http://127.0.0.1:%PORT%/"

call :find_python
if not defined PYTHON_CMD (
    echo Python was not found.
    echo Install Python or add it to PATH, then run this file again.
    pause
    exit /b 1
)

echo Starting URDF editor server...
echo Web root: %WEB_ROOT%
echo URL:      %URL%
echo.
echo Keep this window open while editing. Close it to stop the server.
echo Edits are auto-backed up to "*.bak" before each save.

start "" powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -Command "Start-Sleep -Milliseconds 800; Start-Process '%URL%'"

%PYTHON_CMD% "%WEB_ROOT%urdf_server.py" --port %PORT% --bind 127.0.0.1
if errorlevel 1 (
    echo.
    echo Server exited with error.
    pause
    exit /b %errorlevel%
)

echo.
echo Server stopped.
pause
exit /b 0

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

exit /b 1
