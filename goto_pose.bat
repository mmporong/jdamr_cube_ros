@echo off
setlocal enableextensions

rem Send jdamr_cube to a goal pose (map frame). Requires run_navigation.bat to
rem already be running.
rem Usage: goto_pose.bat X Y [YAW]
rem   example: goto_pose.bat 1.0 -1.8
rem   example: goto_pose.bat 0.0 0.0 1.57

if "%~2"=="" (
    echo Usage: goto_pose.bat X Y [YAW]
    echo   example: goto_pose.bat 1.0 -1.8
    pause
    exit /b 1
)

set "REPO_WIN=%~dp0"
if "%REPO_WIN:~-1%"=="\" set "REPO_WIN=%REPO_WIN:~0,-1%"

for /f "delims=" %%i in ('wsl wslpath -a "%REPO_WIN%"') do set "REPO_WSL=%%i"
if "%REPO_WSL%"=="" (
    echo [ERROR] Failed to resolve WSL path. Is WSL installed?
    pause
    exit /b 1
)

set "GX=%~1"
set "GY=%~2"
set "GYAW=%~3"

if "%GYAW%"=="" (
    wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_goto_pose.sh' %GX% %GY%"
) else (
    wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_goto_pose.sh' %GX% %GY% --yaw %GYAW%"
)

if errorlevel 1 (
    echo.
    echo [ERROR] Goal failed or was rejected. Is run_navigation.bat running?
    pause
    exit /b 1
)

pause
endlocal
