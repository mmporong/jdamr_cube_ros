@echo off
setlocal enableextensions

rem Move the SO-101 arm to a joint target. Requires the simulator to already
rem be running (run_gazebo_slam.bat or run_navigation.bat) so that
rem arm_controller / gripper_controller are active.
rem Usage: arm_control.bat [--shoulder-pan V] [--shoulder-lift V] [--elbow-flex V]
rem                        [--wrist-flex V] [--wrist-roll V] [--gripper V] [--duration SEC]
rem   example: arm_control.bat --shoulder-pan 0.5 --elbow-flex 0.4
rem   example: arm_control.bat --gripper 1.0

if "%~1"=="" (
    echo Usage: arm_control.bat [--shoulder-pan V] [--shoulder-lift V] [--elbow-flex V] [--wrist-flex V] [--wrist-roll V] [--gripper V] [--duration SEC]
    echo   example: arm_control.bat --shoulder-pan 0.5 --elbow-flex 0.4
    echo   example: arm_control.bat --gripper 1.0
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

wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_arm_control.sh' %*"

if errorlevel 1 (
    echo.
    echo [ERROR] Arm goal failed or was rejected. Is the simulator running?
    pause
    exit /b 1
)

pause
endlocal
