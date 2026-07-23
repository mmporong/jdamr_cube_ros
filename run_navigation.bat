@echo off
setlocal enableextensions

rem jdamr_cube_ros: launch Gazebo (room.world) + Nav2 (jdamr_cube_controller),
rem each in its own window, running inside WSL (ROS 2 Jazzy).
rem Requires a saved map first: run_gazebo_slam.bat + save_map.bat.
rem After both windows are up, send goals with goto_pose.bat X Y [YAW].

set "REPO_WIN=%~dp0"
if "%REPO_WIN:~-1%"=="\" set "REPO_WIN=%REPO_WIN:~0,-1%"

for /f "delims=" %%i in ('wsl wslpath -a "%REPO_WIN%"') do set "REPO_WSL=%%i"
if "%REPO_WSL%"=="" (
    echo [ERROR] Failed to resolve WSL path. Is WSL installed?
    pause
    exit /b 1
)

echo ===============================================
echo  jdamr_cube_ws build (WSL / ROS 2 Jazzy)
echo  %REPO_WIN%  -^>  %REPO_WSL%
echo ===============================================

echo Stopping any leftover gazebo/nav2/cartographer/rviz2/teleop processes...
wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_kill.sh'"

wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_build.sh' '%REPO_WSL%'"
if errorlevel 1 (
    echo.
    echo [ERROR] Build failed. See log above.
    pause
    exit /b 1
)

echo.
echo Build OK. Opening Gazebo / Nav2 windows...

start "1. Gazebo (room.world)" wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_run_gazebo.sh'"
timeout /t 12 /nobreak >nul

start "2. Nav2 (jdamr_cube_controller)" wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_run_navigation.sh'"
timeout /t 8 /nobreak >nul

echo.
echo Both windows launched. Nav2 loads ~/maps/jdamr_cube_room.yaml by default
echo (create it first with run_gazebo_slam.bat + save_map.bat if you have not yet).
echo.
echo Send the robot to a goal from another terminal:
echo   goto_pose.bat X Y [YAW]
echo   example: goto_pose.bat 1.0 -1.8

endlocal
