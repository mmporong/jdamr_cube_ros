@echo off
setlocal enableextensions

rem jdamr_cube_ros: launch Gazebo (room.world) + Cartographer/RViz + teleop,
rem each in its own window, running inside WSL (ROS 2 Jazzy).
rem Mirrors readme.md "Gazebo + Cartographer" section 1.

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

echo Stopping any leftover gazebo/cartographer/rviz2/teleop processes...
wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_kill.sh'"

wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_build.sh' '%REPO_WSL%'"
if errorlevel 1 (
    echo.
    echo [ERROR] Build failed. See log above.
    pause
    exit /b 1
)

echo.
echo Build OK. Opening Gazebo / Cartographer+RViz / Teleop windows...

start "1. Gazebo (room.world)" wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_run_gazebo.sh'"
timeout /t 12 /nobreak >nul

start "2. Cartographer + RViz" wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_run_cartographer.sh'"
timeout /t 5 /nobreak >nul

start "3. Teleop (keyboard control)" wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_run_teleop.sh'"
timeout /t 3 /nobreak >nul

start "4. Wrist camera" wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_arm_control.sh' view --camera wrist"
start "5. RGBD overview camera" wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_arm_control.sh' view --camera rgbd"
start "6. RGBD depth camera" wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_arm_control.sh' view --camera depth"

rem Arm joint sliders. Needs arm_controller/gripper_controller to be active and
rem /joint_states to be published, so it starts after Gazebo has settled.
timeout /t 5 /nobreak >nul
start "7. Arm joint control UI" wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_arm_control.sh' ui"

echo.
echo All 7 windows launched. Use the Teleop window to drive the robot and fill the map.
echo Window 7 has sliders for the SO-101 arm joints (q: quit, h: home, r: resync).
echo To save the map from another terminal:
echo   wsl bash -lc "source ~/jdamr_cube_ws/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f ~/maps/jdamr_cube_room"

endlocal
