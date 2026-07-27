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
