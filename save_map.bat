@echo off
setlocal enableextensions

rem Save the /map topic currently published by cartographer_node to .pgm/.yaml.
rem Requires the simulator + cartographer to already be running
rem (e.g. started via run_gazebo_slam.bat).
rem Usage: save_map.bat [map-name]   (default map-name: jdamr_cube_room)

set "REPO_WIN=%~dp0"
if "%REPO_WIN:~-1%"=="\" set "REPO_WIN=%REPO_WIN:~0,-1%"

for /f "delims=" %%i in ('wsl wslpath -a "%REPO_WIN%"') do set "REPO_WSL=%%i"
if "%REPO_WSL%"=="" (
    echo [ERROR] Failed to resolve WSL path. Is WSL installed?
    pause
    exit /b 1
)

set "MAP_NAME=%~1"
if "%MAP_NAME%"=="" set "MAP_NAME=jdamr_cube_room"

echo Saving map as "%MAP_NAME%"...
wsl bash -lc "bash '%REPO_WSL%/scripts/wsl_save_map.sh' '%REPO_WSL%' '%MAP_NAME%'"
if errorlevel 1 (
    echo.
    echo [ERROR] Map save failed. Is the simulator + cartographer running?
    pause
    exit /b 1
)

echo.
echo Done. Files are in %REPO_WIN%\maps\%MAP_NAME%.pgm / .yaml
pause

endlocal
