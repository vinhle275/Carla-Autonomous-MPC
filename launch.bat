@echo off
REM ==============================================================================
REM AUTONOMOUS VEHICLE MPC - WINDOWS LAUNCHER (PURE C++ RUNTIME)
REM ==============================================================================

set "PATH=D:\Appfile\Code\msys64\ucrt64\bin;C:\msys64\ucrt64\bin;C:\Program Files\CMake\bin;%PATH%"
cd /d "%~dp0"

echo ====================================================================
echo   KHOI DONG HE THONG MO PHONG XE TU HANH C++ (AUTOSAR AP RUNTIME)
echo ====================================================================

REM Bien dich incremental dam bao binary luon cap nhat code moi nhat
call build.bat
if not exist "build\autonomous_mpc_unified.exe" (
    echo [ERROR] Bien dich that bai! Khong the khoi dong.
    exit /b 1
)

REM Khoi dong ung dung C++ duy nhat (tu dong mo Cockpit Viewer + CARLA Bridge)
build\autonomous_mpc_unified.exe --with-obstacles %*
