@echo off
REM ==============================================================================
REM AUTONOMOUS VEHICLE MPC - WINDOWS LAUNCHER (PURE C++ RUNTIME)
REM ==============================================================================

set "PATH=D:\Appfile\Code\msys64\ucrt64\bin;C:\Program Files\CMake\bin;%PATH%"
cd /d "%~dp0"

echo ====================================================================
echo   KHOI DONG HE THONG MO PHONG XE TU HANH C++ (AUTOSAR AP RUNTIME)
echo ====================================================================

REM Kiem tra neu binary C++ chua duoc bien dich thi tu dong goi build.bat
if not exist "build\autonomous_mpc_unified.exe" (
    echo [BUILD] Chua tim thay binary C++. Dang tien hanh tu dong bien dich...
    call build.bat
    if not exist "build\autonomous_mpc_unified.exe" (
        echo [ERROR] Bien dich that bai! Khong the khoi dong.
        exit /b 1
    )
)

REM Khoi dong ung dung C++ duy nhat
build\autonomous_mpc_unified.exe --with-obstacles %*
