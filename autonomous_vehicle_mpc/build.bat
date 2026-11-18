@echo off
REM ==============================================================================
REM AUTONOMOUS VEHICLE MPC - WINDOWS BUILD SCRIPT (MinGW UCRT64 + CMake)
REM ==============================================================================

set "PATH=D:\Appfile\Code\msys64\ucrt64\bin;C:\msys64\ucrt64\bin;C:\Program Files\CMake\bin;%PATH%"
cd /d "%~dp0"

echo ====================================================================
echo   BIEN DICH HE THONG MO PHONG XE TU HANH C++ (AUTOSAR ADAPTIVE)
echo ====================================================================

REM Kiem tra neu thu muc build chua co hoac chua duoc cau hinh
if not exist "build\build.ninja" if not exist "build\Makefile" if not exist "build\autonomous_vehicle_mpc.sln" (
    echo [CMAKE] Dang cau hinh he thong bien dich vao thu muc build/...
    if exist "D:\Appfile\Code\msys64\ucrt64\bin\g++.exe" (
        cmake -B build -G "MinGW Makefiles" -DCMAKE_CXX_COMPILER="D:/Appfile/Code/msys64/ucrt64/bin/g++.exe" -DCMAKE_MAKE_PROGRAM="D:/Appfile/Code/msys64/ucrt64/bin/mingw32-make.exe" -DCMAKE_BUILD_TYPE=Release
    ) else if exist "C:\msys64\ucrt64\bin\g++.exe" (
        cmake -B build -G "MinGW Makefiles" -DCMAKE_CXX_COMPILER="C:/msys64/ucrt64/bin/g++.exe" -DCMAKE_MAKE_PROGRAM="C:/msys64/ucrt64/bin/mingw32-make.exe" -DCMAKE_BUILD_TYPE=Release
    ) else (
        cmake -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
    )
    if %ERRORLEVEL% neq 0 (
        echo [ERROR] Cau hinh CMake that bai!
        exit /b 1
    )
)

REM Tien hanh bien dich song song 4 luong
echo [BUILD] Dang bien dich cac module C++...
cmake --build build -j4
if %ERRORLEVEL% equ 0 (
    echo.
    echo ====================================================================
    echo [THANH CONG] Da tao xong file thuc thi:
    echo   - build\autonomous_mpc_unified.exe (Unified Runner)
    echo   - build\autonomous_core.exe        (Process 2 - 50Hz QP-MPC)
    echo   - build\paio_vehicle_interface.exe (Process 1 - 100Hz HAL)
    echo   - build\hmi_diagnostics.exe        (Process 3 - 20Hz HMI)
    echo ====================================================================
) else (
    echo.
    echo [ERROR] Bien dich that bai. Vui long kiem tra lai thong bao loi tren.
    exit /b 1
)
