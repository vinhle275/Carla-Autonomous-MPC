#!/usr/bin/env bash
# ==============================================================================
# AUTONOMOUS VEHICLE MPC - LINUX / WSL2 LAUNCHER (PURE C++ RUNTIME)
# ==============================================================================

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "===================================================================="
echo "  KHOI DONG HE THONG MO PHONG XE TU HANH C++ (AUTOSAR AP RUNTIME)   "
echo "===================================================================="

# Tự động phát hiện IP host Windows nếu chạy từ bên trong WSL2
DEFAULT_HOST="127.0.0.1"
if [ -f "/etc/resolv.conf" ]; then
    WSL_HOST=$(grep nameserver /etc/resolv.conf | awk '{print $2}')
    if [ -n "$WSL_HOST" ]; then
        DEFAULT_HOST="$WSL_HOST"
    fi
fi

# Tìm binary đã biên dịch
TARGET_BIN=""
if [ -f "build_linux/autonomous_mpc_unified" ]; then
    TARGET_BIN="build_linux/autonomous_mpc_unified"
elif [ -f "build/autonomous_mpc_unified" ]; then
    TARGET_BIN="build/autonomous_mpc_unified"
elif [ -f "build/autonomous_mpc_unified.exe" ]; then
    TARGET_BIN="build/autonomous_mpc_unified.exe"
fi

if [ -z "$TARGET_BIN" ]; then
    echo "[BUILD] Đang tiến hành biên dịch ứng dụng C++ cho Linux/WSL..."
    mkdir -p build_linux
    cmake -B build_linux -DCMAKE_BUILD_TYPE=Release
    cmake --build build_linux -j4
    TARGET_BIN="build_linux/autonomous_mpc_unified"
fi

echo "[SYSTEM] Chạy C++ Autonomous Runtime: $TARGET_BIN"
"$TARGET_BIN" --host "$DEFAULT_HOST" --with-obstacles "$@"
