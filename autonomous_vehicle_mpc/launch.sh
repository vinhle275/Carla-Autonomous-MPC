#!/usr/bin/env bash
# ==============================================================================
# AUTONOMOUS VEHICLE MPC - LINUX / WSL2 LAUNCHER (PURE C++ RUNTIME)
# ==============================================================================

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Tự động nạp CMake và g++ từ môi trường conda nếu chưa có trong PATH
if [ -d "$HOME/miniconda3/envs/cpp_mpc_env/bin" ]; then
    export PATH="$HOME/miniconda3/envs/cpp_mpc_env/bin:$PATH"
fi

echo "===================================================================="
echo "  KHOI DONG HE THONG MO PHONG XE TU HANH C++ (AUTOSAR AP RUNTIME)   "
echo "===================================================================="

# Tự động phát hiện IP host Windows nếu chạy từ bên trong WSL2
DEFAULT_HOST="127.0.0.1"
if grep -qi microsoft /proc/version 2>/dev/null; then
    # Lấy Default Gateway của WSL2 (chính là IP vEthernet của Windows host)
    WSL_HOST=$(ip route show default 2>/dev/null | awk '{print $3}')
    if [ -z "$WSL_HOST" ] && [ -f "/etc/resolv.conf" ]; then
        WSL_HOST=$(grep nameserver /etc/resolv.conf | awk '{print $2}')
    fi
    if [ -n "$WSL_HOST" ]; then
        DEFAULT_HOST="$WSL_HOST"
    fi
fi

# Tự động cấu hình & biên dịch incremental cho Linux/WSL (chỉ mất ~0.1s nếu không có thay đổi code)
if [ ! -f "build_linux/Makefile" ] && [ ! -f "build_linux/build.ninja" ]; then
    echo "[BUILD] Đang cấu hình CMake cho Linux/WSL (build_linux/)..."
    mkdir -p build_linux
    cmake -B build_linux -DCMAKE_BUILD_TYPE=Release
fi
cmake --build build_linux -j4
TARGET_BIN="build_linux/autonomous_mpc_unified"

echo "[SYSTEM] Chạy C++ Autonomous Runtime: $TARGET_BIN"
"$TARGET_BIN" --host "$DEFAULT_HOST" --with-obstacles "$@"
