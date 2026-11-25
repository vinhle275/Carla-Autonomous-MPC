#!/usr/bin/env bash
# ==============================================================================
# AUTONOMOUS VEHICLE MPC - AUTOSAR ADAPTIVE MULTI-PROCESS EXECUTION MANAGER
# Khởi chạy độc lập 3 Process ASIL riêng biệt giao tiếp qua POSIX Shared Memory
# ==============================================================================

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Dọn dẹp triệt để các tiến trình cũ còn sót lại từ lần chạy trước
pkill -9 -f "autonomous_core" 2>/dev/null || true
pkill -9 -f "paio_vehicle_interface" 2>/dev/null || true
pkill -9 -f "hmi_diagnostics" 2>/dev/null || true
pkill -9 -f "carla_bridge_server.py" 2>/dev/null || true
rm -f /dev/shm/av_ara_* 2>/dev/null || true

# Tự động phát hiện IP host Windows nếu chạy từ bên trong WSL2
DEFAULT_HOST="127.0.0.1"
if grep -qi microsoft /proc/version 2>/dev/null; then
    WSL_HOST=$(ip route show default 2>/dev/null | awk '{print $3}')
    if [ -z "$WSL_HOST" ] && [ -f "/etc/resolv.conf" ]; then
        WSL_HOST=$(grep nameserver /etc/resolv.conf | awk '{print $2}')
    fi
    if [ -n "$WSL_HOST" ]; then
        DEFAULT_HOST="$WSL_HOST"
    fi
fi

# Nhận diện tham số dòng lệnh
CARLA_HOST="$DEFAULT_HOST"
VIEWER_FLAG="--viewer"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --host)
            CARLA_HOST="$2"
            shift 2
            ;;
        --viewer)
            VIEWER_FLAG="--viewer"
            shift
            ;;
        --no-viewer)
            VIEWER_FLAG="--no-viewer"
            shift
            ;;
        *)
            shift
            ;;
    esac
done

echo "===================================================================="
echo "    AUTOSAR ADAPTIVE PLATFORM RUNTIME (MULTI-PROCESS ARCHITECTURE)  "
echo "===================================================================="
echo "[INFO] CARLA Host: $CARLA_HOST"
echo "[INFO] Viewer mode: $VIEWER_FLAG"

# 1. Biên dịch incremental đảm bảo mã mới nhất
if [ ! -f "build_linux/Makefile" ] && [ ! -f "build_linux/build.ninja" ]; then
    mkdir -p build_linux
    cmake -B build_linux -DCMAKE_BUILD_TYPE=Release
fi
cmake --build build_linux -j4 --target paio_vehicle_interface autonomous_core hmi_diagnostics

# Biến lưu trữ PID của các tiến trình
PID_BRIDGE=""
PID_P1=""
PID_P2=""
PID_P3=""

cleanup() {
    echo ""
    echo "[AUTOSAR EM] Đang dừng tất cả các tiến trình (Fail-Safe Shutdown)..."
    [ -n "$PID_P3" ] && kill -9 "$PID_P3" 2>/dev/null || true
    [ -n "$PID_P2" ] && kill -9 "$PID_P2" 2>/dev/null || true
    [ -n "$PID_P1" ] && kill -9 "$PID_P1" 2>/dev/null || true
    [ -n "$PID_BRIDGE" ] && kill -9 "$PID_BRIDGE" 2>/dev/null || true
    pkill -9 -f "autonomous_core" 2>/dev/null || true
    pkill -9 -f "paio_vehicle_interface" 2>/dev/null || true
    pkill -9 -f "hmi_diagnostics" 2>/dev/null || true
    pkill -9 -f "carla_bridge_server.py" 2>/dev/null || true
    rm -f /dev/shm/av_ara_* 2>/dev/null || true
    echo "[AUTOSAR EM] Toàn bộ hệ thống AUTOSAR đã dừng hoàn tất và an toàn."
}
trap cleanup EXIT INT TERM

# 2. Khởi chạy Cầu nối CARLA Bridge (Python Adapter)
echo "[START] Khởi chạy CARLA Bridge Server..."
python3 tools/carla_bridge/carla_bridge_server.py --host "$CARLA_HOST" --with-obstacles $VIEWER_FLAG &
PID_BRIDGE=$!

# 3. Chờ Bridge mở cổng 20050 (Dùng ss kiểm tra LISTEN, TUYỆT ĐỐI KHÔNG connect thử làm hỏng socket)
echo -n "[AUTOSAR EM] Đang chờ CARLA Bridge khởi tạo"
BRIDGE_READY=0
for i in {1..30}; do
    sleep 1
    echo -n "."
    if ss -lnt 2>/dev/null | grep -q ':20050' || netstat -lnt 2>/dev/null | grep -q ':20050'; then
        BRIDGE_READY=1
        break
    fi
    # Kiểm tra xem tiến trình bridge còn sống không
    if ! kill -0 "$PID_BRIDGE" 2>/dev/null; then
        echo " [ERROR] CARLA Bridge đã thoát bất ngờ!"
        exit 1
    fi
done
echo " [OK]"
sleep 1

# 4. Khởi chạy Process 1: PAIO Vehicle Interface (100Hz - ASIL-D)
echo "[START] Khởi chạy Process 1: PAIO Vehicle Interface (100Hz - ASIL-D)..."
./build_linux/paio_vehicle_interface 127.0.0.1 20050 &
PID_P1=$!
sleep 1

# 5. Khởi chạy Process 2: Autonomous Core (50Hz - ASIL-B/D)
echo "[START] Khởi chạy Process 2: Autonomous Core MPC (50Hz - ASIL-B/D)..."
./build_linux/autonomous_core configs/trajectories/reference_trajectory.csv &
PID_P2=$!
sleep 1

# 6. Khởi chạy Process 3: HMI Diagnostics (20Hz - QM) trên màn hình chính
echo "[START] Khởi chạy Process 3: HMI Diagnostics & Telemetry HUD (20Hz)..."
echo "===================================================================="
./build_linux/hmi_diagnostics simulation_telemetry.csv &
PID_P3=$!

# Giám sát: Đóng ngay lập tức khi Process 2 (về đích) xong HOẶC người dùng đóng cửa sổ Viewer
while kill -0 "$PID_P2" 2>/dev/null && kill -0 "$PID_BRIDGE" 2>/dev/null; do
    sleep 0.1
done
exit 0
