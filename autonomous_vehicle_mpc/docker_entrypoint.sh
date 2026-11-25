#!/usr/bin/env bash
# ==============================================================================
# DOCKER ENTRYPOINT: AUTOSAR ADAPTIVE MULTI-PROCESS EXECUTION MANAGER
# Điều phối 3 tiến trình C++ độc lập + Bridge Server bên trong Docker
# ==============================================================================

set -e

# Xóa Shared Memory cũ
rm -f /dev/shm/av_ara_* 2>/dev/null || true

# Tự động dò tìm IP CARLA host
DEFAULT_HOST=""
if [ -n "$CARLA_HOST" ]; then
    DEFAULT_HOST="$CARLA_HOST"
elif command -v ip >/dev/null 2>&1; then
    DEFAULT_HOST=$(ip route show default 2>/dev/null | awk '{print $3}')
fi
if [ -z "$DEFAULT_HOST" ] && [ -f /etc/resolv.conf ]; then
    DEFAULT_HOST=$(grep nameserver /etc/resolv.conf | awk '{print $2}')
fi
if [ -z "$DEFAULT_HOST" ]; then
    DEFAULT_HOST="127.0.0.1"
fi

CARLA_HOST="$DEFAULT_HOST"
VIEWER_FLAG="--viewer"

# Đọc các tham số dòng lệnh truyền từ docker run
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
echo "  AUTOSAR AP MULTI-PROCESS RUNTIME (INSIDE DOCKER CONTAINER)        "
echo "===================================================================="
echo "[DOCKER EM] CARLA Host: $CARLA_HOST"
echo "[DOCKER EM] Viewer mode: $VIEWER_FLAG"

PID_BRIDGE=""
PID_P1=""
PID_P2=""
PID_P3=""

cleanup() {
    echo ""
    echo "[DOCKER EM] Dừng tất cả tiến trình AUTOSAR trong container..."
    [ -n "$PID_P3" ] && kill -9 "$PID_P3" 2>/dev/null || true
    [ -n "$PID_P2" ] && kill -9 "$PID_P2" 2>/dev/null || true
    [ -n "$PID_P1" ] && kill -9 "$PID_P1" 2>/dev/null || true
    [ -n "$PID_BRIDGE" ] && kill -9 "$PID_BRIDGE" 2>/dev/null || true
    rm -f /dev/shm/av_ara_* 2>/dev/null || true
    echo "[DOCKER EM] Container đã hoàn tất dọn dẹp và thoát an toàn."
}
trap cleanup EXIT INT TERM

# 1. Khởi chạy CARLA Bridge Server (Python Adapter)
echo "[START] Khởi chạy CARLA Bridge Server..."
python3 /app/tools/carla_bridge/carla_bridge_server.py --host "$CARLA_HOST" --with-obstacles $VIEWER_FLAG &
PID_BRIDGE=$!

# 2. Chờ Bridge mở cổng 20050
echo -n "[DOCKER EM] Đang chờ CARLA Bridge khởi tạo"
for i in {1..30}; do
    sleep 1
    echo -n "."
    if ss -lnt 2>/dev/null | grep -q ':20050'; then
        break
    fi
    if ! kill -0 "$PID_BRIDGE" 2>/dev/null; then
        echo " [ERROR] CARLA Bridge đã thoát bất ngờ!"
        exit 1
    fi
done
echo " [OK]"
sleep 1

# 3. Khởi chạy Process 1: PAIO Vehicle Interface (100Hz - ASIL-D)
echo "[START] Khởi chạy Process 1: PAIO Vehicle Interface (100Hz)..."
/app/paio_vehicle_interface 127.0.0.1 20050 &
PID_P1=$!
sleep 1

# 4. Khởi chạy Process 2: Autonomous Core (50Hz - ASIL-B/D)
echo "[START] Khởi chạy Process 2: Autonomous Core MPC (50Hz)..."
/app/autonomous_core /app/configs/trajectories/reference_trajectory.csv &
PID_P2=$!
sleep 1

# 5. Khởi chạy Process 3: HMI Diagnostics (20Hz - QM)
echo "[START] Khởi chạy Process 3: HMI Diagnostics (20Hz)..."
echo "===================================================================="
/app/hmi_diagnostics /app/simulation_telemetry.csv &
PID_P3=$!

# 6. Giám sát: Khi Process 2 về đích HOẶC người dùng đóng cửa sổ Viewer, đóng container ngay
while kill -0 "$PID_P2" 2>/dev/null && kill -0 "$PID_BRIDGE" 2>/dev/null; do
    sleep 0.1
done

exit 0
