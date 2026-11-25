#!/usr/bin/env bash
# ==============================================================================
# AUTOSAR ADAPTIVE PLATFORM - EMBEDDED EXECUTION MANAGER (EM)
# Chạy bên trong Docker Container trên Raspberry Pi 4 (Multi-Process Runtime)
# ==============================================================================

set -e

echo "===================================================================="
echo "  AUTOSAR AP MULTI-PROCESS RUNTIME (RASPBERRY PI 4 - EMBEDDED ECU)  "
echo "===================================================================="

# Xóa Shared Memory cũ
rm -f /dev/shm/av_ara_* 2>/dev/null || true

PID_P1=""
PID_P2=""
PID_P3=""

cleanup() {
    echo ""
    echo "[AUTOSAR EM] Đang dừng an toàn toàn bộ các tiến trình..."
    [ -n "$PID_P3" ] && kill -9 "$PID_P3" 2>/dev/null || true
    [ -n "$PID_P2" ] && kill -9 "$PID_P2" 2>/dev/null || true
    [ -n "$PID_P1" ] && kill -9 "$PID_P1" 2>/dev/null || true
    rm -f /dev/shm/av_ara_* 2>/dev/null || true
    echo "[AUTOSAR EM] ECU đã dừng hoàn tất và chuyển sang trạng thái Safe-State."
}
trap cleanup EXIT INT TERM

# Tham số nhận diện IP Bridge (nếu chạy HIL test mạng với PC) hoặc cổng phần cứng
BRIDGE_IP="${1:-127.0.0.1}"
BRIDGE_PORT="${2:-20050}"
TRAJ_PATH="${3:-configs/trajectories/reference_trajectory.csv}"

# 1. Khởi chạy Process 1: PAIO Vehicle Interface (100Hz - ASIL-D)
echo "[EM] Khởi chạy Process 1: PAIO Vehicle Interface (100Hz)..."
/app/paio_vehicle_interface "$BRIDGE_IP" "$BRIDGE_PORT" &
PID_P1=$!
sleep 0.5

# 2. Khởi chạy Process 2: Autonomous Core MPC (50Hz - ASIL-B/D)
echo "[EM] Khởi chạy Process 2: Autonomous Core MPC (50Hz)..."
/app/autonomous_core "$TRAJ_PATH" &
PID_P2=$!
sleep 0.5

# 3. Khởi chạy Process 3: HMI Diagnostics (20Hz - QM)
echo "[EM] Khởi chạy Process 3: HMI Diagnostics (20Hz)..."
/app/hmi_diagnostics simulation_telemetry.csv &
PID_P3=$!

# Giám sát: Khi Process 2 kết thúc lộ trình, thoát container ngay lập tức
while kill -0 "$PID_P2" 2>/dev/null; do
    sleep 0.1
done

exit 0
