#!/usr/bin/env bash
# ==============================================================================
# SCRIPT KHỞI CHẠY CONTAINER AUTOSAR TRÊN RASPBERRY PI 4
# Chạy trực tiếp trên Raspberry Pi OS 64-bit / Ubuntu Server ARM64
# ==============================================================================

set -e

IMAGE_NAME="av-mpc-rpi4:latest"

echo "===================================================================="
echo "    KHỞI CHẠY AUTONOMOUS VEHICLE MPC TRÊN RASPBERRY PI 4 (ARM64)    "
echo "===================================================================="

# Cấu hình thiết bị phần cứng (cáp USB nối vi điều khiển STM32/Arduino, GPS/IMU)
DEVICE_FLAGS=()
if [ -e "/dev/ttyUSB0" ]; then
    DEVICE_FLAGS+=("--device=/dev/ttyUSB0")
    echo "[HARDWARE] Đã phát hiện cổng kết nối xe: /dev/ttyUSB0"
fi
if [ -e "/dev/ttyACM0" ]; then
    DEVICE_FLAGS+=("--device=/dev/ttyACM0")
    echo "[HARDWARE] Đã phát hiện cổng kết nối xe: /dev/ttyACM0"
fi

# Chạy container đa tiến trình với POSIX Shared Memory và Host Networking
docker run --rm -it \
    --net=host \
    --ipc=host \
    "${DEVICE_FLAGS[@]}" \
    "$IMAGE_NAME" \
    "$@"
