#!/usr/bin/env bash
# ==============================================================================
# SCRIPT CHẠY DOCKER TIỆN LỢI CHO AUTONOMOUS VEHICLE MPC
# Tự động nhận diện IP, hỗ trợ linh hoạt có/không có --no-viewer và mở GUI
# ==============================================================================

set -e

IMAGE_NAME="av-mpc-runtime:latest"

# Cấu hình hiển thị GUI (X11 / WSLg) nếu người dùng muốn xem cửa sổ Cockpit Viewer
DOCKER_GUI_FLAGS=()
if [ -n "$DISPLAY" ]; then
    DOCKER_GUI_FLAGS+=("-e" "DISPLAY=$DISPLAY")
fi
if [ -d "/tmp/.X11-unix" ]; then
    DOCKER_GUI_FLAGS+=("-v" "/tmp/.X11-unix:/tmp/.X11-unix:ro")
fi

# Chạy container với host networking và POSIX SHM
docker run --rm -it \
    --net=host \
    --ipc=host \
    "${DOCKER_GUI_FLAGS[@]}" \
    "$IMAGE_NAME" \
    "$@"
