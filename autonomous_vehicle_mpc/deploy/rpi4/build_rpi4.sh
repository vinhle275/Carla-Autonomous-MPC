#!/usr/bin/env bash
# ==============================================================================
# SCRIPT BIÊN DỊCH CHÉO (CROSS-COMPILE) DOCKER CHO RASPBERRY PI 4 (ARM64)
# Chạy trực tiếp từ WSL2 trên Laptop x86 của bạn
# ==============================================================================

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT_DIR"

IMAGE_NAME="av-mpc-rpi4:latest"
OUTPUT_TAR="deploy/rpi4/av_mpc_rpi4.tar"

echo "===================================================================="
echo "    BẮT ĐẦU BUILD DOCKER ARM64 CHO RASPBERRY PI 4 (AUTOSAR AP)      "
echo "===================================================================="

# Đảm bảo docker buildx đã sẵn sàng
docker buildx create --name rpi_builder --use 2>/dev/null || docker buildx use rpi_builder

# Build image cho kiến trúc ARM64 của Raspberry Pi 4
echo "[BUILDX] Đang biên dịch C++20 trên nền tảng linux/arm64..."
docker buildx build \
    --platform linux/arm64 \
    -f deploy/rpi4/Dockerfile.rpi \
    -t "$IMAGE_NAME" \
    --load \
    .

echo "[SUCCESS] Đã build thành công Docker image: $IMAGE_NAME (linux/arm64)!"

# Xuất image ra file tar để copy sang thẻ nhớ Pi
echo "[EXPORT] Đang xuất image ra file $OUTPUT_TAR..."
docker save "$IMAGE_NAME" -o "$OUTPUT_TAR"

echo "===================================================================="
echo "  HOÀN TẤT! FILE NẠP: $(pwd)/$OUTPUT_TAR"
echo "  Dung lượng: $(du -h "$OUTPUT_TAR" | awk '{print $1}')"
echo "===================================================================="
echo "👉 BƯỚC TIẾP THEO TRÊN RASPBERRY PI 4:"
echo "1. Copy file $OUTPUT_TAR sang Raspberry Pi (qua scp hoặc USB):"
echo "   scp $OUTPUT_TAR pi@<IP_RASPBERRY_PI>:~/"
echo "2. Trên Raspberry Pi, nạp image vào Docker:"
echo "   docker load -i $OUTPUT_TAR"
echo "3. Chạy container bằng script deploy/rpi4/run_rpi4.sh!"
