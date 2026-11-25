# ==============================================================================
# AUTONOMOUS VEHICLE MPC - DOCKER RUNTIME (AUTOSAR MULTI-PROCESS ECU + SIM)
# ==============================================================================

# STAGE 1: BUILD C++ BINARIES
FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake g++ gcc ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . .

# Biên dịch cả 3 tiến trình AUTOSAR độc lập
RUN mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) paio_vehicle_interface autonomous_core hmi_diagnostics

# ==============================================================================
# STAGE 2: RUNTIME MULTI-PROCESS C++ & PYTHON CARLA BRIDGE
# ==============================================================================
FROM ubuntu:22.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    libstdc++6 ca-certificates python3 python3-pip \
    iproute2 procps \
    libx11-6 libxext6 libxrender1 libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# Cài đặt các thư viện Python cho CARLA Bridge Server
RUN pip3 install --no-cache-dir numpy pygame carla==0.9.15

WORKDIR /app

# Copy cả 3 binary C++ độc lập
COPY --from=builder /workspace/build/paio_vehicle_interface /app/
COPY --from=builder /workspace/build/autonomous_core /app/
COPY --from=builder /workspace/build/hmi_diagnostics /app/

# Copy thư mục dữ liệu và script
COPY --from=builder /workspace/configs /app/configs
COPY --from=builder /workspace/tools /app/tools
COPY docker_entrypoint.sh /app/docker_entrypoint.sh

RUN chmod +x /app/paio_vehicle_interface \
             /app/autonomous_core \
             /app/hmi_diagnostics \
             /app/docker_entrypoint.sh

# Execution Manager điều phối 3 tiến trình C++ và Bridge
ENTRYPOINT ["/app/docker_entrypoint.sh"]
CMD []
