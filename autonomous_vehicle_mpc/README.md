# Autonomous Vehicle MPC — AUTOSAR Adaptive Platform

> Hệ thống lái xe tự hành theo quỹ đạo với bộ điều khiển **MPC (Model Predictive Control)** chạy trên kiến trúc đa tiến trình AUTOSAR Adaptive, tích hợp mô phỏng **CARLA 0.9.15**.

---

## Mục Lục

- [Kiến Trúc Hệ Thống](#kiến-trúc-hệ-thống)
- [Yêu Cầu Hệ Thống](#yêu-cầu-hệ-thống)
- [Bước 0: Khởi Động CARLA (Windows)](#bước-0-khởi-động-carla-windows)
- [Hướng 1: Chạy Trực Tiếp (launch_multiprocess.sh)](#hướng-1-chạy-trực-tiếp-launch_multiprocesssh)
- [Hướng 2: Chạy Qua Docker (launch_docker.sh)](#hướng-2-chạy-qua-docker-launch_dockersh)
- [Các Tùy Chọn Dòng Lệnh](#các-tùy-chọn-dòng-lệnh)
- [Hiểu Kết Quả Hiển Thị](#hiểu-kết-quả-hiển-thị)
- [Troubleshooting](#troubleshooting)

---

## Kiến Trúc Hệ Thống

```
┌─────────────────────────────── WSL2 / Docker Container ───────────────────────────────┐
│                                                                                        │
│  ┌─────────────────────────────────────┐                                              │
│  │  CARLA Bridge Server (Python)       │◄──── TCP :2000 ──── CARLA (Windows Host)    │
│  │  carla_bridge_server.py             │                                              │
│  │  · Spawn Ego Vehicle + Obstacles    │──── TCP :20050 ──►┐                         │
│  │  · Camera Stream (160×100 RGB)      │                    │                         │
│  │  · Cockpit Viewer / HUD             │                    │                         │
│  └─────────────────────────────────────┘                    │                         │
│                                                             ▼                         │
│  ┌────────────────────────────────┐    POSIX SHM   ┌───────────────────────────┐     │
│  │  P2: autonomous_core  (50Hz)   │◄──/dev/shm/────►│  P1: paio_vehicle_        │     │
│  │  · QP-MPC (OSQP Solver)        │   av_ara_*      │      interface  (100Hz)   │     │
│  │  · Obstacle Detection          │                 │  · HAL Abstraction Layer  │     │
│  │  · Yellow Line Detection       │                 │  · Watchdog ASIL-D        │     │
│  │  · ASIL-B/D                    │                 │  · Send cmd → Bridge      │     │
│  └────────────────────────────────┘                 └───────────────────────────┘     │
│            │                                                                           │
│            │ POSIX SHM (telemetry)                                                    │
│            ▼                                                                           │
│  ┌────────────────────────────────┐                                                   │
│  │  P3: hmi_diagnostics  (20Hz)   │                                                   │
│  │  · Console HUD Table           │                                                   │
│  │  · CSV Telemetry Log           │                                                   │
│  │  · QM (Quality Management)     │                                                   │
│  └────────────────────────────────┘                                                   │
│                                                                                        │
└────────────────────────────────────────────────────────────────────────────────────────┘
```

| Process | Binary | Tần Số | ASIL | Vai Trò |
|---------|--------|--------|------|---------|
| P1 | `paio_vehicle_interface` | 100 Hz | ASIL-D | HAL, Watchdog, giao tiếp Bridge |
| P2 | `autonomous_core` | 50 Hz | ASIL-B/D | MPC solver, né vật cản, nhận diện vạch |
| P3 | `hmi_diagnostics` | 20 Hz | QM | HUD console, ghi CSV telemetry |

**IPC:** POSIX Shared Memory (`/dev/shm/av_ara_*`) — không dùng socket giữa các process C++.

---

## Yêu Cầu Hệ Thống

### Chung (cả 2 hướng)

| Thành phần | Yêu cầu |
|-----------|---------|
| OS | Windows 10/11 với WSL2 (Ubuntu 22.04) |
| GPU | NVIDIA GPU có ít nhất 4 GB VRAM (RTX 3050 Laptop đủ) |
| CARLA | 0.9.15 tại `D:\Appfile\Carla\CARLA_0.9.15\WindowsNoEditor\` |
| RAM | Tối thiểu 16 GB |
| Disk | ~8 GB (CARLA ~6 GB + Docker image ~2 GB) |

### Hướng 1 — Chạy Trực Tiếp (WSL2)

```
WSL2 Ubuntu 22.04:
  - g++ 11+, cmake 3.20+
  - Python 3.10 với môi trường conda/venv có: carla==0.9.15, numpy, pygame
  - iproute2 (ss command)
```

Cài đặt Python dependencies (nếu chưa có):

```bash
# Tạo venv (nếu chưa có)
cd /mnt/d/Main/Final_Boss/Sim/Carla-Autonomous-MPC/autonomous_vehicle_mpc
python3 -m venv venv
source venv/bin/activate
pip install carla==0.9.15 numpy pygame
```

### Hướng 2 — Chạy Qua Docker

```
Windows:
  - Docker Desktop (đang chạy, WSL2 integration được bật)
WSL2:
  - Lệnh docker hoạt động từ terminal WSL2
```

---

## Bước 0: Khởi Động CARLA (Windows)

> **QUAN TRỌNG:** CARLA phải đang chạy trên Windows TRƯỚC KHI chạy bất kỳ lệnh nào dưới đây.

Mở **PowerShell** hoặc **CMD** trên Windows, `cd` vào thư mục CARLA rồi chạy:

```powershell
# Chế độ ẩn (nhẹ, không cửa sổ CARLA — khuyến nghị cho GPU yếu)
.\CarlaUE4.exe -RenderOffScreen -quality-level=Low -nosound -fps=20

# Chế độ có cửa sổ (nếu muốn thấy CARLA render)
.\CarlaUE4.exe -quality-level=Low -windowed -ResX=640 -ResY=480 -nosound -fps=20
```

> **Lưu ý GPU:** Tránh dùng `-quality-level=Epic` hoặc resolution cao trên GPU dưới 6 GB VRAM — CARLA sẽ crash.

Sau khi CARLA khởi động xong (thấy dòng log `...listening on port 2000`), chuyển sang WSL2 để chạy hướng bên dưới.

---

## Hướng 1: Chạy Trực Tiếp (`launch_multiprocess.sh`)

Chạy toàn bộ hệ thống **trực tiếp trong WSL2** — không cần Docker.

### Các Bước

```bash
# 1. Mở WSL2 terminal, cd vào thư mục dự án
cd /mnt/d/Main/Final_Boss/Sim/Carla-Autonomous-MPC/autonomous_vehicle_mpc

# 2. Kích hoạt Python environment (1 trong 2)
source venv/bin/activate
# hoặc: conda activate carla_env

# 3. Chạy
./launch_multiprocess.sh
```

Script sẽ tự động:
1. **Phát hiện IP Windows host** (IP CARLA) qua `ip route show default`
2. **Build incremental** nếu cần (chỉ build lại file đã thay đổi)
3. Khởi chạy Bridge → chờ port 20050 sẵn sàng → khởi P1 → P2 → P3
4. **Tự tắt toàn bộ** khi xe về đích

### Cửa Sổ Hiển Thị

Khi chạy với `--viewer` (mặc định), script sẽ mở các cửa sổ:
- **Cockpit Camera** — góc nhìn từ mũi xe (160×100 RGB)
- **HUD Overlay** — tốc độ, lỗi lateral, steering, trạng thái chế độ

### Dừng Thủ Công

Nhấn `Ctrl+C` — script sẽ tự kill toàn bộ 4 tiến trình và dọn `/dev/shm/av_ara_*`.

---

## Hướng 2: Chạy Qua Docker (`launch_docker.sh`)

Chạy toàn bộ hệ thống **bên trong Docker container** — môi trường cô lập, không cần cài Python/dependencies trên WSL2.

### Bước 1: Build Image (chỉ cần làm 1 lần)

```bash
cd /mnt/d/Main/Final_Boss/Sim/Carla-Autonomous-MPC/autonomous_vehicle_mpc

# Build Docker image (mất ~5-10 phút lần đầu)
docker build -t av-mpc-runtime:latest .
```

> Build sẽ compile toàn bộ C++ code bên trong image. Chỉ cần build lại khi thay đổi source code.

### Bước 2: Chạy

```bash
./launch_docker.sh
```

Script sẽ:
1. Nhận diện `$DISPLAY` và mount `/tmp/.X11-unix` để hỗ trợ cửa sổ GUI (WSLg)
2. Chạy container với `--net=host --ipc=host` (dùng chung network và shared memory với host)
3. Bên trong container, `docker_entrypoint.sh` tự phát hiện IP CARLA và khởi chạy 4 tiến trình

---

## Các Tùy Chọn Dòng Lệnh

Cả `./launch_multiprocess.sh` và `./launch_docker.sh` đều nhận cùng các tùy chọn:

| Tùy chọn | Mô tả | Mặc định |
|----------|-------|---------|
| `--viewer` | Hiện cửa sổ Cockpit Camera + HUD | ✅ Bật |
| `--no-viewer` | Tắt cửa sổ (nhẹ hơn, chạy server/CI) | — |
| `--host <IP>` | Chỉ định thủ công IP CARLA host | Tự động phát hiện |

### Ví Dụ

```bash
# Chạy không có cửa sổ (headless)
./launch_multiprocess.sh --no-viewer

# Chỉ định IP CARLA thủ công (khi auto-detect sai)
./launch_multiprocess.sh --host 192.168.1.100

# Docker không có viewer
./launch_docker.sh --no-viewer

# Docker với IP cụ thể
./launch_docker.sh --host 172.20.240.1
```

### Chạy Trên Máy Khác (IP Khác Nhau)

Script tự động phát hiện IP CARLA qua `ip route show default`. Nếu IP không đúng:

```bash
# Kiểm tra IP Windows host từ WSL2
ip route show default | awk '{print $3}'

# Chạy với IP tìm được
./launch_multiprocess.sh --host <IP_tìm_được>
```

---

## Hiểu Kết Quả Hiển Thị

```
Step    Speed(km/h) Ref(km/h)   LatErr(m)   Steer     Throttle  Brake     Jerk(m/s3)  QP(ms)    Status
30      18.5        36.0        0.12        -0.03     0.21      0.00      1.20        0.4       CHẾ ĐỘ 1 (BÁM ĐƯỜNG NÉ VẬT CẢN)
```

| Cột | Ý nghĩa |
|-----|---------|
| `Step` | Bước điều khiển (mỗi 0.1s = 5 step @ 50Hz) |
| `Speed` | Tốc độ hiện tại của xe (km/h) |
| `Ref` | Tốc độ tham chiếu từ quỹ đạo (km/h) |
| `LatErr` | Lỗi lateral — khoảng cách xe lệch khỏi quỹ đạo (m) |
| `Steer` | Góc lái (âm = trái, dương = phải) |
| `Throttle` | Mức ga [0.0 – 1.0] |
| `Brake` | Mức phanh [0.0 – 1.0] |
| `Jerk` | Giật gia tốc (m/s³) — càng thấp càng êm |
| `QP(ms)` | Thời gian giải QP-MPC (< 1ms là tốt) |
| `Status` | Chế độ hiện tại |

**Các chế độ (Status):**

| Status | Mô tả |
|--------|-------|
| `CHẾ ĐỘ 1 (BÁM ĐƯỜNG NÉ VẬT CẢN)` | Chạy bình thường theo quỹ đạo |
| `CHẾ ĐỘ 2 (VƯỢT VẬT CẢN)` | Đang né/vượt vật cản trên đường |
| `[VỀ ĐÍCH AN TOÀN]` | Hoàn thành quỹ đạo — hệ thống sẽ tự đóng |

---

## Troubleshooting

### CARLA crash ngay khi khởi động

```
Nguyên nhân: VRAM không đủ
Giải pháp:   Dùng -RenderOffScreen hoặc -windowed -ResX=640 -ResY=480
             Tránh dùng -quality-level=Epic trên GPU < 6GB
```

### Bridge không kết nối được CARLA

```
Nguyên nhân: IP host sai hoặc CARLA chưa khởi động
Kiểm tra:   ip route show default | awk '{print $3}'
Giải pháp:  ./launch_multiprocess.sh --host <IP_đúng>
            Đảm bảo CARLA đang chạy trên Windows (cổng 2000)
```

### Bridge thoát bất ngờ ngay sau khi khởi động

```
Nguyên nhân: Thiếu thư viện Python carla / numpy / pygame
Kiểm tra:   python3 -c "import carla; import numpy; import pygame"
Giải pháp:  source venv/bin/activate
            pip install carla==0.9.15 numpy pygame
```

### Xe không di chuyển (Speed = 0 mãi)

```
Nguyên nhân: P1 (paio_vehicle_interface) chưa kết nối được Bridge
Kiểm tra:   Log phải có dòng "[BRIDGE] C++20 Autonomous Runtime đã kết nối"
Giải pháp:  Chờ thêm vài giây, nếu vẫn lỗi kiểm tra port 20050
            ss -lnt | grep 20050
```

### Docker: lỗi "image not found"

```
Giải pháp:  Chạy build trước:
            docker build -t av-mpc-runtime:latest .
```

### Docker: không hiện cửa sổ GUI

```
Nguyên nhân: WSLg chưa được kích hoạt hoặc biến DISPLAY không đặt
Kiểm tra:   echo $DISPLAY    # phải có giá trị, vd: :0
Giải pháp:  Đảm bảo đang dùng WSL2 với WSLg (Windows 11 / Windows 10 21H2+)
            Hoặc chạy với --no-viewer để bỏ qua GUI
```

### Shared Memory còn sót từ lần chạy trước

```bash
# Xóa thủ công nếu gặp lỗi SHM
rm -f /dev/shm/av_ara_*
```

---

## Cấu Trúc Thư Mục

```
autonomous_vehicle_mpc/
├── apps/
│   ├── autonomous_core/        # P2: MPC solver chính (C++20)
│   ├── paio_vehicle_interface/ # P1: HAL + Watchdog ASIL-D (C++20)
│   └── hmi_diagnostics/        # P3: HUD + CSV logger (C++20)
├── configs/
│   └── trajectories/           # Quỹ đạo tham chiếu (CSV) + OpenDRIVE cache
├── deploy/
│   └── rpi4/                   # Build ARM64 cho Raspberry Pi 4
├── libs/                       # Thư viện dùng chung (IPC, MPC, HAL, Safety)
├── third_party/                # OSQP solver
├── tools/
│   └── carla_bridge/           # carla_bridge_server.py (Python adapter)
├── Dockerfile                  # Image cho laptop simulation
├── docker_entrypoint.sh        # Execution Manager bên trong Docker
├── launch_multiprocess.sh      # 🚀 Chạy native WSL2
└── launch_docker.sh            # 🐳 Chạy qua Docker
```

---

## Triển Khai Raspberry Pi 4 (Tương Lai)

Thư mục `deploy/rpi4/` chứa các script cross-compile cho ARM64 (không cần Python/CARLA):

```bash
# Cross-compile từ máy x86_64
cd deploy/rpi4
./build_rpi4.sh          # Tạo av_mpc_rpi4.tar (ARM64 image)

# Trên Raspberry Pi 4
./run_rpi4.sh            # Chạy container ARM64
```

> Trên Pi, CARLA Bridge chạy trên laptop/server riêng. 3 process C++ trên Pi kết nối qua network.
