# Hệ Thống Điều Khiển Xe Tự Hành (Autonomous Vehicle MPC)
### Kiến trúc: Modern C++20 & AUTOSAR Adaptive Platform (PAIO)

Dự án này là phiên bản chuyển đổi toàn diện từ Python sang **C++20 chuẩn công nghiệp**, loại bỏ toàn bộ các thành phần dư thừa và đáp ứng các tiêu chuẩn thời gian thực khắt khe (**ASIL-B/D**). Hệ thống chạy hoàn toàn bằng ứng dụng C++ thực thi native mà không cần người dùng phải gõ bất kỳ lệnh Python nào.

---

## 1. Cấu Trúc Thư Mục Dự Án

```text
autonomous_vehicle_mpc/
├── CMakeLists.txt                      # Root CMake (C++20, g++, MinGW/Linux)
├── .gitignore                          # Cấu hình bỏ qua build/, file nhị phân và cache
├── build.bat                           # Script tự động cấu hình & biên dịch C++ 1-click
├── launch.bat                          # Khởi động ứng dụng C++ 1-click trên Windows
├── launch.sh                           # Khởi động ứng dụng C++ 1-click trên Linux / WSL2
├── configs/                            # Tham số cấu hình hệ thống
│   ├── vehicle_params.yaml             # Thông số xe, kích thước, cảm biến, Danger Zone
│   ├── mpc_weights.yaml                # Trọng số Q, R, phạt vạch vàng, S-Curve
│   └── trajectories/                   # reference_trajectory.csv, obstacles.csv, moving_obstacles.csv
│
├── libs/                               # CÁC THƯ VIỆN CỐT LÕI (C++20)
│   ├── common/                         # autosar_types.hpp, frenet_utils.hpp, trajectory_loader.hpp
│   ├── ipc_transport/                  # ara::com Zero-Copy Shared Memory (Lock-free triple buffer)
│   ├── mpc_core/                       # Successive Linearization QP-MPC (OSQP/ADMM < 3ms), Anti-Jerk
│   ├── safety_guard/                   # SAT Danger Zone (2.4m x 6m), State Machine Mode 1/2, Actuator Mapping
│   ├── perception_core/                # Dynamic Road Corridor, Dual-Zone OBB Tracking, Yellow Line Shield
│   └── hal/                            # Hardware Abstraction Layer, CARLA TCP Client Bridge
│
├── apps/                               # CÁC TIẾN TRÌNH AUTOSAR ADAPTIVE RUNTIME
│   ├── paio_vehicle_interface/         # PROCESS 1: ASIL-D 100Hz (CAN/CARLA IO, 100ms Watchdog)
│   ├── autonomous_core/                # PROCESS 2: ASIL-B/D 50Hz (Perception, Frenet, QP-MPC, Safety)
│   ├── hmi_diagnostics/                # PROCESS 3: QM 20Hz (Cockpit HUD Terminal, SSD Blackbox Logger)
│   └── unified_app/                    # UNIFIED RUNNER: Chạy toàn bộ hệ thống bằng 1 binary C++ duy nhất
│
└── tools/
    ├── carla_bridge/                   # CARLA Adapter Daemon ngầm (C++ tự động quản lý vòng đời)
    └── scenario_builder/               # PROCESS 0: Công cụ tiền kỳ nạp bản đồ, vẽ quỹ đạo và đặt vật cản
```

---

## 2. Hướng Dẫn Biên Dịch Chuẩn Xác (Sau Khi Xóa Hoặc Gitignore `build/`)

### Cách 1: Sử dụng file `build.bat` (Khuyên dùng - 1 click)
Mỗi khi bạn clone repo mới hoặc sau khi xóa thư mục `build/`:
```cmd
cd D:\Main\Final_Boss\Sim\autonomous_vehicle_mpc
build.bat
```
Script sẽ tự động:
1. Nạp đúng đường dẫn MinGW UCRT64 và CMake vào PATH.
2. Tự tạo thư mục `build/` và cấu hình CMake bằng MinGW Makefiles.
3. Tự động biên dịch song song 4 luồng (`-j4`) ra các file `.exe`.

### Cách 2: Biên dịch thủ công bằng lệnh CMake
```powershell
$env:PATH = "D:\Appfile\Code\msys64\ucrt64\bin;C:\Program Files\CMake\bin;" + $env:PATH
cd D:\Main\Final_Boss\Sim\autonomous_vehicle_mpc
cmake -B build -G "MinGW Makefiles" -DCMAKE_CXX_COMPILER="D:/Appfile/Code/msys64/ucrt64/bin/g++.exe" -DCMAKE_MAKE_PROGRAM="D:/Appfile/Code/msys64/ucrt64/bin/mingw32-make.exe" -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

---

## 3. Cách Khởi Động & Mô Phỏng

### Bước 1: Khởi động CARLA Simulator (nếu chưa chạy)
Trên PowerShell hoặc CMD:
```powershell
cd D:\Appfile\Carla\CARLA_0.9.15\WindowsNoEditor
.\CarlaUE4.exe -carla-server -dx11 -quality-level=Low -windowed -ResX=800 -ResY=600
```

### Bước 2: Khởi động hệ thống tự hành C++
Mở cửa sổ Terminal thứ hai:
```cmd
cd D:\Main\Final_Boss\Sim\autonomous_vehicle_mpc
launch.bat
```
*(Hoặc chạy trực tiếp: `.\build\autonomous_mpc_unified.exe --with-obstacles`)*

> **Lưu ý:** `launch.bat` đã được tích hợp sẵn tính năng tự động gọi `build.bat` nếu thư mục `build/` chưa có hoặc file thực thi chưa được biên dịch. Bạn không cần lo lắng về việc thư mục `build/` bị xóa hay nằm trong `.gitignore`.
