# Carla-Autonomous-MPC

Dự án này là hệ thống độc lập hoàn chỉnh kết hợp **Tạo Quỹ đạo Tham chiếu (Trajectory Generation)** và **Bộ điều khiển Dự báo Mô hình (Model Predictive Control - MPC)** cho xe tự lái trong CARLA Simulator.

---

## 🌟 Tính năng hệ thống

1. **Độc lập hoàn toàn (Self-contained)**: Toàn bộ công cụ vẽ/tạo đường đi, bộ điều khiển MPC và lưu trữ dữ liệu đều nằm trọn vẹn trong thư mục này, sẵn sàng để đẩy lên GitHub.
2. **Công cụ Tạo đường đi 2D (Interactive Path Generator)**:
   - Giao diện Matplotlib 2D cho phép click chuột chọn điểm neo trực tiếp trên bản đồ CARLA.
   - Thuật toán nội suy **PCHIP Spline** làm mịn đường và tự động điều chỉnh tốc độ $v_{ref}$ khi vào cua gấp.
3. **Bộ điều khiển MPC Phi tuyến (CasADi + IPOPT)**:
   - Mô hình xe đạp động học (Kinematic Bicycle Model).
   - Tầm nhìn dự báo $N = 10$ bước thời gian thực (10Hz).
   - Tối ưu hóa đồng thời sai số bám đường $(x, y)$, sai số hướng góc $\theta$, sai số tốc độ $v$, và độ êm góc lái/tăng tốc.
4. **Hiển thị 3D & Đồ thị Đánh giá**:
   - Vẽ đường tham chiếu 3D và quỹ đạo dự báo $N$ bước màu đỏ trực tiếp trong CARLA.
   - Tự động xuất đồ thị đánh giá hiệu năng bám đường (Path tracking error), tốc độ và tín hiệu điều khiển sau khi hoàn tất.

---

## 📁 Cấu trúc Thư mục Dự án

```text
Carla-Autonomous-MPC/
├── README.md                           # Tài liệu hướng dẫn chi tiết
├── requirements.txt                    # Danh sách thư viện Python cần thiết
├── data/
│   └── reference_trajectory.csv        # Dữ liệu đường đi tham chiếu (x, y, theta, v)
└── src/
    ├── __init__.py
    ├── config.py                       # Cấu hình tham số MPC, xe và đường dẫn CSV
    ├── carla_simulator.py              # Đóng gói kết nối CARLA Simulator & vẽ 3D
    ├── vehicle_model.py                # Mô hình xe Kinematic Bicycle Model
    ├── mpc_controller.py               # Bộ điều khiển tối ưu MPC (CasADi)
    ├── help_functions.py               # Hàm nạp CSV, cập nhật vị trí xe & N-step horizon
    ├── logger.py                       # Ghi log & vẽ biểu đồ đánh giá
    ├── generate_path.py                # Script độc lập: Tạo đường đi mới từ bản đồ 2D
    ├── main.py                         # Script chính: Khởi chạy xe MPC bám đường
    └── trajectory_generator/           # Mô-đun phụ trách sinh đường đi PCHIP
        ├── __init__.py
        ├── waypoint_reader.py          # GUI 2D Matplotlib chọn điểm neo
        └── spline_gen.py               # Nội suy PCHIP & hồ sơ tốc độ theo độ cong
```

---

## 🚀 Hướng dẫn Sử dụng

### 1. Tạo Đường đi Tham chiếu mới (Optional)
Bật CARLA Simulator Server (`./CarlaUE4.exe`), sau đó chạy lệnh:
```bash
cd Carla-Autonomous-MPC
python src/generate_path.py
```
- **Click chuột trái**: Chấm chọn các điểm neo (Anchor points).
- **Click chuột phải**: Xóa điểm vừa chọn (Undo).
- **Nhấn ENTER**: Lưu đường đi vào `data/reference_trajectory.csv` và vẽ lên CARLA 3D.

---

### 2. Khởi chạy Mô phỏng Xe MPC Bám đường
Chạy lệnh thực thi bộ điều khiển MPC:
```bash
cd Carla-Autonomous-MPC
python src/main.py
```

---

## ⚙️ Thông số Cấu hình Chính (`src/config.py`)

- `N = 10`: Tầm nhìn dự báo của bộ điều khiển MPC.
- `dt = 0.1`: Chu kỳ điều khiển thời gian thực ($10\text{Hz}$).
- `TRAJECTORY_CSV_PATH`: Đường dẫn tới file lưu trữ dữ liệu quỹ đạo (`data/reference_trajectory.csv`).