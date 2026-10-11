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
   - Khớp tầm nhìn dự báo theo khoảng cách thực tế $v \cdot dt$ để triệt tiêu hoàn toàn hiện tượng rung lắc/chao đảo.
   - Tối ưu hóa đồng thời sai số bám đường $(x, y)$, sai số hướng góc $\theta$, sai số tốc độ $v$, và độ êm góc lái/tăng tốc.
4. **Tự động Lưu Biểu đồ Kết quả (Automatic Plot Saver)**:
   - Tự động xuất và lưu các biểu đồ đánh giá (2D Path Tracking, Speed Tracking, Control Signals, và Báo cáo tổng hợp 3-in-1) dưới dạng file PNG sắc nét trong thư mục `plots/`.

---

## 📁 Cấu trúc Thư mục Dự án

```text
Carla-Autonomous-MPC/
├── README.md                           # Tài liệu hướng dẫn chi tiết
├── requirements.txt                    # Danh sách thư viện Python cần thiết
├── data/
│   └── reference_trajectory.csv        # Dữ liệu đường đi tham chiếu (x, y, theta, v)
├── plots/                              # [MỚI] Thư mục chứa các ảnh biểu đồ kết quả (.png)
│   ├── path_tracking.png
│   ├── speed_tracking.png
│   ├── control_signals.png
│   └── summary_report.png
└── src/
    ├── __init__.py
    ├── config.py                       # Cấu hình tham số MPC, xe, đường dẫn CSV & plots/
    ├── carla_simulator.py              # Đóng gói kết nối CARLA Simulator & vẽ 3D
    ├── vehicle_model.py                # Mô hình xe Kinematic Bicycle Model
    ├── mpc_controller.py               # Bộ điều khiển tối ưu MPC (CasADi)
    ├── help_functions.py               # Hàm nạp CSV, cập nhật vị trí xe & N-step spatial horizon
    ├── logger.py                       # Ghi log & tự động lưu đồ thị báo cáo
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

Khi chạy xong, toàn bộ biểu đồ sẽ tự động xuất ra màn hình và được lưu ảnh sắc nét vào thư mục **[plots/](file:///d:/Main/Final_Boss/Sim/Carla-Autonomous-MPC/plots)**.