# BẢN KẾ HOẠCH THỰC HIỆN: COCKPIT DASHBOARD ĐA MÀN HÌNH (REVISED PLAN)

---

## I. THIẾT KẾ BỐ CỤC GIAO DIỆN (COCKPIT LAYOUT)

### 1. Quy chuẩn kích thước tổng thể
- **Độ phân giải tổng cửa sổ**: `1440 × 900 px` (Tỷ lệ 16:10 chuẩn giao diện điều khiển giám sát).
- **Phân bổ theo chiều dọc**:
  - **Top Bar (Thanh thông số hợp nhất)**: `1440 × 60 px` ($y = 0 \to 60$).
  - **Khu vực Camera (Main View & 2 Sub-Cams)**: `1440 × 600 px` ($y = 60 \to 660$).
  - **Khu vực 2 Đồ thị Realtime (Ngang nhau)**: `1440 × 240 px` ($y = 660 \to 900$).

### 2. Tỷ lệ hình học các khung nhìn Camera
- **Main View (Third-person / Chase Cam)**:
  - Chiều rộng: $W_{main} = 960\text{ px}$.
  - Chiều cao: $H_{main} = 600\text{ px}$.
  - Tỷ lệ khung hình: $16:10$.
- **Camera phụ 1 & 2 (Bird-Eye View & Front View)**:
  - Chiều rộng: $W_{sub} = \frac{1}{2} W_{main} = \mathbf{480\text{ px}}$ (chuẩn 1/2 chiều rộng Main View).
  - Chiều cao mỗi camera: $H_{sub} = \frac{1}{2} H_{main} = \mathbf{300\text{ px}}$ (tỷ lệ $16:10$).
  - Xếp chồng 2 camera phụ ở cột phải: $300\text{ px} + 300\text{ px} = 600\text{ px}$ (khớp hoàn hảo chiều cao Main View).

### 3. Sơ đồ bố cục chi tiết (ASCII Layout)

```
┌────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│ [TOP BAR FRAME - THÔNG SỐ ĐIỀU KHIỂN & CẢM BIẾN HỢP NHẤT]                                              │
│ Tốc độ: 28.5/36.0 km/h | Lái: +3.2° | MPC: NÉ PHẢI (#105) | s=84.2/158.8m | Obs: 2 | Vàng: SOLID (0.8m)│
├─────────────────────────────────────────────────────────┬──────────────────────────────────────────────┤
│                                                         │ [CAMERA PHỤ 1: BIRD-EYE VIEW (TOP-DOWN)]     │
│                                                         │  • Kích thước: 480 × 300 px                  │
│                                                         │  • Góc nhìn: Trên cao bám xe (x=0, z=18m)    │
│                                                         │  • Quan sát: 2 mép làn & vật cản bao quanh   │
│               MAIN VIEW: CHASE CAM                      ├──────────────────────────────────────────────┤
│                                                         │ [CAMERA PHỤ 2: FRONT VIEW (HOOD-CAM)]        │
│  • Kích thước: 960 × 600 px                             │  • Kích thước: 480 × 300 px                  │
│  • Góc nhìn: Sau đuôi xe (x=-6.5m, z=2.8m, pitch=-12°) │  • Góc nhìn: Mui xe trực diện (x=2.2, z=1.2) │
│  • Hiển thị toàn cảnh xe né vật cản & đường đi dự đoán  │  • Cấp ảnh trực tiếp cho Yellow Detector     │
├─────────────────────────────────────────────────────────┴──────────────────────────────────────────────┤
│ 2 ĐỒ THỊ REALTIME ĐẶT NGANG NHAU (CHIỀU CAO 240 px)                                                    │
├────────────────────────────────────────────┬───────────────────────────────────────────────────────────┤
│ [ĐỒ THỊ 1: VẬN TỐC THỰC TẾ vs V_REF]       │ [ĐỒ THỊ 2: ĐỘ GIẬT JERK (da/dt - m/s³)]                   │
│  • Kích thước: 720 × 240 px (hoặc 960x240) │  • Kích thước: 720 × 240 px (hoặc 480x240)                │
│  • v_ego (Xanh Cyan) vs v_ref (Nét đứt Cam)│  • Thang đo: -5.0 m/s³ đến +5.0 m/s³ (Vạch 0 trung tâm)   │
│  • Cửa sổ trượt: 10 giây gần nhất (100 pts)│  • Dải màu êm ái: Xanh (|j|<=2) -> Vàng -> Đỏ (|j|>3.5)   │
└────────────────────────────────────────────┴───────────────────────────────────────────────────────────┘
```

---

## II. CHI TIẾT CÁC THÀNH PHẦN

### 1. Frame [TOP BAR]: Bảng điều khiển & Cảm biến hợp nhất (1440 × 60 px)
- Nền bán trong suốt `(10, 15, 25, 220)` kèm đường viền dưới màu Cyan công nghệ `(0, 180, 220)`.
- **Phân chia 3 cụm thông số trực quan**:
  - **Cụm 1 (Động học xe)**:
    - Vận tốc: `Tốc độ: 28.5 km/h` (Màu xanh nếu bám tốt, vàng nếu lệch $v_{ref}$).
    - Mục tiêu: `Mục tiêu: 30.0 km/h`.
    - Góc bẻ lái: `Góc lái: +3.2°` (có biểu tượng mũi tên đánh lái $\leftarrow$ hoặc $\rightarrow$).
  - **Cụm 2 (Trạng thái điều khiển MPC)**:
    - Nhãn chế độ: `CHẾ ĐỘ 1: BÁM ĐƯỜNG` / `NÉ PHẢI (#105)` / `DỪNG AN TOÀN (ROADBLOCK)`.
    - Tiến độ quỹ đạo: `Tiến độ: s=84.2m / 158.8m (53%)`.
  - **Cụm 3 (Nhận thức & Cảm biến)**:
    - Vật cản: `Vật cản: 2`.
    - Vạch vàng: `SOLID | d=0.85m | 3.2ms` (Vàng tươi nếu vạch liền, vàng nhạt nếu vạch đứt, xám nếu không có).
    - Hướng dẫn phím: `[ESC]: Đóng | [M]: Ẩn HUD`.

---

### 2. Hệ thống Camera (CARLA Multi-Sensor Setup)
- **Sensor 1 (Main - Chase Cam)**:
  - Resolution: `960 × 600`, FOV: `90°`.
  - Transform: `carla.Transform(carla.Location(x=-6.5, z=2.8), carla.Rotation(pitch=-12.0, yaw=0.0))`.
  - Nhiệm vụ: Giám sát toàn diện hành vi bám làn, đánh lái và dời quỹ đạo của xe.
- **Sensor 2 (Sub 1 - Bird-Eye View)**:
  - Resolution: `480 × 300`, FOV: `90°`.
  - Transform: `carla.Transform(carla.Location(x=-1.5, z=18.0), carla.Rotation(pitch=-80.0, yaw=0.0))`.
  - Nhiệm vụ: Cung cấp góc nhìn không gian 2D mặt đường, kiểm tra mép xe với lề đường và vật cản hai bên hông.
- **Sensor 3 (Sub 2 - Front View / Hood-Cam)**:
  - Resolution: `480 × 300`, FOV: `90°`.
  - Transform: `carla.Transform(carla.Location(x=2.2, z=1.2), carla.Rotation(pitch=-5.0, yaw=0.0))`.
  - Nhiệm vụ: Tầm nhìn trước kính lái xe; **chia sẻ trực tiếp buffer ảnh cho `RealtimeYellowLineDetector`** để nhận diện vạch kẻ đường chính xác nhất.

---

### 3. Hai Đồ thị Realtime đặt ngang nhau (1440 × 240 px)
Cả 2 đồ thị được vẽ trực tiếp bằng Pygame Primitives (không dùng Matplotlib, render 60+ FPS, độ trễ 0ms).

#### Đồ thị 1: Vận tốc thực tế ($v$) vs Vận tốc tham chiếu ($v_{ref}$)
- **Kích thước**: `720 × 240 px` (Bên trái hoặc theo tỷ lệ).
- **Trục tung (Y-axis)**: $0 \to 40\text{ km/h}$ với các đường lưới mờ cách nhau $10\text{ km/h}$.
- **Trục hoành (X-axis)**: Cửa sổ trượt 10 giây (100 mẫu gần nhất tại $dt=0.1s$).
- **Đường vẽ**:
  - $v_{ref}$: Đường nét đứt màu Vàng Cam `(255, 170, 0)`.
  - $v_{ego}$: Đường nét liền màu Cyan sáng `(0, 240, 255)` độ dày 2px.
- **Chỉ số số học góc đồ thị**: Giá trị tức thời $v$ và sai số $\Delta v = v - v_{ref}$.

#### Đồ thị 2: Độ giật Jerk ($j = \Delta a / \Delta t$ - $m/s^3$)
- **Kích thước**: `720 × 240 px` (Bên phải).
- **Cách tính toán Jerk**:
  - Gia tốc tức thời: $a_k = \frac{v_k - v_{k-1}}{dt}$ (hoặc lấy từ `ego_vehicle.get_acceleration()`).
  - Jerk: $j_k = \frac{a_k - a_{k-1}}{dt}$ ($m/s^3$).
  - Bộ lọc thông thấp (EMA): $j_{filtered} = \alpha \cdot j_k + (1 - \alpha) \cdot j_{prev}$ ($\alpha = 0.35$).
- **Trục tung (Y-axis)**: Đối xứng từ $-5.0\text{ m/s}^3$ đến $+5.0\text{ m/s}^3$. Vạch tim $0.0$ màu xám nhạt `(120, 120, 120)`.
- **Dải phân vùng màu an toàn**:
  - Dải xanh mờ ($|j| \le 2.0\text{ m/s}^3$): Vùng hành khách cảm thấy êm ái, thoải mái.
  - Vạch cảnh báo vàng ($2.0 < |j| \le 3.5\text{ m/s}^3$): Chuyển làn, tăng tốc hoặc hãm phanh vừa.
  - Vạch đỏ ($|j| > 3.5\text{ m/s}^3$): Phanh gấp hoặc giật cục.
- **Đường vẽ Jerk**: Đường cong đổi màu linh hoạt theo giá trị tuyệt đối của $|j|$.

---

## III. KẾ HOẠCH TRIỂN KHAI THEO TỪNG BƯỚC

### Bước 1: Cập nhật Cấu hình ([src/config.py](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py))
- Thêm các hằng số:
  - `COCKPIT_WINDOW_WIDTH = 1440`
  - `COCKPIT_WINDOW_HEIGHT = 900`
  - `MAIN_VIEW_WIDTH = 960`, `MAIN_VIEW_HEIGHT = 600`
  - `SUB_VIEW_WIDTH = 480`, `SUB_VIEW_HEIGHT = 300`
  - `PLOT_HEIGHT = 240`, `PLOT_WIDTH = 720`
  - `TELEMETRY_HISTORY_LEN = 100`

### Bước 2: Viết lại Module Camera & Dashboard ([src/camera_viewer.py](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/camera_viewer.py))
- Khởi tạo 3 sensor camera: `chase_sensor`, `bev_sensor`, `front_sensor`.
- Lưu giữ 3 Pygame surface độc lập: `surface_main`, `surface_bev`, `surface_front`.
- Xây dựng lớp con `TelemetryPlotter`:
  - Quản lý deques cho `v_history`, `v_ref_history`, `jerk_history`.
  - Hàm `draw_speed_plot(surface, rect)`
  - Hàm `draw_jerk_plot(surface, rect)`
- Hàm vẽ `_draw_top_bar(hud_info)`: Vẽ thanh ngang hợp nhất trên cùng.
- Hàm `render()`: Blit cả 3 màn hình camera, vẽ thanh Top Bar và 2 biểu đồ sóng.

### Bước 3: Tích hợp vào Vòng lặp Điều khiển Chính ([src/main.py](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/main.py))
- Tính toán $a_{ego}$ và $jerk_{ego}$ ở mỗi chu kỳ $dt$.
- Truyền ảnh từ `camera_viewer.get_front_frame()` trực tiếp vào `yellow_detector` thay vì lấy từ chase camera.
- Đóng gói đầy đủ `hud_info` truyền vào `camera_viewer.render(hud_info)`.
- Đảm bảo giải phóng sạch sẽ cả 3 cảm biến trong khối `finally` hoặc khi bấm `ESC`.

### Bước 4: Kiểm thử và tinh chỉnh
- Kiểm tra độ ổn định FPS trên CARLA (đảm bảo $\ge 25$ - 30 FPS).
- Kiểm tra hiển thị 2 đồ thị khi xe tăng tốc, vào cua, lách qua vật cản và dừng khẩn cấp.