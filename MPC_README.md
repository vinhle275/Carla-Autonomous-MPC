# HƯỚNG DẪN ĐIỀU KHIỂN CARLA MPC & NÉ VẬT CẢN

## 1. CƠ CHẾ PHÁT HIỆN & THEO DÕI VẬT CẢN

Mã nguồn: [src/obstacle_detector.py](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/obstacle_detector.py)

### 1.1 Cảm biến kết hợp 2 vùng (Dual-Zone Perception)
- **Vùng nhìn trước tầm xa (Front Far FOV)**: Góc quét $120^\circ$ ($[-60^\circ, +60^\circ]$), tầm xa $25.0\text{ m}$. Phát hiện vật cản từ xa trên tim đường để giảm tốc và chuẩn bị mở góc lái sớm.
- **Vùng sườn xe bao quanh (Surround Close FOV)**: Góc quét $360^\circ$, bán kính $10.0\text{ m}$. Giám sát vật cản ngang sườn xe khi đang lướt song song (tránh mất góc nhìn khi mũi xe đánh lái chệch hướng).

### 1.2 Bản đồ theo dõi có bộ nhớ (Object Tracking Map)
- **Vấn đề**: Khi xe bẻ lái né, camera trước quay đi hướng khác khiến vật cản lọt ra ngoài FOV. Nếu không có bộ nhớ, xe sẽ lầm tưởng hết vật cản và bẻ lái ngược lại đâm vào vật thể.
- **Xử lý**: Lưu vật cản vào từ điển bộ nhớ `tracked_obstacles` theo ID duy nhất.
- **Điều kiện xóa (Drop condition)**: Chỉ giải phóng vật cản khỏi bộ nhớ khi vật thể đã trôi hẳn về phía sau đuôi xe quá $5.0\text{ m}$ ($d_{\text{long}} < -5.0\text{m}$).

### 1.3 Tọa độ & Bán kính an toàn
- **Hệ tọa độ thân xe (Ego-frame)**:
  - $d_{\text{long}}$: Khoảng cách dọc theo hướng mũi xe ($d_{\text{long}} > 0$: ở phía trước).
  - $d_{\text{lat}}$: Độ lệch ngang ($d_{\text{lat}} < 0$: lệch phải; $d_{\text{lat}} > 0$: lệch trái).
- **Bán kính an toàn**: $r_{\text{safe}} = r_{\text{bbox}} + r_{\text{margin}} \approx 2.0\text{m} \dots 2.2\text{m}$ (khoảng đệm an toàn dự phòng $0.35\text{m}$).

---

## 2. THUẬT TOÁN XỬ LÝ NÉ TRÁNH CỦA BỘ MPC

Mã nguồn: [src/linearized_mpc_controller.py](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/linearized_mpc_controller.py)

### 2.1 Khóa hướng né (Side Determination & Lock)
Khi vật cản vào vùng nguy hiểm ($0 \le d_{\text{long}} \le 22\text{m}$, $|d_{\text{lat}}| \le 2.6\text{m}$):
- Vật cản lệch phải ($d_{\text{lat}} < -0.15\text{m}$): Né sang **TRÁI**.
- Vật cản lệch trái ($d_{\text{lat}} > +0.15\text{m}$): Né sang **PHẢI**.
- Vật cản chính giữa tim đường: Né sang **TRÁI** (theo cấu hình ưu tiên mặc định).
- Hướng né được **khóa cố định** (`active_obstacle`) suốt chu trình vượt để chống đổi hướng giật cục.

### 2.2 Hàm dời làn mềm mại chữ S (C¹ Cosine Transition Curve)
Quỹ đạo tham chiếu của MPC được biến dạng sang bên theo hàm Cosine chia 4 pha liên tục:
- **Pha 1 (Tiếp cận: $-22\text{m} \le p_k < -2\text{m}$)**: Đánh lái từ từ mở rộng làn né sang $2.2\text{m}$.
- **Pha 2 (Chạy qua: $-2\text{m} \le p_k \le 3.5\text{m}$)**: Giữ ổn định $100\%$ độ dời làn $2.2\text{m}$, xe chạy thẳng song song vật cản (vô lăng thẳng $\approx 0^\circ$).
- **Pha 3 (Nhập làn: $3.5\text{m} < p_k \le 14\text{m}$)**: Đánh lái từ từ đưa xe trở lại tim đường cũ sau khi đuôi xe qua khỏi vật cản.
- **Pha 4 (Hoàn tất: $p_k > 14\text{m}$)**: Trở lại hoàn toàn quỹ đạo danh định ban đầu.

### 2.3 Điều phối tốc độ an toàn
- Bình thường: Vận tốc hành trình $10.0\text{ m/s}$ ($36\text{ km/h}$).
- Tiếp cận vật cản ($\le 18\text{m}$): Giảm tốc êm dịu xuống $3.0\text{ m/s}$ ($11\text{ km/h}$).
- Vượt qua an toàn: Tự động tăng tốc trở lại $10.0\text{ m/s}$.

### 2.4 Ràng buộc tiếp diện nửa không gian & Slack QP
- Mỗi bước dự đoán $k$, bổ sung mặt phẳng tiếp diện bảo vệ: $\mathbf{a}_k^T \mathbf{p}_k \le b_k + \epsilon_k$.
- Biến bù Slack $\epsilon_k \ge 0$ được phạt nặng ($W_{\text{slack}} = 20000.0$) trong hàm mục tiêu, đảm bảo bài toán QP **luôn khả thi 100%** (không bao giờ lỗi infeasible).
- Phạt gia tốc góc lái lớn (`FINE_STEER_DOT_COEF = 600.0`) triệt tiêu hoàn toàn rung lắc vô lăng.

---

## 3. HƯỚNG DẪN CHẠY MÔ PHỎNG

### 3.1 Khởi động CARLA Server
```bash
./CarlaUE4.sh -quality-level=Low -benchmark -fps=20
```

### 3.2 Tạo Quỹ đạo tham chiếu mới (ref_trac)
*(Click chuột trái chọn điểm trên bản đồ 2D, nhấn ENTER để lưu CSV và vẽ 3D lên CARLA)*
```bash
python src/generate_path.py
```

### 3.3 Chạy Bộ điều khiển MPC

**Trường hợp 1: Chạy CÓ VẬT CẢN (Obstacle Avoidance Mode)**
```bash
python src/main.py --with-obstacles
```

**Trường hợp 2: Chạy KHÔNG VẬT CẢN (Pure Path Tracking Mode)**
```bash
python src/main.py --no-obstacles
```
*(hoặc chạy mặc định: `python src/main.py`)*

---

## 4. BẢNG CẤU HÌNH NHANH (src/config.py)

| Tham số | Giá trị | Ý nghĩa |
| :--- | :--- | :--- |
| `ENABLE_OBSTACLES` | `False` | Mặc định không sinh vật cản (bật nhanh bằng CLI `--with-obstacles`) |
| `TARGET_SPEED_MS` | `10.0` | Vận tốc hành trình đường thẳng ($36\text{ km/h}$) |
| `OBSTACLE_CAUTION_SPEED_MS` | `3.0` | Vận tốc an toàn khi lướt qua vật cản ($11\text{ km/h}$) |
| `N` / `dt` | `10` / `0.1s` | Chân trời dự đoán ($1.0\text{s}$) và chu kỳ lấy mẫu ($10\text{ Hz}$) |
| `SENSOR_DETECTION_RANGE_M` | `25.0` | Tầm quét xa phía trước ($120^\circ$) |
| `SENSOR_SURROUND_RANGE_M` | `10.0` | Bán kính cảm biến bao quanh sườn xe ($360^\circ$) |
| `SENSOR_TRACK_DROP_BEHIND_M` | `5.0` | Cự ly sau lưng xe để xóa vật cản khỏi bộ nhớ |
| `OBSTACLE_AVOIDANCE_MARGIN_M` | `0.35` | Khoảng đệm an toàn dự phòng bao quanh vật thể |
| `FINE_STEER_DOT_COEF` | `600.0` | Trọng số phạt gia tốc góc lái (chống rung lắc) |
| `MPC_W_SLACK` | `20000.0` | Trọng số biến bù Slack (đảm bảo QP luôn khả thi) |
