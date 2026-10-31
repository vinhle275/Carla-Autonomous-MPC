# BÁO CÁO CƠ CHẾ NÉ VẬT CẢN, QUYẾT ĐỊNH ĐÁNH LÁI TRÁI/PHẢI VÀ DỪNG LẠI (ROADBLOCK)

Tài liệu này tổng hợp chi tiết toàn bộ kiến trúc và thuật toán điều khiển né vật cản đa kịch bản đang được triển khai trực tiếp trong:
- Mã nguồn bộ điều khiển: [src/linearized_mpc_controller.py](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/linearized_mpc_controller.py)
- Cấu hình thông số hệ thống: [src/config.py](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py)

---

## 1. TỔNG QUAN HỆ THỐNG ĐIỀU KHIỂN NÉ VẬT CẢN (LMPC)

Hệ thống sử dụng mô hình kết hợp 2 tầng (Hierarchical Local Planning & Control):
1. **Tầng 1 - Quy hoạch quỹ đạo cục bộ Frenet (Frenet Local Reference Planning)**:
   - Chiếu vị trí xe và các chướng ngại vật lên hệ tọa độ Frenet dọc theo quỹ đạo tham chiếu tim đường: tọa độ dọc $s$ và độ lệch ngang $d$.
   - Phân tích mặt cắt ngang (Cross-sectional Slices) và tính toán khoảng trống khả dụng (Free Gaps) trên dải hành lang mặt đường $[-5.25\,\text{m}, +5.25\,\text{m}]$.
   - Quyết định hướng né (Trái / Phải), biên độ dịch tâm xe $d_{\text{target}}$, làm phẳng biên độ chuỗi vật cản cùng phía, và tạo đường cong mượt $(x_{\text{ref\_mod}}, y_{\text{ref\_mod}}, \theta_{\text{ref\_mod}})$.
   - Phát hiện các trường hợp bít làn hoặc không đủ khoảng cách an toàn để đảo làn nhằm kích hoạt trạng thái dừng khẩn cấp (`is_roadblock = True`).
2. **Tầng 2 - Tối ưu hóa điều khiển QP-MPC (Constrained Quadratic Programming Optimization)**:
   - Bám sát quỹ đạo cục bộ đã dịch chuyển.
   - Kích hoạt siêu phẳng an toàn (Half-plane constraints) cho các vật cản gần xe kèm biến bù nới lỏng $\epsilon_{\text{slack}}$ ($L_1 + L_2$ Penalty).
   - Ép tâm xe nằm trong hành lang mặt đường $[-5.25\,\text{m}, +5.25\,\text{m}]$ với biến bù nới lỏng lấn lề $\epsilon_{\text{road}}$ ($8000 \epsilon_{\text{road}}^2 + 800 \epsilon_{\text{road}}$).
   - Khi có cờ `is_roadblock`, giữ nguyên độ lệch làn đang né (`hold_corridor_d`), tự động hạ trần tốc độ theo profile phanh an toàn $v_{\text{target}} = \sqrt{2 a_{\text{decel}} d_{\text{stop}}}$ và dừng xe dứt khoát trước vật cản.

---

## 2. QUY TRÌNH XỬ LÝ VÀ PHÂN TÍCH MẶT CẮT (CROSS-SECTIONAL SLICES)

### 2.1. Lọc và chiếu vật cản vào hệ quy chiếu Frenet
- Xe chỉ quan tâm đến các vật cản nằm trong phạm vi cảm biến:
  - Tọa độ dọc: từ sau đuôi xe $s_{\text{ego}} - 12.0\,\text{m}$ đến trước mũi xe $s_{\text{ego}} + \text{SENSOR\_DETECTION\_RANGE\_M}$ (với $\text{SENSOR\_DETECTION\_RANGE\_M} = 30.0\,\text{m}$).
  - Tọa độ ngang: nằm trên mặt đường mở rộng $[road_{\text{min}} - 1.5\,\text{m}, road_{\text{max}} + 1.5\,\text{m}]$ (với $road_{\text{min}} = -5.25\,\text{m}$, $road_{\text{max}} = +5.25\,\text{m}$).
- Hàm `compute_obstacle_frenet_bounds(obs)` trích xuất tất cả các đỉnh của hộp bao hướng (OBB) vật cản để xác định chính xác phạm vi chiếm dụng thực tế $[s_{\text{rear}}, s_{\text{front}}] \times [d_{\text{min}}, d_{\text{max}}]$.
- Bán kính an toàn mở rộng (Clearance Radius) bao quanh thân xe và vật cản:
  ```
  r_clearance = EGO_HALF_WIDTH_M + OBSTACLE_AVOIDANCE_MARGIN_M
              = 1.0m + 0.35m = 1.35m
  ```
- Dải cấm chiếm dụng ngang đối với **tâm xe Ego** trên hệ trục Frenet:
  ```
  blocked_interval = [d_min - 1.35m, d_max + 1.35m]
  ```

### 2.2. Phân chia mặt cắt ngang (Slices)
- Thay vì gộp nhóm theo cự ly lớn dễ gây hiểu nhầm các vật cản so le là cùng một cụm, thuật toán gộp các vật cản thành các lát cắt ngang đứng hàng ngang (khoảng cách dọc $\le 2.0\,\text{m}$):
  - Mỗi lát cắt `sl` tổng hợp toàn bộ các dải chắn ngang `blocked` của các vật cản trong lát cắt đó.
  - Sau đó gộp các đoạn chắn trùng nhau để tìm ra các dải bị bít kín $[b_{\text{low}}, b_{\text{high}}]$.

---

## 3. CƠ CHẾ TÌM KHE TRỐNG VÀ QUYẾT ĐỊNH ĐÁNH LÁI TRÁI / PHẢI

### 3.1. Tìm khe trống khả dụng (`find_free_gaps_in_corridor`)
- Trên toàn bộ bề rộng đường từ $road_{\text{min}} = -5.25\,\text{m}$ đến $road_{\text{max}} = +5.25\,\text{m}$, thuật toán loại bỏ tất cả các khoảng bị chắn bởi vật cản (đã nở rộng $1.35\,\text{m}$).
- Do dải cấm đã bao gồm toàn bộ nửa thân xe ($1.0\,\text{m}$) và khoảng đệm an toàn ($0.35\,\text{m}$), khe trống $(g_{\text{low}}, g_{\text{high}})$ là không gian dành riêng cho **tâm xe Ego**. Khe chỉ cần có bề rộng tối thiểu:
  ```
  min_gap_width = 0.30m
  ```
  là tâm xe đã có thể lọt qua mà thân xe vẫn cách mép vật cản ít nhất $0.35\,\text{m}$.

### 3.2. Tiêu chí chọn hướng đánh lái (Trái hay Phải) và tính $d_{\text{target}}$
Hàm `select_optimal_shift_from_gaps` thực hiện đánh giá chi phí cho từng khe trống:
1. **Kiểm tra ràng buộc khóa hướng (`enforce_sign`)**:
   - Nếu xe đã cam kết hướng né và khoảng cách dọc không đủ để đảo làn, thuật toán chỉ tìm các khe trống nằm cùng phía cam kết (`same_side_gaps`):
     - Né Phải (`enforce_sign > 0`): $g_{\text{high}} \ge 1.65\,\text{m}$ và bề rộng khe $\ge 0.8\,\text{m}$.
     - Né Trái (`enforce_sign < 0`): $g_{\text{low}} \le -1.65\,\text{m}$ và bề rộng khe $\ge 0.8\,\text{m}$.
   - Nếu làn né bị chặn không còn khe nào đạt chuẩn $\rightarrow$ trả về `None, None` (kích hoạt Roadblock).

2. **Vị trí tâm xe mục tiêu trong khe trống**:
   - Lấy tâm khe trống nhưng co vào một khoảng đệm an toàn $pad = \min(0.15, 0.20 \times gap\_width)$.
   - Đảm bảo biên độ dịch tối thiểu $1.65\,\text{m}$ khi né:
     ```
     Nếu d_cand > 0 (né phải): d_cand = min(1.65m, g_high - pad)
     Nếu d_cand < 0 (né trái): d_cand = max(-1.65m, g_low + pad)
     ```
     Biên độ tối thiểu $1.65\,\text{m}$ giúp xe thoát hoàn toàn khỏi tim đường và siêu phẳng va chạm, không lách ngập ngừng.

3. **Hàm chi phí chọn hướng (Cost Function)**:
   ```
   Cost = |d_cand| + Penalty
   ```
   - $|d_{\text{cand}}|$: Ưu tiên khe trống gần tim đường nhất (đánh lái ít nhất, xe ổn định nhất).
   - `Penalty`:
     - Nếu đổi hướng ngược với hướng đã né trước đó (`preferred_sign`): cộng thêm phạt nặng `1.50` để ưu tiên giữ nguyên làn né.
     - Nếu xe chưa cam kết hướng nào: ưu tiên theo cấu hình `OBSTACLE_PREFER_BYPASS_SIDE` (nếu ngược hướng ưu tiên thì cộng phạt nhẹ `0.10`).

4. **Cơ chế nhìn xa đồng bộ (Lookahead Window 16m)**:
   - Trước khi xe cam kết đánh lái cho lát cắt đầu tiên, hệ thống nhìn xa $16\,\text{m}$ để gộp tất cả các dải chắn của các lát cắt trong vùng này.
   - Nếu tồn tại một "hành lang chung" (Common Corridor) thông suốt cả chuỗi, xe sẽ chọn hướng đi theo hành lang chung đó ngay từ đầu, tránh tình trạng vừa rẽ trái lát cắt 1 đã phải ngoặt gắt sang phải ở lát cắt 2.

### 3.3. Cơ chế Khóa hướng né (Locked / Committed Side)
Khi xe đã bắt đầu mở lái né một chướng ngại vật:
- Điều kiện cam kết: Xe nằm trong khoảng cách khóa $(s_{\text{rear}} - \text{EGO\_FRONT\_OVERHANG\_M} - s_{\text{ego}}) \le lock\_dist$ (với $lock\_dist = \text{OBSTACLE\_APPROACH\_RAMP\_M} + \text{OBSTACLE\_APPROACH\_MARGIN\_M} = 13.0 + 1.0 = 14.0\,\text{m}$) hoặc xe đã lệch ngang thực tế $|d_{\text{ego}}| \ge 0.65\,\text{m}$.
- Khi đã cam kết hướng `committed_sign`, các lát cắt tiếp theo bắt buộc phải tuân theo hướng này nếu:
  1. Bản thân lát cắt mới đã có vật cản nằm trong vùng khóa của `active_chain`.
  2. Khoảng trống dọc giữa 2 lát cắt không đủ để xe đảo làn an toàn:
     $$ds_{\text{gap}} < ds_{\text{req}} = (\text{EGO\_FRONT} + \text{EGO\_REAR}) + 2.0 \times (|curr\_d\_target| + 1.85)$$
  3. Thân xe thực tế đang lệch về phía `curr_sign` ($d_{\text{ego}} \cdot curr\_sign \ge 0.65$) và cự ly dọc tới lát cắt không đủ để đánh chéo qua tim đường:
     $$dist_{\text{ego\_to\_sl}} < ds_{\text{req\_ego}} = 2.2 \times (|d_{\text{ego}}| + 1.85) + 2.0$$
- **Lọc vật cản mép lề không ảnh hưởng**: Các vật cản nằm xa ngoài lề đường có $|d_{\text{target}}| < 0.05\,\text{m}$ sẽ bị loại bỏ, không thêm vào `active_chain` và không khóa hướng né sai.

---

## 4. QUYẾT ĐỊNH DỪNG LẠI (ROADBLOCK EMERGENCY STOP)

Hệ thống quyết định dừng xe an toàn khi phát hiện không thể tiếp tục lưu thông qua 2 trường hợp chính và thực thi phanh qua 2 giai đoạn.

### 4.1. Các trường hợp ra quyết định kích hoạt dừng xe (`self.is_roadblock = True`)

1. **Trường hợp 1: Không còn khe hở trên toàn bộ mặt đường (Full Roadblock)**
   - **Vị trí code**: [linearized_mpc_controller.py: dòng 460 - 470](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/linearized_mpc_controller.py#L460-L470)
   - **Tình huống**: Cụm vật cản dàn hàng ngang chắn kín từ mép trái sang mép phải mặt đường (`not gaps`). Không còn khe nào trên dải $[-5.25\,\text{m}, +5.25\,\text{m}]$ có bề rộng $\ge 0.30\,\text{m}$.
   - **Quyết định**: Kích hoạt `self.is_roadblock = True`, lưu khoảng cách tới vật cản và dừng xe.

2. **Trường hợp 2: Làn đang né bị bít kín và không đủ khoảng cách đảo làn (Staggered Roadblock)**
   - **Vị trí code**: [linearized_mpc_controller.py: dòng 475 - 487](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/linearized_mpc_controller.py#L475-L487)
   - **Tình huống**: Xe đang né dở chướng ngại vật (ví dụ bên phải), xuất hiện vật cản tiếp theo chắn ngang làn né phải. Do khoảng cách dọc quá ngắn, xe không thể lách ngược lại làn trái mà không va chạm. Hàm `select_optimal_shift_from_gaps(..., enforce_sign=curr_sign)` trả về `None`.
   - **Quyết định**: Dừng khẩn cấp ngay trên chính làn né hiện tại.

### 4.2. Quá trình thực thi dừng xe (Braking Execution)

Nằm trong hàm [solve: dòng 838 - 874](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/linearized_mpc_controller.py#L838-L874):

1. **Duy trì hành lang né khi dừng (Hold Corridor)**:
   - Nếu dừng khi xe đang ở làn né (`hold_corridor_d != 0`), hệ thống giữ nguyên quỹ đạo tham chiếu cục bộ `x_ref_mod, y_ref_mod` ở làn né kéo dài qua vị trí vật cản (`blocking_slice['s_front'] + 15.0m`).
   - **Mục đích**: Xe phanh thẳng hàng ngay trên làn né hiện tại, tuyệt đối không bị kéo ngược về tim đường gây đâm vào đuôi chướng ngại vật phía trước.

2. **Giai đoạn 1: Giảm tốc mượt theo quỹ đạo phanh an toàn (`d_stop > 0.25m`)**:
   - Điểm dừng an toàn cách vật cản: `ROADBLOCK_SAFE_STOP_DISTANCE_M = 3.5m`.
   - Khoảng cách còn lại tới vạch dừng: `d_stop = max(0.0, d_to_barrier - 3.5m)`.
   - Vận tốc mục tiêu tại mỗi bước $k$ trong chân trời dự đoán $N = 10$ bước ($t = k \cdot dt$):
     $$v_{\text{target}}(k) = \sqrt{2.0 \times \text{ROADBLOCK\_DECELERATION\_M\_S\_2} \times d_{\text{stop}}(k)}$$
     $$v_{\text{ref\_adapted}}[k] = \min(v_{\text{ref\_base}}[k], v_{\text{target}}(k))$$
     (với gia tốc phanh êm ái `ROADBLOCK_DECELERATION_M_S_2 = 3.0 m/s²`).
   - Bài toán QP được giải với cờ `is_roadblock = True`, cho phép hạ cận dưới vận tốc `min_v = 0.0 m/s` để bộ giải tối ưu lực phanh mượt mà dừng đúng vạch.

3. **Giai đoạn 2: Khóa phanh đứng yên (`d_stop <= 0.25m` hoặc `curr_v < 0.25 m/s`)**:
   - Khi xe đã tiến đến sát vạch dừng cách vật cản $3.5\,\text{m}$ hoặc vận tốc thực tế đã triệt tiêu về gần 0:
     ```python
     if d_stop <= 0.25 or (curr_v < 0.25 and d_to_barrier <= ROADBLOCK_SAFE_STOP_DISTANCE_M + 1.5):
         self.is_success = True
         self.init_nominal_trajectory(x_init, x_ref_mod, y_ref_mod, theta_ref_mod, np.zeros_like(v_ref))
         return 0.0, -3.5  # steer = 0.0 rad, gia tốc phanh cứng = -3.5 m/s²
     ```
   - Giữ góc lái thẳng $0.0\,\text{rad}$ và khóa phanh cứng $-3.5\,\text{m/s}²$ để xe đứng yên hoàn toàn.

---

## 5. BẢNG TỔNG HỢP CÁC THÔNG SỐ CẤU HÌNH LIÊN QUAN

| Tên thông số | Giá trị | File cấu hình | Ý nghĩa thực tế |
| :--- | :--- | :--- | :--- |
| `DEFAULT_ROAD_HALF_WIDTH_M` | `5.25m` | [config.py: L116](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py#L116) | Nửa bề rộng hành lang mặt đường khả dụng cho tâm xe Ego (mặt đường rộng $10.5\,\text{m}$) |
| `EGO_HALF_WIDTH_M` | `1.0m` | [config.py: L120](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py#L120) | Nửa chiều rộng thân xe Ego (thân xe rộng $2.0\,\text{m}$) |
| `EGO_FRONT_OVERHANG_M` | `2.4m` | [config.py: L118](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py#L118) | Khoảng cách từ tâm xe tới cản trước |
| `EGO_REAR_OVERHANG_M` | `2.4m` | [config.py: L119](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py#L119) | Khoảng cách từ tâm xe tới cản sau |
| `OBSTACLE_AVOIDANCE_MARGIN_M` | `0.35m` | [config.py: L129](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py#L129) | Khoảng đệm an toàn dự phòng ngoài thân xe ($35\,\text{cm}$) |
| `OBSTACLE_APPROACH_RAMP_M` | `13.0m` | [config.py: L93](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py#L93) | Quãng đường bắt đầu mở lái chuyển làn trước vật cản |
| `OBSTACLE_HOLD_DISTANCE_M` | `1.5m` | [config.py: L94](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py#L94) | Quãng đường tiếp tục giữ làn né sau khi qua vật cản |
| `OBSTACLE_RETURN_RAMP_M` | `10.0m` | [config.py: L95](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py#L95) | Quãng đường chuyển làn êm ái trở về tim đường |
| `ROADBLOCK_SAFE_STOP_DISTANCE_M` | `3.5m` | [config.py: L106](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py#L106) | Khoảng cách dừng cách cản trước của vật cản chặn đường |
| `ROADBLOCK_DECELERATION_M_S_2` | `3.0 m/s²` | [config.py: L107](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py#L107) | Gia tốc phanh hãm dừng êm khi gặp vật cản bít đường |
| `ENABLE_ROADBLOCK_EMERGENCY_STOP` | `True` | [config.py: L105](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py#L105) | Bật tính năng phanh dừng an toàn thay vì cố lách |
| `OBSTACLE_PREFER_BYPASS_SIDE` | `'left'` | [config.py: L137](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py#L137) | Hướng né ưu tiên danh định khi 2 bên trống như nhau |
| `MPC_W_SLACK` | `20000.0` | [config.py: L127](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py#L127) | Trọng số phạt bậc 2 ($L_2$) cho biến bù né vật cản $\epsilon_{\text{slack}}$ |
| `MPC_W_SLACK_LINEAR` | `500.0` | [config.py: L128](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/config.py#L128) | Trọng số phạt bậc 1 ($L_1$) cho biến bù né vật cản $\epsilon_{\text{slack}}$ |
| Phạt nới lỏng biên đường | `8000.0 / 800.0` | [LMPC: L733](file:///home/tam-phuc/carla_simulator/Carla-Autonomous-MPC/src/linearized_mpc_controller.py#L733) | Trọng số phạt biến bù lấn lề $8000 \epsilon_{\text{road}}^2 + 800 \epsilon_{\text{road}}$ |

---

## 6. CHI TIẾT HÌNH HỌC VÀ CÁC CÔNG THỨC TOÁN HỌC CỐT LÕI

### 6.1. Chiếu tọa độ Frenet $(s, d)$
- Đường tham chiếu uốn cong toàn cục $(x, y)$ được biến đổi về hệ trục Frenet:
  - $s$ (dọc đường): Khoảng cách tích lũy dọc theo tim đường.
  - $d$ (ngang đường): Khoảng cách vuông góc từ vị trí xe/vật cản tới tiếp tuyến tim đường theo vector pháp tuyến trái $\vec{n} = [-\sin\theta, \cos\theta]^T$.
  - $d = 0.0\,\text{m}$: Tim đường.
  - $d > 0$: Lệch sang Trái.
  - $d < 0$: Lệch sang Phải.

### 6.2. Công thức tính bán kính an toàn $r_{\text{safe}}$ và $r_{\text{clearance}}$
- Thân xe Ego: Chiều rộng $2.0\,\text{m}$ $\rightarrow$ Nửa chiều rộng `EGO_HALF_WIDTH_M = 1.0m`.
- Khoảng đệm an toàn `OBSTACLE_AVOIDANCE_MARGIN_M = 0.35m`.
- Bán kính an toàn:
  $$r_{\text{safe}} = \text{EGO\_HALF\_WIDTH\_M} + \text{OBSTACLE\_AVOIDANCE\_MARGIN\_M} = 1.0\,\text{m} + 0.35\,\text{m} = \mathbf{1.35\,\text{m}}$$
- Dải cấm ngang cho tâm xe khi vật cản có biên độ $[d_{\text{min}}, d_{\text{max}}]$:
  $$\text{Blocked} = [d_{\text{min}} - 1.35\,\text{m}, d_{\text{max}} + 1.35\,\text{m}]$$

### 6.3. Giới hạn hành lang mặt đường và biến bù nới lỏng ($\epsilon_{\text{road}}$)
- Làn đường hiện tại trải dài từ $d = -1.75\,\text{m}$ đến $+1.75\,\text{m}$ (rộng $3.5\,\text{m}$).
- Các làn bên cạnh mở rộng ra tới mép ngoài mặt đường:
  $$\text{Mép lề trái } (-5.25\,\text{m}) \longleftrightarrow \text{Tim đường } (0.0\,\text{m}) \longleftrightarrow \text{Mép lề phải } (+5.25\,\text{m})$$
- Ràng buộc biên mềm trong bộ giải QP của MPC:
  $$-5.25 - \epsilon_{\text{road}, k} \le d_{\text{lat}, k} \le 5.25 + \epsilon_{\text{road}, k}, \quad \epsilon_{\text{road}, k} \ge 0$$
- Chi phí phạt trong hàm mục tiêu:
  $$\text{cost} \mathrel{+}= 8000.0 \cdot \epsilon_{\text{road}, k}^2 + 800.0 \cdot \epsilon_{\text{road}, k}$$
  - Khi xe chạy trong lòng đường ($|d_{\text{lat}}| \le 5.25\,\text{m}$): $\epsilon_{\text{road}} = 0$, chi phí phạt bằng 0.
  - Khi bị ép cấn lề để tránh va chạm trực diện: solver chấp nhận $\epsilon_{\text{road}} = |d_{\text{lat}}| - 5.25 > 0$ và chịu phạt, sau đó lực phạt kéo xe quay lại lòng đường ngay khi thoát vật cản.