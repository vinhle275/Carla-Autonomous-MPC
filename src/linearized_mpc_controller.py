import casadi as ca
import numpy as np
import math
import time

from src.config import (
    N, dt, L, TARGET_SPEED_MS,
    MAX_CONTROL_WHEEL_ANGLE_RAD, MAX_CONTROL_ACCELERATION_M_S_2, MAX_CONTROL_BRAKING_M_S_2,
    FINE_X_COEF, FINE_Y_COEF, FINE_V_COEF, FINE_STEER_COEF, FINE_ACC_COEF,
    FINE_STEER_DOT_COEF, FINE_ACC_DOT_COEF, FINE_THETA_COEF, FINE_LATERAL_COEF,
    MPC_MAX_LINEARIZATION_ITER, MPC_W_SLACK, MPC_W_SLACK_LINEAR,
    OBSTACLE_AVOIDANCE_MARGIN_M,
    OBSTACLE_CAUTION_SPEED_MS, OBSTACLE_SLOWDOWN_DISTANCE_M,
    OBSTACLE_SLOWDOWN_LATERAL_CORRIDOR_M, OBSTACLE_PREFER_BYPASS_SIDE
)
from src.help_functions import calculate_lateral_deviation, normalize_angle

class LinearizedMpcController:
    """
    Bộ điều khiển MPC Tuyến tính hóa lặp (Successive Linearization QP-MPC)
    kết hợp Dời tham chiếu cục bộ (Local Reference Shifting) và Ràng buộc Biến bù nới lỏng (Slack Variables):
    - Đưa bài toán phi tuyến (NMPC) về dạng bài toán Quy hoạch Toàn phương (QP).
    - Tự động nắn quỹ đạo tham chiếu S-curve vòng qua vật cản mượt mà, triệt tiêu xung đột gradient.
    - Khống chế cận dưới vận tốc (Anti-Stall), giải quyết triệt để hiện tượng Freezing Robot.
    - Tự động hạ tốc độ an toàn khi phát hiện vật cản (Adaptive Speed Governor).
    - Triệt tiêu hiện tượng đánh lái loạn xạ sau khi vượt qua vật cản.
    """
    def __init__(self, horizon=N, sample_time=dt):
        self.horizon = horizon
        self.dt = sample_time
        self.L = L

        # Quỹ đạo danh định (Nominal Trajectory) để tuyến tính hóa
        self.x_nom = None  # Shape: (4, horizon + 1)
        self.u_nom = None  # Shape: (2, horizon)

        # Quỹ đạo tham chiếu đã được dời cục bộ (cho trực quan hóa 3D)
        self.x_ref_mod = None
        self.y_ref_mod = None

        # Bộ theo dõi trạng thái né vật cản có nhớ (Stateful Bypass Tracker)
        # Giữ nguyên làn né khi vượt qua vật cản, không bị trả lái sớm khi vật cản rơi khỏi FOV cảm biến
        self.active_obstacle = None

        # Bộ đệm điều khiển dự phòng (Control buffer)
        self.control_buffer = {
            "acceleration": [0.0] * self.horizon,
            "wheel_angle": [0.0] * self.horizon
        }
        self.buffer_index = 0
        self.last_cost = 0.0
        self.is_success = False

    def _eval_discrete_dynamics(self, x, u):
        """Tính toán trạng thái tiếp theo theo mô hình động học xe đạp phi tuyến"""
        pos_x, pos_y, theta, v = x[0], x[1], x[2], x[3]
        delta, a = u[0], u[1]

        next_x = pos_x + v * np.cos(theta) * self.dt
        next_y = pos_y + v * np.sin(theta) * self.dt
        next_theta = theta + (v / self.L) * np.tan(delta) * self.dt
        next_v = v + a * self.dt
        return np.array([next_x, next_y, next_theta, next_v])

    def get_linearized_matrices(self, x_bar, u_bar):
        """
        Tính toán ma trận Jacobian rời rạc A_k, B_k và vector bù d_k tại điểm danh định (x_bar, u_bar):
        x_{k+1} = A_k * x_k + B_k * u_k + d_k
        """
        pos_x, pos_y, theta, v = x_bar[0], x_bar[1], x_bar[2], x_bar[3]
        delta, a = u_bar[0], u_bar[1]

        # Ma trận A (4x4)
        A = np.eye(4)
        A[0, 2] = -v * np.sin(theta) * self.dt
        A[0, 3] = np.cos(theta) * self.dt
        A[1, 2] = v * np.cos(theta) * self.dt
        A[1, 3] = np.sin(theta) * self.dt
        A[2, 3] = (np.tan(delta) / self.L) * self.dt

        # Ma trận B (4x2)
        B = np.zeros((4, 2))
        cos_delta = np.cos(delta)
        sec_delta_sq = 1.0 / (cos_delta ** 2 + 1e-6)
        B[2, 0] = (v / self.L) * sec_delta_sq * self.dt
        B[3, 1] = self.dt

        # Vector bù phi tuyến hằng số d (4,)
        x_next_exact = self._eval_discrete_dynamics(x_bar, u_bar)
        d = x_next_exact - (A @ x_bar + B @ u_bar)

        return A, B, d

    def init_nominal_trajectory(self, x_init, x_ref, y_ref, theta_ref, v_ref):
        """Khởi tạo quỹ đạo danh định từ tham chiếu nếu chưa có Warm-start"""
        self.x_nom = np.zeros((4, self.horizon + 1))
        self.u_nom = np.zeros((2, self.horizon))

        self.x_nom[:, 0] = x_init
        for k in range(self.horizon):
            self.x_nom[0, k + 1] = x_ref[k]
            self.x_nom[1, k + 1] = y_ref[k]
            self.x_nom[2, k + 1] = theta_ref[k]
            self.x_nom[3, k + 1] = v_ref[k]
            self.u_nom[0, k] = 0.0
            self.u_nom[1, k] = 0.0

    def compute_local_reference_shift(self, x_init, x_ref, y_ref, theta_ref, v_ref, detected_obstacles):
        """
        Kỹ thuật Local Reference Shifting có nhớ (Stateful Local Reference Shifting):
        - Khi phát hiện vật cản: Khóa mục tiêu (Latched Bypass), tự động uốn đường cong Cosine sang một bên.
        - Giữ nguyên làn né (Hold lane) xuyên suốt khi xe đang chạy ngang qua vật cản,
          kể cả khi vật cản rơi khỏi trường nhìn phía trước (FOV) của cảm biến.
        - Chỉ khi đuôi xe đã vượt qua vật cản an toàn (> 3.5m) mới bắt đầu đánh lái từ từ sang phía ngược lại
          để nhập trở lại tim đường cũ.
        """
        tx0 = math.cos(theta_ref[0])
        ty0 = math.sin(theta_ref[0])
        x_ego, y_ego = x_init[0], x_init[1]

        # 1. Kiểm tra trạng thái của vật cản đang né hiện tại (nếu có)
        if self.active_obstacle is not None:
            x_o = self.active_obstacle['x']
            y_o = self.active_obstacle['y']
            # Tiến độ dọc của xe so với vật cản
            progress_ego = tx0 * (x_ego - x_o) + ty0 * (y_ego - y_o)

            # Nếu xe đã vượt qua vật cản hơn 14m và đã nhập làn xong -> Giải phóng trạng thái né
            if progress_ego > 14.0:
                self.active_obstacle = None
            else:
                # Nếu vật cản vẫn còn nhìn thấy phía trước (xe chưa qua ngang hàng), cập nhật nhẹ vị trí nếu có
                if progress_ego < 0.0 and detected_obstacles:
                    for obs in detected_obstacles:
                        if obs.get('id') == self.active_obstacle.get('id'):
                            self.active_obstacle['x'] = obs['x']
                            self.active_obstacle['y'] = obs['y']
                            break

        # 2. Nếu chưa có vật cản nào đang né, tìm kiếm vật cản nguy hiểm mới từ cảm biến
        if self.active_obstacle is None:
            if detected_obstacles and len(detected_obstacles) > 0:
                crit_obs = [
                    obs for obs in detected_obstacles
                    if 0.0 <= obs['d_long'] <= 22.0 and
                       abs(obs['d_lat']) <= (OBSTACLE_SLOWDOWN_LATERAL_CORRIDOR_M + 0.6)
                ]
                if crit_obs:
                    primary_obs = min(crit_obs, key=lambda o: o['dist'])
                    d_lat_obs = primary_obs['d_lat']
                    if d_lat_obs < -0.15:
                        side_sign = 1.0   # né sang TRÁI
                    elif d_lat_obs > 0.15:
                        side_sign = -1.0  # né sang PHẢI
                    else:
                        side_sign = 1.0 if OBSTACLE_PREFER_BYPASS_SIDE == 'left' else -1.0

                    r_safe = primary_obs['radius'] + OBSTACLE_AVOIDANCE_MARGIN_M
                    A_shift = max(r_safe + 0.45, 1.95)

                    self.active_obstacle = {
                        'id': primary_obs.get('id'),
                        'type': primary_obs.get('type'),
                        'x': primary_obs['x'],
                        'y': primary_obs['y'],
                        'radius': primary_obs['radius'],
                        'r_safe': r_safe,
                        'side_sign': side_sign,
                        'A_shift': A_shift
                    }

        # 3. Nếu không có vật cản nào đang kích hoạt -> Trả về tham chiếu gốc
        if self.active_obstacle is None:
            return np.copy(x_ref), np.copy(y_ref), np.copy(theta_ref), np.copy(v_ref), False

        # 4. Tính toán quỹ đạo uốn lượn S-curve cho cả đoạn chân trời N bước
        x_o = self.active_obstacle['x']
        y_o = self.active_obstacle['y']
        side_sign = self.active_obstacle['side_sign']
        A_shift = self.active_obstacle['A_shift']

        x_ref_mod = np.copy(x_ref)
        y_ref_mod = np.copy(y_ref)
        theta_ref_mod = np.copy(theta_ref)

        for k in range(len(x_ref)):
            th_k = theta_ref[k]
            tx_k = math.cos(th_k)
            ty_k = math.sin(th_k)
            nx_k = -math.sin(th_k)
            ny_k = math.cos(th_k)

            # Tọa độ dọc của waypoint k so với vật cản
            # progress_k < 0: waypoint ở trước vật cản
            # progress_k = 0: waypoint ngang hàng vật cản
            # progress_k > 0: waypoint ở sau vật cản
            progress_k = tx_k * (x_ref[k] - x_o) + ty_k * (y_ref[k] - y_o)

            if progress_k < -2.0:
                # Giai đoạn tiếp cận: đánh lái mở từ từ sang trái/phải
                dist_before = -progress_k
                if dist_before <= 22.0:
                    w = 0.5 * (1.0 + math.cos(math.pi * (dist_before - 2.0) / 20.0))
                else:
                    w = 0.0
            elif -2.0 <= progress_k <= 3.5:
                # Giai đoạn chạy ngang qua vật cản: GIỮ NGUYÊN HOÀN TOÀN LÀN NÉ
                w = 1.0
            elif 3.5 < progress_k <= 14.0:
                # Giai đoạn vượt qua xong: đánh lái từ từ trả lại tim đường
                dist_after = progress_k - 3.5
                w = 0.5 * (1.0 + math.cos(math.pi * dist_after / 10.5))
            else:
                w = 0.0

            shift_lat = side_sign * A_shift * w
            x_ref_mod[k] = x_ref[k] + shift_lat * nx_k
            y_ref_mod[k] = y_ref[k] + shift_lat * ny_k

        # Làm mịn hướng tiếp tuyến theta_ref_mod cho quỹ đạo lách né
        for k in range(len(x_ref) - 1):
            dx = x_ref_mod[k + 1] - x_ref_mod[k]
            dy = y_ref_mod[k + 1] - y_ref_mod[k]
            th_target = math.atan2(dy, dx)
            d_th = normalize_angle(th_target - theta_ref_mod[k])
            theta_ref_mod[k] = theta_ref_mod[k] + d_th
        if len(x_ref) > 1:
            theta_ref_mod[-1] = theta_ref_mod[-2]

        return x_ref_mod, y_ref_mod, theta_ref_mod, np.copy(v_ref), True

    def solve_qp_iteration(self, x_init, x_ref, y_ref, theta_ref, v_ref, obstacles=None):
        """
        Xây dựng và giải bài toán Quy hoạch Toàn phương (QP) tại một vòng lặp tuyến tính hóa:
        - Khống chế cận dưới vận tốc (Anti-Stall) triệt tiêu hoàn toàn Freezing Robot.
        - Áp dụng mặt phẳng tiếp tuyến phân luồng theo khoảng cách Euclid tuyến tính hóa.
        """
        opti = ca.Opti()
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_cpu_time': self.dt * 0.9,
            'ipopt.tol': 1e-4
        }
        opti.solver('ipopt', opts)

        # Biến quyết định
        X = opti.variable(4, self.horizon + 1)
        U = opti.variable(2, self.horizon)

        # Ràng buộc trạng thái xuất phát
        opti.subject_to(X[:, 0] == x_init)

        cost = 0

        # Cận dưới vận tốc chống chết máy / dừng khựng (Anti-Stall Velocity Bound)
        min_v = min(float(x_init[3]), 1.5)

        # Tuyến tính hóa động học xe dọc theo chân trời dự đoán
        for k in range(self.horizon):
            x_bar_k = self.x_nom[:, k]
            u_bar_k = self.u_nom[:, k]
            A_k, B_k, d_k = self.get_linearized_matrices(x_bar_k, u_bar_k)

            # Ràng buộc động học tuyến tính hóa (Linear Equality Constraints)
            opti.subject_to(X[:, k + 1] == A_k @ X[:, k] + B_k @ U[:, k] + d_k)

            # Ràng buộc giới hạn phần cứng (Box Constraints)
            opti.subject_to(U[0, k] <= MAX_CONTROL_WHEEL_ANGLE_RAD)
            opti.subject_to(U[0, k] >= -MAX_CONTROL_WHEEL_ANGLE_RAD)
            opti.subject_to(U[1, k] <= MAX_CONTROL_ACCELERATION_M_S_2)
            opti.subject_to(U[1, k] >= MAX_CONTROL_BRAKING_M_S_2)

            # Ràng buộc vận tốc tiến an toàn (Không cho solver chọn đứng yên)
            opti.subject_to(X[3, k + 1] >= min_v)
            opti.subject_to(X[3, k + 1] <= 15.0)

            # Thiết lập điểm mớm (Warm-start)
            opti.set_initial(X[:, k], x_bar_k)
            opti.set_initial(U[:, k], u_bar_k)

            # Chi phí bám quỹ đạo (Quadratic Tracking Cost)
            fine_x = FINE_X_COEF * (X[0, k] - x_ref[k]) ** 2
            fine_y = FINE_Y_COEF * (X[1, k] - y_ref[k]) ** 2
            fine_v = FINE_V_COEF * (X[3, k] - v_ref[k]) ** 2
            fine_theta = FINE_THETA_COEF * (X[2, k] - theta_ref[k]) ** 2

            # Độ lệch ngang vuông góc so với quỹ đạo tham chiếu
            if k < self.horizon - 1:
                lat_dev = calculate_lateral_deviation(
                    X[0, k], X[1, k],
                    x_ref[k], y_ref[k],
                    x_ref[k + 1], y_ref[k + 1]
                )
            else:
                lat_dev = 0.0
            fine_lat = FINE_LATERAL_COEF * (lat_dev ** 2)

            # Phạt biên độ điều khiển
            fine_steer = FINE_STEER_COEF * (U[0, k] ** 2)
            fine_acc = FINE_ACC_COEF * (U[1, k] ** 2)

            # Phạt tốc độ giật lái / giật ga (Smoothness - phạt nặng để không rung lắc)
            fine_steer_dot = 0
            fine_acc_dot = 0
            if k > 0:
                fine_steer_dot = FINE_STEER_DOT_COEF * ((U[0, k] - U[0, k - 1]) ** 2)
                fine_acc_dot = FINE_ACC_DOT_COEF * ((U[1, k] - U[1, k - 1]) ** 2)

            cost += (fine_x + fine_y + fine_v + fine_theta + fine_lat +
                     fine_steer + fine_acc + fine_steer_dot + fine_acc_dot)

        opti.set_initial(X[:, self.horizon], self.x_nom[:, self.horizon])

        # =====================================================================
        # RÀNG BUỘC NÉ VẬT CẢN DẠNG TIẾP DIỆN PHÂN CÁCH TUYẾN TÍNH HÓA & BIẾN BÙ SLACK
        # =====================================================================
        if obstacles and len(obstacles) > 0:
            active_obs = obstacles[:3]
            num_obs = len(active_obs)

            # Khởi tạo ma trận biến bù Slack Variable
            eps_slack = opti.variable(num_obs, self.horizon)
            opti.subject_to(eps_slack >= 0)

            for m, obs in enumerate(active_obs):
                x_o = obs['x']
                y_o = obs['y']
                r_safe = obs['radius'] + OBSTACLE_AVOIDANCE_MARGIN_M

                for k in range(self.horizon):
                    x_bar_k = self.x_nom[0, k]
                    y_bar_k = self.x_nom[1, k]
                    dx_nom = x_bar_k - x_o
                    dy_nom = y_bar_k - y_o
                    dist_nom = math.sqrt(dx_nom ** 2 + dy_nom ** 2) + 1e-4

                    # Chỉ kích hoạt mặt phẳng tiếp tuyến phân cách khi xe ở gần vật cản
                    if dist_nom <= r_safe + 5.0:
                        nx_bar = dx_nom / dist_nom
                        ny_bar = dy_nom / dist_nom

                        # Ràng buộc siêu phẳng tiếp diện nửa không gian an toàn (Supporting Hyperplane):
                        # nx_bar * (X[0,k] - x_o) + ny_bar * (X[1,k] - y_o) + eps_slack >= r_safe
                        lhs = nx_bar * (X[0, k] - x_o) + ny_bar * (X[1, k] - y_o) + eps_slack[m, k]
                        opti.subject_to(lhs >= r_safe)

                        # Phạt vi phạm vùng an toàn
                        cost += MPC_W_SLACK * (eps_slack[m, k] ** 2) + MPC_W_SLACK_LINEAR * eps_slack[m, k]

        opti.minimize(cost)

        sol = opti.solve()
        x_opt = sol.value(X)
        u_opt = sol.value(U)
        cost_opt = float(sol.value(cost))
        return x_opt, u_opt, cost_opt

    def solve(self, x_init, x_ref, y_ref, theta_ref, v_ref, detected_obstacles=None):
        """
        Thực hiện toàn bộ chu trình Tuyến tính hóa lặp (Iterative Linearization Loop):
        1. Dời tạm tham chiếu cục bộ có nhớ (Stateful Local Reference Shifting).
        2. Tự động điều chỉnh profile tốc độ an toàn (Adaptive Speed Governor).
        3. Khởi tạo/Cập nhật quỹ đạo danh định mượt mà.
        4. Chạy 1-2 vòng lặp giải QP với danh sách vật cản bảo vệ đầy đủ.
        5. Cập nhật nghiệm tối ưu và dịch chuyển Warm-start cho chu kỳ sau.
        """
        # =====================================================================
        # 1. DỜI TẠM THAM CHIẾU CỤC BỘ CÓ NHỚ (STATEFUL REFERENCE SHIFTING)
        # =====================================================================
        x_ref_mod, y_ref_mod, theta_ref_mod, v_ref_adapted, has_shift = self.compute_local_reference_shift(
            x_init, x_ref, y_ref, theta_ref, v_ref, detected_obstacles
        )
        self.x_ref_mod = x_ref_mod
        self.y_ref_mod = y_ref_mod

        # =====================================================================
        # 2. BỘ THÍCH ỨNG TỐC ĐỘ THEO TRẠNG THÁI VƯỢT VẬT CẢN (SPEED GOVERNOR)
        # =====================================================================
        v_ref_adapted = np.copy(v_ref)
        if self.active_obstacle is not None:
            tx0 = math.cos(theta_ref[0])
            ty0 = math.sin(theta_ref[0])
            progress_ego = tx0 * (x_init[0] - self.active_obstacle['x']) + ty0 * (x_init[1] - self.active_obstacle['y'])

            if progress_ego < -2.0:
                # Đang tiếp cận: giảm dần từ TARGET_SPEED_MS về OBSTACLE_CAUTION_SPEED_MS
                dist_to_obs = -progress_ego
                ratio = np.clip((dist_to_obs - 4.0) / (OBSTACLE_SLOWDOWN_DISTANCE_M - 4.0), 0.0, 1.0)
                base_speed = max(float(v_ref[0]), TARGET_SPEED_MS)
                v_safe = OBSTACLE_CAUTION_SPEED_MS + ratio * (base_speed - OBSTACLE_CAUTION_SPEED_MS)
                v_ref_adapted = np.minimum(v_ref_adapted, v_safe)
                v_ref_adapted = np.maximum(v_ref_adapted, OBSTACLE_CAUTION_SPEED_MS)
            elif -2.0 <= progress_ego <= 3.5:
                # Đang chạy ngang qua vật cản: duy trì vận tốc ổn định 3.0 m/s
                v_ref_adapted = np.minimum(v_ref_adapted, OBSTACLE_CAUTION_SPEED_MS)
                v_ref_adapted = np.maximum(v_ref_adapted, OBSTACLE_CAUTION_SPEED_MS)
            else:
                # Đang nhập làn trở lại: tăng tốc mượt mà từ 3.0 m/s lên TARGET_SPEED_MS
                return_dist = progress_ego - 3.5
                ratio = np.clip(return_dist / 10.0, 0.0, 1.0)
                v_ramp = OBSTACLE_CAUTION_SPEED_MS + ratio * (TARGET_SPEED_MS - OBSTACLE_CAUTION_SPEED_MS)
                v_ref_adapted = np.minimum(v_ref_adapted, v_ramp)

        # Tập hợp danh sách vật cản bảo vệ (gồm vật cản đang né có nhớ + vật cản từ cảm biến)
        all_obs = []
        if self.active_obstacle is not None:
            all_obs.append(self.active_obstacle)
        if detected_obstacles:
            for obs in detected_obstacles:
                if self.active_obstacle is None or obs.get('id') != self.active_obstacle.get('id'):
                    all_obs.append(obs)

        # Khởi tạo / Đồng bộ lại nominal trajectory nếu chưa có hoặc vừa có độ lệch dời làn lớn
        if self.x_nom is None or (has_shift and np.max(np.abs(self.x_nom[1, :self.horizon] - y_ref_mod)) > 1.2):
            self.init_nominal_trajectory(x_init, x_ref_mod, y_ref_mod, theta_ref_mod, v_ref_adapted)

        x_opt = None
        u_opt = None

        try:
            # Vòng lặp tuyến tính hóa lặp (Iterative / Successive Loop)
            for it in range(MPC_MAX_LINEARIZATION_ITER):
                x_opt, u_opt, cost_val = self.solve_qp_iteration(
                    x_init, x_ref_mod, y_ref_mod, theta_ref_mod, v_ref_adapted, obstacles=all_obs
                )
                self.x_nom = x_opt
                self.u_nom = u_opt
                self.last_cost = cost_val

            self.is_success = True

            # Lấy tín hiệu bước đầu tiên (Receding Horizon Principle)
            steer_rad = float(u_opt[0, 0])
            acc_m_s2 = float(u_opt[1, 0])

            # Cập nhật buffer dự phòng
            self.control_buffer["wheel_angle"] = list(u_opt[0, 1:]) + [float(u_opt[0, -1])]
            self.control_buffer["acceleration"] = list(u_opt[1, 1:]) + [float(u_opt[1, -1])]
            self.buffer_index = 0

            # Dịch chuyển Warm-start cho chu kỳ điều khiển tiếp theo (k + 1)
            self.x_nom[:, :-1] = x_opt[:, 1:]
            self.x_nom[:, -1] = x_opt[:, -1]
            self.u_nom[:, :-1] = u_opt[:, 1:]
            self.u_nom[:, -1] = u_opt[:, -1]

            return steer_rad, acc_m_s2

        except Exception as e:
            # Fallback an toàn khi solver gặp sự cố
            self.is_success = False
            if self.buffer_index < len(self.control_buffer["wheel_angle"]):
                steer_rad = self.control_buffer["wheel_angle"][self.buffer_index]
                acc_m_s2 = self.control_buffer["acceleration"][self.buffer_index]
                self.buffer_index += 1
            else:
                steer_rad = 0.0
                acc_m_s2 = -1.0  # Giảm tốc nhẹ an toàn, không phanh chết máy

            # Reset lại nominal trajectory
            self.init_nominal_trajectory(x_init, x_ref_mod, y_ref_mod, theta_ref_mod, v_ref_adapted)
            return steer_rad, acc_m_s2

    def get_optimized_cost(self):
        return self.last_cost

