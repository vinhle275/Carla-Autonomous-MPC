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

        # Quỹ đạo tim đường toàn cục để tra cứu chính xác vị trí tim đường tại các khúc cua
        self.x_traj = None
        self.y_traj = None
        self.theta_traj = None

    def set_global_trajectory(self, x_traj, y_traj, theta_traj):
        """Thiết lập quỹ đạo toàn cục để định vị tim đường chuẩn xác, không bị méo góc khi vào cua"""
        self.x_traj = np.array(x_traj)
        self.y_traj = np.array(y_traj)
        self.theta_traj = np.array(theta_traj)

    def calculate_lateral_to_road(self, x_obs, y_obs, x_ref=None, y_ref=None, theta_ref=None):
        """
        Tính khoảng cách vuông góc d_lat_road từ vật cản (x_obs, y_obs) đến tim đường:
        - Sử dụng quỹ đạo toàn cục self.x_traj nếu có, hoặc đoạn chân trời x_ref.
        - Trả về: (d_lat_road, nx, ny, idx_match)
          + d_lat_road > 0: Vật cản nằm bên PHẢI tim đường (+Y trong hệ tọa độ CARLA).
          + d_lat_road < 0: Vật cản nằm bên TRÁI tim đường (-Y trong hệ tọa độ CARLA).
        """
        if self.x_traj is not None and len(self.x_traj) > 0:
            dists_sq = (self.x_traj - x_obs) ** 2 + (self.y_traj - y_obs) ** 2
            idx = int(np.argmin(dists_sq))
            xr = self.x_traj[idx]
            yr = self.y_traj[idx]
            th = self.theta_traj[idx]
        elif x_ref is not None and len(x_ref) > 0:
            dists_sq = (x_ref - x_obs) ** 2 + (y_ref - y_obs) ** 2
            idx = int(np.argmin(dists_sq))
            xr = x_ref[idx]
            yr = y_ref[idx]
            th = theta_ref[idx]
        else:
            return 0.0, 0.0, 1.0, 0

        nx = -math.sin(th)
        ny = math.cos(th)
        d_lat_road = nx * (x_obs - xr) + ny * (y_obs - yr)
        return d_lat_road, nx, ny, idx

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
        curr_v = float(x_init[3])
        for k in range(self.horizon):
            self.x_nom[0, k + 1] = x_ref[k]
            self.x_nom[1, k + 1] = y_ref[k]
            self.x_nom[2, k + 1] = theta_ref[k]
            # Mượt hóa danh định tốc độ (tăng tốc dần từ vận tốc hiện tại, tránh sốc gia tốc)
            curr_v = min(curr_v + 3.0 * self.dt, float(v_ref[k]))
            self.x_nom[3, k + 1] = max(curr_v, float(v_ref[k]) if x_init[3] > float(v_ref[k]) else curr_v)
            self.u_nom[0, k] = 0.0
            self.u_nom[1, k] = 0.0

    @staticmethod
    def extract_obstacle_points(obs):
        """
        Trích xuất tập hợp các điểm biểu diễn hình thể/bề mặt của vật cản (Sample Points / Point Cloud):
        - Hỗ trợ cả điểm tiếp xúc bề mặt gần nhất, các điểm góc của hộp bao OBB (trong mô phỏng),
          lẫn mảng điểm quét từ LiDAR 2D Point Cloud / Siêu âm (khi triển khai trên xe RC giai đoạn 2).
        """
        pts = []
        if 'x' in obs and 'y' in obs:
            pts.append((obs['x'], obs['y']))

        # Nếu có thông tin Bounding Box (OBB) từ Detector
        if 'center_x' in obs and 'extent_x' in obs:
            cx = obs['center_x']
            cy = obs['center_y']
            hl = obs.get('extent_x', 0.6)
            hw = obs.get('extent_y', 0.6)
            yaw = obs.get('box_yaw', 0.0)
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)

            pts.append((cx, cy))
            for dx, dy in [(-hl, -hw), (-hl, hw), (hl, -hw), (hl, hw), (0.0, -hw), (0.0, hw)]:
                pts.append((
                    cx + dx * cos_y - dy * sin_y,
                    cy + dx * sin_y + dy * cos_y
                ))

        # Nếu có danh sách điểm bề mặt quét từ LiDAR Point Cloud (cho xe RC)
        if 'surface_points' in obs and obs['surface_points']:
            pts.extend(obs['surface_points'])

        return pts

    def analyze_spatial_occupancy(self, primary_obs, candidate_obstacles, x_ref, y_ref, theta_ref):
        """
        Thuật toán Phân tích Độ chiếm dụng Không gian & Khoảng trống Khả dụng (Spatial Occupancy & Clearance Analysis):
        - Giải quyết triệt để edge case: vật cản có 1 phần nhỏ chớm sang một bên tim đường,
          nhưng phần lớn khối tích/đuôi xe hoặc các vật cản kế tiếp trong cụm lại nằm dạt sang bên còn lại.
        - Trả về: (side_sign, A_shift, span_left, span_right)
        """
        # 1. Thu thập tất cả các điểm biểu diễn của primary_obs và các vật cản lân cận trong cụm (cách dưới 8.5m)
        all_pts = self.extract_obstacle_points(primary_obs)
        if candidate_obstacles:
            p_dist = primary_obs.get('dist', 10.0)
            for c_obs in candidate_obstacles:
                if c_obs.get('id') != primary_obs.get('id'):
                    if abs(c_obs.get('dist', p_dist) - p_dist) <= 8.5:
                        all_pts.extend(self.extract_obstacle_points(c_obs))

        # 2. Chiếu tất cả các điểm lên tim đường để xác định độ chiếm dụng không gian hai bên làn
        d_lats = []
        for px, py in all_pts:
            d_lat, _, _, _ = self.calculate_lateral_to_road(px, py, x_ref, y_ref, theta_ref)
            d_lats.append(d_lat)

        if not d_lats:
            d_lats = [0.0]

        d_lat_min = min(d_lats)  # Mép lấn sang TRÁI nhất (< 0 nếu sang trái)
        d_lat_max = max(d_lats)  # Mép lấn sang PHẢI nhất (> 0 nếu sang phải)
        d_lat_mean = float(np.mean(d_lats))

        # Độ lấn sâu của bề mặt sang hai bên tim đường
        span_left = max(0.0, -d_lat_min)
        span_right = max(0.0, d_lat_max)

        # 3. Tiêu chí phân luồng tối ưu (Dominant Clearance Rule):
        # CARLA / Road Frame:
        # - d_lat > 0: Bên PHẢI tim đường -> Xe né sang TRÁI (-nx, -ny) -> side_sign = -1.0
        # - d_lat < 0: Bên TRÁI tim đường -> Xe né sang PHẢI (+nx, +ny) -> side_sign = +1.0
        bias = span_right - span_left
        if bias > 0.20:
            # Chiếm dụng bên PHẢI lớn hơn hẳn bên TRÁI -> Làn TRÁI thông thoáng -> Né TRÁI
            side_sign = -1.0
        elif bias < -0.20:
            # Chiếm dụng bên TRÁI lớn hơn hẳn bên PHẢI -> Làn PHẢI thông thoáng -> Né PHẢI
            side_sign = 1.0
        else:
            # Cân bằng hoặc đối xứng quanh tim đường -> Dùng trọng tâm d_lat_mean hoặc cấu hình ưu tiên
            if d_lat_mean > 0.15:
                side_sign = -1.0
            elif d_lat_mean < -0.15:
                side_sign = 1.0
            else:
                side_sign = -1.0 if OBSTACLE_PREFER_BYPASS_SIDE == 'left' else 1.0

        # 4. Tính toán biên độ dời làn thích ứng (Adaptive Clearance A_shift):
        # Đảm bảo thân xe (nửa rộng ~1.0m + lề an toàn) cách mép ngoài cùng của vật cản một khoảng an toàn
        if side_sign < 0:
            # Né TRÁI: mép vật cản gần làn né nhất là span_left
            A_shift = max(1.95, span_left + 1.0 + OBSTACLE_AVOIDANCE_MARGIN_M)
        else:
            # Né PHẢI: mép vật cản gần làn né nhất là span_right
            A_shift = max(1.95, span_right + 1.0 + OBSTACLE_AVOIDANCE_MARGIN_M)

        # Giới hạn A_shift tối đa để vừa đủ thoát cụm vật cản mà không lao ra ngoài lề đường (tối đa 3.60m)
        A_shift = min(A_shift, 4.0)

        return side_sign, A_shift, span_left, span_right

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
        nx0 = -ty0
        ny0 = tx0
        x_ego, y_ego = x_init[0], x_init[1]

        # 1. Thu thập và phân loại vật cản nguy hiểm từ cảm biến (hành lang mở rộng 4.2m, tầm xa 26.0m)
        crit_obs = []
        if detected_obstacles and len(detected_obstacles) > 0:
            for obs in detected_obstacles:
                d_long_ego = tx0 * (obs['x'] - x_ego) + ty0 * (obs['y'] - y_ego)
                d_lat_road, _, _, _ = self.calculate_lateral_to_road(obs['x'], obs['y'], x_ref, y_ref, theta_ref)
                if 0.0 <= d_long_ego <= 26.0 and abs(d_lat_road) <= 4.2:
                    obs_c = dict(obs)
                    obs_c['d_long_ego'] = d_long_ego
                    obs_c['d_lat_road'] = d_lat_road
                    crit_obs.append(obs_c)

        # 2. Quản lý trạng thái vật cản đang né hiện tại (nếu có)
        if self.active_obstacle is not None:
            x_o = self.active_obstacle['x']
            y_o = self.active_obstacle['y']
            # Tiến độ dọc của xe so với vật cản
            if self.x_traj is not None and len(self.x_traj) > 0:
                _, _, _, idx_ego = self.calculate_lateral_to_road(x_ego, y_ego)
                _, _, _, idx_o = self.calculate_lateral_to_road(x_o, y_o)
                progress_ego = (idx_ego - idx_o) * 0.1
            else:
                progress_ego = tx0 * (x_ego - x_o) + ty0 * (y_ego - y_o)

            # Cơ chế cập nhật từ xa có kiểm soát (Early Foresight & Latched Direction):
            # - Khi xe còn ở xa (progress_ego < -12.0m): Cho phép cập nhật lại side_sign nếu cảm biến thấy thêm vật cản mới
            # - Khi xe đã vào cự ly tiếp cận (-12.0m <= progress_ego < -2.0m): KHÓA CỨNG side_sign để triệt tiêu hiện tượng giật vô lăng;
            #   chỉ mở rộng biên độ né A_shift nếu phát hiện vật cản phình rộng
            if progress_ego < -12.0 and crit_obs:
                primary = min(crit_obs, key=lambda o: o['dist'])
                side_sign, A_shift, _, _ = self.analyze_spatial_occupancy(primary, crit_obs, x_ref, y_ref, theta_ref)
                self.active_obstacle['side_sign'] = side_sign
                self.active_obstacle['A_shift'] = max(A_shift, self.active_obstacle['A_shift'])
                self.active_obstacle['x'] = primary['x']
                self.active_obstacle['y'] = primary['y']
            elif progress_ego < -2.0 and crit_obs:
                primary = min(crit_obs, key=lambda o: o['dist'])
                _, A_shift, _, _ = self.analyze_spatial_occupancy(primary, crit_obs, x_ref, y_ref, theta_ref)
                self.active_obstacle['A_shift'] = max(A_shift, self.active_obstacle['A_shift'])

            # Chaining cho cụm nhiều vật cản gần nhau (Obstacle Cluster Chaining):
            # Khi xe bắt đầu vượt qua vật cản hiện tại (progress_ego >= -0.5m), chuyển giao sang vật kế tiếp trong cụm
            # đồng thời DUY TRÌ HƯỚNG NÉ preserved_side để xe không bị đánh võng giữa cụm
            if progress_ego >= -0.5 and crit_obs:
                next_cluster_obs = [
                    o for o in crit_obs
                    if o.get('id') != self.active_obstacle.get('id') and o.get('d_long_ego', 1.0) >= 0.5
                ]
                if next_cluster_obs:
                    next_obs = min(next_cluster_obs, key=lambda o: o['dist'])
                    preserved_side = self.active_obstacle['side_sign']

                    _, new_A_shift, _, _ = self.analyze_spatial_occupancy(
                        next_obs, next_cluster_obs, x_ref, y_ref, theta_ref
                    )
                    new_A_shift = max(new_A_shift, self.active_obstacle['A_shift'])
                    r_safe = next_obs['radius'] + OBSTACLE_AVOIDANCE_MARGIN_M

                    self.active_obstacle = {
                        'id': next_obs.get('id'),
                        'type': next_obs.get('type'),
                        'x': next_obs['x'],
                        'y': next_obs['y'],
                        'radius': next_obs['radius'],
                        'r_safe': r_safe,
                        'side_sign': preserved_side,
                        'A_shift': new_A_shift
                    }
                    x_o = self.active_obstacle['x']
                    y_o = self.active_obstacle['y']
                    if self.x_traj is not None and len(self.x_traj) > 0:
                        _, _, _, idx_o = self.calculate_lateral_to_road(x_o, y_o)
                        progress_ego = (idx_ego - idx_o) * 0.1
                    else:
                        progress_ego = tx0 * (x_ego - x_o) + ty0 * (y_ego - y_o)

            # Nếu xe đã vượt qua vật cản hơn 11m và đã nhập làn xong -> Giải phóng trạng thái né
            if progress_ego > 11.0:
                self.active_obstacle = None
            else:
                if progress_ego < 0.0 and detected_obstacles:
                    for obs in detected_obstacles:
                        if obs.get('id') == self.active_obstacle.get('id'):
                            self.active_obstacle['x'] = obs['x']
                            self.active_obstacle['y'] = obs['y']
                            break

        # 3. Nếu chưa có vật cản nào đang né, kích hoạt từ danh sách crit_obs
        if self.active_obstacle is None and crit_obs:
            primary_obs = min(crit_obs, key=lambda o: o['dist'])
            side_sign, A_shift, span_left, span_right = self.analyze_spatial_occupancy(
                primary_obs, crit_obs, x_ref, y_ref, theta_ref
            )
            r_safe = primary_obs['radius'] + OBSTACLE_AVOIDANCE_MARGIN_M
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

        # 4. Nếu không có vật cản nào đang kích hoạt -> Trả về tham chiếu gốc
        if self.active_obstacle is None:
            return np.copy(x_ref), np.copy(y_ref), np.copy(theta_ref), np.copy(v_ref), False

        # 5. Tính toán quỹ đạo uốn lượn S-curve cho cả đoạn chân trời N bước
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
            progress_k = tx_k * (x_ref[k] - x_o) + ty_k * (y_ref[k] - y_o)

            if progress_k < -2.0:
                # Giai đoạn tiếp cận: mở góc lái dứt khoát trong 11m (từ -13m đến -2m)
                dist_before = -progress_k
                if dist_before <= 13.0:
                    w = 0.5 * (1.0 + math.cos(math.pi * (dist_before - 2.0) / 11.0))
                else:
                    w = 0.0
            elif -2.0 <= progress_k <= 4.0:
                # Giai đoạn chạy ngang qua vật cản: GIỮ NGUYÊN HOÀN TOÀN LÀN NÉ (thoát hẳn đuôi xe)
                w = 1.0
            elif 4.0 < progress_k <= 10.5:
                # Giai đoạn vượt qua xong: đánh lái từ từ trả lại tim đường (trong quãng đường 6.5m gọn gàng, dứt khoát)
                dist_after = progress_k - 4.0
                w = 0.5 * (1.0 + math.cos(math.pi * dist_after / 6.5))
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
            'ipopt.max_cpu_time': 1.5,
            'ipopt.max_iter': 40,
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
            # Chỉ áp dụng siêu phẳng an toàn cho các vật cản ở phía trước hoặc ngang xe (d_long >= -0.8m)
            # Loại bỏ các vật cản đã đi qua sau đuôi xe để tránh siêu phẳng ngược chiều gây xung đột
            tx_ego = math.cos(x_init[2])
            ty_ego = math.sin(x_init[2])
            ahead_obs = [
                obs for obs in obstacles
                if (tx_ego * (obs['x'] - x_init[0]) + ty_ego * (obs['y'] - x_init[1])) >= -0.8
            ]
            active_obs = ahead_obs[:2]
            num_obs = len(active_obs)

            if num_obs > 0:
                # Khởi tạo ma trận biến bù Slack Variable
                eps_slack = opti.variable(num_obs, self.horizon)
                opti.subject_to(ca.vec(eps_slack) >= 0)

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

        try:
            sol = opti.solve()
            x_opt = sol.value(X)
            u_opt = sol.value(U)
            cost_opt = float(sol.value(cost))
        except Exception:
            # Thu hồi nghiệm tốt nhất (Best iterate) từ Ipopt thay vì để chương trình bị ngắt khựng xe
            x_opt = opti.debug.value(X)
            u_opt = opti.debug.value(U)
            cost_opt = float(opti.debug.value(cost))

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
            if self.x_traj is not None and len(self.x_traj) > 0:
                _, _, _, idx_ego = self.calculate_lateral_to_road(x_init[0], x_init[1])
                _, _, _, idx_o = self.calculate_lateral_to_road(self.active_obstacle['x'], self.active_obstacle['y'])
                progress_ego = (idx_ego - idx_o) * 0.1
            else:
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
            elif -2.0 <= progress_ego <= 2.0:
                # Đang chạy ngang qua vật cản: duy trì vận tốc an toàn ổn định (6.5 m/s)
                v_ref_adapted = np.minimum(v_ref_adapted, OBSTACLE_CAUTION_SPEED_MS)
                v_ref_adapted = np.maximum(v_ref_adapted, OBSTACLE_CAUTION_SPEED_MS)
            else:
                # Ngay khi mũi xe qua vật cản: tăng tốc mượt mà từ OBSTACLE_CAUTION_SPEED_MS lên TARGET_SPEED_MS
                return_dist = progress_ego - 2.0
                ratio = np.clip(return_dist / 6.5, 0.0, 1.0)
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
        if self.x_nom is None:
            self.init_nominal_trajectory(x_init, x_ref_mod, y_ref_mod, theta_ref_mod, v_ref_adapted)
        else:
            max_dev = np.max(np.hypot(self.x_nom[0, 1:] - x_ref_mod, self.x_nom[1, 1:] - y_ref_mod))
            if has_shift and max_dev > 1.2:
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
                acc_m_s2 = 0.5  # Duy trì gia tốc dương nhẹ để xe tiếp tục lăn bánh vượt qua, không phanh chết máy

            # Reset lại nominal trajectory
            self.init_nominal_trajectory(x_init, x_ref_mod, y_ref_mod, theta_ref_mod, v_ref_adapted)
            return steer_rad, acc_m_s2

    def get_optimized_cost(self):
        return self.last_cost

