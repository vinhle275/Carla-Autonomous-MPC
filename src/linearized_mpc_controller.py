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
    OBSTACLE_TRIGGER_LATERAL_CORRIDOR_M, OBSTACLE_PREFER_BYPASS_SIDE,
    OBSTACLE_APPROACH_RAMP_M, OBSTACLE_APPROACH_MARGIN_M,
    OBSTACLE_HOLD_DISTANCE_M, OBSTACLE_RETURN_RAMP_M,
    ENABLE_ROADBLOCK_EMERGENCY_STOP, ROADBLOCK_SAFE_STOP_DISTANCE_M, ROADBLOCK_DECELERATION_M_S_2,
    OBSTACLE_SPATIAL_CLUSTER_RANGE_M
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
        self.active_chain = {}  # Bộ nhớ chuỗi vật cản liên tiếp (Active Chain Envelope Tracker)

        # Bộ đệm điều khiển dự phòng (Control buffer)
        self.control_buffer = {
            "acceleration": [0.0] * self.horizon,
            "wheel_angle": [0.0] * self.horizon
        }
        self.buffer_index = 0
        self.last_cost = 0.0
        self.is_success = False

        # Trạng thái phong tỏa toàn bộ đường (Total Roadblock Emergency Stop)
        self.is_roadblock = False
        self.roadblock_barrier_dist = None

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
        # 1. Thu thập tất cả các điểm biểu diễn của primary_obs và các vật cản lân cận trong cụm (cách dưới OBSTACLE_SPATIAL_CLUSTER_RANGE_M dọc theo đường)
        all_pts = self.extract_obstacle_points(primary_obs)
        if candidate_obstacles:
            p_long = primary_obs.get('d_long_ego', primary_obs.get('dist', 10.0))
            for c_obs in candidate_obstacles:
                if c_obs.get('id') != primary_obs.get('id'):
                    c_long = c_obs.get('d_long_ego', c_obs.get('dist', p_long))
                    if abs(c_long - p_long) <= OBSTACLE_SPATIAL_CLUSTER_RANGE_M:
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
            A_shift = max(1.5, span_left + 1.0 + OBSTACLE_AVOIDANCE_MARGIN_M)
        else:
            # Né PHẢI: mép vật cản gần làn né nhất là span_right
            A_shift = max(1.5, span_right + 1.0 + OBSTACLE_AVOIDANCE_MARGIN_M)
            
        # Giới hạn A_shift tối đa để vừa đủ thoát cụm vật cản mà không lao ra ngoài lề đường (tối đa 3.60m)
        # A_shift = min(A_shift, 5.0)

        return side_sign, A_shift, span_left, span_right

    def calc_obstacle_weight(self, progress):
        """
        Hàm tính trọng số chuyển làn Cosine mượt mà theo tiến độ dọc:
        - Trước vật cản: Mở góc lái chuyển làn thoai thoải từ 0 -> 1.0
        - Chạy ngang qua vật cản: Giữ vững 100% làn né (1.0)
        - Vượt qua xong: Đánh lái từ từ trả lại tim đường từ 1.0 -> 0.0
        """
        app_start = OBSTACLE_APPROACH_RAMP_M + OBSTACLE_APPROACH_MARGIN_M
        ret_end = OBSTACLE_HOLD_DISTANCE_M + OBSTACLE_RETURN_RAMP_M

        if progress < -OBSTACLE_APPROACH_MARGIN_M:
            dist_before = -progress
            if dist_before <= app_start:
                return 0.5 * (1.0 + math.cos(math.pi * (dist_before - OBSTACLE_APPROACH_MARGIN_M) / OBSTACLE_APPROACH_RAMP_M))
            else:
                return 0.0
        elif -OBSTACLE_APPROACH_MARGIN_M <= progress <= OBSTACLE_HOLD_DISTANCE_M:
            return 1.0
        elif OBSTACLE_HOLD_DISTANCE_M < progress <= ret_end:
            dist_after = progress - OBSTACLE_HOLD_DISTANCE_M
            return 0.5 * (1.0 + math.cos(math.pi * dist_after / OBSTACLE_RETURN_RAMP_M))
        else:
            return 0.0

    def compute_local_reference_shift(self, x_init, x_ref, y_ref, theta_ref, v_ref, detected_obstacles):
        """
        Kỹ thuật Local Reference Shifting dạng Đường Bao Chuỗi Vật Cản (Active Chain Envelope Reference Shifting):
        - Giải quyết triệt để hiện tượng đánh lái loạng choạng khi gặp chuỗi nhiều vật cản liên tiếp.
        - Thay vì chỉ né từng vật cản đơn lẻ rồi cố trả lái giữa các vật cản gần nhau (< 25m),
          hệ thống duy trì một Active Chain Envelope bao phủ toàn bộ chuỗi vật cản:
          + Giữ vững 100% làn né (Hold lane) xuyên suốt từ vật cản đầu tiên đến hết vật cản cuối cùng.
          + Không bao giờ đánh lái quay về tim đường giữa 2 vật cản nếu cự ly giữa chúng không đủ an toàn.
          + Chỉ khi đuôi xe đã vượt qua vật cản CUỐI CÙNG trong chuỗi mới bắt đầu trả lái nhập làn từ từ.
        """
        tx0 = math.cos(theta_ref[0])
        ty0 = math.sin(theta_ref[0])
        nx0 = -ty0
        ny0 = tx0
        x_ego, y_ego = x_init[0], x_init[1]

        # 1. Thu thập toàn bộ vật cản phía trước trên mặt đường
        crit_obs = []
        all_road_obs = []
        if detected_obstacles and len(detected_obstacles) > 0:
            for obs in detected_obstacles:
                d_long_ego = tx0 * (obs['x'] - x_ego) + ty0 * (obs['y'] - y_ego)
                d_lat_road, _, _, _ = self.calculate_lateral_to_road(obs['x'], obs['y'], x_ref, y_ref, theta_ref)
                # Giữ vật cản từ -10.0m sau lưng đến 28.0m phía trước để duy trì đuôi trả làn an toàn
                if -10.0 <= d_long_ego <= 28.0 and abs(d_lat_road) <= 5.5:
                    obs_c = dict(obs)
                    obs_c['d_long_ego'] = d_long_ego
                    obs_c['d_lat_road'] = d_lat_road
                    all_road_obs.append(obs_c)
                    if abs(d_lat_road) <= OBSTACLE_TRIGGER_LATERAL_CORRIDOR_M:
                        crit_obs.append(obs_c)

        # Kiểm tra phong tỏa toàn diện mặt đường (Roadblock Detection):
        # Nếu có vật cản chắn làn xe ta VÀ cả làn bên trái lẫn làn bên phải đều bị chặn:
        self.is_roadblock = False
        self.roadblock_barrier_dist = None

        if ENABLE_ROADBLOCK_EMERGENCY_STOP and crit_obs:
            primary_barrier = min(crit_obs, key=lambda o: o['d_long_ego'])
            p_long = primary_barrier['d_long_ego']
            left_blocked = any(
                o for o in all_road_obs
                if -5.0 <= o['d_lat_road'] <= -1.2 and abs(o['d_long_ego'] - p_long) <= 6.5
            )
            right_blocked = any(
                o for o in all_road_obs
                if 1.2 <= o['d_lat_road'] <= 5.0 and abs(o['d_long_ego'] - p_long) <= 6.5
            )
            if left_blocked and right_blocked:
                self.is_roadblock = True
                self.roadblock_barrier_dist = p_long
                self.active_obstacle = None
                if hasattr(self, 'active_chain'):
                    self.active_chain.clear()
                return np.copy(x_ref), np.copy(y_ref), np.copy(theta_ref), np.copy(v_ref), False

        # 2. Quản lý trạng thái Chuỗi Vật cản Đang Né (Active Chain Tracker)
        if not hasattr(self, 'active_chain') or self.active_chain is None:
            self.active_chain = {}

        # Giải phóng các vật cản mà xe đã hoàn toàn vượt qua và hoàn thành đoạn trả làn
        release_dist = OBSTACLE_HOLD_DISTANCE_M + OBSTACLE_RETURN_RAMP_M + 1.0
        to_remove = []
        for obs_id, obs_data in self.active_chain.items():
            prog_ego = tx0 * (x_ego - obs_data['x']) + ty0 * (y_ego - obs_data['y'])
            if prog_ego > release_dist:
                to_remove.append(obs_id)
        for obs_id in to_remove:
            del self.active_chain[obs_id]

        # Nếu đang có chuỗi né dở dang
        if self.active_chain:
            first_obs = next(iter(self.active_chain.values()))
            prog_ego = tx0 * (x_ego - first_obs['x']) + ty0 * (y_ego - first_obs['y'])

            # CƠ CHẾ QUAN SÁT TỪ XA CÓ KIỂM SOÁT (EARLY FORESIGHT):
            # - Khi xe còn ở xa (prog_ego < -12.0m): Xe vẫn đang chạy thẳng trong làn, chưa bắt đầu bẻ lái né.
            #   Cho phép cập nhật lại hướng né side_sign nếu cảm biến quét thấy thêm vật cản mới ở các làn lân cận.
            # - Khi xe đã vào cự ly tiếp cận (-12.0m <= prog_ego): KHÓA CỨNG side_sign để triệt tiêu hiện tượng giật lắc vô lăng.
            if prog_ego < -12.0 and crit_obs:
                primary = min(crit_obs, key=lambda o: o['d_long_ego'])
                side_sign_updated, A_shift_updated, _, _ = self.analyze_spatial_occupancy(
                    primary, all_road_obs, x_ref, y_ref, theta_ref
                )
                fixed_side = side_sign_updated
                for obs_item in self.active_chain.values():
                    obs_item['side_sign'] = fixed_side
                    obs_item['A_shift'] = max(obs_item['A_shift'], A_shift_updated)
            else:
                fixed_side = first_obs['side_sign']

            # Cập nhật và tự động nối dài chuỗi với bất kỳ vật cản nào mới xuất hiện phía trước trong làn xe
            for o in crit_obs:
                if o['id'] not in self.active_chain and o['d_long_ego'] >= -2.0:
                    _, A_shift_o, _, _ = self.analyze_spatial_occupancy(o, all_road_obs, x_ref, y_ref, theta_ref)
                    self.active_chain[o['id']] = {
                        'id': o['id'],
                        'type': o.get('type', 'vehicle'),
                        'x': o['x'],
                        'y': o['y'],
                        'radius': o.get('radius', 1.0),
                        'r_safe': o.get('radius', 1.0) + OBSTACLE_AVOIDANCE_MARGIN_M,
                        'side_sign': fixed_side,
                        'A_shift': A_shift_o
                    }

        # Nếu chưa có chuỗi né nào, khởi tạo chuỗi mới từ danh sách crit_obs phía trước
        elif crit_obs:
            ahead_obs = [o for o in crit_obs if o['d_long_ego'] >= -1.0]
            if ahead_obs:
                primary = min(ahead_obs, key=lambda o: o['d_long_ego'])
                side_sign, A_shift, _, _ = self.analyze_spatial_occupancy(primary, all_road_obs, x_ref, y_ref, theta_ref)
                self.active_chain[primary['id']] = {
                    'id': primary['id'],
                    'type': primary.get('type', 'vehicle'),
                    'x': primary['x'],
                    'y': primary['y'],
                    'radius': primary.get('radius', 1.0),
                    'r_safe': primary.get('radius', 1.0) + OBSTACLE_AVOIDANCE_MARGIN_M,
                    'side_sign': side_sign,
                    'A_shift': A_shift
                }
                # Thêm tất cả các vật cản kế tiếp trong chuỗi (cách dưới 25m) vào cùng một chuỗi
                for o in ahead_obs:
                    if o['id'] != primary['id'] and o['d_long_ego'] <= 25.0:
                        _, A_shift_o, _, _ = self.analyze_spatial_occupancy(o, all_road_obs, x_ref, y_ref, theta_ref)
                        self.active_chain[o['id']] = {
                            'id': o['id'],
                            'type': o.get('type', 'vehicle'),
                            'x': o['x'],
                            'y': o['y'],
                            'radius': o.get('radius', 1.0),
                            'r_safe': o.get('radius', 1.0) + OBSTACLE_AVOIDANCE_MARGIN_M,
                            'side_sign': side_sign,
                            'A_shift': A_shift_o
                        }

        # Cập nhật self.active_obstacle đại diện (phục vụ bộ điều tốc v_ref và visualization)
        if self.active_chain:
            candidates = list(self.active_chain.values())
            ahead = [o for o in candidates if (tx0 * (o['x'] - x_ego) + ty0 * (o['y'] - y_ego)) >= -1.0]
            if ahead:
                self.active_obstacle = min(ahead, key=lambda o: tx0 * (o['x'] - x_ego) + ty0 * (o['y'] - y_ego))
            else:
                self.active_obstacle = max(candidates, key=lambda o: tx0 * (o['x'] - x_ego) + ty0 * (o['y'] - y_ego))
        else:
            self.active_obstacle = None
            return np.copy(x_ref), np.copy(y_ref), np.copy(theta_ref), np.copy(v_ref), False

        # 3. Tính toán đường bao tham chiếu dời cục bộ (Active Chain Envelope Reference)
        side_sign = self.active_obstacle['side_sign']
        x_ref_mod = np.copy(x_ref)
        y_ref_mod = np.copy(y_ref)
        theta_ref_mod = np.copy(theta_ref)

        chain_list = list(self.active_chain.values())

        for k in range(len(x_ref)):
            th_k = theta_ref[k]
            tx_k = math.cos(th_k)
            ty_k = math.sin(th_k)
            nx_k = -math.sin(th_k)
            ny_k = math.cos(th_k)

            # Tính độ dời bao ngoài (envelope) từ tất cả vật cản trong chain
            shift_k_max = 0.0
            for obs_item in chain_list:
                prog_k = tx_k * (x_ref[k] - obs_item['x']) + ty_k * (y_ref[k] - obs_item['y'])
                w = self.calc_obstacle_weight(prog_k)
                shift_val = obs_item['A_shift'] * w
                if shift_val > shift_k_max:
                    shift_k_max = shift_val

            total_shift = side_sign * shift_k_max
            x_ref_mod[k] = x_ref[k] + total_shift * nx_k
            y_ref_mod[k] = y_ref[k] + total_shift * ny_k

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

    def solve_qp_iteration(self, x_init, x_ref, y_ref, theta_ref, v_ref, obstacles=None, is_roadblock=False):
        """
        Xây dựng và giải bài toán Quy hoạch Toàn phương (QP) tại một vòng lặp tuyến tính hóa:
        - Khống chế cận dưới vận tốc (Anti-Stall) triệt tiêu hoàn toàn Freezing Robot.
        - Khi gặp Roadblock: Cho phép min_v = 0.0 để xe phanh dừng khẩn cấp an toàn.
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

        # Cận dưới vận tốc: Cho phép về 0 khi phanh dừng trước Roadblock; bình thường giữ Anti-Stall
        if is_roadblock:
            min_v = 0.0
        else:
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
                    # (x_o, y_o) đã là điểm bề mặt ngoài cùng của vật cản từ Detector.
                    # Khoảng cách từ tâm xe ta tới mép vật cản cần >= (nửa bề rộng xe ta ~1.0m + khoảng đệm an toàn)
                    r_safe = 1.0 + OBSTACLE_AVOIDANCE_MARGIN_M

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
        # 2. XỬ LÝ PHANH DỪNG KHẨN CẤP AN TOÀN KHI GẶP ROADBLOCK (SAFE EMERGENCY STOP)
        # =====================================================================
        if self.is_roadblock and self.roadblock_barrier_dist is not None:
            d_to_barrier = self.roadblock_barrier_dist
            d_stop = max(0.0, d_to_barrier - ROADBLOCK_SAFE_STOP_DISTANCE_M)
            curr_v = float(x_init[3])

            # Nếu xe đã tới gần điểm dừng an toàn (d_stop <= 0.25m) hoặc đã dừng hẳn (curr_v < 0.25 m/s)
            if d_stop <= 0.25 or (curr_v < 0.25 and d_to_barrier <= ROADBLOCK_SAFE_STOP_DISTANCE_M + 1.5):
                self.is_success = True
                self.init_nominal_trajectory(x_init, x_ref_mod, y_ref_mod, theta_ref_mod, np.zeros_like(v_ref))
                # Phanh cứng để xe đứng yên vững vàng, bánh lái thẳng 0 độ
                return 0.0, -3.5

            # Tạo profile giảm tốc mượt mà dừng đúng điểm d_stop cách hàng rào ROADBLOCK_SAFE_STOP_DISTANCE_M
            v_ref_adapted = np.zeros_like(v_ref)
            for k in range(len(v_ref)):
                d_stop_k = max(0.0, d_stop - curr_v * k * self.dt)
                v_target_k = math.sqrt(2.0 * ROADBLOCK_DECELERATION_M_S_2 * d_stop_k)
                v_ref_adapted[k] = min(float(v_ref[k]), v_target_k)

            if self.x_nom is None:
                self.init_nominal_trajectory(x_init, x_ref_mod, y_ref_mod, theta_ref_mod, v_ref_adapted)

            try:
                x_opt, u_opt, cost_val = self.solve_qp_iteration(
                    x_init, x_ref_mod, y_ref_mod, theta_ref_mod, v_ref_adapted, obstacles=None, is_roadblock=True
                )
                self.is_success = True
                self.last_cost = cost_val
                steer_rad = float(u_opt[0, 0])
                acc_m_s2 = float(u_opt[1, 0])

                # Warm-start
                self.x_nom = x_opt
                self.u_nom = u_opt
                return steer_rad, acc_m_s2
            except Exception:
                return 0.0, -ROADBLOCK_DECELERATION_M_S_2

        # =====================================================================
        # 3. BỘ THÍCH ỨNG TỐC ĐỘ THEO TRẠNG THÁI VƯỢT VẬT CẢN (SPEED GOVERNOR)
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

            if progress_ego < -OBSTACLE_APPROACH_MARGIN_M:
                # Đang tiếp cận: giảm dần từ TARGET_SPEED_MS về OBSTACLE_CAUTION_SPEED_MS
                dist_to_obs = -progress_ego
                ratio = np.clip((dist_to_obs - 4.0) / (OBSTACLE_SLOWDOWN_DISTANCE_M - 4.0), 0.0, 1.0)
                base_speed = max(float(v_ref[0]), TARGET_SPEED_MS)
                v_safe = OBSTACLE_CAUTION_SPEED_MS + ratio * (base_speed - OBSTACLE_CAUTION_SPEED_MS)
                v_ref_adapted = np.minimum(v_ref_adapted, v_safe)
                v_ref_adapted = np.maximum(v_ref_adapted, OBSTACLE_CAUTION_SPEED_MS)
            elif -OBSTACLE_APPROACH_MARGIN_M <= progress_ego <= 2.0:
                # Đang chạy ngang qua vật cản: duy trì vận tốc an toàn ổn định (OBSTACLE_CAUTION_SPEED_MS)
                v_ref_adapted = np.minimum(v_ref_adapted, OBSTACLE_CAUTION_SPEED_MS)
                v_ref_adapted = np.maximum(v_ref_adapted, OBSTACLE_CAUTION_SPEED_MS)
            else:
                # Ngay khi mũi xe qua vật cản: tăng tốc mượt mà từ OBSTACLE_CAUTION_SPEED_MS lên TARGET_SPEED_MS
                return_dist = progress_ego - 2.0
                ratio = np.clip(return_dist / OBSTACLE_RETURN_RAMP_M, 0.0, 1.0)
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

