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
    OBSTACLE_SPATIAL_CLUSTER_RANGE_M,
    DEFAULT_ROAD_HALF_WIDTH_M, MAX_LATERAL_ACCEL_M_S_2,
    EGO_FRONT_OVERHANG_M, EGO_REAR_OVERHANG_M, EGO_HALF_WIDTH_M
)
from src.help_functions import calculate_lateral_deviation, normalize_angle


class LinearizedMpcController:
    """
    Bộ điều khiển MPC Tuyến tính hóa lặp (Successive Linearization QP-MPC):
    - Đồng bộ chính xác chỉ số thời gian X[:, k] <-> x_ref[k] (triệt tiêu dao động hình sin).
    - Đường bao ngoài chuẩn (True Outer Envelope) cho cụm vật cản cùng phía và chuyển tiếp mượt cho Slalom.
    - Ngắt siêu phẳng đẩy ngang ngay khi xe bước vào pha trả làn (Return-to-Lane), giúp nhập làn êm, không lắc.
    - Duy trì liên tục quỹ đạo Warm-start x_nom, không reset đột ngột giữa chừng.
    """
    def __init__(self, horizon=N, sample_time=dt):
        self.horizon = horizon
        self.dt = sample_time
        self.L = L

        self.x_nom = None  # Shape: (4, horizon + 1)
        self.u_nom = None  # Shape: (2, horizon)
        self.u_prev = np.zeros(2)

        self.x_ref_mod = None
        self.y_ref_mod = None

        self.active_obstacle = None
        self.active_chain = {}

        self.control_buffer = {
            "acceleration": [0.0] * self.horizon,
            "wheel_angle": [0.0] * self.horizon
        }
        self.buffer_index = 0
        self.last_cost = 0.0
        self.is_success = False

        self.is_roadblock = False
        self.roadblock_barrier_dist = None

        # Quỹ đạo toàn cục & độ dài cung Frenet
        self.x_traj = None
        self.y_traj = None
        self.theta_traj = None
        self.s_traj = None
        self.is_closed_loop = False
        self.current_traj_idx = 0

    def set_global_trajectory(self, x_traj, y_traj, theta_traj):
        """Thiết lập quỹ đạo toàn cục và tính trước độ dài cung tích lũy s_traj"""
        self.x_traj = np.array(x_traj, dtype=float)
        self.y_traj = np.array(y_traj, dtype=float)
        self.theta_traj = np.array(theta_traj, dtype=float)
        if len(self.x_traj) > 1:
            dx = np.diff(self.x_traj)
            dy = np.diff(self.y_traj)
            self.s_traj = np.concatenate(([0.0], np.cumsum(np.hypot(dx, dy))))
            self.is_closed_loop = (
                math.hypot(self.x_traj[-1] - self.x_traj[0], self.y_traj[-1] - self.y_traj[0]) < 2.0
            )
        else:
            self.s_traj = np.zeros(len(self.x_traj))
            self.is_closed_loop = False

    def project_to_frenet(self, x, y, hint_idx=None, window_back=200, window_fwd=450):
        """
        Chiếu tọa độ (x, y) lên quỹ đạo toàn cục trong cửa sổ cục bộ quanh hint_idx:
        - Tránh nhảy nhầm nhánh đường trên quỹ đạo hình số 8, chữ U hoặc đường vòng khép kín.
        - Trả về: (s_proj, d_proj, nx, ny, best_idx)
        """
        if self.x_traj is None or len(self.x_traj) == 0:
            return 0.0, 0.0, 0.0, 1.0, 0

        n_pts = len(self.x_traj)
        if hint_idx is None:
            hint_idx = self.current_traj_idx

        start_i = max(0, hint_idx - window_back)
        end_i = min(n_pts, hint_idx + window_fwd)
        if end_i <= start_i:
            start_i, end_i = 0, n_pts

        sub_x = self.x_traj[start_i:end_i]
        sub_y = self.y_traj[start_i:end_i]
        dists_sq = (sub_x - x) ** 2 + (sub_y - y) ** 2
        best_idx = start_i + int(np.argmin(dists_sq))

        xr = self.x_traj[best_idx]
        yr = self.y_traj[best_idx]
        th = self.theta_traj[best_idx]
        tx = math.cos(th)
        ty = math.sin(th)
        nx = -ty
        ny = tx

        s_proj = float(self.s_traj[best_idx]) + tx * (x - xr) + ty * (y - yr)
        d_proj = nx * (x - xr) + ny * (y - yr)
        return s_proj, d_proj, nx, ny, best_idx

    def calculate_lateral_to_road(self, x_obs, y_obs, x_ref=None, y_ref=None, theta_ref=None):
        """Tương thích ngược: trả về (d_lat_road, nx, ny, idx)"""
        if self.x_traj is not None and len(self.x_traj) > 0:
            _, d_lat, nx, ny, idx = self.project_to_frenet(x_obs, y_obs, hint_idx=self.current_traj_idx)
            return d_lat, nx, ny, idx
        elif x_ref is not None and len(x_ref) > 0:
            dists_sq = (x_ref - x_obs) ** 2 + (y_ref - y_obs) ** 2
            idx = int(np.argmin(dists_sq))
            xr, yr, th = x_ref[idx], y_ref[idx], theta_ref[idx]
            nx, ny = -math.sin(th), math.cos(th)
            d_lat = nx * (x_obs - xr) + ny * (y_obs - yr)
            return d_lat, nx, ny, idx
        return 0.0, 0.0, 1.0, 0

    def _eval_discrete_dynamics(self, x, u):
        pos_x, pos_y, theta, v = x[0], x[1], x[2], x[3]
        delta, a = u[0], u[1]
        next_x = pos_x + v * np.cos(theta) * self.dt
        next_y = pos_y + v * np.sin(theta) * self.dt
        next_theta = theta + (v / self.L) * np.tan(delta) * self.dt
        next_v = v + a * self.dt
        return np.array([next_x, next_y, next_theta, next_v])

    def get_linearized_matrices(self, x_bar, u_bar):
        pos_x, pos_y, theta, v = x_bar[0], x_bar[1], x_bar[2], x_bar[3]
        delta, a = u_bar[0], u_bar[1]

        A = np.eye(4)
        A[0, 2] = -v * np.sin(theta) * self.dt
        A[0, 3] = np.cos(theta) * self.dt
        A[1, 2] = v * np.cos(theta) * self.dt
        A[1, 3] = np.sin(theta) * self.dt
        A[2, 3] = (np.tan(delta) / self.L) * self.dt

        B = np.zeros((4, 2))
        cos_delta = np.cos(delta)
        sec_delta_sq = 1.0 / (cos_delta ** 2 + 1e-6)
        B[2, 0] = (v / self.L) * sec_delta_sq * self.dt
        B[3, 1] = self.dt

        x_next_exact = self._eval_discrete_dynamics(x_bar, u_bar)
        d = x_next_exact - (A @ x_bar + B @ u_bar)
        return A, B, d

    def init_nominal_trajectory(self, x_init, x_ref, y_ref, theta_ref, v_ref):
        """Khởi tạo quỹ đạo danh định êm ái từ tham chiếu khi mới bắt đầu"""
        self.x_nom = np.zeros((4, self.horizon + 1))
        self.u_nom = np.zeros((2, self.horizon))

        self.x_nom[:, 0] = x_init
        curr_v = float(x_init[3])
        for k in range(self.horizon):
            self.x_nom[0, k + 1] = x_ref[k]
            self.x_nom[1, k + 1] = y_ref[k]
            self.x_nom[2, k + 1] = theta_ref[k]
            curr_v = min(curr_v + 3.0 * self.dt, float(v_ref[k]))
            self.x_nom[3, k + 1] = max(curr_v, float(v_ref[k]) if x_init[3] > float(v_ref[k]) else curr_v)
            self.u_nom[0, k] = 0.0
            self.u_nom[1, k] = 0.0

    @staticmethod
    def extract_obstacle_points(obs):
        pts = []
        if 'x' in obs and 'y' in obs:
            pts.append((obs['x'], obs['y']))
        if 'center_x' in obs and 'extent_x' in obs:
            cx, cy = obs['center_x'], obs['center_y']
            hl = obs.get('extent_x', 0.6)
            hw = obs.get('extent_y', 0.6)
            yaw = obs.get('box_yaw', 0.0)
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            pts.append((cx, cy))
            for dx, dy in [(-hl, -hw), (-hl, hw), (hl, -hw), (hl, hw), (0.0, -hw), (0.0, hw), (-hl, 0.0), (hl, 0.0)]:
                pts.append((cx + dx * cos_y - dy * sin_y, cy + dx * sin_y + dy * cos_y))
        if 'surface_points' in obs and obs['surface_points']:
            pts.extend(obs['surface_points'])
        return pts

    def compute_obstacle_frenet_bounds(self, obs):
        """Tính khoảng chiếm dụng 2D [s_rear, s_front] x [d_min, d_max] của toàn bộ OBB vật cản"""
        pts = self.extract_obstacle_points(obs)
        s_vals, d_vals = [], []
        for px, py in pts:
            s_p, d_p, _, _, _ = self.project_to_frenet(px, py, hint_idx=self.current_traj_idx)
            s_vals.append(s_p)
            d_vals.append(d_p)
        if not s_vals:
            return 0.0, 0.0, 0.0, 0.0
        return float(min(s_vals)), float(max(s_vals)), float(min(d_vals)), float(max(d_vals))

    @staticmethod
    def find_free_gaps_in_corridor(blocked_intervals, road_min=-DEFAULT_ROAD_HALF_WIDTH_M, road_max=DEFAULT_ROAD_HALF_WIDTH_M, min_gap_width=0.30):
        """Tìm danh sách các khe trống khả dụng [g_low, g_high] cho tâm xe Ego trên mặt cắt ngang"""
        if not blocked_intervals:
            return [(road_min, road_max)]
        sorted_iv = sorted(blocked_intervals, key=lambda iv: iv[0])
        merged = []
        for low, high in sorted_iv:
            if not merged or low > merged[-1][1]:
                merged.append([low, high])
            else:
                merged[-1][1] = max(merged[-1][1], high)

        free_gaps = []
        cursor = road_min
        for b_low, b_high in merged:
            if b_low > cursor:
                free_gaps.append((cursor, min(b_low, road_max)))
            cursor = max(cursor, b_high)
            if cursor >= road_max:
                break
        if cursor < road_max:
            free_gaps.append((cursor, road_max))

        return [(g_l, g_h) for g_l, g_h in free_gaps if (g_h - g_l) >= min_gap_width]

    @staticmethod
    def select_optimal_shift_from_gaps(valid_gaps, preferred_sign=None):
        """
        Chọn vị trí dịch tâm xe d_target trong khe trống gần tim đường nhất,
        đảm bảo biên độ dịch tối thiểu 1.5m khi né để dứt khoát thoát khỏi vùng siêu phẳng của vật cản.
        """
        best_d = None
        best_cost = float('inf')
        best_gap = None
        for g_low, g_high in valid_gaps:
            pad = min(0.20, 0.25 * (g_high - g_low))
            d_cand = float(np.clip(0.0, g_low + pad, g_high - pad))
            if 0.0 < abs(d_cand) < 1.5:
                # Mở rộng nhẹ về phía khe trống để tâm xe cách đủ xa siêu phẳng của vật cản
                if d_cand > 0:
                    d_cand = min(1.5, g_high - pad)
                else:
                    d_cand = max(-1.5, g_low + pad)

            cost = abs(d_cand)
            if preferred_sign is not None and d_cand != 0.0 and (d_cand * preferred_sign < 0):
                cost += 0.60
            elif preferred_sign is None and abs(d_cand) > 0.0:
                pref_dir = -1.0 if OBSTACLE_PREFER_BYPASS_SIDE == 'left' else 1.0
                if d_cand * pref_dir < 0:
                    cost += 0.10
            if cost < best_cost:
                best_cost = cost
                best_d = d_cand
                best_gap = (g_low, g_high)
        return best_d, best_gap

    def calc_span_obstacle_weight(self, s_query, s_rear, s_front):
        """
        Hàm trọng số S-Curve mượt mà theo khoảng dọc [s_rear, s_front]:
        - Mở lái hoàn tất trước mũi xe: s_hold_start = s_rear - EGO_FRONT_OVERHANG_M - OBSTACLE_APPROACH_MARGIN_M
        - Giữ làn né 100% đến khi thoát thân xe: s_hold_end = s_front + OBSTACLE_HOLD_DISTANCE_M
        """
        s_hold_start = s_rear - EGO_FRONT_OVERHANG_M - OBSTACLE_APPROACH_MARGIN_M
        s_app_start = s_hold_start - OBSTACLE_APPROACH_RAMP_M
        s_hold_end = s_front + OBSTACLE_HOLD_DISTANCE_M
        s_ret_end = s_hold_end + OBSTACLE_RETURN_RAMP_M

        if s_query < s_app_start:
            return 0.0
        elif s_query < s_hold_start:
            u = (s_query - s_app_start) / max(OBSTACLE_APPROACH_RAMP_M, 1.0)
            return 0.5 * (1.0 - math.cos(math.pi * u))
        elif s_query <= s_hold_end:
            return 1.0
        elif s_query <= s_ret_end:
            u = (s_query - s_hold_end) / max(OBSTACLE_RETURN_RAMP_M, 1.0)
            return 0.5 * (1.0 + math.cos(math.pi * u))
        else:
            return 0.0

    def compute_local_reference_shift(self, x_init, x_ref, y_ref, theta_ref, v_ref, detected_obstacles):
        """
        Quy hoạch đường bao tham chiếu cục bộ:
        - Duy trì đường bao cực đại phẳng (Flat Outer Envelope) giữa các vật cản cùng phía (không bị trũng/nhảy bậc).
        - Hỗ trợ lách qua khe trống (Gate) và chuyển làn mượt khi gặp vật cản so le (Slalom).
        """
        x_ego, y_ego = float(x_init[0]), float(x_init[1])
        s_ego, d_ego, _, _, idx_ego = self.project_to_frenet(x_ego, y_ego, hint_idx=self.current_traj_idx)
        self.current_traj_idx = idx_ego

        r_clearance = EGO_HALF_WIDTH_M + OBSTACLE_AVOIDANCE_MARGIN_M
        road_min = -DEFAULT_ROAD_HALF_WIDTH_M
        road_max = DEFAULT_ROAD_HALF_WIDTH_M

        # 1. Thu thập các vật cản trên mặt đường trong tầm nhìn
        road_obs = []
        if detected_obstacles:
            for obs in detected_obstacles:
                s_r, s_f, d_min, d_max = self.compute_obstacle_frenet_bounds(obs)
                if (s_f >= s_ego - 12.0) and (s_r <= s_ego + 30.0) and (d_max >= road_min - 1.5) and (d_min <= road_max + 1.5):
                    obs_c = dict(obs)
                    obs_c['s_rear'] = s_r
                    obs_c['s_front'] = s_f
                    obs_c['s_center'] = 0.5 * (s_r + s_f)
                    obs_c['d_min'] = d_min
                    obs_c['d_max'] = d_max
                    obs_c['d_long_ego'] = s_r - s_ego
                    obs_c['d_lat_road'] = 0.5 * (d_min + d_max)
                    road_obs.append(obs_c)

        # Giải phóng các vật cản mà xe đã hoàn tất toàn bộ đoạn trả làn
        to_remove = []
        for obs_id, obs_item in self.active_chain.items():
            s_ret_done = obs_item['s_front'] + OBSTACLE_HOLD_DISTANCE_M + OBSTACLE_RETURN_RAMP_M + 1.0
            if s_ego > s_ret_done:
                to_remove.append(obs_id)
        for obs_id in to_remove:
            del self.active_chain[obs_id]

        # 2. Phân cụm dọc theo mặt cắt (các vật cản cách nhau <= OBSTACLE_SPATIAL_CLUSTER_RANGE_M)
        ahead_road_obs = [o for o in road_obs if o['s_front'] >= s_ego - 1.0]
        ahead_road_obs.sort(key=lambda o: o['s_rear'])

        clusters = []
        for o in ahead_road_obs:
            if not clusters or (o['s_rear'] - clusters[-1]['s_front']) > OBSTACLE_SPATIAL_CLUSTER_RANGE_M:
                clusters.append({
                    'obs_list': [o],
                    's_rear': o['s_rear'],
                    's_front': o['s_front'],
                    'blocked': [(o['d_min'] - r_clearance, o['d_max'] + r_clearance)]
                })
            else:
                clusters[-1]['obs_list'].append(o)
                clusters[-1]['s_rear'] = min(clusters[-1]['s_rear'], o['s_rear'])
                clusters[-1]['s_front'] = max(clusters[-1]['s_front'], o['s_front'])
                clusters[-1]['blocked'].append((o['d_min'] - r_clearance, o['d_max'] + r_clearance))

        # 3. Tìm khe trống cho từng cụm
        self.is_roadblock = False
        self.roadblock_barrier_dist = None

        prev_sign = None
        if self.active_obstacle is not None:
            prev_sign = self.active_obstacle.get('side_sign', None)

        cluster_plans = []
        for cl in clusters:
            affects_lane = any(
                (b_l < 0.15 and b_h > -0.15) or (min(abs(b_l), abs(b_h)) < OBSTACLE_TRIGGER_LATERAL_CORRIDOR_M)
                for b_l, b_h in cl['blocked']
            )
            gaps = self.find_free_gaps_in_corridor(cl['blocked'], road_min, road_max)
            if not gaps:
                if ENABLE_ROADBLOCK_EMERGENCY_STOP and cl['s_rear'] >= s_ego - 1.0:
                    self.is_roadblock = True
                    self.roadblock_barrier_dist = max(0.0, cl['s_rear'] - EGO_FRONT_OVERHANG_M - s_ego)
                    self.active_obstacle = None
                    self.active_chain.clear()
                    return np.copy(x_ref), np.copy(y_ref), np.copy(theta_ref), np.copy(v_ref), False
                continue

            d_target, _ = self.select_optimal_shift_from_gaps(gaps, preferred_sign=prev_sign)
            cl['gaps'] = gaps
            cl['d_target'] = d_target if affects_lane else 0.0
            cl['affects_lane'] = affects_lane
            if abs(cl['d_target']) > 0.05:
                prev_sign = -1.0 if cl['d_target'] < 0 else 1.0
            cluster_plans.append(cl)

        # Kiểm tra tính khả thi chuyển làn giữa 2 cụm liên tiếp
        for i in range(len(cluster_plans) - 1):
            c1 = cluster_plans[i]
            c2 = cluster_plans[i + 1]
            ds_gap = c2['s_rear'] - c1['s_front']
            dd = abs(c2['d_target'] - c1['d_target'])
            ds_req = (EGO_FRONT_OVERHANG_M + EGO_REAR_OVERHANG_M) + 1.2 * dd
            if ds_gap < ds_req and dd > 1.2:
                combined_blocked = c1['blocked'] + c2['blocked']
                common_gaps = self.find_free_gaps_in_corridor(combined_blocked, road_min, road_max)
                if common_gaps:
                    d_common, _ = self.select_optimal_shift_from_gaps(common_gaps, preferred_sign=prev_sign)
                    c1['d_target'] = d_common
                    c2['d_target'] = d_common
                elif ds_gap < (EGO_FRONT_OVERHANG_M + EGO_REAR_OVERHANG_M) and ENABLE_ROADBLOCK_EMERGENCY_STOP and c1['s_rear'] >= s_ego - 1.0:
                    self.is_roadblock = True
                    self.roadblock_barrier_dist = max(0.0, c1['s_rear'] - EGO_FRONT_OVERHANG_M - s_ego)
                    self.active_obstacle = None
                    self.active_chain.clear()
                    return np.copy(x_ref), np.copy(y_ref), np.copy(theta_ref), np.copy(v_ref), False

        # 4. Cập nhật vào self.active_chain (Khóa hướng né ngay khi vào vùng mở lái S-curve, không ghi đè thu hẹp s_rear/s_front)
        lock_dist = OBSTACLE_APPROACH_RAMP_M + OBSTACLE_APPROACH_MARGIN_M
        for cl in cluster_plans:
            if not cl['affects_lane'] and abs(cl['d_target']) < 0.05:
                continue
            d_target = cl['d_target']
            side_sign = -1.0 if d_target < 0 else 1.0
            A_shift = abs(d_target)
            for o in cl['obs_list']:
                oid = o['id']
                if oid not in self.active_chain:
                    self.active_chain[oid] = {
                        'id': oid, 'type': o.get('type', 'vehicle'),
                        'x': o['x'], 'y': o['y'],
                        'center_x': o.get('center_x', o['x']),
                        'center_y': o.get('center_y', o['y']),
                        'extent_x': o.get('extent_x', 0.6),
                        'extent_y': o.get('extent_y', 0.6),
                        'box_yaw': o.get('box_yaw', 0.0),
                        's_rear': cl['s_rear'], 's_front': cl['s_front'],
                        'd_min': o['d_min'], 'd_max': o['d_max'],
                        'side_sign': side_sign, 'A_shift': A_shift,
                        'd_target': d_target
                    }
                else:
                    # Giữ nguyên bao ngoài của cụm, không thu hẹp s_rear/s_front gây giật trọng số w
                    self.active_chain[oid]['s_rear'] = min(self.active_chain[oid]['s_rear'], cl['s_rear'])
                    self.active_chain[oid]['s_front'] = max(self.active_chain[oid]['s_front'], cl['s_front'])
                    if (self.active_chain[oid]['s_rear'] - EGO_FRONT_OVERHANG_M - s_ego) > lock_dist:
                        self.active_chain[oid]['side_sign'] = side_sign
                        self.active_chain[oid]['A_shift'] = A_shift
                        self.active_chain[oid]['d_target'] = d_target
                    else:
                        locked_sign = self.active_chain[oid]['side_sign']
                        if locked_sign * side_sign > 0:
                            self.active_chain[oid]['A_shift'] = max(self.active_chain[oid]['A_shift'], A_shift)
                            self.active_chain[oid]['d_target'] = locked_sign * self.active_chain[oid]['A_shift']

        if not self.active_chain:
            self.active_obstacle = None
            return np.copy(x_ref), np.copy(y_ref), np.copy(theta_ref), np.copy(v_ref), False

        chain_list = list(self.active_chain.values())
        ahead_chain = [o for o in chain_list if o['s_front'] + OBSTACLE_HOLD_DISTANCE_M >= s_ego]
        if ahead_chain:
            self.active_obstacle = min(ahead_chain, key=lambda o: o['s_rear'])
        else:
            self.active_obstacle = max(chain_list, key=lambda o: o['s_front'])

        # 5. Tính toán đường bao dịch ngang d_shift_k (True Outer Envelope cho cùng bên, không bị trũng giữa 2 vật cản)
        n_ref = len(x_ref)
        x_ref_mod = np.copy(x_ref)
        y_ref_mod = np.copy(y_ref)
        theta_ref_mod = np.copy(theta_ref)

        for k in range(n_ref):
            s_k, _, nx_k, ny_k, _ = self.project_to_frenet(x_ref[k], y_ref[k], hint_idx=idx_ego)
            max_pos_shift = 0.0
            min_neg_shift = 0.0

            for obs_item in chain_list:
                w = self.calc_span_obstacle_weight(s_k, obs_item['s_rear'], obs_item['s_front'])
                if w > 1e-4:
                    d_t = obs_item.get('d_target', obs_item['side_sign'] * obs_item['A_shift'])
                    val = w * d_t
                    if val > max_pos_shift:
                        max_pos_shift = val
                    elif val < min_neg_shift:
                        min_neg_shift = val

            # Nếu chỉ né 1 phía, lấy chuẩn biên cực đại (phẳng 100% suốt cụm); nếu chuyển bên Slalom, cộng đại số mượt
            d_shift_k = max_pos_shift + min_neg_shift

            x_ref_mod[k] = x_ref[k] + d_shift_k * nx_k
            y_ref_mod[k] = y_ref[k] + d_shift_k * ny_k

        # 6. Làm mịn góc tiếp tuyến theta_ref_mod (Giảm hệ số d_th khi trả làn để xe nhập làn tiệm cận, không vọt lố)
        for k in range(n_ref - 1):
            dx = x_ref_mod[k + 1] - x_ref_mod[k]
            dy = y_ref_mod[k + 1] - y_ref_mod[k]
            if math.hypot(dx, dy) > 1e-3:
                th_target = math.atan2(dy, dx)
                d_th = normalize_angle(th_target - theta_ref[k])
                # Làm dịu góc hướng tiếp tuyến (0.75 * d_th) để tránh ép mũi xe lao quá gắt cắt qua tim đường khi nhập làn
                theta_ref_mod[k] = theta_ref[k] + 0.75 * d_th
        if n_ref > 1:
            theta_ref_mod[-1] = theta_ref_mod[-2]

        return x_ref_mod, y_ref_mod, theta_ref_mod, np.copy(v_ref), True

    def compute_safe_speed_profile(self, x_init, x_ref_mod, y_ref_mod, theta_ref_mod, v_ref_base):
        """Điều chỉnh vận tốc mục tiêu an toàn khi tiếp cận và đi qua vật cản"""
        v_adapted = np.copy(v_ref_base)
        s_ego, _, _, _, _ = self.project_to_frenet(x_init[0], x_init[1], hint_idx=self.current_traj_idx)

        if self.active_obstacle is not None:
            s_rear = self.active_obstacle.get('s_rear', s_ego + 10.0)
            s_front = self.active_obstacle.get('s_front', s_rear + 2.0)

            dist_to_obs = (s_rear - EGO_FRONT_OVERHANG_M) - s_ego
            dist_after_obs = s_ego - (s_front + OBSTACLE_HOLD_DISTANCE_M)

            if dist_to_obs > OBSTACLE_APPROACH_MARGIN_M:
                ratio = float(np.clip((dist_to_obs - OBSTACLE_APPROACH_MARGIN_M) /
                                      max(OBSTACLE_SLOWDOWN_DISTANCE_M, 1.0), 0.0, 1.0))
                v_gov = OBSTACLE_CAUTION_SPEED_MS + ratio * (TARGET_SPEED_MS - OBSTACLE_CAUTION_SPEED_MS)
                v_adapted = np.minimum(v_adapted, v_gov)
            elif dist_after_obs <= 0.0:
                v_adapted = np.minimum(v_adapted, OBSTACLE_CAUTION_SPEED_MS)
            else:
                ratio = float(np.clip(dist_after_obs / max(OBSTACLE_RETURN_RAMP_M, 1.0), 0.0, 1.0))
                v_ramp = OBSTACLE_CAUTION_SPEED_MS + ratio * (TARGET_SPEED_MS - OBSTACLE_CAUTION_SPEED_MS)
                v_adapted = np.minimum(v_adapted, v_ramp)

        return v_adapted

    def solve_qp_iteration(self, x_init, x_ref, y_ref, theta_ref, v_ref, obstacles=None, is_roadblock=False):
        """
        Giải bài toán QP tại một vòng lặp tuyến tính hóa:
        - Khớp chính xác chỉ số thời gian X[:, k] với x_ref[k] (k = 0..N-1) để triệt tiêu hoàn toàn dao động hình sin.
        - Chỉ kích hoạt siêu phẳng khi vật cản còn ở phía trước tâm xe (không đẩy ngang xe khi đang nhập làn).
        """
        opti = ca.Opti()
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_cpu_time': 1.5,
            'ipopt.max_iter': 40,
            'ipopt.tol': 1e-4,
            'ipopt.sb': 'yes'
        }
        opti.solver('ipopt', opts)

        X = opti.variable(4, self.horizon + 1)
        U = opti.variable(2, self.horizon)

        opti.subject_to(X[:, 0] == x_init)

        cost = 0

        if is_roadblock:
            min_v = 0.0
        else:
            min_v = min(float(x_init[3]), 1.5)

        for k in range(self.horizon):
            x_bar_k = self.x_nom[:, k]
            u_bar_k = self.u_nom[:, k]
            A_k, B_k, d_k = self.get_linearized_matrices(x_bar_k, u_bar_k)

            opti.subject_to(X[:, k + 1] == A_k @ X[:, k] + B_k @ U[:, k] + d_k)

            opti.subject_to(U[0, k] <= MAX_CONTROL_WHEEL_ANGLE_RAD)
            opti.subject_to(U[0, k] >= -MAX_CONTROL_WHEEL_ANGLE_RAD)
            opti.subject_to(U[1, k] <= MAX_CONTROL_ACCELERATION_M_S_2)
            opti.subject_to(U[1, k] >= MAX_CONTROL_BRAKING_M_S_2)

            opti.subject_to(X[3, k + 1] >= min_v)
            opti.subject_to(X[3, k + 1] <= 15.0)

            opti.set_initial(X[:, k], x_bar_k)
            opti.set_initial(U[:, k], u_bar_k)

            # Chi phí bám quỹ đạo đồng bộ thời gian thực tại bước k (t = k * dt)
            fine_x = FINE_X_COEF * (X[0, k] - x_ref[k]) ** 2
            fine_y = FINE_Y_COEF * (X[1, k] - y_ref[k]) ** 2
            fine_v = FINE_V_COEF * (X[3, k] - v_ref[k]) ** 2
            fine_theta = FINE_THETA_COEF * (X[2, k] - theta_ref[k]) ** 2

            if k < self.horizon - 1:
                lat_dev = calculate_lateral_deviation(
                    X[0, k], X[1, k],
                    x_ref[k], y_ref[k],
                    x_ref[k + 1], y_ref[k + 1]
                )
            else:
                lat_dev = 0.0
            fine_lat = FINE_LATERAL_COEF * (lat_dev ** 2)

            fine_steer = FINE_STEER_COEF * (U[0, k] ** 2)
            fine_acc = FINE_ACC_COEF * (U[1, k] ** 2)

            # Phạt độ mượt nội bộ chân trời (k > 0) - Không khóa cứng U[0, 0] vào u_prev để tránh trễ pha gây lắc
            fine_steer_dot = 0.0
            fine_acc_dot = 0.0
            if k > 0:
                fine_steer_dot = FINE_STEER_DOT_COEF * ((U[0, k] - U[0, k - 1]) ** 2)
                fine_acc_dot = FINE_ACC_DOT_COEF * ((U[1, k] - U[1, k - 1]) ** 2)

            cost += (fine_x + fine_y + fine_v + fine_theta + fine_lat +
                     fine_steer + fine_acc + fine_steer_dot + fine_acc_dot)

        opti.set_initial(X[:, self.horizon], self.x_nom[:, self.horizon])

        # RÀNG BUỘC SIÊU PHẲNG NÉ VẬT CẢN (Chỉ kích hoạt cho vật cản phía trước mũi/tâm xe, ngắt khi đã vào pha nhập làn)
        if obstacles and len(obstacles) > 0:
            tx_ego = math.cos(x_init[2])
            ty_ego = math.sin(x_init[2])
            ahead_obs = [
                obs for obs in obstacles
                if (tx_ego * (obs['x'] - x_init[0]) + ty_ego * (obs['y'] - x_init[1])) >= 0.0
            ]
            active_obs = ahead_obs[:2]
            num_obs = len(active_obs)

            if num_obs > 0:
                eps_slack = opti.variable(num_obs, self.horizon)
                opti.subject_to(ca.vec(eps_slack) >= 0)

                for m, obs in enumerate(active_obs):
                    x_o = obs['x']
                    y_o = obs['y']
                    r_safe = EGO_HALF_WIDTH_M + OBSTACLE_AVOIDANCE_MARGIN_M

                    for k in range(self.horizon):
                        x_bar_k = self.x_nom[0, k]
                        y_bar_k = self.x_nom[1, k]
                        dx_nom = x_bar_k - x_o
                        dy_nom = y_bar_k - y_o
                        dist_nom = math.sqrt(dx_nom ** 2 + dy_nom ** 2) + 1e-4

                        if dist_nom <= r_safe + 3.5:
                            nx_bar = dx_nom / dist_nom
                            ny_bar = dy_nom / dist_nom
                            lhs = nx_bar * (X[0, k] - x_o) + ny_bar * (X[1, k] - y_o) + eps_slack[m, k]
                            opti.subject_to(lhs >= r_safe)
                            cost += MPC_W_SLACK * (eps_slack[m, k] ** 2) + MPC_W_SLACK_LINEAR * eps_slack[m, k]

        opti.minimize(cost)

        try:
            sol = opti.solve()
            x_opt = np.array(sol.value(X), dtype=float)
            u_opt = np.array(sol.value(U), dtype=float)
            cost_opt = float(sol.value(cost))
        except Exception:
            x_opt = np.array(opti.debug.value(X), dtype=float)
            u_opt = np.array(opti.debug.value(U), dtype=float)
            cost_opt = float(opti.debug.value(cost))

        return x_opt, u_opt, cost_opt

    def solve(self, x_init, x_ref, y_ref, theta_ref, v_ref, detected_obstacles=None):
        """
        Thực hiện toàn bộ chu trình Tuyến tính hóa lặp (Iterative Linearization Loop):
        1. Dời tạm tham chiếu cục bộ đa tình huống (Universal Local Reference Shifting).
        2. Tự động điều chỉnh profile tốc độ an toàn (Speed Governor).
        3. Duy trì liên tục quỹ đạo danh định Warm-start (không reset đột ngột).
        4. Chạy vòng lặp giải QP.
        """
        x_ref_mod, y_ref_mod, theta_ref_mod, _, has_shift = self.compute_local_reference_shift(
            x_init, x_ref, y_ref, theta_ref, v_ref, detected_obstacles
        )
        self.x_ref_mod = x_ref_mod
        self.y_ref_mod = y_ref_mod

        # Xử lý Roadblock Emergency Stop
        if self.is_roadblock and self.roadblock_barrier_dist is not None:
            d_to_barrier = self.roadblock_barrier_dist
            d_stop = max(0.0, d_to_barrier - ROADBLOCK_SAFE_STOP_DISTANCE_M)
            curr_v = float(x_init[3])

            if d_stop <= 0.25 or (curr_v < 0.25 and d_to_barrier <= ROADBLOCK_SAFE_STOP_DISTANCE_M + 1.5):
                self.is_success = True
                self.init_nominal_trajectory(x_init, x_ref_mod, y_ref_mod, theta_ref_mod, np.zeros_like(v_ref))
                return 0.0, -3.5

            v_ref_adapted = np.zeros_like(v_ref)
            for k in range(len(v_ref)):
                d_stop_k = max(0.0, d_stop - curr_v * k * self.dt)
                v_target_k = math.sqrt(2.0 * ROADBLOCK_DECELERATION_M_S_2 * d_stop_k)
                v_ref_adapted[k] = min(float(v_ref[k]), v_target_k)

            if self.x_nom is None:
                self.init_nominal_trajectory(x_init, x_ref_mod, y_ref_mod, theta_ref_mod, v_ref_adapted)

            try:
                x_opt, u_opt, cost_val = self.solve_qp_iteration(
                    x_init, x_ref_mod, y_ref_mod, theta_ref_mod, v_ref_adapted,
                    obstacles=None, is_roadblock=True
                )
                self.is_success = True
                self.last_cost = cost_val
                steer_rad = float(u_opt[0, 0])
                acc_m_s2 = float(u_opt[1, 0])
                self.x_nom = x_opt
                self.u_nom = u_opt
                return steer_rad, acc_m_s2
            except Exception:
                return 0.0, -ROADBLOCK_DECELERATION_M_S_2

        # Tính profile vận tốc an toàn
        v_ref_adapted = self.compute_safe_speed_profile(x_init, x_ref_mod, y_ref_mod, theta_ref_mod, v_ref)

        all_obs = []
        if self.active_obstacle is not None:
            all_obs.append(self.active_obstacle)
        if detected_obstacles:
            for obs in detected_obstacles:
                if self.active_obstacle is None or obs.get('id') != self.active_obstacle.get('id'):
                    all_obs.append(obs)

        # Chỉ khởi tạo x_nom lần đầu tiên; giữ liên tục nghiệm Warm-start của xe trong suốt quá trình chạy
        if self.x_nom is None:
            self.init_nominal_trajectory(x_init, x_ref_mod, y_ref_mod, theta_ref_mod, v_ref_adapted)

        x_opt = None
        u_opt = None

        try:
            for it in range(MPC_MAX_LINEARIZATION_ITER):
                x_opt, u_opt, cost_val = self.solve_qp_iteration(
                    x_init, x_ref_mod, y_ref_mod, theta_ref_mod, v_ref_adapted,
                    obstacles=all_obs
                )
                self.x_nom = x_opt
                self.u_nom = u_opt
                self.last_cost = cost_val

            self.is_success = True

            steer_rad = float(u_opt[0, 0])
            acc_m_s2 = float(u_opt[1, 0])
            self.u_prev = np.array([steer_rad, acc_m_s2])

            self.control_buffer["wheel_angle"] = list(u_opt[0, 1:]) + [float(u_opt[0, -1])]
            self.control_buffer["acceleration"] = list(u_opt[1, 1:]) + [float(u_opt[1, -1])]
            self.buffer_index = 0

            self.x_nom[:, :-1] = x_opt[:, 1:]
            self.x_nom[:, -1] = x_opt[:, -1]
            self.u_nom[:, :-1] = u_opt[:, 1:]
            self.u_nom[:, -1] = u_opt[:, -1]

            return steer_rad, acc_m_s2

        except Exception:
            self.is_success = False
            if self.buffer_index < len(self.control_buffer["wheel_angle"]):
                steer_rad = self.control_buffer["wheel_angle"][self.buffer_index]
                acc_m_s2 = self.control_buffer["acceleration"][self.buffer_index]
                self.buffer_index += 1
            else:
                steer_rad = 0.0
                acc_m_s2 = 0.5

            self.init_nominal_trajectory(x_init, x_ref_mod, y_ref_mod, theta_ref_mod, v_ref_adapted)
            return steer_rad, acc_m_s2

    def get_optimized_cost(self):
        return self.last_cost
