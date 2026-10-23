import math
import numpy as np
from enum import Enum

from src.config import (
    ENABLE_EMERGENCY_BRAKE_MODE,
    EMERGENCY_ZONE_WIDTH_M,
    EMERGENCY_ZONE_LENGTH_M,
    EMERGENCY_CLEAR_HOLD_TIME_SEC,
    EMERGENCY_TRIGGER_ONLY_MOVING,
    EMERGENCY_MOVING_SPEED_THRESHOLD_MS,
    EGO_FRONT_OVERHANG_M,
    EGO_HALF_WIDTH_M
)
from src.help_functions import normalize_angle


class VehicleMode(Enum):
    """2 chế độ hoạt động của xe tự hành"""
    MODE_1_NORMAL = 1            # Chế độ 1: Bám đường né vật cản theo MPC (Path Tracking & Obstacle Avoidance)
    MODE_2_EMERGENCY_BRAKE = 2   # Chế độ 2: Phanh gấp tối đa, bỏ qua mọi ràng buộc (Emergency Hard Braking)


class CollisionDangerZone:
    """
    Vùng Nguy Hiểm Va Chạm (Collision Danger Zone):
    - Chỉ tồn tại ở TRƯỚC ĐẦU XE (tính từ cản trước xe EGO_FRONT_OVERHANG_M trở về phía trước).
    - KHÔNG tồn tại xung quanh thân xe hay sau xe (không gây phanh nhầm khi lách qua sườn vật cản tĩnh).
    - Kích thước:
        + Bề rộng (W): EMERGENCY_ZONE_WIDTH_M (tùy chỉnh trong config.py, đối xứng qua trục dọc xe)
        + Chiều dài (L): EMERGENCY_ZONE_LENGTH_M (tùy chỉnh trong config.py)
    - Sử dụng thuật toán Trục Phân Tách SAT (Separating Axis Theorem) kết hợp kiểm tra điểm
      để phát hiện chính xác mọi trường hợp vật cản xâm lấn vào vùng nguy hiểm.
    """
    def __init__(self,
                 zone_width=EMERGENCY_ZONE_WIDTH_M,
                 zone_length=EMERGENCY_ZONE_LENGTH_M,
                 front_overhang=EGO_FRONT_OVERHANG_M):
        self.zone_width = float(zone_width)
        self.zone_length = float(zone_length)
        self.front_overhang = float(front_overhang)

    def get_danger_zone_corners_world(self, ego_x, ego_y, ego_yaw_rad):
        """
        Tính 4 góc của hộp vùng nguy hiểm trong hệ tọa độ toàn cục thế giới (World Frame).
        Thứ tự trả về: [sau-phải, trước-phải, trước-trái, sau-trái] (tính từ mũi xe hướng tới).
        """
        cos_th = math.cos(ego_yaw_rad)
        sin_th = math.sin(ego_yaw_rad)

        t_vec = np.array([cos_th, sin_th])      # Vector đơn vị dọc thân xe
        n_vec = np.array([-sin_th, cos_th])     # Vector đơn vị ngang thân xe (hướng trái)

        center_ego = np.array([ego_x, ego_y])
        bumper_front = center_ego + self.front_overhang * t_vec
        zone_center = bumper_front + (self.zone_length / 2.0) * t_vec

        hl = self.zone_length / 2.0
        hw = self.zone_width / 2.0

        c_rear_right = zone_center - hl * t_vec - hw * n_vec
        c_front_right = zone_center + hl * t_vec - hw * n_vec
        c_front_left = zone_center + hl * t_vec + hw * n_vec
        c_rear_left = zone_center - hl * t_vec + hw * n_vec

        return np.array([c_rear_right, c_front_right, c_front_left, c_rear_left])

    @staticmethod
    def get_obstacle_obb_corners_world(obs):
        """Tính 4 góc của Bounding Box vật cản trong hệ tọa độ thế giới"""
        cx = float(obs.get('center_x', obs.get('x', 0.0)))
        cy = float(obs.get('center_y', obs.get('y', 0.0)))
        hl = float(obs.get('extent_x', 0.6))
        hw = float(obs.get('extent_y', 0.6))
        yaw = float(obs.get('box_yaw', 0.0))

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        t_obs = np.array([cos_y, sin_y])
        n_obs = np.array([-sin_y, cos_y])
        c_center = np.array([cx, cy])

        c1 = c_center - hl * t_obs - hw * n_obs
        c2 = c_center + hl * t_obs - hw * n_obs
        c3 = c_center + hl * t_obs + hw * n_obs
        c4 = c_center - hl * t_obs + hw * n_obs
        return np.array([c1, c2, c3, c4])

    @staticmethod
    def _sat_overlap_on_axis(axis, corners_a, corners_b):
        """Kiểm tra hình chiếu của 2 đa giác lên một trục xem có tách rời không"""
        proj_a = [np.dot(c, axis) for c in corners_a]
        proj_b = [np.dot(c, axis) for c in corners_b]
        min_a, max_a = min(proj_a), max(proj_a)
        min_b, max_b = min(proj_b), max(proj_b)
        return not (max_a < min_b or max_b < min_a)

    def check_obb_intersection(self, zone_corners, obs_corners, ego_yaw_rad, obs_yaw_rad):
        """
        Áp dụng Separating Axis Theorem (SAT) kiểm tra giao cắt giữa Vùng nguy hiểm và Bounding Box vật cản.
        4 trục kiểm tra: 2 trục trực giao của Vùng nguy hiểm + 2 trục trực giao của Vật cản.
        """
        # Trục của vùng nguy hiểm
        ax1 = np.array([math.cos(ego_yaw_rad), math.sin(ego_yaw_rad)])
        ax2 = np.array([-math.sin(ego_yaw_rad), math.cos(ego_yaw_rad)])
        # Trục của vật cản
        ax3 = np.array([math.cos(obs_yaw_rad), math.sin(obs_yaw_rad)])
        ax4 = np.array([-math.sin(obs_yaw_rad), math.cos(obs_yaw_rad)])

        for ax in [ax1, ax2, ax3, ax4]:
            norm = np.linalg.norm(ax)
            if norm > 1e-6:
                unit_ax = ax / norm
                if not self._sat_overlap_on_axis(unit_ax, zone_corners, obs_corners):
                    return False  # Tìm thấy trục phân tách -> Không giao nhau
        return True  # Không có trục phân tách nào -> Giao nhau!

    def check_point_in_zone(self, px, py, ego_x, ego_y, ego_yaw_rad):
        """
        Kiểm tra điểm (px, py) có nằm trong vùng nguy hiểm trước mũi xe hay không.
        Quy đổi sang hệ tọa độ cục bộ của xe Ego:
        - X_local: Khoảng cách dọc tính từ tâm xe
        - Y_local: Khoảng cách ngang tính từ tim xe
        """
        dx = px - ego_x
        dy = py - ego_y
        cos_th = math.cos(ego_yaw_rad)
        sin_th = math.sin(ego_yaw_rad)

        x_local = dx * cos_th + dy * sin_th
        y_local = -dx * sin_th + dy * cos_th

        d_front = x_local - self.front_overhang
        hw = self.zone_width / 2.0

        # Kiểm tra nằm ở phía trước đầu xe và trong phạm vi giới hạn
        in_length = (0.0 <= d_front <= self.zone_length)
        in_width = (abs(y_local) <= hw)

        return in_length and in_width, d_front, y_local

    def is_obstacle_in_zone(self, obs, ego_x, ego_y, ego_yaw_rad):
        """
        Kiểm tra toàn diện xem vật cản có nằm trong hoặc giao cắt với vùng nguy hiểm hay không.
        Trả về: (is_inside, min_dist_from_bumper, lat_offset)
        """
        zone_corners = self.get_danger_zone_corners_world(ego_x, ego_y, ego_yaw_rad)
        obs_corners = self.get_obstacle_obb_corners_world(obs)
        obs_yaw = float(obs.get('box_yaw', 0.0))

        # 1. Kiểm tra SAT giao cắt hộp
        is_intersect = self.check_obb_intersection(zone_corners, obs_corners, ego_yaw_rad, obs_yaw)

        # 2. Kiểm tra các điểm biên và điểm gần nhất của vật cản
        test_points = list(obs_corners)
        if 'x' in obs and 'y' in obs:
            test_points.append(np.array([obs['x'], obs['y']]))
        if 'center_x' in obs and 'center_y' in obs:
            test_points.append(np.array([obs['center_x'], obs['center_y']]))

        min_d_front = float('inf')
        lat_at_min = 0.0
        any_pt_inside = False

        for pt in test_points:
            in_zone, d_fr, y_loc = self.check_point_in_zone(pt[0], pt[1], ego_x, ego_y, ego_yaw_rad)
            if in_zone:
                any_pt_inside = True
            if d_fr < min_d_front:
                min_d_front = d_fr
                lat_at_min = y_loc

        # Nếu giao cắt OBB hoặc có điểm lọt vào trong vùng
        is_inside = is_intersect or any_pt_inside
        if min_d_front == float('inf'):
            min_d_front = max(0.0, obs.get('dist', 10.0) - self.front_overhang)

        return is_inside, min_d_front, lat_at_min


class EmergencySafetyManager:
    """
    Bộ Quản Lý Trạng Thái An Toàn & 2 Chế Độ Điều Khiển:
    - Chế độ 1: Bám đường né vật cản theo MPC
    - Chế độ 2: Phanh gấp tối đa, bỏ qua mọi ràng buộc
    Chuyển chế độ:
    - Chế độ 1 -> Chế độ 2: Ngay khi phát hiện vật cản xâm nhập vùng nguy hiểm trước đầu xe.
    - Chế độ 2 -> Chế độ 1: Khi vùng nguy hiểm hoàn toàn không còn vật cản trong ít nhất EMERGENCY_CLEAR_HOLD_TIME_SEC (giây).
    """
    def __init__(self,
                 zone_width=EMERGENCY_ZONE_WIDTH_M,
                 zone_length=EMERGENCY_ZONE_LENGTH_M,
                 clear_hold_time_sec=EMERGENCY_CLEAR_HOLD_TIME_SEC,
                 trigger_only_moving=EMERGENCY_TRIGGER_ONLY_MOVING,
                 moving_speed_threshold_ms=EMERGENCY_MOVING_SPEED_THRESHOLD_MS):
        self.zone = CollisionDangerZone(
            zone_width=zone_width,
            zone_length=zone_length,
            front_overhang=EGO_FRONT_OVERHANG_M
        )
        self.clear_hold_time_sec = float(clear_hold_time_sec)
        self.trigger_only_moving = bool(trigger_only_moving)
        self.moving_speed_threshold_ms = float(moving_speed_threshold_ms)

        self.current_mode = VehicleMode.MODE_1_NORMAL
        self.clear_timer = 0.0
        self.prev_obs_positions = {}   # actor_id -> (center_x, center_y)
        self.danger_obstacles = []
        self.last_triggered_obs = None
        self.mode_switches = 0
        self.total_emergency_braking_time = 0.0

    def reset(self):
        """Reset trạng thái bộ quản lý an toàn"""
        self.current_mode = VehicleMode.MODE_1_NORMAL
        self.clear_timer = 0.0
        self.prev_obs_positions.clear()
        self.danger_obstacles.clear()
        self.last_triggered_obs = None

    def is_obstacle_moving(self, obs, dt, moving_actor_ids=None):
        """
        Xác định vật cản có đang di chuyển không:
        1. Thuộc danh sách moving_actor_ids của MovingObstacleManager.
        2. Tốc độ thực tế từ CARLA actor.get_velocity().
        3. Tốc độ theo dõi độ dời vị trí tâm vật cản giữa 2 bước scan liên tiếp.
        """
        actor_id = obs.get('id')

        # 1. Kiểm tra danh sách moving_actor_ids
        if moving_actor_ids is not None and actor_id in moving_actor_ids:
            return True, 9.0

        # 2. Kiểm tra CARLA actor velocity
        actor = obs.get('actor')
        if actor is not None:
            try:
                vel = actor.get_velocity()
                sp = math.hypot(vel.x, vel.y)
                if sp >= self.moving_speed_threshold_ms:
                    return True, sp
            except Exception:
                pass

        # 3. Kiểm tra độ dịch chuyển qua các bước thời gian (position tracking)
        curr_x = float(obs.get('center_x', obs.get('x', 0.0)))
        curr_y = float(obs.get('center_y', obs.get('y', 0.0)))

        if actor_id in self.prev_obs_positions and dt > 1e-4:
            prev_x, prev_y = self.prev_obs_positions[actor_id]
            ds = math.hypot(curr_x - prev_x, curr_y - prev_y)
            sp_est = ds / dt
            self.prev_obs_positions[actor_id] = (curr_x, curr_y)
            if sp_est >= self.moving_speed_threshold_ms:
                return True, sp_est
        else:
            self.prev_obs_positions[actor_id] = (curr_x, curr_y)

        return False, 0.0

    def update(self, ego_x, ego_y, ego_yaw_rad, ego_v, detected_obstacles, dt, moving_actor_ids=None):
        """
        Cập nhật trạng thái an toàn mỗi bước dt:
        - Quét tất cả vật cản xem có xâm phạm vùng nguy hiểm trước đầu xe không.
        - Xử lý chuyển chế độ (Mode 1 <-> Mode 2).
        Trả về: (current_mode, is_danger_triggered, danger_obs_list, status_info)
        """
        if not ENABLE_EMERGENCY_BRAKE_MODE:
            self.current_mode = VehicleMode.MODE_1_NORMAL
            return self.current_mode, False, [], {"mode": self.current_mode, "text": "CHẾ ĐỘ 1 (BÁM ĐƯỜNG)"}

        # 1. Quét tìm tất cả các vật cản trong vùng nguy hiểm
        obstacles_in_zone = []
        triggering_obstacles = []

        if detected_obstacles:
            for obs in detected_obstacles:
                in_zone, dist_front, lat_off = self.zone.is_obstacle_in_zone(
                    obs, ego_x, ego_y, ego_yaw_rad
                )
                if in_zone:
                    is_moving, obs_speed = self.is_obstacle_moving(obs, dt, moving_actor_ids)
                    obs_report = dict(obs)
                    obs_report['dist_front'] = dist_front
                    obs_report['lat_off'] = lat_off
                    obs_report['is_moving'] = is_moving
                    obs_report['speed'] = obs_speed
                    obstacles_in_zone.append(obs_report)

                    # Điều kiện kích hoạt:
                    # - Nếu trigger_only_moving = True: vật cản phải đang di chuyển
                    # - Nếu trigger_only_moving = False: bất kỳ vật cản nào lọt vào vùng
                    if (not self.trigger_only_moving) or is_moving:
                        triggering_obstacles.append(obs_report)

        self.danger_obstacles = obstacles_in_zone

        # 2. Máy trạng thái (Finite State Machine) chuyển đổi giữa 2 chế độ
        mode_changed = False

        if self.current_mode == VehicleMode.MODE_1_NORMAL:
            # Điều kiện chuyển sang Chế độ 2: Có vật cản xâm nhập vùng nguy hiểm
            if len(triggering_obstacles) > 0:
                self.current_mode = VehicleMode.MODE_2_EMERGENCY_BRAKE
                self.clear_timer = 0.0
                self.last_triggered_obs = triggering_obstacles[0]
                self.mode_switches += 1
                mode_changed = True
                print(f"\n[CẢNH BÁO NGUY HIỂM] Vật cản #{self.last_triggered_obs.get('id')} xâm nhập vùng nguy hiểm trước đầu xe "
                      f"(Cách cản trước: {self.last_triggered_obs['dist_front']:.1f}m, Lệch: {self.last_triggered_obs['lat_off']:+.1f}m)! "
                      f"-> CHUYỂN SANG CHẾ ĐỘ 2 (PHANH GẤP TỐI ĐA)!")

        elif self.current_mode == VehicleMode.MODE_2_EMERGENCY_BRAKE:
            self.total_emergency_braking_time += dt

            # Điều kiện chuyển từ Chế độ 2 về Chế độ 1:
            # Vùng nguy hiểm phải hoàn toàn KHÔNG CÓ vật cản liên tục sau khoảng thời gian clear_hold_time_sec
            if len(obstacles_in_zone) == 0:
                self.clear_timer += dt
                if self.clear_timer + 1e-4 >= self.clear_hold_time_sec:
                    self.current_mode = VehicleMode.MODE_1_NORMAL
                    self.clear_timer = 0.0
                    self.mode_switches += 1
                    mode_changed = True
                    print(f"\n[AN TOÀN TRỞ LẠI] Vùng nguy hiểm trước đầu xe đã thông thoáng liên tục "
                          f"{self.clear_hold_time_sec:.1f}s! -> CHUYỂN VỀ CHẾ ĐỘ 1 (BÁM ĐƯỜNG NÉ VẬT CẢN)!")
            else:
                # Vẫn còn vật cản trong vùng nguy hiểm -> reset bộ đếm thời gian an toàn
                self.clear_timer = 0.0

        # 3. Chuẩn bị thông tin hiển thị trạng thái
        is_triggered = (self.current_mode == VehicleMode.MODE_2_EMERGENCY_BRAKE)
        if is_triggered:
            if len(obstacles_in_zone) > 0:
                near_obs = obstacles_in_zone[0]
                status_text = f"PHANH GẤP [CHẾ ĐỘ 2] - VẬT CẢN ({near_obs['dist_front']:.1f}m)"
            else:
                rem_time = max(0.0, self.clear_hold_time_sec - self.clear_timer)
                status_text = f"PHANH GẤP [CHỜ AN TOÀN: {rem_time:.1f}s]"
        else:
            status_text = "CHẾ ĐỘ 1 (BÁM ĐƯỜNG NÉ VẬT CẢN)"

        status_info = {
            'mode': self.current_mode,
            'is_emergency': is_triggered,
            'status_text': status_text,
            'danger_count': len(obstacles_in_zone),
            'clear_timer': self.clear_timer,
            'hold_time': self.clear_hold_time_sec,
            'mode_changed': mode_changed
        }

        return self.current_mode, is_triggered, obstacles_in_zone, status_info
