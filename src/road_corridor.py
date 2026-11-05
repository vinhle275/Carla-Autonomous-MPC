import math
import numpy as np
from typing import Optional, Dict, Tuple, Any

from src.config import (
    EGO_HALF_WIDTH_M,
    OBSTACLE_AVOIDANCE_MARGIN_M,
    DEFAULT_ROAD_HALF_WIDTH_M,
    YELLOW_LINE_CLEARANCE_M
)

class RoadCorridorTracker:
    """
    Module Theo Dõi & Nhận Diện Hành Lang Mặt Đường Động (Dynamic Drivable Road Corridor):
    - Đọc cấu trúc làn đường theo thời gian thực từ CARLA OpenDRIVE HD Map.
    - Tự động nhận diện số làn cùng chiều bên trái/phải, mép vỉa hè, hộ lan lề đường và vạch kẻ.
    - Cung cấp giới hạn biên pháp lý [road_min, road_max] chuẩn xác cho tâm xe dọc theo từng waypoint,
      thay thế hoàn toàn việc hardcode tĩnh DEFAULT_ROAD_HALF_WIDTH_M (+-5.25m).
    """
    def __init__(self, carla_map=None):
        self.carla_map = carla_map
        # r_clearance cho mép mặt đường / vạch kẻ biên: Nửa thân xe (0.93m) + đệm lề (0.15m) = 1.08m
        # Giúp tâm xe có đủ không gian tự nhiên trong làn mà không bao giờ bị bóp nghẹt thành dải hẹp
        self.r_clearance = float(EGO_HALF_WIDTH_M + 0.15)

        # Mảng dữ liệu hành lang tiền tính toán dọc theo quỹ đạo toàn cục
        self.road_min_traj: Optional[np.ndarray] = None
        self.road_max_traj: Optional[np.ndarray] = None
        self.d_left_legal_traj: Optional[np.ndarray] = None
        self.d_right_legal_traj: Optional[np.ndarray] = None
        self.is_yellow_solid_traj: Optional[np.ndarray] = None
        self.s_traj: Optional[np.ndarray] = None
        self.x_traj: Optional[np.ndarray] = None
        self.y_traj: Optional[np.ndarray] = None
        self.theta_traj: Optional[np.ndarray] = None
        self.n_pts: int = 0

    def set_map(self, carla_map):
        """Cập nhật đối tượng carla_map"""
        self.carla_map = carla_map

    def compute_waypoint_corridor(self, x: float, y: float, theta_rad: float) -> Dict[str, Any]:
        """
        Tính toán biên hành lang ngang tại một tọa độ (x, y, theta_rad) bất kỳ.
        Trả về từ điển gồm:
          - road_min: Giới hạn an toàn tâm xe bên trái (m) (đã trừ bán kính thân xe + clearance)
          - road_max: Giới hạn an toàn tâm xe bên phải (m) (đã trừ bán kính thân xe + clearance)
          - d_left_legal: Mép mặt đường hợp pháp bên trái (m)
          - d_right_legal: Mép mặt đường hợp pháp bên phải (m)
          - d_left_physical: Mép mặt đường vật lý bên trái (m)
          - d_right_physical: Mép mặt đường vật lý bên phải (m)
          - is_yellow_solid: True nếu biên trái bị giới hạn bởi vạch vàng nét liền
          - lane_width: Chiều rộng làn xe hiện tại (m)
          - lane_id: ID làn OpenDRIVE
        """
        # Giá trị mặc định dự phòng nếu không kết nối được bản đồ CARLA
        default_res = {
            'road_min': -DEFAULT_ROAD_HALF_WIDTH_M + self.r_clearance,
            'road_max':  DEFAULT_ROAD_HALF_WIDTH_M - self.r_clearance,
            'd_left_legal': -DEFAULT_ROAD_HALF_WIDTH_M,
            'd_right_legal': DEFAULT_ROAD_HALF_WIDTH_M,
            'd_left_physical': -DEFAULT_ROAD_HALF_WIDTH_M,
            'd_right_physical': DEFAULT_ROAD_HALF_WIDTH_M,
            'is_yellow_solid': False,
            'lane_width': 3.5,
            'lane_id': 0
        }

        if self.carla_map is None:
            return default_res

        try:
            import carla
            loc = carla.Location(x=float(x), y=float(y), z=0.5)
            wp = self.carla_map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            if wp is None:
                return default_res

            # Vector pháp tuyến vuông góc hướng sang PHẢI trong CARLA Frenet:
            # nx = -sin(theta), ny = cos(theta) (d > 0 là BÊN PHẢI, d < 0 là BÊN TRÁI)
            nx = -math.sin(theta_rad)
            ny = math.cos(theta_rad)

            # Độ lệch tâm làn waypoint so với điểm tham chiếu (x, y)
            dx = wp.transform.location.x - x
            dy = wp.transform.location.y - y
            d_center = nx * dx + ny * dy

            curr_half_w = float(wp.lane_width / 2.0)

            # -------------------------------------------------------------
            # 1. Quét tìm mép đường và các làn cùng chiều bên PHẢI (d > 0)
            # -------------------------------------------------------------
            d_right_phys = d_center + curr_half_w
            d_right_legal = d_right_phys
            curr_r = wp

            while True:
                rm = curr_r.right_lane_marking
                is_r_solid = rm and rm.type in (
                    carla.LaneMarkingType.Solid,
                    carla.LaneMarkingType.SolidSolid,
                    carla.LaneMarkingType.SolidBroken
                )

                next_r = curr_r.get_right_lane()
                # Chỉ quét các làn Driving cùng chiều (loại bỏ Shoulder để xe không đi vào làn đỗ xe / lề cỏ)
                if (next_r is not None and
                    next_r.lane_type == carla.LaneType.Driving and
                    (next_r.lane_id * wp.lane_id > 0)):
                    w_r = float(next_r.lane_width)
                    d_right_phys += w_r
                    d_right_legal += w_r
                    curr_r = next_r
                else:
                    # Đã gặp mép ngoài cùng (Shoulder / Sidewalk / Curb / Hộ lan mép phải)
                    break

            # -------------------------------------------------------------
            # 2. Quét tìm mép đường và các làn cùng chiều bên TRÁI (d < 0)
            # -------------------------------------------------------------
            d_left_phys = d_center - curr_half_w
            d_left_legal = d_left_phys
            is_yellow_solid = False
            curr_l = wp

            while True:
                lm = curr_l.left_lane_marking
                is_l_yellow = lm and (lm.color == carla.LaneMarkingColor.Yellow)
                is_l_solid = lm and lm.type in (
                    carla.LaneMarkingType.Solid,
                    carla.LaneMarkingType.SolidSolid,
                    carla.LaneMarkingType.SolidBroken
                )
                is_l_broken = lm and lm.type in (
                    carla.LaneMarkingType.Broken,
                    carla.LaneMarkingType.BrokenSolid,
                    carla.LaneMarkingType.BrokenBroken
                )

                next_l = curr_l.get_left_lane()
                if next_l is not None and next_l.lane_type == carla.LaneType.Driving:
                    if next_l.lane_id * wp.lane_id > 0:
                        # Làn cùng chiều bên trái (Same direction) -> mượn làn hợp pháp
                        w_l = float(next_l.lane_width)
                        d_left_phys -= w_l
                        d_left_legal -= w_l
                        curr_l = next_l
                    else:
                        # Làn ngược chiều (Opposing traffic)
                        if is_l_yellow and is_l_solid:
                            is_yellow_solid = True
                            # Vạch vàng nét liền phân cách làn đối diện -> Dừng biên hợp pháp tại đây
                            break
                        elif is_l_broken:
                            # Vạch đứt cho phép mượn làn đối diện
                            w_l = float(next_l.lane_width)
                            d_left_phys -= w_l
                            d_left_legal -= w_l
                            break
                        else:
                            if is_l_solid:
                                is_yellow_solid = is_l_yellow
                            break
                else:
                    # Gặp vỉa hè / dải phân cách cứng bên trái
                    if is_l_yellow and is_l_solid:
                        is_yellow_solid = True
                    break

            # XỬ LÝ KHÔNG GIAN GIAO LỘ (JUNCTION / INTERSECTION):
            # Trong CARLA OpenDRIVE, các đường kết nối nội bộ trong giao lộ thường chỉ mô hình 1 làn ảo đơn lẻ
            # khiến get_left_lane / get_right_lane trả về None. Tại đây mặt đường thực tế rất rộng rãi,
            # không được phép bóp nghẹt hành lang khiến xe giật lái hoặc cản trở quỹ đạo vào cua.
            if getattr(wp, 'is_junction', False):
                d_left_legal = min(d_left_legal, -5.25)
                d_right_legal = max(d_right_legal, 4.50)
                is_yellow_solid = False

            # Giới hạn an toàn cho tâm xe Ego sau khi trừ clearance (thân xe + lề)
            road_min = d_left_legal + self.r_clearance
            road_max = d_right_legal - self.r_clearance

            # Đảm bảo không gian leeway tối thiểu tự nhiên (+-0.45m) quanh tim đường để MPC bám quỹ đạo thoải mái
            # mà không bao giờ bóp nghẹt thành dải hẹp hay cưỡng ép mở rộng sang dải đỗ xe / vỉa hè
            road_min = min(road_min, -0.45)
            road_max = max(road_max,  0.45)

            # Đảm bảo tính nhất quán: road_min <= road_max - 0.5m
            if road_min > road_max - 0.5:
                road_min = road_max - 0.5

            return {
                'road_min': float(road_min),
                'road_max': float(road_max),
                'd_left_legal': float(d_left_legal),
                'd_right_legal': float(d_right_legal),
                'd_left_physical': float(d_left_phys),
                'd_right_physical': float(d_right_phys),
                'is_yellow_solid': bool(is_yellow_solid),
                'lane_width': float(wp.lane_width),
                'lane_id': int(wp.lane_id)
            }

        except Exception:
            return default_res

    def build_trajectory_corridor(self, x_traj, y_traj, theta_traj):
        """
        Tiền tính toán toàn bộ hành lang dọc theo quỹ đạo toàn cục để truy vấn siêu tốc O(1).
        Áp dụng lọc làm mịn cục bộ (Median / Moving Average Filter) để chống răng cưa tại các điểm nối OpenDRIVE.
        """
        n = len(x_traj)
        self.n_pts = n
        if n == 0 or self.carla_map is None:
            return

        road_min_arr = np.zeros(n, dtype=float)
        road_max_arr = np.zeros(n, dtype=float)
        d_left_arr = np.zeros(n, dtype=float)
        d_right_arr = np.zeros(n, dtype=float)
        yellow_solid_arr = np.zeros(n, dtype=bool)

        for i in range(n):
            res = self.compute_waypoint_corridor(x_traj[i], y_traj[i], theta_traj[i])
            road_min_arr[i] = res['road_min']
            road_max_arr[i] = res['road_max']
            d_left_arr[i] = res['d_left_legal']
            d_right_arr[i] = res['d_right_legal']
            yellow_solid_arr[i] = res['is_yellow_solid']

        # Làm mịn nhẹ (Window 5 waypoints ~ 1.0m) để chống đột biến tại các mối nối đường OpenDRIVE
        def smooth_box(arr, win=5):
            pad = win // 2
            padded = np.pad(arr, pad, mode='edge')
            smoothed = np.convolve(padded, np.ones(win)/win, mode='valid')
            return smoothed

        self.road_min_traj = smooth_box(road_min_arr, win=5)
        self.road_max_traj = smooth_box(road_max_arr, win=5)
        self.d_left_legal_traj = smooth_box(d_left_arr, win=5)
        self.d_right_legal_traj = smooth_box(d_right_arr, win=5)
        self.is_yellow_solid_traj = yellow_solid_arr
        self.x_traj = np.array(x_traj, dtype=float)
        self.y_traj = np.array(y_traj, dtype=float)
        self.theta_traj = np.array(theta_traj, dtype=float)

        dx = np.diff(x_traj)
        dy = np.diff(y_traj)
        self.s_traj = np.concatenate(([0.0], np.cumsum(np.hypot(dx, dy))))

        print(f"[ROAD CORRIDOR] Đã tiền tính toán hành lang động cho {n} waypoints:")
        print(f"  - Biên trái road_min: min={self.road_min_traj.min():.2f}m, max={self.road_min_traj.max():.2f}m")
        print(f"  - Biên phải road_max: min={self.road_max_traj.min():.2f}m, max={self.road_max_traj.max():.2f}m")

    def get_corridor_at_idx(self, idx: int) -> Tuple[float, float, bool]:
        """Truy vấn biên hành lang tại chỉ số waypoint (O(1)) -> (road_min, road_max, is_yellow_solid)"""
        if self.road_min_traj is None or self.n_pts == 0:
            return -DEFAULT_ROAD_HALF_WIDTH_M + self.r_clearance, DEFAULT_ROAD_HALF_WIDTH_M - self.r_clearance, False
        clamped_i = max(0, min(self.n_pts - 1, int(idx)))
        return (
            float(self.road_min_traj[clamped_i]),
            float(self.road_max_traj[clamped_i]),
            bool(self.is_yellow_solid_traj[clamped_i])
        )

    def get_corridor_at_s(self, s_query: float) -> Tuple[float, float, bool]:
        """Truy vấn biên hành lang theo khoảng cách dọc s (m) trên quỹ đạo -> (road_min, road_max, is_yellow_solid)"""
        if self.s_traj is None or self.n_pts == 0:
            return -DEFAULT_ROAD_HALF_WIDTH_M + self.r_clearance, DEFAULT_ROAD_HALF_WIDTH_M - self.r_clearance, False
        idx = int(np.clip(np.searchsorted(self.s_traj, s_query), 0, self.n_pts - 1))
        return self.get_corridor_at_idx(idx)

    def get_horizon_corridor(self, current_idx: int, horizon_len: int = 10) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Trả về mảng biên (road_min_h, road_max_h, is_yellow_solid_h) độ dài horizon_len cho QP-MPC
        """
        if self.road_min_traj is None or self.n_pts == 0:
            rm = np.full(horizon_len, -DEFAULT_ROAD_HALF_WIDTH_M + self.r_clearance)
            rx = np.full(horizon_len, DEFAULT_ROAD_HALF_WIDTH_M - self.r_clearance)
            ys = np.full(horizon_len, False)
            return rm, rx, ys

        idx_arr = np.clip(np.arange(current_idx, current_idx + horizon_len), 0, self.n_pts - 1)
        return (
            self.road_min_traj[idx_arr].copy(),
            self.road_max_traj[idx_arr].copy(),
            self.is_yellow_solid_traj[idx_arr].copy()
        )

    def get_horizon_corridor_at_s(self, s_array: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Truy vấn mảng biên (road_min_h, road_max_h, is_yellow_solid_h) theo tọa độ cung s chính xác của từng bước dự báo
        """
        h_len = len(s_array)
        if self.road_min_traj is None or self.s_traj is None or self.n_pts == 0:
            rm = np.full(h_len, -DEFAULT_ROAD_HALF_WIDTH_M + self.r_clearance)
            rx = np.full(h_len, DEFAULT_ROAD_HALF_WIDTH_M - self.r_clearance)
            ys = np.full(h_len, False)
            return rm, rx, ys

        idx_arr = np.clip(np.searchsorted(self.s_traj, s_array), 0, self.n_pts - 1)
        return (
            self.road_min_traj[idx_arr].copy(),
            self.road_max_traj[idx_arr].copy(),
            self.is_yellow_solid_traj[idx_arr].copy()
        )

    def draw_debug_corridor(self, carla_sim, current_idx: int, lookahead_pts: int = 40, life_time: float = 0.2):
        """Vẽ 2 đường biên xanh lá/cyan và magenta hiển thị hành lang mặt đường hợp pháp trên CARLA 3D"""
        if self.d_left_legal_traj is None or self.x_traj is None or carla_sim is None or carla_sim.world is None:
            return

        try:
            import carla
            debug = carla_sim.world.debug
            end_idx = min(self.n_pts, current_idx + lookahead_pts)
            if end_idx <= current_idx + 1:
                return

            color_left = carla.Color(0, 255, 255)    # Cyan cho mép trái hợp pháp
            color_right = carla.Color(255, 0, 255)   # Magenta cho mép phải hợp pháp

            for i in range(current_idx, end_idx - 1):
                # Waypoint i
                th_i = self.theta_traj[i]
                nx_i = -math.sin(th_i)
                ny_i = math.cos(th_i)
                # Left point i
                lx_i = self.x_traj[i] + self.d_left_legal_traj[i] * nx_i
                ly_i = self.y_traj[i] + self.d_left_legal_traj[i] * ny_i
                # Right point i
                rx_i = self.x_traj[i] + self.d_right_legal_traj[i] * nx_i
                ry_i = self.y_traj[i] + self.d_right_legal_traj[i] * ny_i

                # Waypoint i+1
                th_next = self.theta_traj[i+1]
                nx_next = -math.sin(th_next)
                ny_next = math.cos(th_next)
                # Left point i+1
                lx_next = self.x_traj[i+1] + self.d_left_legal_traj[i+1] * nx_next
                ly_next = self.y_traj[i+1] + self.d_left_legal_traj[i+1] * ny_next
                # Right point i+1
                rx_next = self.x_traj[i+1] + self.d_right_legal_traj[i+1] * nx_next
                ry_next = self.y_traj[i+1] + self.d_right_legal_traj[i+1] * ny_next

                debug.draw_line(
                    carla.Location(x=lx_i, y=ly_i, z=0.3),
                    carla.Location(x=lx_next, y=ly_next, z=0.3),
                    thickness=0.08, color=color_left, life_time=life_time
                )
                debug.draw_line(
                    carla.Location(x=rx_i, y=ry_i, z=0.3),
                    carla.Location(x=rx_next, y=ry_next, z=0.3),
                    thickness=0.08, color=color_right, life_time=life_time
                )
        except Exception:
            pass
