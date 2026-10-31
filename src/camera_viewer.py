from collections import deque
import math
import numpy as np
import carla
import pygame

from src.config import (
    COCKPIT_WINDOW_WIDTH,
    COCKPIT_WINDOW_HEIGHT,
    TOP_BAR_HEIGHT,
    MAIN_VIEW_WIDTH,
    MAIN_VIEW_HEIGHT,
    SUB_VIEW_WIDTH,
    SUB_VIEW_HEIGHT,
    PLOT_WIDTH,
    PLOT_HEIGHT,
    TELEMETRY_HISTORY_LEN,
)


class TelemetryPlotter:
    """
    Bộ vẽ 2 đồ thị thời gian thực (Realtime Telemetry Plots) bằng Pygame Primitives:
    - Đồ thị 1 (Trái - 720x240): Vận tốc thực tế (v_ego) vs Vận tốc tham chiếu (v_ref) [km/h].
    - Đồ thị 2 (Phải - 720x240): Độ giật dọc xe Jerk (da/dt) [m/s^3] với dải phân vùng êm ái.
    - Không sử dụng Matplotlib -> tốc độ vẽ 60+ FPS, độ trễ ~0ms.
    """

    def __init__(self, history_len: int = TELEMETRY_HISTORY_LEN):
        self.history_len = history_len
        self.v_history = deque(maxlen=history_len)
        self.v_ref_history = deque(maxlen=history_len)
        self.jerk_history = deque(maxlen=history_len)

        self.font_title = pygame.font.SysFont('DejaVu Sans', 13, bold=True)
        self.font_axis = pygame.font.SysFont('DejaVu Sans', 11)
        self.font_value = pygame.font.SysFont('DejaVu Sans', 12, bold=True)

    def update(self, v_kmh: float, v_ref_kmh: float, jerk_ms3: float):
        """Cập nhật mẫu dữ liệu mới vào hàng đợi cửa sổ trượt"""
        self.v_history.append(float(v_kmh))
        self.v_ref_history.append(float(v_ref_kmh))
        self.jerk_history.append(float(jerk_ms3))

    @staticmethod
    def _draw_dashed_line(surface, color, start_pos, end_pos, width=1, dash_length=6):
        """Vẽ đoạn thẳng nét đứt trên Pygame surface"""
        x1, y1 = start_pos
        x2, y2 = end_pos
        dl = math.hypot(x2 - x1, y2 - y1)
        if dl < 1e-3:
            return
        dashes = max(1, int(dl // (dash_length * 2)))
        for i in range(dashes + 1):
            t_start = (i * 2 * dash_length) / dl
            t_end = min(1.0, ((i * 2 + 1) * dash_length) / dl)
            if t_start >= 1.0:
                break
            sx = int(x1 + (x2 - x1) * t_start)
            sy = int(y1 + (y2 - y1) * t_start)
            ex = int(x1 + (x2 - x1) * t_end)
            ey = int(y1 + (y2 - y1) * t_end)
            pygame.draw.line(surface, color, (sx, sy), (ex, ey), width)

    def draw_speed_plot(self, surface: pygame.Surface, rect: pygame.Rect):
        """
        Vẽ Đồ thị 1: Vận tốc thực tế (v_ego) vs Vận tốc tham chiếu (v_ref) [km/h]
        - Trục tung: 0 -> 40 km/h (lưới ngang mỗi 10 km/h).
        - Trục hoành: Cửa sổ trượt 100 mẫu (10 giây tại dt=0.1s).
        """
        # Nền và khung viền đồ thị
        pygame.draw.rect(surface, (12, 17, 26), rect)
        pygame.draw.rect(surface, (30, 65, 90), rect, 1)

        # Vùng vẽ đường biểu diễn bên trong (chừa lề cho nhãn trục và tiêu đề)
        pad_left = 44
        pad_right = 16
        pad_top = 32
        pad_bottom = 24
        plot_x = rect.x + pad_left
        plot_y = rect.y + pad_top
        plot_w = rect.width - pad_left - pad_right
        plot_h = rect.height - pad_top - pad_bottom

        # Khung nền vùng plot
        plot_rect = pygame.Rect(plot_x, plot_y, plot_w, plot_h)
        pygame.draw.rect(surface, (8, 12, 20), plot_rect)

        # Giới hạn trục tung (tối thiểu 40 km/h)
        max_v = 40.0
        if self.v_history or self.v_ref_history:
            peak = max(
                max(self.v_history, default=0.0),
                max(self.v_ref_history, default=0.0),
            )
            if peak > 38.0:
                max_v = math.ceil(peak / 10.0) * 10.0

        # Vẽ lưới ngang (mỗi 10 km/h)
        step_v = 10.0
        val = 0.0
        while val <= max_v + 0.1:
            ratio = val / max_v
            gy = int(plot_y + plot_h - ratio * plot_h)
            grid_color = (45, 58, 75) if val in (0.0, max_v) else (25, 34, 48)
            pygame.draw.line(surface, grid_color, (plot_x, gy), (plot_x + plot_w, gy), 1)
            lbl = self.font_axis.render(f"{int(val)}", True, (130, 145, 165))
            surface.blit(lbl, (rect.x + 8, gy - 7))
            val += step_v

        # Vẽ lưới dọc (mỗi 25 mẫu = 2.5 giây)
        for idx_step in range(0, self.history_len + 1, 25):
            gx = int(plot_x + (idx_step / max(1, self.history_len - 1)) * plot_w)
            pygame.draw.line(surface, (22, 30, 42), (gx, plot_y), (gx, plot_y + plot_h), 1)
            sec_ago = -((self.history_len - idx_step) * 0.1)
            t_lbl = self.font_axis.render(f"{sec_ago:.1f}s" if idx_step < self.history_len else "0.0s", True, (110, 125, 145))
            surface.blit(t_lbl, (gx - 14, plot_y + plot_h + 4))

        # Tiêu đề và chú giải
        title_surf = self.font_title.render("ĐỒ THỊ 1: VẬN TỐC XE (v_ego) vs THAM CHIẾU (v_ref) [km/h]", True, (0, 220, 255))
        surface.blit(title_surf, (rect.x + 12, rect.y + 7))

        # Chỉ số tức thời ở góc phải trên của đồ thị
        curr_v = self.v_history[-1] if self.v_history else 0.0
        curr_v_ref = self.v_ref_history[-1] if self.v_ref_history else 0.0
        dv = curr_v - curr_v_ref
        dv_color = (0, 240, 180) if abs(dv) <= 3.5 else (255, 200, 60)
        stat_str = f"v: {curr_v:4.1f} | v_ref: {curr_v_ref:4.1f} | Δv: {dv:+4.1f} km/h"
        stat_surf = self.font_value.render(stat_str, True, dv_color)
        surface.blit(stat_surf, (rect.right - stat_surf.get_width() - 12, rect.y + 8))

        # Vẽ đường v_ref (Nét đứt màu Vàng Cam) và v_ego (Nét liền màu Cyan sáng)
        n_pts = len(self.v_history)
        if n_pts >= 2:
            pts_v = []
            pts_ref = []
            denom = max(1, self.history_len - 1)
            x_offset = self.history_len - n_pts
            for i in range(n_pts):
                px = int(plot_x + ((x_offset + i) / denom) * plot_w)
                v_clamped = max(0.0, min(max_v, self.v_history[i]))
                vr_clamped = max(0.0, min(max_v, self.v_ref_history[i]))
                py_v = int(plot_y + plot_h - (v_clamped / max_v) * plot_h)
                py_r = int(plot_y + plot_h - (vr_clamped / max_v) * plot_h)
                pts_v.append((px, py_v))
                pts_ref.append((px, py_r))

            # Đường v_ref nét đứt màu Vàng Cam (255, 170, 0)
            for i in range(len(pts_ref) - 1):
                if i % 2 == 0:
                    pygame.draw.line(surface, (255, 170, 0), pts_ref[i], pts_ref[i + 1], 2)

            # Đường v_ego nét liền màu Cyan sáng (0, 240, 255) dày 2px
            pygame.draw.lines(surface, (0, 240, 255), False, pts_v, 2)
            pygame.draw.circle(surface, (0, 255, 255), pts_v[-1], 4)

        pygame.draw.rect(surface, (45, 70, 95), plot_rect, 1)

    def draw_jerk_plot(self, surface: pygame.Surface, rect: pygame.Rect):
        """
        Vẽ Đồ thị 2: Độ giật dọc xe Jerk (j = da/dt) [m/s^3]
        - Trục tung: Đối xứng từ -5.0 m/s^3 đến +5.0 m/s^3 (vạch 0.0 ở chính giữa).
        - Dải phân vùng an toàn:
            + Xanh (|j| <= 2.0): Vùng êm ái.
            + Vàng (2.0 < |j| <= 3.5): Chuyển làn / tăng giảm tốc vừa.
            + Đỏ (|j| > 3.5): Phanh gấp / giật cục.
        """
        pygame.draw.rect(surface, (12, 17, 26), rect)
        pygame.draw.rect(surface, (30, 65, 90), rect, 1)

        pad_left = 46
        pad_right = 16
        pad_top = 32
        pad_bottom = 24
        plot_x = rect.x + pad_left
        plot_y = rect.y + pad_top
        plot_w = rect.width - pad_left - pad_right
        plot_h = rect.height - pad_top - pad_bottom

        plot_rect = pygame.Rect(plot_x, plot_y, plot_w, plot_h)
        pygame.draw.rect(surface, (8, 12, 20), plot_rect)

        max_j = 5.0  # Thang đo đối xứng [-5.0, +5.0] m/s^3

        def j_to_y(j_val: float) -> int:
            j_clamped = max(-max_j, min(max_j, j_val))
            ratio = (j_clamped + max_j) / (2.0 * max_j)  # 0.0 ở -5, 1.0 ở +5
            return int(plot_y + plot_h - ratio * plot_h)

        # Dải xanh mờ vùng êm ái (|j| <= 2.0 m/s^3)
        y_pos2 = j_to_y(2.0)
        y_neg2 = j_to_y(-2.0)
        comfort_band = pygame.Surface((plot_w, max(1, y_neg2 - y_pos2)), pygame.SRCALPHA)
        comfort_band.fill((0, 180, 100, 26))
        surface.blit(comfort_band, (plot_x, y_pos2))

        # Vạch ngưỡng cảnh báo Vàng (|j| = 2.0) và Đỏ (|j| = 3.5)
        for j_thr, col in [
            (2.0, (140, 120, 30)),
            (-2.0, (140, 120, 30)),
            (3.5, (150, 45, 45)),
            (-3.5, (150, 45, 45)),
        ]:
            ty = j_to_y(j_thr)
            self._draw_dashed_line(surface, col, (plot_x, ty), (plot_x + plot_w, ty), width=1, dash_length=5)

        # Vạch trung tâm 0.0 m/s^3
        y_zero = j_to_y(0.0)
        pygame.draw.line(surface, (120, 120, 120), (plot_x, y_zero), (plot_x + plot_w, y_zero), 1)

        # Nhãn trục tung
        for j_lbl in (-5.0, -3.5, -2.0, 0.0, 2.0, 3.5, 5.0):
            ly = j_to_y(j_lbl)
            lbl = self.font_axis.render(f"{j_lbl:+.1f}" if j_lbl != 0.0 else " 0.0", True, (130, 145, 165))
            surface.blit(lbl, (rect.x + 6, ly - 7))

        # Lưới dọc theo thời gian
        for idx_step in range(0, self.history_len + 1, 25):
            gx = int(plot_x + (idx_step / max(1, self.history_len - 1)) * plot_w)
            pygame.draw.line(surface, (22, 30, 42), (gx, plot_y), (gx, plot_y + plot_h), 1)
            sec_ago = -((self.history_len - idx_step) * 0.1)
            t_lbl = self.font_axis.render(f"{sec_ago:.1f}s" if idx_step < self.history_len else "0.0s", True, (110, 125, 145))
            surface.blit(t_lbl, (gx - 14, plot_y + plot_h + 4))

        # Tiêu đề
        title_surf = self.font_title.render("ĐỒ THỊ 2: ĐỘ GIẬT JERK (da/dt) [m/s³]", True, (0, 220, 255))
        surface.blit(title_surf, (rect.x + 12, rect.y + 7))

        # Chỉ số tức thời & phân loại trạng thái êm ái
        curr_j = self.jerk_history[-1] if self.jerk_history else 0.0
        abs_j = abs(curr_j)
        if abs_j <= 2.0:
            j_color = (0, 235, 140)
            j_tag = "ÊM ÁI"
        elif abs_j <= 3.5:
            j_color = (255, 200, 40)
            j_tag = "CHUYỂN LÀN / HÃM VỪA"
        else:
            j_color = (255, 65, 65)
            j_tag = "GIẬT MẠNH / PHANH GẤP"

        stat_str = f"Jerk: {curr_j:+5.2f} m/s³ [{j_tag}]"
        stat_surf = self.font_value.render(stat_str, True, j_color)
        surface.blit(stat_surf, (rect.right - stat_surf.get_width() - 12, rect.y + 8))

        # Vẽ đường cong Jerk đổi màu linh hoạt theo |j|
        n_pts = len(self.jerk_history)
        if n_pts >= 2:
            denom = max(1, self.history_len - 1)
            x_offset = self.history_len - n_pts
            pts_j = []
            for i in range(n_pts):
                px = int(plot_x + ((x_offset + i) / denom) * plot_w)
                py = j_to_y(self.jerk_history[i])
                pts_j.append((px, py, abs(self.jerk_history[i])))

            for i in range(len(pts_j) - 1):
                x1, y1, aj1 = pts_j[i]
                x2, y2, aj2 = pts_j[i + 1]
                seg_j = max(aj1, aj2)
                if seg_j <= 2.0:
                    seg_col = (0, 230, 140)
                elif seg_j <= 3.5:
                    seg_col = (255, 200, 40)
                else:
                    seg_col = (255, 60, 60)
                pygame.draw.line(surface, seg_col, (x1, y1), (x2, y2), 2)

            last_x, last_y, _ = pts_j[-1]
            pygame.draw.circle(surface, j_color, (last_x, last_y), 4)

        pygame.draw.rect(surface, (45, 70, 95), plot_rect, 1)


class CameraViewer:
    """
    Cửa sổ Cockpit Dashboard Đa Màn Hình (1440 x 900 px - Tỷ lệ 16:10):
    - Top Bar (1440 x 60 px): Hợp nhất 3 cụm thông số (Động học, Trạng thái MPC, Cảm biến & Vạch vàng).
    - Khu vực Camera (1440 x 600 px):
        + Main View (Chase Cam): 960 x 600 px bên trái.
        + Camera phụ 1 (Bird-Eye View): 480 x 300 px góc trên bên phải.
        + Camera phụ 2 (Front View / Hood-Cam): 480 x 300 px góc dưới bên phải (cấp ảnh trực tiếp cho Yellow Line Detector).
    - Khu vực 2 Đồ thị Realtime (1440 x 240 px):
        + Đồ thị 1 (720 x 240 px): Vận tốc thực tế v_ego vs v_ref.
        + Đồ thị 2 (720 x 240 px): Độ giật dọc xe Jerk (da/dt - m/s^3).
    """

    def __init__(self, world, ego_vehicle, width: int = COCKPIT_WINDOW_WIDTH, height: int = COCKPIT_WINDOW_HEIGHT):
        self.world = world
        self.ego_vehicle = ego_vehicle
        self.width = COCKPIT_WINDOW_WIDTH
        self.height = COCKPIT_WINDOW_HEIGHT

        # Khởi tạo Pygame
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(
            (self.width, self.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        pygame.display.set_caption(
            "CARLA Autonomous MPC — Cockpit Multi-Camera & Realtime Telemetry Dashboard ([M]: Ẩn/Hiện HUD | [ESC]: Đóng)"
        )

        # Fonts cho Top Bar và nhãn Camera
        self.font_main = pygame.font.SysFont('DejaVu Sans', 15, bold=True)
        self.font_sub = pygame.font.SysFont('DejaVu Sans', 12)
        self.font_cam_tag = pygame.font.SysFont('DejaVu Sans', 12, bold=True)

        # 3 Pygame Surfaces độc lập cho 3 góc nhìn Camera
        self.surface_main = None
        self.surface_bev = None
        self.surface_front = None

        # Buffer RGB NumPy (H, W, 3) từ Front View cấp trực tiếp cho Yellow Line Detector
        self.front_rgb_frame = None
        # Tương thích ngược nếu module bên ngoài truy cập camera_viewer.surface
        self.surface = None

        # 3 Sensor Actors trong CARLA
        self.chase_sensor = None
        self.bev_sensor = None
        self.front_sensor = None

        self._is_active = True
        self.show_hud = True

        # Bộ vẽ 2 đồ thị Realtime
        self.plotter = TelemetryPlotter(history_len=TELEMETRY_HISTORY_LEN)

        # Khởi tạo 3 cảm biến Camera gắn lên xe Ego
        self._spawn_cameras()

    def _spawn_cameras(self):
        """Khởi tạo đồng thời 3 cảm biến Camera RGB (Chase, Bird-Eye, Front Hood-Cam)"""
        bp_lib = self.world.get_blueprint_library()

        # 1. Sensor 1: Main View - Chase Cam (960 x 600, FOV 90)
        chase_bp = bp_lib.find('sensor.camera.rgb')
        chase_bp.set_attribute('image_size_x', str(MAIN_VIEW_WIDTH))
        chase_bp.set_attribute('image_size_y', str(MAIN_VIEW_HEIGHT))
        chase_bp.set_attribute('fov', '90')
        chase_tf = carla.Transform(
            carla.Location(x=-6.5, z=2.8),
            carla.Rotation(pitch=-12.0, yaw=0.0)
        )
        self.chase_sensor = self.world.spawn_actor(chase_bp, chase_tf, attach_to=self.ego_vehicle)
        self.chase_sensor.listen(self._on_chase_image)

        # 2. Sensor 2: Sub 1 - Bird-Eye View (480 x 300, FOV 90)
        bev_bp = bp_lib.find('sensor.camera.rgb')
        bev_bp.set_attribute('image_size_x', str(SUB_VIEW_WIDTH))
        bev_bp.set_attribute('image_size_y', str(SUB_VIEW_HEIGHT))
        bev_bp.set_attribute('fov', '90')
        bev_tf = carla.Transform(
            carla.Location(x=-1.5, z=18.0),
            carla.Rotation(pitch=-80.0, yaw=0.0)
        )
        self.bev_sensor = self.world.spawn_actor(bev_bp, bev_tf, attach_to=self.ego_vehicle)
        self.bev_sensor.listen(self._on_bev_image)

        # 3. Sensor 3: Sub 2 - Front View / Hood-Cam (480 x 300, FOV 90)
        front_bp = bp_lib.find('sensor.camera.rgb')
        front_bp.set_attribute('image_size_x', str(SUB_VIEW_WIDTH))
        front_bp.set_attribute('image_size_y', str(SUB_VIEW_HEIGHT))
        front_bp.set_attribute('fov', '90')
        front_tf = carla.Transform(
            carla.Location(x=2.2, z=1.2),
            carla.Rotation(pitch=-5.0, yaw=0.0)
        )
        self.front_sensor = self.world.spawn_actor(front_bp, front_tf, attach_to=self.ego_vehicle)
        self.front_sensor.listen(self._on_front_image)

        print(
            f"[COCKPIT DASHBOARD] Đã kích hoạt 3 Camera ({MAIN_VIEW_WIDTH}x{MAIN_VIEW_HEIGHT} Chase + "
            f"2x {SUB_VIEW_WIDTH}x{SUB_VIEW_HEIGHT} BEV/Front) & 2 Đồ thị Realtime ({self.width}x{self.height})."
        )

    @staticmethod
    def _carla_image_to_rgb(image) -> np.ndarray:
        """Chuyển đổi CARLA raw_data (BGRA) sang mảng RGB uint8 (H, W, 3)"""
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        rgb = array[:, :, :3][:, :, ::-1]
        return rgb

    def _on_chase_image(self, image):
        if not self._is_active:
            return
        rgb = self._carla_image_to_rgb(image)
        self.surface_main = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))

    def _on_bev_image(self, image):
        if not self._is_active:
            return
        rgb = self._carla_image_to_rgb(image)
        self.surface_bev = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))

    def _on_front_image(self, image):
        if not self._is_active:
            return
        rgb = self._carla_image_to_rgb(image)
        self.front_rgb_frame = np.ascontiguousarray(rgb)
        self.surface_front = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
        self.surface = self.surface_front

    def get_front_frame(self):
        """Trả về mảng ảnh RGB (H, W, 3) mới nhất từ Front View (Hood-Cam) cho Yellow Line Detector"""
        return self.front_rgb_frame

    def _draw_cam_label(self, x: int, y: int, text: str, accent_color=(0, 210, 255)):
        """Vẽ nhãn góc trên mỗi khung nhìn camera"""
        if not self.show_hud:
            return
        txt_surf = self.font_cam_tag.render(text, True, (235, 245, 255))
        bg_w = txt_surf.get_width() + 18
        bg_h = 24
        badge = pygame.Surface((bg_w, bg_h), pygame.SRCALPHA)
        badge.fill((10, 15, 25, 195))
        pygame.draw.rect(badge, accent_color, (0, 0, 4, bg_h))
        self.display.blit(badge, (x + 8, y + 8))
        self.display.blit(txt_surf, (x + 18, y + 12))

    def _draw_top_bar(self, hud_info: dict):
        """
        Vẽ Frame [TOP BAR] (1440 x 60 px tại y = 0..60):
        - Cụm 1 (Động học xe): Tốc độ thực tế, Mục tiêu, Góc lái + mũi tên hướng lái.
        - Cụm 2 (Trạng thái điều khiển MPC): Nhãn chế độ & Tiến độ quỹ đạo s (m) và %.
        - Cụm 3 (Nhận thức & Cảm biến): Số vật cản, Vạch vàng (SOLID/DASHED/NONE), Phím tắt.
        """
        top_surface = pygame.Surface((self.width, TOP_BAR_HEIGHT), pygame.SRCALPHA)
        top_surface.fill((10, 15, 25, 230))
        self.display.blit(top_surface, (0, 0))

        # Viền dưới màu Cyan công nghệ (hoặc Đỏ chớp nếu đang phanh khẩn cấp Chế độ 2)
        is_emerg = hud_info.get('is_emergency', False)
        border_col = (255, 40, 40) if is_emerg else (0, 180, 220)
        pygame.draw.line(self.display, border_col, (0, TOP_BAR_HEIGHT - 2), (self.width, TOP_BAR_HEIGHT - 2), 2)

        # Vạch ngăn cách dọc giữa 3 cụm thông số
        pygame.draw.line(self.display, (35, 60, 85), (465, 8), (465, TOP_BAR_HEIGHT - 8), 1)
        pygame.draw.line(self.display, (35, 60, 85), (960, 8), (960, TOP_BAR_HEIGHT - 8), 1)

        # --- CỤM 1: ĐỘNG HỌC XE (x = 15..455) ---
        speed_kmh = hud_info.get('speed_kmh', 0.0)
        target_kmh = hud_info.get('target_kmh', 0.0)
        steer_deg = hud_info.get('steer_deg', 0.0)

        speed_color = (0, 255, 180) if abs(speed_kmh - target_kmh) <= 5.0 else (255, 205, 50)
        if abs(steer_deg) < 0.8:
            steer_arrow = "↑"
        elif steer_deg > 0:
            steer_arrow = "→"
        else:
            steer_arrow = "←"

        c1_line1 = self.font_main.render(
            f"Tốc độ: {speed_kmh:4.1f} km/h  |  Góc lái: {steer_deg:+5.1f}° {steer_arrow}",
            True,
            speed_color
        )
        c1_line2 = self.font_sub.render(
            f"Mục tiêu: {target_kmh:4.1f} km/h  |  Sai lệch Δv: {speed_kmh - target_kmh:+4.1f} km/h",
            True,
            (185, 200, 220)
        )
        self.display.blit(c1_line1, (15, 8))
        self.display.blit(c1_line2, (15, 34))

        # --- CỤM 2: TRẠNG THÁI ĐIỀU KHIỂN MPC (x = 480..945) ---
        bypass_text = hud_info.get('bypass_text', 'CHẾ ĐỘ 1: BÁM ĐƯỜNG')
        if "PHANH GẤP" in bypass_text or is_emerg:
            bypass_color = (255, 55, 55)
        elif "ROADBLOCK" in bypass_text or "DỪNG" in bypass_text:
            bypass_color = (255, 110, 60)
        elif "NÉ" in bypass_text:
            bypass_color = (255, 180, 40)
        else:
            bypass_color = (70, 225, 255)

        s_ego_m = hud_info.get('s_ego_m', None)
        s_total_m = hud_info.get('s_total_m', None)
        current_idx = hud_info.get('current_idx', 0)
        total_idx = max(1, hud_info.get('total_idx', 1))

        if s_ego_m is not None and s_total_m is not None and s_total_m > 0.1:
            pct = min(100.0, max(0.0, (s_ego_m / s_total_m) * 100.0))
            prog_str = f"Tiến độ: s={s_ego_m:.1f}m / {s_total_m:.1f}m ({pct:.0f}%)  [WP: {current_idx}/{total_idx}]"
        else:
            pct = (current_idx / total_idx) * 100.0
            prog_str = f"Tiến độ: WP {current_idx}/{total_idx} ({pct:.0f}%)"

        c2_line1 = self.font_main.render(f"MPC: {bypass_text}", True, bypass_color)
        c2_line2 = self.font_sub.render(prog_str, True, (185, 200, 220))
        self.display.blit(c2_line1, (480, 8))
        self.display.blit(c2_line2, (480, 34))

        # --- CỤM 3: NHẬN THỨC & CẢM BIẾN (x = 975..1430) ---
        obs_count = hud_info.get('obs_count', 0)
        yellow_info = hud_info.get('yellow_line', '---')
        if 'SOLID' in yellow_info:
            yl_color = (255, 230, 0)
        elif 'DASHED' in yellow_info:
            yl_color = (255, 200, 80)
        else:
            yl_color = (160, 170, 185)

        c3_line1 = self.font_main.render(
            f"Vật cản: {obs_count}  |  Vạch vàng: {yellow_info}",
            True,
            yl_color
        )
        c3_line2 = self.font_sub.render(
            "[ESC]: Đóng cửa sổ  |  [M]: Ẩn/Hiện nhãn HUD",
            True,
            (150, 165, 185)
        )
        self.display.blit(c3_line1, (975, 8))
        self.display.blit(c3_line2, (975, 34))

    def render(self, hud_info=None):
        """
        Cập nhật toàn bộ giao diện Cockpit Dashboard (1440 x 900):
        1. Xử lý phím bấm (ESC/Q để thoát, M để ẩn/hiện HUD).
        2. Vẽ 3 khung nhìn Camera (Main Chase Cam 960x600 + BEV 480x300 + Front 480x300).
        3. Cập nhật & vẽ 2 Đồ thị Realtime ngang nhau (720x240 x 2).
        4. Vẽ thanh Top Bar hợp nhất (1440x60).
        """
        if not self._is_active:
            return True

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._is_active = False
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    self._is_active = False
                    return False
                elif event.key == pygame.K_m:
                    self.show_hud = not self.show_hud

        self.display.fill((10, 14, 22))

        cam_y = TOP_BAR_HEIGHT  # y = 60

        # 1. Vẽ MAIN VIEW: CHASE CAM (960 x 600 tại x=0, y=60)
        if self.surface_main is not None:
            self.display.blit(self.surface_main, (0, cam_y))
        else:
            pygame.draw.rect(self.display, (18, 22, 30), (0, cam_y, MAIN_VIEW_WIDTH, MAIN_VIEW_HEIGHT))
            wait_txt = self.font_main.render("Đang kết nối Main Chase Camera (960x600)...", True, (180, 190, 205))
            self.display.blit(wait_txt, (MAIN_VIEW_WIDTH // 2 - 170, cam_y + MAIN_VIEW_HEIGHT // 2 - 10))
        self._draw_cam_label(0, cam_y, "MAIN VIEW: CHASE CAM (960×600)", accent_color=(0, 210, 255))

        # 2. Vẽ CAMERA PHỤ 1: BIRD-EYE VIEW (480 x 300 tại x=960, y=60)
        sub_x = MAIN_VIEW_WIDTH  # x = 960
        if self.surface_bev is not None:
            self.display.blit(self.surface_bev, (sub_x, cam_y))
        else:
            pygame.draw.rect(self.display, (16, 20, 28), (sub_x, cam_y, SUB_VIEW_WIDTH, SUB_VIEW_HEIGHT))
            wait_bev = self.font_sub.render("Đang kết nối Bird-Eye Cam (480x300)...", True, (170, 180, 195))
            self.display.blit(wait_bev, (sub_x + 115, cam_y + SUB_VIEW_HEIGHT // 2 - 8))
        self._draw_cam_label(sub_x, cam_y, "CAM PHỤ 1: BIRD-EYE VIEW (480×300)", accent_color=(0, 230, 150))

        # 3. Vẽ CAMERA PHỤ 2: FRONT VIEW / HOOD-CAM (480 x 300 tại x=960, y=360)
        front_y = cam_y + SUB_VIEW_HEIGHT  # y = 360
        if self.surface_front is not None:
            self.display.blit(self.surface_front, (sub_x, front_y))
        else:
            pygame.draw.rect(self.display, (16, 20, 28), (sub_x, front_y, SUB_VIEW_WIDTH, SUB_VIEW_HEIGHT))
            wait_front = self.font_sub.render("Đang kết nối Front Hood-Cam (480x300)...", True, (170, 180, 195))
            self.display.blit(wait_front, (sub_x + 110, front_y + SUB_VIEW_HEIGHT // 2 - 8))
        self._draw_cam_label(
            sub_x, front_y,
            "CAM PHỤ 2: FRONT VIEW / YELLOW DETECTOR (480×300)",
            accent_color=(255, 215, 0)
        )

        # Đường phân cách mảnh giữa các khung nhìn Camera
        pygame.draw.line(self.display, (0, 160, 200), (sub_x, cam_y), (sub_x, cam_y + MAIN_VIEW_HEIGHT), 2)
        pygame.draw.line(self.display, (0, 160, 200), (sub_x, front_y), (self.width, front_y), 2)

        # 4. Cập nhật dữ liệu và vẽ 2 Đồ thị Realtime (y = 660..900)
        plot_y = TOP_BAR_HEIGHT + MAIN_VIEW_HEIGHT  # y = 660
        if hud_info is not None:
            v_kmh = hud_info.get('speed_kmh', 0.0)
            v_ref_kmh = hud_info.get('target_kmh', 0.0)
            jerk_ms3 = hud_info.get('jerk_ms3', 0.0)
            self.plotter.update(v_kmh, v_ref_kmh, jerk_ms3)

        rect_speed_plot = pygame.Rect(0, plot_y, PLOT_WIDTH, PLOT_HEIGHT)
        rect_jerk_plot = pygame.Rect(PLOT_WIDTH, plot_y, PLOT_WIDTH, PLOT_HEIGHT)
        self.plotter.draw_speed_plot(self.display, rect_speed_plot)
        self.plotter.draw_jerk_plot(self.display, rect_jerk_plot)

        # 5. Vẽ thanh thông số hợp nhất Top Bar (y = 0..60)
        if hud_info is not None and self.show_hud:
            self._draw_top_bar(hud_info)
        elif hud_info is not None and not self.show_hud:
            # Khi ẩn HUD vẫn giữ nền tối gọn cho dải Top Bar
            pygame.draw.rect(self.display, (10, 15, 25), (0, 0, self.width, TOP_BAR_HEIGHT))
            hint = self.font_sub.render("Đã ẩn HUD — Nhấn [M] để hiện lại thanh thông số | [ESC]: Đóng", True, (140, 155, 175))
            self.display.blit(hint, (15, 22))

        pygame.display.flip()
        return True

    def destroy(self):
        """Giải phóng an toàn cả 3 cảm biến Camera CARLA và đóng cửa sổ Pygame"""
        self._is_active = False
        sensors = [
            ("Chase Cam", self.chase_sensor),
            ("Bird-Eye Cam", self.bev_sensor),
            ("Front Hood-Cam", self.front_sensor),
        ]
        for name, sensor in sensors:
            if sensor is not None:
                try:
                    if sensor.is_alive:
                        sensor.stop()
                        sensor.destroy()
                except Exception as e:
                    print(f"[COCKPIT DASHBOARD] Cảnh báo khi giải phóng {name}: {e}")
        self.chase_sensor = None
        self.bev_sensor = None
        self.front_sensor = None
        print("[COCKPIT DASHBOARD] Đã giải phóng toàn bộ 3 sensor camera an toàn.")
        try:
            pygame.quit()
        except Exception:
            pass
