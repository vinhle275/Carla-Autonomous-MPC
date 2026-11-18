#!/usr/bin/env python3
"""
==============================================================================
CARLA SIMULATION BRIDGE & COCKPIT MULTI-CAMERA DASHBOARD FOR C++20 AUTOSAR AP
- Streams VehicleState + Live OpenDRIVE Corridor + 160x100 RGB Front Hood-Cam
  into C++20 Shared Memory (PerceptionSensorService) over low-latency TCP.
- Receives 50Hz/100Hz C++ QP-MPC Actuator & Telemetry packets (449 bytes).
- Displays the 1440x900 Pygame Cockpit Dashboard (Chase Cam 960x600, BEV 480x300,
  Front Hood-Cam 480x300, Top Bar HUD, and 2 Realtime Plots: Speed & Jerk).
- Cross-platform compatible on both Ubuntu Linux and Windows 10/11.
==============================================================================
"""

import sys
import os
import time
import math
import socket
import struct
import threading
import argparse
import glob
from collections import deque

import numpy as np

# Thêm đường dẫn carla egg hoặc wheel nếu chưa có trong sys.path
for base_search in [
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../Carla-Autonomous-MPC/carla/dist"),
    "D:/Appfile/Carla/CARLA_0.9.15/WindowsNoEditor/PythonAPI/carla/dist",
    "/mnt/d/Appfile/Carla/CARLA_0.9.15/WindowsNoEditor/PythonAPI/carla/dist"
]:
    eggs = glob.glob(os.path.join(base_search, 'carla-*.egg'))
    for egg in eggs:
        if egg not in sys.path:
            sys.path.append(egg)

try:
    import carla
except ImportError:
    print("[ERROR] Thư viện CARLA Python API không tìm thấy! Hãy đảm bảo đã cài đặt carla package.")
    sys.exit(1)

try:
    import pygame
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False

# ==============================================================================
# HẰNG SỐ GIAO DIỆN COCKPIT DASHBOARD & HÌNH HỌC XE (ĐỒNG BỘ VỚI C++ & PYTHON)
# ==============================================================================
COCKPIT_WINDOW_WIDTH = 1440
COCKPIT_WINDOW_HEIGHT = 900
TOP_BAR_HEIGHT = 60
MAIN_VIEW_WIDTH = 960
MAIN_VIEW_HEIGHT = 600
SUB_VIEW_WIDTH = 480
SUB_VIEW_HEIGHT = 300
PLOT_WIDTH = 720
PLOT_HEIGHT = 240
TELEMETRY_HISTORY_LEN = 100

CAM_SHM_W = 160
CAM_SHM_H = 100
CAM_SHM_BYTES = CAM_SHM_W * CAM_SHM_H * 3  # 48000 bytes

EGO_HALF_WIDTH_M = 1.00
OBSTACLE_AVOIDANCE_MARGIN_M = 0.35
R_CLEARANCE = EGO_HALF_WIDTH_M + OBSTACLE_AVOIDANCE_MARGIN_M  # 1.35m bất biến
DEFAULT_ROAD_HALF_WIDTH_M = 5.25
EGO_FRONT_OVERHANG_M = 2.40
EMERGENCY_ZONE_WIDTH_M = 2.40
EMERGENCY_ZONE_LENGTH_M = 6.00


def clip(v, lo, hi):
    return max(lo, min(v, hi))


def get_cross_platform_font(size: int, bold: bool = False):
    """Chọn font hỗ trợ tiếng Việt Unicode tốt nhất trên cả Ubuntu và Windows"""
    font_candidates = ['DejaVu Sans', 'Segoe UI', 'Arial', 'Liberation Sans', 'Tahoma']
    for name in font_candidates:
        try:
            f = pygame.font.SysFont(name, size, bold=bold)
            if f is not None:
                return f
        except Exception:
            continue
    return pygame.font.SysFont(None, size, bold=bold)


def load_csv(filepath):
    data = []
    if not os.path.exists(filepath):
        return data
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        if not lines:
            return data
        header = [h.strip() for h in lines[0].split(',')]
        for l in lines[1:]:
            parts = [p.strip() for p in l.split(',') if p.strip()]
            if len(parts) >= len(header):
                row = {}
                for idx, h in enumerate(header):
                    try:
                        row[h] = float(parts[idx])
                    except ValueError:
                        row[h] = parts[idx]
                data.append(row)
    return data


# ==============================================================================
# BỘ VẼ 2 ĐỒ THỊ THỜI GIAN THỰC (REALTIME TELEMETRY PLOTS)
# ==============================================================================
class TelemetryPlotter:
    def __init__(self, history_len: int = TELEMETRY_HISTORY_LEN):
        self.history_len = history_len
        self.v_history = deque(maxlen=history_len)
        self.v_ref_history = deque(maxlen=history_len)
        self.jerk_history = deque(maxlen=history_len)

        self.font_title = get_cross_platform_font(13, bold=True)
        self.font_axis = get_cross_platform_font(11, bold=False)
        self.font_value = get_cross_platform_font(12, bold=True)

    def update(self, v_kmh: float, v_ref_kmh: float, jerk_ms3: float):
        self.v_history.append(float(v_kmh))
        self.v_ref_history.append(float(v_ref_kmh))
        self.jerk_history.append(float(jerk_ms3))

    @staticmethod
    def _draw_dashed_line(surface, color, start_pos, end_pos, width=1, dash_length=6):
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

    def draw_speed_plot(self, surface, rect):
        pygame.draw.rect(surface, (12, 17, 26), rect)
        pygame.draw.rect(surface, (30, 65, 90), rect, 1)

        pad_left, pad_right, pad_top, pad_bottom = 44, 16, 32, 24
        plot_x = rect.x + pad_left
        plot_y = rect.y + pad_top
        plot_w = rect.width - pad_left - pad_right
        plot_h = rect.height - pad_top - pad_bottom

        plot_rect = pygame.Rect(plot_x, plot_y, plot_w, plot_h)
        pygame.draw.rect(surface, (8, 12, 20), plot_rect)

        max_v = 40.0
        if self.v_history or self.v_ref_history:
            peak = max(max(self.v_history, default=0.0), max(self.v_ref_history, default=0.0))
            if peak > 38.0:
                max_v = math.ceil(peak / 10.0) * 10.0

        val = 0.0
        while val <= max_v + 0.1:
            ratio = val / max_v
            gy = int(plot_y + plot_h - ratio * plot_h)
            grid_color = (45, 58, 75) if val in (0.0, max_v) else (25, 34, 48)
            pygame.draw.line(surface, grid_color, (plot_x, gy), (plot_x + plot_w, gy), 1)
            lbl = self.font_axis.render(f"{int(val)}", True, (130, 145, 165))
            surface.blit(lbl, (rect.x + 8, gy - 7))
            val += 10.0

        for idx_step in range(0, self.history_len + 1, 25):
            gx = int(plot_x + (idx_step / max(1, self.history_len - 1)) * plot_w)
            pygame.draw.line(surface, (22, 30, 42), (gx, plot_y), (gx, plot_y + plot_h), 1)
            sec_ago = -((self.history_len - idx_step) * 0.1)
            t_lbl = self.font_axis.render(f"{sec_ago:.1f}s" if idx_step < self.history_len else "0.0s", True, (110, 125, 145))
            surface.blit(t_lbl, (gx - 14, plot_y + plot_h + 4))

        title_surf = self.font_title.render("ĐỒ THỊ 1: VẬN TỐC XE (v_ego) vs THAM CHIẾU (v_ref) [km/h]", True, (0, 220, 255))
        surface.blit(title_surf, (rect.x + 12, rect.y + 7))

        curr_v = self.v_history[-1] if self.v_history else 0.0
        curr_v_ref = self.v_ref_history[-1] if self.v_ref_history else 0.0
        dv = curr_v - curr_v_ref
        dv_color = (0, 240, 180) if abs(dv) <= 3.5 else (255, 200, 60)
        stat_str = f"v: {curr_v:4.1f} | v_ref: {curr_v_ref:4.1f} | Δv: {dv:+4.1f} km/h"
        stat_surf = self.font_value.render(stat_str, True, dv_color)
        surface.blit(stat_surf, (rect.right - stat_surf.get_width() - 12, rect.y + 8))

        n_pts = len(self.v_history)
        if n_pts >= 2:
            pts_v, pts_ref = [], []
            denom = max(1, self.history_len - 1)
            x_offset = self.history_len - n_pts
            for i in range(n_pts):
                px = int(plot_x + ((x_offset + i) / denom) * plot_w)
                v_c = max(0.0, min(max_v, self.v_history[i]))
                vr_c = max(0.0, min(max_v, self.v_ref_history[i]))
                pts_v.append((px, int(plot_y + plot_h - (v_c / max_v) * plot_h)))
                pts_ref.append((px, int(plot_y + plot_h - (vr_c / max_v) * plot_h)))

            for i in range(len(pts_ref) - 1):
                if i % 2 == 0:
                    pygame.draw.line(surface, (255, 170, 0), pts_ref[i], pts_ref[i + 1], 2)

            pygame.draw.lines(surface, (0, 240, 255), False, pts_v, 2)
            pygame.draw.circle(surface, (0, 255, 255), pts_v[-1], 4)

        pygame.draw.rect(surface, (45, 70, 95), plot_rect, 1)

    def draw_jerk_plot(self, surface, rect):
        pygame.draw.rect(surface, (12, 17, 26), rect)
        pygame.draw.rect(surface, (30, 65, 90), rect, 1)

        pad_left, pad_right, pad_top, pad_bottom = 46, 16, 32, 24
        plot_x = rect.x + pad_left
        plot_y = rect.y + pad_top
        plot_w = rect.width - pad_left - pad_right
        plot_h = rect.height - pad_top - pad_bottom

        plot_rect = pygame.Rect(plot_x, plot_y, plot_w, plot_h)
        pygame.draw.rect(surface, (8, 12, 20), plot_rect)

        max_j = 5.0

        def j_to_y(j_val: float) -> int:
            j_clamped = max(-max_j, min(max_j, j_val))
            ratio = (j_clamped + max_j) / (2.0 * max_j)
            return int(plot_y + plot_h - ratio * plot_h)

        y_pos2 = j_to_y(2.0)
        y_neg2 = j_to_y(-2.0)
        comfort_band = pygame.Surface((plot_w, max(1, y_neg2 - y_pos2)), pygame.SRCALPHA)
        comfort_band.fill((0, 180, 100, 26))
        surface.blit(comfort_band, (plot_x, y_pos2))

        for j_thr, col in [(2.0, (140, 120, 30)), (-2.0, (140, 120, 30)), (3.5, (150, 45, 45)), (-3.5, (150, 45, 45))]:
            ty = j_to_y(j_thr)
            self._draw_dashed_line(surface, col, (plot_x, ty), (plot_x + plot_w, ty), width=1, dash_length=5)

        y_zero = j_to_y(0.0)
        pygame.draw.line(surface, (120, 120, 120), (plot_x, y_zero), (plot_x + plot_w, y_zero), 1)

        for j_lbl in (-5.0, -3.5, -2.0, 0.0, 2.0, 3.5, 5.0):
            ly = j_to_y(j_lbl)
            lbl = self.font_axis.render(f"{j_lbl:+.1f}" if j_lbl != 0.0 else " 0.0", True, (130, 145, 165))
            surface.blit(lbl, (rect.x + 6, ly - 7))

        for idx_step in range(0, self.history_len + 1, 25):
            gx = int(plot_x + (idx_step / max(1, self.history_len - 1)) * plot_w)
            pygame.draw.line(surface, (22, 30, 42), (gx, plot_y), (gx, plot_y + plot_h), 1)
            sec_ago = -((self.history_len - idx_step) * 0.1)
            t_lbl = self.font_axis.render(f"{sec_ago:.1f}s" if idx_step < self.history_len else "0.0s", True, (110, 125, 145))
            surface.blit(t_lbl, (gx - 14, plot_y + plot_h + 4))

        title_surf = self.font_title.render("ĐỒ THỊ 2: ĐỘ GIẬT JERK (da/dt) [m/s³]", True, (0, 220, 255))
        surface.blit(title_surf, (rect.x + 12, rect.y + 7))

        curr_j = self.jerk_history[-1] if self.jerk_history else 0.0
        abs_j = abs(curr_j)
        if abs_j <= 2.0:
            j_color, j_tag = (0, 235, 140), "ÊM ÁI"
        elif abs_j <= 3.5:
            j_color, j_tag = (255, 200, 40), "CHUYỂN LÀN / HÃM VỪA"
        else:
            j_color, j_tag = (255, 65, 65), "GIẬT MẠNH / PHANH GẤP"

        stat_str = f"Jerk: {curr_j:+5.2f} m/s³ [{j_tag}]"
        stat_surf = self.font_value.render(stat_str, True, j_color)
        surface.blit(stat_surf, (rect.right - stat_surf.get_width() - 12, rect.y + 8))

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
                seg_col = (0, 230, 140) if seg_j <= 2.0 else ((255, 200, 40) if seg_j <= 3.5 else (255, 60, 60))
                pygame.draw.line(surface, seg_col, (x1, y1), (x2, y2), 2)

            last_x, last_y, _ = pts_j[-1]
            pygame.draw.circle(surface, j_color, (last_x, last_y), 4)

        pygame.draw.rect(surface, (45, 70, 95), plot_rect, 1)


# ==============================================================================
# CỬA SỐ COCKPIT MULTI-CAMERA & REALTIME TELEMETRY DASHBOARD (1440 x 900)
# ==============================================================================
class CockpitCameraViewer:
    """
    Quản lý 3 Camera RGB gắn trên xe Ego trong CARLA và giao diện Pygame Cockpit (1440x900):
    - Luôn kích hoạt Front Hood-Cam (x=2.2m, z=1.2m, pitch=-5°, FOV=90°) để truyền ảnh RGB
      160x100x3 thời gian thực sang C++20 YellowLineDetector (kể cả khi chạy headless).
    - Khi bật chế độ cửa sổ (--viewer), hiển thị đầy đủ 3 Camera + Top Bar HUD + 2 Đồ thị.
    """

    def __init__(self, world, ego_vehicle, enable_window: bool = True):
        self.world = world
        self.ego_vehicle = ego_vehicle
        self.width = COCKPIT_WINDOW_WIDTH
        self.height = COCKPIT_WINDOW_HEIGHT
        self.enable_window = bool(enable_window and HAS_PYGAME)

        self.chase_sensor = None
        self.bev_sensor = None
        self.front_sensor = None

        self.surface_main = None
        self.surface_bev = None
        self.surface_front = None

        # Khung hình RGB 160x100x3 (48000 bytes) cấp trực tiếp cho C++20 YellowLineDetector
        self.front_lock = threading.Lock()
        self.front_rgb_160x100_bytes = None
        self.front_frame_seq = 0

        self._is_active = True
        self.show_hud = True
        self.plotter = None
        self._last_plot_time = 0.0

        if self.enable_window:
            try:
                pygame.init()
                pygame.font.init()
                self.display = pygame.display.set_mode(
                    (self.width, self.height),
                    pygame.HWSURFACE | pygame.DOUBLEBUF
                )
                pygame.display.set_caption(
                    "CARLA Autonomous MPC (C++20 Runtime) — Cockpit Multi-Camera & Realtime Telemetry Dashboard ([M]: Ẩn/Hiện HUD | [ESC]: Đóng)"
                )
                self.font_main = get_cross_platform_font(15, bold=True)
                self.font_sub = get_cross_platform_font(12, bold=False)
                self.font_cam_tag = get_cross_platform_font(12, bold=True)
                self.plotter = TelemetryPlotter(history_len=TELEMETRY_HISTORY_LEN)
            except Exception as e:
                print(f"[COCKPIT VIEWER] Không mở được cửa sổ hiển thị Pygame ({e}), tự động chuyển sang chế độ Headless Camera.")
                self.enable_window = False

        self._spawn_cameras()

    def _spawn_cameras(self):
        bp_lib = self.world.get_blueprint_library()

        if self.enable_window:
            # 1. Main View: Chase Cam (960 x 600, FOV 90)
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

            # 2. Sub 1: Bird-Eye View (480 x 300, FOV 90)
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

        # 3. Sub 2: Front View / Hood-Cam (480 x 300, FOV 90) — luôn kích hoạt cho C++20 YellowLineDetector
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

        if self.enable_window:
            print(
                f"[COCKPIT DASHBOARD] Đã kích hoạt 3 Camera ({MAIN_VIEW_WIDTH}x{MAIN_VIEW_HEIGHT} Chase + "
                f"2x {SUB_VIEW_WIDTH}x{SUB_VIEW_HEIGHT} BEV/Front) & 2 Đồ thị Realtime ({self.width}x{self.height})."
            )
        else:
            print("[COCKPIT DASHBOARD] Đã kích hoạt Front Hood-Cam (160x100 RGB Stream) cho C++20 YellowLineDetector.")

    @staticmethod
    def _carla_image_to_rgb(image) -> np.ndarray:
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        return array[:, :, :3][:, :, ::-1]

    def _on_chase_image(self, image):
        if not self._is_active or not self.enable_window:
            return
        rgb = self._carla_image_to_rgb(image)
        self.surface_main = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))

    def _on_bev_image(self, image):
        if not self._is_active or not self.enable_window:
            return
        rgb = self._carla_image_to_rgb(image)
        self.surface_bev = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))

    def _on_front_image(self, image):
        if not self._is_active:
            return
        rgb = self._carla_image_to_rgb(image)
        # Downsample 480x300 -> 160x100 (stride 3) giữ nguyên tỷ lệ khung hình 16:10 và FOV 90° cho C++ IPM
        rgb_small = np.ascontiguousarray(rgb[::3, ::3, :], dtype=np.uint8)
        if rgb_small.shape == (CAM_SHM_H, CAM_SHM_W, 3):
            raw_bytes = rgb_small.tobytes()
            with self.front_lock:
                self.front_rgb_160x100_bytes = raw_bytes
                self.front_frame_seq += 1
        if self.enable_window:
            self.surface_front = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))

    def pop_latest_front_rgb_bytes(self, last_seq: int):
        """Lấy gói 48000 bytes RGB 160x100 nếu có frame mới hơn last_seq"""
        with self.front_lock:
            if self.front_rgb_160x100_bytes is not None and self.front_frame_seq != last_seq:
                return self.front_rgb_160x100_bytes, self.front_frame_seq
        return None, last_seq

    def _draw_cam_label(self, x: int, y: int, text: str, accent_color=(0, 210, 255)):
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

    def _draw_corridor_badge(self, x: int, y: int, road_min: float, road_max: float, prefix="HÀNH LANG", accent=(0, 210, 225)):
        if not self.show_hud or road_min is None or road_max is None:
            return
        text = f"{prefix}: [{road_min:+.2f}m ; {road_max:+.2f}m]"
        txt_surf = self.font_cam_tag.render(text, True, (230, 250, 255))
        bg_w = txt_surf.get_width() + 18
        bg_h = 24
        badge = pygame.Surface((bg_w, bg_h), pygame.SRCALPHA)
        badge.fill((10, 16, 26, 205))
        pygame.draw.rect(badge, accent, (0, 0, 4, bg_h))
        pygame.draw.rect(badge, (35, 75, 105), (0, 0, bg_w, bg_h), 1)
        self.display.blit(badge, (x, y))
        self.display.blit(txt_surf, (x + 10, y + 4))

    def _draw_top_bar(self, hud_info: dict):
        top_surface = pygame.Surface((self.width, TOP_BAR_HEIGHT), pygame.SRCALPHA)
        top_surface.fill((10, 15, 25, 230))
        self.display.blit(top_surface, (0, 0))

        is_emerg = hud_info.get('is_emergency', False)
        border_col = (255, 40, 40) if is_emerg else (0, 180, 220)
        pygame.draw.line(self.display, border_col, (0, TOP_BAR_HEIGHT - 2), (self.width, TOP_BAR_HEIGHT - 2), 2)

        pygame.draw.line(self.display, (35, 60, 85), (465, 8), (465, TOP_BAR_HEIGHT - 8), 1)
        pygame.draw.line(self.display, (35, 60, 85), (960, 8), (960, TOP_BAR_HEIGHT - 8), 1)

        # --- CỤM 1: ĐỘNG HỌC XE ---
        speed_kmh = hud_info.get('speed_kmh', 0.0)
        target_kmh = hud_info.get('target_kmh', 0.0)
        steer_deg = hud_info.get('steer_deg', 0.0)

        speed_color = (0, 255, 180) if abs(speed_kmh - target_kmh) <= 5.0 else (255, 205, 50)
        steer_arrow = "↑" if abs(steer_deg) < 0.8 else ("→" if steer_deg > 0 else "←")

        c1_line1 = self.font_main.render(
            f"Tốc độ: {speed_kmh:4.1f} km/h  |  Góc lái: {steer_deg:+5.1f}° {steer_arrow}",
            True, speed_color
        )
        c1_line2 = self.font_sub.render(
            f"Mục tiêu: {target_kmh:4.1f} km/h  |  Sai lệch Δv: {speed_kmh - target_kmh:+4.1f} km/h",
            True, (185, 200, 220)
        )
        self.display.blit(c1_line1, (15, 8))
        self.display.blit(c1_line2, (15, 34))

        # --- CỤM 2: TRẠNG THÁI ĐIỀU KHIỂN MPC C++20 ---
        bypass_text = hud_info.get('bypass_text', 'CHẾ ĐỘ 1: BÁM ĐƯỜNG')
        if "PHANH GẤP" in bypass_text or is_emerg:
            bypass_color = (255, 55, 55)
        elif "ROADBLOCK" in bypass_text or "DỪNG" in bypass_text:
            bypass_color = (255, 110, 60)
        elif "NÉ" in bypass_text:
            bypass_color = (255, 180, 40)
        else:
            bypass_color = (70, 225, 255)

        s_ego_m = hud_info.get('s_ego_m', 0.0)
        s_total_m = hud_info.get('s_total_m', 0.0)
        current_idx = hud_info.get('current_idx', 0)
        total_idx = max(1, hud_info.get('total_idx', 1))
        road_min = hud_info.get('road_min', -0.45)
        road_max = hud_info.get('road_max', 3.90)
        corridor_str = f"Road: [{road_min:+.2f}m; {road_max:+.2f}m]"

        if s_total_m > 0.1:
            pct = min(100.0, max(0.0, (s_ego_m / s_total_m) * 100.0))
            prog_str = f"s={s_ego_m:.1f}m ({pct:.0f}%) [WP {current_idx}]  |  {corridor_str}"
        else:
            pct = (current_idx / total_idx) * 100.0
            prog_str = f"WP {current_idx}/{total_idx} ({pct:.0f}%)  |  {corridor_str}"

        c2_line1 = self.font_main.render(f"C++ MPC: {bypass_text}", True, bypass_color)
        c2_line2 = self.font_sub.render(prog_str, True, (185, 200, 220))
        self.display.blit(c2_line1, (480, 8))
        self.display.blit(c2_line2, (480, 34))

        # --- CỤM 3: NHẬN THỨC & CẢM BIẾN ---
        obs_count = hud_info.get('obs_count', 0)
        yellow_info = hud_info.get('yellow_line', '---')
        if 'SOLID' in yellow_info:
            yl_color = (255, 230, 0)
        elif 'DASHED' in yellow_info:
            yl_color = (255, 200, 80)
        else:
            yl_color = (160, 170, 185)

        c3_line1 = self.font_main.render(f"Vật cản: {obs_count}  |  Vạch vàng: {yellow_info}", True, yl_color)
        c3_line2 = self.font_sub.render("[ESC]: Đóng cửa sổ  |  [M]: Ẩn/Hiện nhãn HUD", True, (150, 165, 185))
        self.display.blit(c3_line1, (975, 8))
        self.display.blit(c3_line2, (975, 34))

    def render(self, hud_info: dict = None) -> bool:
        if not self._is_active or not self.enable_window:
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
        cam_y = TOP_BAR_HEIGHT

        # 1. Main Chase Cam (960 x 600)
        if self.surface_main is not None:
            self.display.blit(self.surface_main, (0, cam_y))
        else:
            pygame.draw.rect(self.display, (18, 22, 30), (0, cam_y, MAIN_VIEW_WIDTH, MAIN_VIEW_HEIGHT))
            wait_txt = self.font_main.render("Đang kết nối Main Chase Camera (960x600)...", True, (180, 190, 205))
            self.display.blit(wait_txt, (MAIN_VIEW_WIDTH // 2 - 170, cam_y + MAIN_VIEW_HEIGHT // 2 - 10))
        self._draw_cam_label(0, cam_y, "MAIN VIEW: CHASE CAM (960×600)", accent_color=(0, 210, 255))

        # 2. Sub 1: Bird-Eye View (480 x 300)
        sub_x = MAIN_VIEW_WIDTH
        if self.surface_bev is not None:
            self.display.blit(self.surface_bev, (sub_x, cam_y))
        else:
            pygame.draw.rect(self.display, (16, 20, 28), (sub_x, cam_y, SUB_VIEW_WIDTH, SUB_VIEW_HEIGHT))
            wait_bev = self.font_sub.render("Đang kết nối Bird-Eye Cam (480x300)...", True, (170, 180, 195))
            self.display.blit(wait_bev, (sub_x + 115, cam_y + SUB_VIEW_HEIGHT // 2 - 8))
        self._draw_cam_label(sub_x, cam_y, "CAM PHỤ 1: BIRD-EYE VIEW (480×300)", accent_color=(0, 230, 150))

        # 3. Sub 2: Front View / Hood-Cam (480 x 300)
        front_y = cam_y + SUB_VIEW_HEIGHT
        if self.surface_front is not None:
            self.display.blit(self.surface_front, (sub_x, front_y))
        else:
            pygame.draw.rect(self.display, (16, 20, 28), (sub_x, front_y, SUB_VIEW_WIDTH, SUB_VIEW_HEIGHT))
            wait_front = self.font_sub.render("Đang kết nối Front Hood-Cam (480x300)...", True, (170, 180, 195))
            self.display.blit(wait_front, (sub_x + 110, front_y + SUB_VIEW_HEIGHT // 2 - 8))
        self._draw_cam_label(
            sub_x, front_y,
            "CAM PHỤ 2: FRONT VIEW / C++20 YELLOW DETECTOR (480×300)",
            accent_color=(255, 215, 0)
        )

        pygame.draw.line(self.display, (0, 160, 200), (sub_x, cam_y), (sub_x, cam_y + MAIN_VIEW_HEIGHT), 2)
        pygame.draw.line(self.display, (0, 160, 200), (sub_x, front_y), (self.width, front_y), 2)

        if hud_info is not None and self.show_hud:
            r_min = hud_info.get('road_min', -0.45)
            r_max = hud_info.get('road_max', 3.90)
            yl_src = hud_info.get('yellow_source_str', 'CV')
            self._draw_corridor_badge(12, cam_y + MAIN_VIEW_HEIGHT - 32, r_min, r_max, prefix="HÀNH LANG ĐƯỜNG")
            self._draw_corridor_badge(sub_x + 8, cam_y + SUB_VIEW_HEIGHT - 30, r_min, r_max, prefix="HÀNH LANG BEV")
            self._draw_corridor_badge(
                sub_x + 8, front_y + SUB_VIEW_HEIGHT - 30, r_min, r_max,
                prefix=f"{yl_src} CORRIDOR", accent=(255, 215, 0)
            )

        # 4. Vẽ 2 Đồ thị Realtime (y = 660..900) — lấy mẫu chuẩn 10Hz (0.1s/điểm -> cửa sổ 10.0s)
        plot_y = TOP_BAR_HEIGHT + MAIN_VIEW_HEIGHT
        if hud_info is not None and self.plotter is not None:
            now_plot = time.time()
            if now_plot - self._last_plot_time >= 0.095:
                self._last_plot_time = now_plot
                self.plotter.update(
                    hud_info.get('speed_kmh', 0.0),
                    hud_info.get('target_kmh', 0.0),
                    hud_info.get('jerk_ms3', 0.0)
                )
            self.plotter.draw_speed_plot(self.display, pygame.Rect(0, plot_y, PLOT_WIDTH, PLOT_HEIGHT))
            self.plotter.draw_jerk_plot(self.display, pygame.Rect(PLOT_WIDTH, plot_y, PLOT_WIDTH, PLOT_HEIGHT))

        # 5. Vẽ thanh thông số Top Bar (y = 0..60)
        if hud_info is not None and self.show_hud:
            self._draw_top_bar(hud_info)
        elif hud_info is not None and not self.show_hud:
            pygame.draw.rect(self.display, (10, 15, 25), (0, 0, self.width, TOP_BAR_HEIGHT))
            hint = self.font_sub.render("Đã ẩn HUD — Nhấn [M] để hiện lại thanh thông số | [ESC]: Đóng", True, (140, 155, 175))
            self.display.blit(hint, (15, 22))

        pygame.display.flip()
        return True

    def destroy(self):
        self._is_active = False
        for name, sensor in [
            ("Chase Cam", self.chase_sensor),
            ("Bird-Eye Cam", self.bev_sensor),
            ("Front Hood-Cam", self.front_sensor),
        ]:
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
        if self.enable_window:
            try:
                pygame.quit()
            except Exception:
                pass


# ==============================================================================
# CARLA SIMULATION BRIDGE DAEMON
# ==============================================================================
class CarlaBridgeDaemon:
    def __init__(self, host='127.0.0.1', port=2000, bridge_port=20050, with_obstacles=True, with_viewer=True):
        self.host = host
        self.port = port
        self.bridge_port = bridge_port
        self.with_obstacles = with_obstacles
        self.with_viewer = with_viewer

        self.client = None
        self.world = None
        self.carla_map = None
        self.spectator = None
        self.ego_vehicle = None
        self.viewer = None

        self.spawned_actors = []
        self.actor_metadata = {}
        self.moving_obstacle_entries = []

        # Dữ liệu quỹ đạo & hành lang động OpenDRIVE tiền tính toán
        self.traj_x = np.array([], dtype=float)
        self.traj_y = np.array([], dtype=float)
        self.traj_theta = np.array([], dtype=float)
        self.corridor_min = np.array([], dtype=float)
        self.corridor_max = np.array([], dtype=float)
        self.corridor_yellow = np.array([], dtype=bool)
        self.current_wp_idx = 0

        # Điều khiển & Telemetry nhận từ C++20 (bảo vệ thread-safe)
        self.cmd_lock = threading.Lock()
        self.latest_throttle = 0.0
        self.latest_brake = 0.18
        self.latest_steer = 0.0
        self.latest_telem = {
            'wheel_angle_rad': 0.0,
            'accel_cmd': 0.0,
            'v_ref_m_s': 10.0,
            'jerk_m_s3': 0.0,
            'road_min_m': -0.45,
            'road_max_m': 3.90,
            's_ego_m': 0.0,
            's_total_m': 760.88,
            'd_yellow_m': 1.75,
            'detect_ms': 0.0,
            'current_idx': 0,
            'total_idx': 7607,
            'obs_count': 0,
            'active_obs_id': -1,
            'bypass_side': 0,
            'is_emergency': 0,
            'is_roadblock': 0,
            'yellow_line_type': 1,  # 1 = SOLID
            'yellow_source': 2,     # 1 = CV, 2 = MAP
            'horizon_x': [],
            'horizon_y': [],
        }

        self.running = threading.Event()
        self.running.set()

        self.connect_carla()

    def connect_carla(self):
        hosts = [self.host]
        if os.path.exists('/etc/resolv.conf'):
            try:
                with open('/etc/resolv.conf', 'r') as f:
                    for line in f:
                        if line.strip().startswith('nameserver'):
                            ns = line.split()[1].strip()
                            if ns not in hosts:
                                hosts.append(ns)
            except Exception:
                pass
        for h in ['127.0.0.1', 'localhost', '172.20.240.1']:
            if h not in hosts:
                hosts.append(h)

        for h in hosts:
            try:
                print(f"[BRIDGE] Kết nối tới CARLA Server tại {h}:{self.port}...")
                c = carla.Client(h, self.port)
                c.set_timeout(10.0)
                w = c.get_world()
                self.client = c
                self.world = w
                self.carla_map = self.world.get_map()
                self.spectator = self.world.get_spectator()

                try:
                    settings = self.world.get_settings()
                    if settings.synchronous_mode:
                        print("[BRIDGE] Phát hiện CARLA đang ở Synchronous Mode. Đang chuyển về Asynchronous Mode...")
                        settings.synchronous_mode = False
                        settings.fixed_delta_seconds = 0.0
                        self.world.apply_settings(settings)
                        try:
                            self.world.tick()
                        except Exception:
                            pass
                except Exception as e:
                    print(f"[BRIDGE] Cảnh báo cấu hình settings CARLA: {e}")

                print(f"[BRIDGE] Kết nối CARLA Server thành công ({h}:{self.port})!")
                break
            except Exception:
                continue

        if self.client is None:
            raise ConnectionError("Không thể kết nối đến CARLA Server! Hãy đảm bảo CarlaUE4 đang chạy.")

    def compute_waypoint_corridor(self, x: float, y: float, theta_rad: float):
        """
        Trích xuất biên hành lang pháp lý [road_min, road_max, is_yellow_solid] trực tiếp
        từ CARLA OpenDRIVE HD Map tại tọa độ (x, y, theta_rad) — đồng bộ 100% với src/road_corridor.py.
        """
        default_min = -DEFAULT_ROAD_HALF_WIDTH_M + R_CLEARANCE
        default_max = DEFAULT_ROAD_HALF_WIDTH_M - R_CLEARANCE
        if self.carla_map is None:
            return default_min, default_max, True

        try:
            loc = carla.Location(x=float(x), y=float(y), z=0.5)
            wp = self.carla_map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            if wp is None:
                return default_min, default_max, True

            nx = -math.sin(theta_rad)
            ny = math.cos(theta_rad)
            dx = wp.transform.location.x - x
            dy = wp.transform.location.y - y
            d_center = nx * dx + ny * dy
            if abs(d_center) <= 0.40:
                d_center = 0.0

            curr_half_w = float(wp.lane_width / 2.0)

            # 1. Quét các làn Driving cùng chiều bên PHẢI (d > 0)
            d_right_legal = d_center + curr_half_w
            curr_r = wp
            has_r_solid = False
            found_r_lane = False
            while True:
                rm = curr_r.right_lane_marking
                is_r_solid = rm and rm.type in (
                    carla.LaneMarkingType.Solid,
                    carla.LaneMarkingType.SolidSolid,
                    carla.LaneMarkingType.SolidBroken
                )
                if is_r_solid:
                    has_r_solid = True
                    break
                next_r = curr_r.get_right_lane()
                if (next_r is not None and
                        next_r.lane_type == carla.LaneType.Driving and
                        (next_r.lane_id * wp.lane_id > 0)):
                    d_right_legal += float(next_r.lane_width)
                    curr_r = next_r
                    found_r_lane = True
                else:
                    break

            # 2. Quét các làn Driving bên TRÁI (d < 0) & kiểm tra vạch vàng nét liền
            d_left_legal = d_center - curr_half_w
            is_yellow_solid = False
            has_l_solid = False
            found_l_lane = False
            curr_l = wp
            while True:
                lm = curr_l.left_lane_marking
                is_l_yellow = bool(lm and (lm.color == carla.LaneMarkingColor.Yellow))
                is_l_solid = bool(lm and lm.type in (
                    carla.LaneMarkingType.Solid,
                    carla.LaneMarkingType.SolidSolid,
                    carla.LaneMarkingType.SolidBroken
                ))
                is_l_broken = bool(lm and lm.type in (
                    carla.LaneMarkingType.Broken,
                    carla.LaneMarkingType.BrokenSolid,
                    carla.LaneMarkingType.BrokenBroken
                ))

                next_l = curr_l.get_left_lane()
                if next_l is not None and next_l.lane_type == carla.LaneType.Driving:
                    if next_l.lane_id * wp.lane_id > 0:
                        d_left_legal -= float(next_l.lane_width)
                        curr_l = next_l
                        found_l_lane = True
                    else:
                        if is_l_yellow and is_l_solid:
                            is_yellow_solid = True
                            has_l_solid = True
                            break
                        elif is_l_broken:
                            d_left_legal -= float(next_l.lane_width)
                            found_l_lane = True
                            break
                        else:
                            if is_l_solid:
                                is_yellow_solid = is_l_yellow
                                has_l_solid = True
                            break
                else:
                    if is_l_yellow and is_l_solid:
                        is_yellow_solid = True
                        has_l_solid = True
                    elif is_l_solid:
                        has_l_solid = True
                    break

            if getattr(wp, 'is_junction', False):
                if not is_yellow_solid and not has_l_solid and not found_l_lane:
                    d_left_legal = min(d_left_legal, -5.25)
                if not has_r_solid and not found_r_lane:
                    d_right_legal = max(d_right_legal, 4.50)

            road_min = min(d_left_legal + R_CLEARANCE, -0.45)
            road_max = max(d_right_legal - R_CLEARANCE, 0.45)
            if road_min > road_max - 0.5:
                road_min = road_max - 0.5

            return float(road_min), float(road_max), bool(is_yellow_solid)
        except Exception:
            return default_min, default_max, True

    def precompute_opendrive_corridor(self, traj_data):
        """Tiền tính toán hành lang động OpenDRIVE dọc theo toàn bộ quỹ đạo (kèm lọc mượt 5 điểm)"""
        n = len(traj_data)
        self.traj_x = np.array([r['x'] for r in traj_data], dtype=float)
        self.traj_y = np.array([r['y'] for r in traj_data], dtype=float)
        self.traj_theta = np.array([r['theta_rad'] for r in traj_data], dtype=float)

        rmin_arr = np.zeros(n, dtype=float)
        rmax_arr = np.zeros(n, dtype=float)
        yl_arr = np.zeros(n, dtype=bool)

        for i in range(n):
            rm, rx, yl = self.compute_waypoint_corridor(self.traj_x[i], self.traj_y[i], self.traj_theta[i])
            rmin_arr[i] = rm
            rmax_arr[i] = rx
            yl_arr[i] = yl

        def smooth_box(arr, win=5):
            pad = win // 2
            padded = np.pad(arr, pad, mode='edge')
            return np.convolve(padded, np.ones(win) / win, mode='valid')

        self.corridor_min = smooth_box(rmin_arr, win=5)
        self.corridor_max = smooth_box(rmax_arr, win=5)
        self.corridor_yellow = yl_arr

        # Xuất file cache hành lang OpenDRIVE toàn cục để C++20 RoadCorridorTracker truy vấn chính xác tại mọi s_rear phía trước
        try:
            base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
            cache_path = os.path.join(base_dir, "configs/trajectories/opendrive_corridor_cache.csv")
            os.makedirs(os.path.dirname(cache_path), exist_ok=True)
            with open(cache_path, 'w', newline='') as f_cache:
                f_cache.write("road_min,road_max,is_yellow_solid\n")
                for i in range(n):
                    yl_int = 1 if self.corridor_yellow[i] else 0
                    f_cache.write(f"{self.corridor_min[i]:.4f},{self.corridor_max[i]:.4f},{yl_int}\n")
        except Exception as e:
            print(f"[ROAD CORRIDOR] Cảnh báo ghi cache hành lang OpenDRIVE: {e}")

        print(
            f"[ROAD CORRIDOR] Đã trích xuất hành lang động OpenDRIVE cho {n} waypoints: "
            f"road_min=[{self.corridor_min.min():+.2f}m..{self.corridor_min.max():+.2f}m], "
            f"road_max=[{self.corridor_max.min():+.2f}m..{self.corridor_max.max():+.2f}m]"
        )

    def query_live_corridor(self, ego_x: float, ego_y: float):
        """Truy vấn biên hành lang OpenDRIVE và trạng thái giao lộ theo vị trí xe thực tế"""
        is_junction = 0
        try:
            wp_ego = self.carla_map.get_waypoint(
                carla.Location(x=float(ego_x), y=float(ego_y), z=0.5),
                project_to_road=True, lane_type=carla.LaneType.Driving
            )
            if wp_ego is not None and getattr(wp_ego, 'is_junction', False):
                is_junction = 1
        except Exception:
            pass

        n = len(self.traj_x)
        if n == 0:
            return -0.45, 3.90, 1, is_junction

        # Tìm waypoint gần nhất trong cửa sổ cục bộ quanh current_wp_idx
        lo = max(0, self.current_wp_idx - 30)
        hi = min(n, self.current_wp_idx + 120)
        sub_dx = self.traj_x[lo:hi] - ego_x
        sub_dy = self.traj_y[lo:hi] - ego_y
        d2 = sub_dx * sub_dx + sub_dy * sub_dy
        best_local = int(np.argmin(d2))
        if d2[best_local] > 100.0:
            all_d2 = (self.traj_x - ego_x) ** 2 + (self.traj_y - ego_y) ** 2
            self.current_wp_idx = int(np.argmin(all_d2))
        else:
            self.current_wp_idx = lo + best_local

        idx = self.current_wp_idx
        return (
            float(self.corridor_min[idx]),
            float(self.corridor_max[idx]),
            1 if self.corridor_yellow[idx] else 0,
            is_junction
        )

    def clean_old_actors(self):
        print("[BRIDGE] Đang làm sạch các actor cũ trên bản đồ...")
        try:
            actors = self.world.get_actors()
            destroy_cmds = []
            for s in actors.filter('*sensor.camera*'):
                if s.is_alive:
                    destroy_cmds.append(carla.command.DestroyActor(s.id))
            for v in actors.filter('*vehicle*'):
                if v.is_alive:
                    destroy_cmds.append(carla.command.DestroyActor(v.id))
            for p in actors.filter('*static.prop*'):
                if p.is_alive:
                    destroy_cmds.append(carla.command.DestroyActor(p.id))
            if destroy_cmds:
                self.client.apply_batch_sync(destroy_cmds, False)
        except Exception as e:
            print(f"[BRIDGE] Cảnh báo dọn dẹp actors: {e}")

    def draw_static_3d_paths(self, traj_data):
        """Vẽ quỹ đạo tham chiếu toàn cục (chấm xanh lá) lên không gian 3D CARLA"""
        try:
            debug = self.world.debug
            last_z = 0.5
            for i in range(0, len(traj_data), 5):
                x_c = traj_data[i]['x']
                y_c = traj_data[i]['y']
                wp = self.carla_map.get_waypoint(
                    carla.Location(x=x_c, y=y_c, z=0.0),
                    project_to_road=True, lane_type=carla.LaneType.Any
                )
                z_c = (wp.transform.location.z + 0.3) if wp else last_z
                last_z = z_c
                debug.draw_point(
                    carla.Location(x=x_c, y=y_c, z=z_c),
                    size=0.09, color=carla.Color(0, 255, 0), life_time=600.0
                )
        except Exception:
            pass

    def draw_dynamic_3d_overlays(self, ego_x: float, ego_y: float, ego_z: float, ego_yaw_rad: float,
                                 horizon_x, horizon_y, is_emergency: bool):
        """Vẽ đường dự báo N bước (đỏ) và vùng nguy hiểm khẩn cấp (khi kích hoạt Mode 2) trên CARLA 3D"""
        try:
            debug = self.world.debug
            if horizon_x and len(horizon_x) >= 2:
                z_line = ego_z + 0.25
                for i in range(0, len(horizon_x) - 1, 2):
                    j = min(i + 2, len(horizon_x) - 1)
                    p1 = carla.Location(x=float(horizon_x[i]), y=float(horizon_y[i]), z=z_line)
                    p2 = carla.Location(x=float(horizon_x[j]), y=float(horizon_y[j]), z=z_line)
                    debug.draw_line(p1, p2, thickness=0.09, color=carla.Color(255, 0, 0), life_time=0.12)

            if is_emergency:
                d_center = EGO_FRONT_OVERHANG_M + EMERGENCY_ZONE_LENGTH_M / 2.0
                box_x = ego_x + d_center * math.cos(ego_yaw_rad)
                box_y = ego_y + d_center * math.sin(ego_yaw_rad)
                box_z = ego_z + 0.4
                box = carla.BoundingBox(
                    carla.Location(x=box_x, y=box_y, z=box_z),
                    carla.Vector3D(x=EMERGENCY_ZONE_LENGTH_M / 2.0, y=EMERGENCY_ZONE_WIDTH_M / 2.0, z=0.4)
                )
                rot = carla.Rotation(pitch=0.0, yaw=math.degrees(ego_yaw_rad), roll=0.0)
                debug.draw_box(box, rot, thickness=0.08, color=carla.Color(255, 0, 0, 220), life_time=0.12)
                debug.draw_string(
                    carla.Location(x=box_x, y=box_y, z=box_z + 1.2),
                    "[!] VÙNG NGUY HIỂM - PHANH GẤP!",
                    draw_shadow=True, color=carla.Color(255, 50, 50), life_time=0.12
                )
        except Exception:
            pass

    def setup_simulation(self, traj_path, obs_path, moving_obs_path):
        self.clean_old_actors()

        traj_data = load_csv(traj_path)
        if not traj_data:
            raise FileNotFoundError(f"Không tìm thấy quỹ đạo tại {traj_path}")

        self.precompute_opendrive_corridor(traj_data)

        x0 = traj_data[0]['x']
        y0 = traj_data[0]['y']
        th0 = traj_data[0]['theta_rad']

        bp_lib = self.world.get_blueprint_library()
        ego_bp = bp_lib.filter('vehicle.tesla.model3')[0]

        wp = self.carla_map.get_waypoint(carla.Location(x=x0, y=y0, z=0.0), project_to_road=True, lane_type=carla.LaneType.Any)
        z0 = (wp.transform.location.z + 0.5) if wp else 0.5
        yaw_deg = math.degrees(th0)

        tf = carla.Transform(carla.Location(x=x0, y=y0, z=z0), carla.Rotation(pitch=0.0, yaw=yaw_deg, roll=0.0))
        self.ego_vehicle = self.world.try_spawn_actor(ego_bp, tf)
        if self.ego_vehicle is None:
            tf.location.z += 0.3
            self.ego_vehicle = self.world.spawn_actor(ego_bp, tf)

        try:
            self.ego_vehicle.set_autopilot(False)
        except Exception:
            pass

        self.spawned_actors.append(self.ego_vehicle)
        print(f"[BRIDGE] Đã tạo xe Ego tại ({x0:.2f}, {y0:.2f}, z={z0:.2f}, yaw={yaw_deg:.1f}°)")

        if self.spectator:
            cam_x = x0 - 7.5 * math.cos(th0)
            cam_y = y0 - 7.5 * math.sin(th0)
            cam_z = z0 + 3.5
            self.spectator.set_transform(carla.Transform(
                carla.Location(x=cam_x, y=cam_y, z=cam_z),
                carla.Rotation(pitch=-15.0, yaw=yaw_deg, roll=0.0)
            ))

        # Tạo vật cản tĩnh
        if self.with_obstacles and os.path.exists(obs_path):
            obs_data = load_csv(obs_path)
            obs_bp = bp_lib.filter('vehicle.toyota.prius')[0]
            for ob in obs_data:
                ox, oy = ob['x'], ob['y']
                best_idx = min(range(len(traj_data)), key=lambda i: (traj_data[i]['x'] - ox)**2 + (traj_data[i]['y'] - oy)**2)
                oyaw = math.degrees(traj_data[best_idx]['theta_rad'])
                owp = self.carla_map.get_waypoint(carla.Location(x=ox, y=oy, z=0.0), project_to_road=True, lane_type=carla.LaneType.Any)
                oz = (owp.transform.location.z + 0.3) if owp else 0.5
                otf = carla.Transform(carla.Location(x=ox, y=oy, z=oz), carla.Rotation(pitch=0.0, yaw=oyaw, roll=0.0))
                try:
                    act = self.world.try_spawn_actor(obs_bp, otf)
                    if act is not None:
                        act.apply_control(carla.VehicleControl(hand_brake=True, throttle=0.0, brake=1.0))
                        self.spawned_actors.append(act)
                        bb = act.bounding_box.extent
                        self.actor_metadata[act.id] = {
                            'extent_x': max(float(bb.x), 0.4),
                            'extent_y': max(float(bb.y), 0.4),
                            'is_vehicle': 1,
                            'vx': 0.0,
                            'vy': 0.0,
                        }
                except Exception:
                    pass
            print(f"[BRIDGE] Đã tạo {len(self.spawned_actors) - 1} vật cản tĩnh.")

        # Tạo vật cản di động (đồng bộ 100% với MovingObstacleManager trong src/create_obstacles/obstacle_generator.py)
        if self.with_obstacles and os.path.exists(moving_obs_path):
            moving_data = load_csv(moving_obs_path)
            groups = {}
            for row in moving_data:
                oid = int(row['obstacle_id'])
                groups.setdefault(oid, []).append(row)

            prius_bps = bp_lib.filter('vehicle.toyota.prius')
            m_bp = prius_bps[0] if prius_bps else bp_lib.filter('vehicle.audi.etron')[0]
            for oid, wps in groups.items():
                wps.sort(key=lambda r: int(r['wp_index']))
                if len(wps) < 2:
                    continue
                path_xy = [(float(w['x']), float(w['y'])) for w in wps]
                start_w = wps[0]
                mwp = self.carla_map.get_waypoint(
                    carla.Location(x=path_xy[0][0], y=path_xy[0][1], z=0.0),
                    project_to_road=True, lane_type=carla.LaneType.Any
                )
                mz = (mwp.transform.location.z + 0.4) if mwp else 0.5
                dx0 = path_xy[1][0] - path_xy[0][0]
                dy0 = path_xy[1][1] - path_xy[0][1]
                init_yaw_deg = math.degrees(math.atan2(dy0, dx0))
                mtf = carla.Transform(
                    carla.Location(x=path_xy[0][0], y=path_xy[0][1], z=mz),
                    carla.Rotation(pitch=0.0, yaw=init_yaw_deg, roll=0.0)
                )
                try:
                    m_act = self.world.try_spawn_actor(m_bp, mtf)
                    if m_act is None:
                        mtf.location.z += 0.5
                        m_act = self.world.try_spawn_actor(m_bp, mtf)
                    if m_act is not None:
                        try:
                            m_act.set_autopilot(False)
                            m_act.set_simulate_physics(False)
                        except Exception:
                            pass
                        self.spawned_actors.append(m_act)
                        bb = m_act.bounding_box.extent
                        self.actor_metadata[m_act.id] = {
                            'extent_x': max(float(bb.x), 0.4),
                            'extent_y': max(float(bb.y), 0.4),
                            'is_vehicle': 1,
                            'vx': 0.0,
                            'vy': 0.0,
                        }
                        s_accum = [0.0]
                        for idx_p in range(1, len(path_xy)):
                            dx = path_xy[idx_p][0] - path_xy[idx_p - 1][0]
                            dy = path_xy[idx_p][1] - path_xy[idx_p - 1][1]
                            s_accum.append(s_accum[-1] + math.hypot(dx, dy))

                        self.moving_obstacle_entries.append({
                            'actor': m_act,
                            'path_xy': path_xy,
                            's_accum': s_accum,
                            'total_s': s_accum[-1],
                            'current_s': 0.0,
                            'speed_ms': float(start_w.get('speed_ms', 9.0)),
                            'road_z': mz,
                            'finished': False,
                        })
                except Exception:
                    pass
            print(f"[BRIDGE] Đã tạo {len(self.moving_obstacle_entries)} xe vật cản di động.")

        # Vẽ quỹ đạo 3D lên bản đồ CARLA
        self.draw_static_3d_paths(traj_data)

        # Khởi tạo Cockpit Camera Viewer (3 Camera + 2 Đồ thị Realtime + Stream RGB 160x100 cho C++20)
        self.viewer = CockpitCameraViewer(self.world, self.ego_vehicle, enable_window=self.with_viewer)

    def update_moving_obstacles_in_memory(self, dt=0.02):
        for mob in self.moving_obstacle_entries:
            act = mob['actor']
            if not act.is_alive or mob.get('finished', False):
                continue

            mob['current_s'] += mob['speed_ms'] * dt
            if mob['current_s'] >= mob['total_s']:
                mob['current_s'] = mob['total_s']
                mob['finished'] = True
                if act.id in self.actor_metadata:
                    self.actor_metadata[act.id]['vx'] = 0.0
                    self.actor_metadata[act.id]['vy'] = 0.0
                continue

            cur_s = mob['current_s']
            path_xy = mob['path_xy']
            s_accum = mob['s_accum']

            seg_idx = 0
            for idx_s in range(len(s_accum) - 1):
                if s_accum[idx_s] <= cur_s <= s_accum[idx_s + 1]:
                    seg_idx = idx_s
                    break

            seg_len = max(s_accum[seg_idx + 1] - s_accum[seg_idx], 1e-6)
            ratio = (cur_s - s_accum[seg_idx]) / seg_len

            cur_x = path_xy[seg_idx][0] + ratio * (path_xy[seg_idx + 1][0] - path_xy[seg_idx][0])
            cur_y = path_xy[seg_idx][1] + ratio * (path_xy[seg_idx + 1][1] - path_xy[seg_idx][1])
            yaw_rad = math.atan2(path_xy[seg_idx + 1][1] - path_xy[seg_idx][1],
                                 path_xy[seg_idx + 1][0] - path_xy[seg_idx][0])

            if act.id in self.actor_metadata:
                self.actor_metadata[act.id]['vx'] = mob['speed_ms'] * math.cos(yaw_rad)
                self.actor_metadata[act.id]['vy'] = mob['speed_ms'] * math.sin(yaw_rad)

            try:
                act.set_transform(carla.Transform(
                    carla.Location(x=cur_x, y=cur_y, z=mob['road_z']),
                    carla.Rotation(pitch=0.0, yaw=math.degrees(yaw_rad), roll=0.0)
                ))
            except Exception:
                pass

    def update_spectator(self, ego_tf):
        if not self.spectator:
            return
        try:
            yaw_rad = math.radians(ego_tf.rotation.yaw)
            cam_x = ego_tf.location.x - 7.5 * math.cos(yaw_rad)
            cam_y = ego_tf.location.y - 7.5 * math.sin(yaw_rad)
            cam_z = ego_tf.location.z + 3.5
            self.spectator.set_transform(carla.Transform(
                carla.Location(x=cam_x, y=cam_y, z=cam_z),
                carla.Rotation(pitch=-15.0, yaw=ego_tf.rotation.yaw, roll=0.0)
            ))
        except Exception:
            pass

    def _receiver_loop(self, client_sock):
        """
        Thread riêng đọc gói lệnh điều khiển & Telemetry ACTR (449 bytes) từ C++20:
        Format: <4s13d3Iib4B40d (4 + 104 + 12 + 4 + 1 + 4 + 320 = 449 bytes)
        """
        PKT_FMT = '<4s13d3Iib4B40d'
        PKT_SIZE = struct.calcsize(PKT_FMT)  # 449 bytes

        while self.running.is_set():
            try:
                cmd_buf = b''
                while len(cmd_buf) < PKT_SIZE and self.running.is_set():
                    chunk = client_sock.recv(PKT_SIZE - len(cmd_buf))
                    if not chunk:
                        return
                    cmd_buf += chunk

                if len(cmd_buf) == PKT_SIZE:
                    unpacked = struct.unpack(PKT_FMT, cmd_buf)
                    magic = unpacked[0]
                    if magic == b'ACTR':
                        (
                            _, thr, brk, str_val, wheel_rad, accel_cmd,
                            v_ref, jerk_f, r_min, r_max, s_ego, s_tot, d_yl, det_ms,
                            cur_idx, tot_idx, obs_cnt, act_obs_id, bypass_side,
                            is_emerg, is_rb, yl_type, yl_src
                        ) = unpacked[:23]
                        hx = list(unpacked[23:43])
                        hy = list(unpacked[43:63])

                        with self.cmd_lock:
                            self.latest_throttle = float(clip(thr, 0.0, 1.0))
                            self.latest_brake = float(clip(brk, 0.0, 1.0))
                            self.latest_steer = float(clip(str_val, -1.0, 1.0))
                            self.latest_telem = {
                                'wheel_angle_rad': float(wheel_rad),
                                'accel_cmd': float(accel_cmd),
                                'v_ref_m_s': float(v_ref),
                                'jerk_m_s3': float(jerk_f),
                                'road_min_m': float(r_min),
                                'road_max_m': float(r_max),
                                's_ego_m': float(s_ego),
                                's_total_m': float(s_tot),
                                'd_yellow_m': float(d_yl),
                                'detect_ms': float(det_ms),
                                'current_idx': int(cur_idx),
                                'total_idx': int(tot_idx),
                                'obs_count': int(obs_cnt),
                                'active_obs_id': int(act_obs_id),
                                'bypass_side': int(bypass_side),
                                'is_emergency': int(is_emerg),
                                'is_roadblock': int(is_rb),
                                'yellow_line_type': int(yl_type),
                                'yellow_source': int(yl_src),
                                'horizon_x': hx,
                                'horizon_y': hy,
                            }
            except (socket.timeout, BlockingIOError):
                continue
            except Exception:
                break

    def build_hud_info(self, v_ego_ms: float, accel_ego: float, telem: dict) -> dict:
        """Chuyển đổi Telemetry từ C++20 sang cấu trúc hiển thị chuẩn trên Cockpit Dashboard"""
        is_emerg = bool(telem.get('is_emergency', 0))
        is_rb = bool(telem.get('is_roadblock', 0))
        bypass_side = telem.get('bypass_side', 0)
        act_obs_id = telem.get('active_obs_id', -1)

        if is_emerg:
            bypass_txt = "CHẾ ĐỘ 2: PHANH GẤP KHẨN CẤP!"
        elif is_rb:
            bypass_txt = "DỪNG AN TOÀN (ROADBLOCK)"
        elif bypass_side != 0 and act_obs_id >= 0:
            side_tag = "PHẢI" if bypass_side > 0 else "TRÁI"
            bypass_txt = f"NÉ {side_tag} (#{act_obs_id})"
        else:
            bypass_txt = "CHẾ ĐỘ 1: BÁM ĐƯỜNG"

        yl_type_code = telem.get('yellow_line_type', 1)
        yl_src_code = telem.get('yellow_source', 2)
        yl_type_str = "SOLID" if yl_type_code == 1 else ("DASHED" if yl_type_code == 2 else "NONE")
        yl_src_str = "CV" if yl_src_code == 1 else ("MAP" if yl_src_code == 2 else "NONE")
        d_yl = telem.get('d_yellow_m', 1.75)
        det_ms = telem.get('detect_ms', 0.0)
        yellow_hud = f"{yl_type_str} [{yl_src_str}] | d={d_yl:.2f}m | {det_ms:.1f}ms"

        return {
            'speed_kmh': v_ego_ms * 3.6,
            'target_kmh': telem.get('v_ref_m_s', 10.0) * 3.6,
            'steer_deg': math.degrees(telem.get('wheel_angle_rad', 0.0)),
            'accel_ms2': accel_ego,
            'jerk_ms3': telem.get('jerk_m_s3', 0.0),
            'bypass_text': bypass_txt,
            'obs_count': telem.get('obs_count', 0),
            'current_idx': telem.get('current_idx', self.current_wp_idx),
            'total_idx': max(1, telem.get('total_idx', len(self.traj_x))),
            's_ego_m': telem.get('s_ego_m', 0.0),
            's_total_m': telem.get('s_total_m', 760.88),
            'is_emergency': is_emerg,
            'yellow_line': yellow_hud,
            'yellow_source_str': yl_src_str,
            'road_min': telem.get('road_min_m', -0.45),
            'road_max': telem.get('road_max_m', 3.90),
        }

    def run_bridge_server(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('0.0.0.0', self.bridge_port))
        server_sock.listen(1)
        print(f"[BRIDGE] Lắng nghe kết nối từ C++20 Runtime tại cổng {self.bridge_port}...")

        client_sock, addr = server_sock.accept()
        client_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"[BRIDGE] C++20 Autonomous Runtime đã kết nối từ {addr}!")

        client_sock.settimeout(0.5)
        recv_thread = threading.Thread(target=self._receiver_loop, args=(client_sock,), daemon=True)
        recv_thread.start()

        prev_v = 0.0
        prev_a = 0.0
        cycle = 0
        last_sent_cam_seq = -1
        last_tick_time = time.time()

        try:
            while self.running.is_set():
                try:
                    snapshot = self.world.wait_for_tick(seconds=1.5)
                except RuntimeError:
                    continue

                now_time = time.time()
                dt = max(now_time - last_tick_time, 0.001)
                last_tick_time = now_time
                cycle += 1

                ego_snap = snapshot.find(self.ego_vehicle.id)
                if ego_snap is None:
                    print("[BRIDGE] Xe Ego không tồn tại trong snapshot.")
                    break

                tf = ego_snap.get_transform()
                vel = ego_snap.get_velocity()
                v = math.hypot(vel.x, vel.y)
                yaw_rad = math.radians(tf.rotation.yaw)
                accel = (v - prev_v) / dt
                jerk = (accel - prev_a) / dt
                prev_v = v
                prev_a = accel

                # 1. Truy vấn biên hành lang OpenDRIVE thời gian thực tại vị trí xe Ego
                map_rmin, map_rmax, map_yl_solid, is_junction = self.query_live_corridor(tf.location.x, tf.location.y)

                # 2. Lấy khung hình RGB 160x100x3 mới nhất từ Front Hood-Cam (nếu có frame mới)
                cam_bytes = None
                if self.viewer is not None:
                    cam_bytes, last_sent_cam_seq = self.viewer.pop_latest_front_rgb_bytes(last_sent_cam_seq)
                has_cam = 1 if (cam_bytes is not None and len(cam_bytes) == CAM_SHM_BYTES) else 0

                # 3. Lấy danh sách vật cản trong bán kính 38m quanh xe Ego từ WorldSnapshot
                actors_to_send = []
                for act in self.spawned_actors:
                    if act.id == self.ego_vehicle.id or not act.is_alive:
                        continue
                    act_snap = snapshot.find(act.id)
                    if act_snap is None:
                        continue
                    atf = act_snap.get_transform()
                    avel = act_snap.get_velocity()
                    d = math.hypot(atf.location.x - tf.location.x, atf.location.y - tf.location.y)
                    if d < 38.0:
                        meta = self.actor_metadata.get(act.id, {})
                        ext_x = meta.get('extent_x', 2.4)
                        ext_y = meta.get('extent_y', 1.0)
                        vx_act = avel.x if math.hypot(avel.x, avel.y) > 0.05 else meta.get('vx', 0.0)
                        vy_act = avel.y if math.hypot(avel.x, avel.y) > 0.05 else meta.get('vy', 0.0)
                        is_veh = meta.get('is_vehicle', 1)
                        actors_to_send.append((
                            act.id, atf.location.x, atf.location.y, atf.location.z,
                            math.radians(atf.rotation.yaw),
                            ext_x, ext_y, vx_act, vy_act, is_veh
                        ))

                # 4. Đóng gói gói tin nhị phân CARL sang C++20:
                #    - VehicleStateData (69 bytes): <QIdddddddB
                #    - BridgeMapCamHeader (24 bytes): <ddBBBBHH
                #    - Actors (4 + 69*N bytes)
                #    - Optional Camera RGB 160x100x3 (48000 bytes khi has_cam == 1)
                now_ns = int(time.time() * 1e9)
                state_bytes = struct.pack(
                    '<QIdddddddB',
                    now_ns, cycle,
                    tf.location.x, tf.location.y, yaw_rad,
                    v, self.latest_steer, accel, jerk, 1
                )
                map_cam_hdr = struct.pack(
                    '<ddBBBBHH',
                    map_rmin, map_rmax,
                    map_yl_solid, is_junction, has_cam, 0,
                    CAM_SHM_W, CAM_SHM_H
                )
                actors_bytes = struct.pack('<I', len(actors_to_send))
                for a in actors_to_send:
                    actors_bytes += struct.pack(
                        '<iddddddddB',
                        a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9]
                    )

                payload = state_bytes + map_cam_hdr + actors_bytes
                if has_cam:
                    payload += cam_bytes
                header = struct.pack('<4sI', b'CARL', len(payload))

                try:
                    client_sock.sendall(header + payload)
                except Exception:
                    print("[BRIDGE] C++20 App đã ngắt kết nối socket.")
                    break

                # 5. Đọc lệnh chấp hành & Telemetry mới nhất từ C++20
                with self.cmd_lock:
                    thr = self.latest_throttle
                    brk = self.latest_brake
                    steer = self.latest_steer
                    telem_snap = dict(self.latest_telem)

                ctrl = carla.VehicleControl(throttle=thr, brake=brk, steer=steer)
                self.ego_vehicle.apply_control(ctrl)

                # 6. Cập nhật vật cản di động & vẽ 3D Horizon / Danger Zone trên CARLA
                self.update_moving_obstacles_in_memory(dt=dt)
                if cycle % 3 == 0:
                    self.draw_dynamic_3d_overlays(
                        tf.location.x, tf.location.y, tf.location.z, yaw_rad,
                        telem_snap.get('horizon_x', []),
                        telem_snap.get('horizon_y', []),
                        bool(telem_snap.get('is_emergency', 0))
                    )
                if cycle % 2 == 0:
                    self.update_spectator(tf)

                # 7. Cập nhật giao diện Cockpit Multi-Camera Dashboard (1440 x 900)
                if self.viewer is not None and self.viewer.enable_window:
                    hud_info = self.build_hud_info(v, accel, telem_snap)
                    if not self.viewer.render(hud_info):
                        print("\n[COCKPIT DASHBOARD] Người dùng đã đóng cửa sổ Cockpit Viewer.")
                        break

        except KeyboardInterrupt:
            print("\n[BRIDGE] Dừng theo yêu cầu người dùng (Ctrl+C).")
        except Exception as e:
            import traceback
            print(f"\n[BRIDGE ERROR] Lỗi trong vòng lặp bridge: {e}")
            traceback.print_exc()
        finally:
            self.running.clear()
            try:
                client_sock.close()
                server_sock.close()
            except Exception:
                pass
            self.clean_up()

    def clean_up(self):
        print("[BRIDGE] Đang dọn dẹp cảm biến Camera và các actor xe đã spawn...")
        if self.viewer is not None:
            try:
                self.viewer.destroy()
            except Exception:
                pass
            self.viewer = None
        try:
            destroy_cmds = [carla.command.DestroyActor(a.id) for a in self.spawned_actors if a.is_alive]
            if destroy_cmds and self.client:
                self.client.apply_batch_sync(destroy_cmds, False)
        except Exception:
            pass
        print("[BRIDGE] Dọn dẹp hoàn tất. Daemon thoát.")


def main():
    parser = argparse.ArgumentParser(description="CARLA Simulation Bridge & Cockpit Dashboard for C++20 MPC")
    parser.add_argument('--with-obstacles', action='store_true', default=True, help="Spawn vật cản tĩnh và động")
    parser.add_argument('--no-obstacles', action='store_true', help="Tắt vật cản")
    parser.add_argument('--viewer', action='store_true', default=True, help="Mở cửa sổ Cockpit Multi-Camera Dashboard (1440x900)")
    parser.add_argument('--no-viewer', action='store_true', help="Chạy ngầm không mở cửa sổ Pygame (vẫn giữ Front Camera cho C++ CV)")
    parser.add_argument('--host', type=str, default='127.0.0.1', help="CARLA host")
    parser.add_argument('--port', type=int, default=2000, help="CARLA port")
    parser.add_argument('--bridge-port', type=int, default=20050, help="TCP bridge port for C++")
    args = parser.parse_args()

    with_obs = not args.no_obstacles
    with_viewer = not args.no_viewer

    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
    traj_path = os.path.join(base_dir, "configs/trajectories/reference_trajectory.csv")
    obs_path = os.path.join(base_dir, "configs/trajectories/obstacles.csv")
    mobs_path = os.path.join(base_dir, "configs/trajectories/moving_obstacles.csv")

    server = CarlaBridgeDaemon(
        host=args.host,
        port=args.port,
        bridge_port=args.bridge_port,
        with_obstacles=with_obs,
        with_viewer=with_viewer
    )
    server.setup_simulation(traj_path, obs_path, mobs_path)
    server.run_bridge_server()


if __name__ == '__main__':
    main()
