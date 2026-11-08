"""
yellow_line_detector.py
=======================
Nhận diện Vạch Kẻ & Hành Lang Mặt Đường bằng Computer Vision cổ điển (Front Hood-Cam).

Quy trình (< 5 ms / frame):
  1. Mask màu trên ảnh gốc: vạch VÀNG (Hue ~28, S thấp vì CARLA vẽ vàng nhạt) và vạch TRẮNG.
  2. IPM GIẢI TÍCH: ma trận homography tính trực tiếp từ nội/ngoại tham số camera
     (FOV, độ cao, góc pitch) → Bird's-Eye View đúng tỷ lệ mét, không cần đoán điểm.
  3. Histogram cột + Sliding Window cho TỪNG vạch → vị trí ngang + tỷ lệ lấp đầy
     (phân loại LIỀN / ĐỨT).
  4. Đi từ tim xe ra 2 bên: vạch ĐỨT → đi qua (làn cùng chiều), vạch LIỀN / VÀNG → dừng.
     Suy ra hành lang tâm xe [road_min, road_max] (theo hệ đường tham chiếu Frenet).
  5. Fallback CARLA Map khi camera không đủ tin cậy.

Quy ước dấu: d < 0 là bên TRÁI, d > 0 là bên PHẢI (giống toàn bộ MPC).
"""

from __future__ import annotations

import os
import time
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple, List

for _qt_env in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_PLUGIN_PATH"):
    os.environ.pop(_qt_env, None)
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import cv2
import numpy as np

from src.config import (
    YELLOW_H_LOW, YELLOW_H_HIGH, YELLOW_S_LOW, YELLOW_V_LOW,
    WHITE_S_HIGH, WHITE_V_LOW,
    EGO_HALF_WIDTH_M,
    OBSTACLE_AVOIDANCE_MARGIN_M,
    YELLOW_LINE_CLEARANCE_M,
    FRONT_CAM_X_M, FRONT_CAM_Z_M, FRONT_CAM_PITCH_DEG, FRONT_CAM_FOV_DEG,
)


class LineType(Enum):
    NONE   = "NONE"
    DASHED = "DASHED"
    SOLID  = "SOLID"


@dataclass
class LaneLine:
    """Một vạch kẻ phát hiện trong BEV (offset tính theo thân xe Ego)."""
    d_ego_m: float          # Khoảng cách ngang tới tâm xe (m), âm = trái
    color: str              # 'yellow' | 'white'
    line_type: LineType     # SOLID / DASHED
    fill_ratio: float


@dataclass
class YellowLineResult:
    line_type:        LineType             = LineType.NONE   # Loại vạch VÀNG bên trái
    poly_coeffs:      Optional[np.ndarray] = None
    d_yellow_line_m:  Optional[float]      = None   # Khoảng cách tâm xe → vạch vàng (m, >= 0)
    road_min_m:       Optional[float]      = None   # Biên trái tâm xe (hệ đường tham chiếu)
    road_max_m:       Optional[float]      = None   # Biên phải tâm xe (hệ đường tham chiếu)
    d_right_line_m:   Optional[float]      = None   # Tâm xe → biên phải hợp pháp (m)
    right_line_type:  LineType             = LineType.NONE
    left_line_type:   LineType             = LineType.NONE
    confidence:       float                = 0.0
    source:           str                  = "NONE"  # 'CV' | 'MAP' | 'NONE'
    lines:            List[LaneLine]       = field(default_factory=list)
    n_valid_windows:  int                  = 0
    gap_density:      float                = 1.0
    detect_time_ms:   float                = 0.0
    debug_bev:        Optional[np.ndarray] = None


class RealtimeYellowLineDetector:
    # Không gian BEV (hệ camera chiếu xuống mặt đất): ngang [-8, +8] m, dọc [2, 20] m trước camera
    BEV_RES = 0.05          # m / px  (20 px/m)
    BEV_LAT = 8.0
    BEV_X_MIN, BEV_X_MAX = 2.0, 20.0
    LANE_WIDTH_DEFAULT = 3.5

    def __init__(self, debug_draw: bool = True, road_half_w: float = 5.25, img_w: int = 480, img_h: int = 300):
        self.debug_draw = debug_draw
        self.road_half_w = road_half_w
        self.img_w, self.img_h = img_w, img_h
        self.r_clearance = float(EGO_HALF_WIDTH_M + OBSTACLE_AVOIDANCE_MARGIN_M)  # nửa xe + lề biên đường (1.35m)
        self.r_clear_yellow = float(EGO_HALF_WIDTH_M + YELLOW_LINE_CLEARANCE_M)

        self.bev_w = int(round(2 * self.BEV_LAT / self.BEV_RES))                       # 320
        self.bev_h = int(round((self.BEV_X_MAX - self.BEV_X_MIN) / self.BEV_RES))      # 360
        self._M = self._build_analytic_ipm()

        self._ema_min: Optional[float] = None
        self._ema_max: Optional[float] = None
        self._ema_alpha = 0.15   # Giảm từ 0.35 -> 0.15 để lọc mịn dằn xóc/rung camera, triệt tiêu giật biên độ ngang 5-10cm

        self._lower_y = np.array([YELLOW_H_LOW, YELLOW_S_LOW, YELLOW_V_LOW], np.uint8)
        self._upper_y = np.array([YELLOW_H_HIGH, 255, 255], np.uint8)
        self._k_wide = np.ones((1, 16), np.uint8)
        self._k_dil = np.ones((3, 5), np.uint8)

    # ------------------------------------------------------------------ IPM
    def _ground_to_pixel(self, X: float, Y: float) -> Tuple[float, float]:
        """Điểm mặt đất (X tới trước camera, Y sang phải) → pixel ảnh (mô hình pinhole CARLA)."""
        f = (self.img_w / 2.0) / math.tan(math.radians(FRONT_CAM_FOV_DEG) / 2.0)
        cx, cy = self.img_w / 2.0, self.img_h / 2.0
        th = math.radians(-FRONT_CAM_PITCH_DEG)   # góc chúc xuống (dương)
        h = FRONT_CAM_Z_M
        depth = X * math.cos(th) + h * math.sin(th)
        up = X * math.sin(th) - h * math.cos(th)
        return cx + f * Y / depth, cy - f * up / depth

    def _ground_to_bev(self, X: float, Y: float) -> Tuple[float, float]:
        return (Y + self.BEV_LAT) / self.BEV_RES, (self.BEV_X_MAX - X) / self.BEV_RES

    def _build_analytic_ipm(self) -> np.ndarray:
        pts_g = [(4.0, -2.0), (4.0, 2.0), (16.0, 2.0), (16.0, -2.0)]
        src = np.float32([self._ground_to_pixel(X, Y) for X, Y in pts_g])
        dst = np.float32([self._ground_to_bev(X, Y) for X, Y in pts_g])
        return cv2.getPerspectiveTransform(src, dst)

    def _bev_col_to_lat(self, col: float) -> float:
        return col * self.BEV_RES - self.BEV_LAT

    def _bev_row_to_x(self, row: float) -> float:
        return self.BEV_X_MAX - row * self.BEV_RES

    # ------------------------------------------------------------------ API
    def detect(self, rgb_frame: Optional[np.ndarray] = None, ego_vehicle=None, carla_map=None,
               ref_xy: Optional[Tuple[float, float]] = None, ref_theta: Optional[float] = None) -> YellowLineResult:
        t0 = time.perf_counter()
        result = YellowLineResult()

        # Độ lệch ngang của xe so với đường tham chiếu (để quy đổi offset CV từ hệ thân xe → hệ Frenet)
        d_ego_ref = 0.0
        if ego_vehicle is not None and ref_xy is not None:
            try:
                tf = ego_vehicle.get_transform()
                th = float(ref_theta) if ref_theta is not None else math.radians(tf.rotation.yaw)
                d_ego_ref = -math.sin(th) * (tf.location.x - ref_xy[0]) + math.cos(th) * (tf.location.y - ref_xy[1])
            except Exception:
                d_ego_ref = 0.0

        if rgb_frame is not None:
            try:
                result = self._process_camera_frame(rgb_frame, d_ego_ref)
            except Exception as e:
                print(f"\n[CV LANE] Lỗi xử lý frame: {e}")
                result = YellowLineResult()

        if result.confidence < 0.5 and ego_vehicle is not None and carla_map is not None:
            self._map_fallback(result, ego_vehicle, carla_map, ref_xy, ref_theta=ref_theta)

        if result.road_min_m is not None and result.road_max_m is not None:
            result.road_min_m = min(result.road_min_m, -0.45)
            result.road_max_m = max(result.road_max_m, 0.45)
            if self._ema_min is None or abs(result.road_min_m - self._ema_min) > 2.5:
                self._ema_min = result.road_min_m   # nhảy làn thật → reset EMA, không trễ
            else:
                self._ema_min += self._ema_alpha * (result.road_min_m - self._ema_min)
            if self._ema_max is None or abs(result.road_max_m - self._ema_max) > 2.5:
                self._ema_max = result.road_max_m
            else:
                self._ema_max += self._ema_alpha * (result.road_max_m - self._ema_max)
            result.road_min_m, result.road_max_m = float(self._ema_min), float(self._ema_max)

        result.detect_time_ms = (time.perf_counter() - t0) * 1000.0
        return result

    # ------------------------------------------------------------------ CV CORE
    def _process_camera_frame(self, rgb_frame: np.ndarray, d_ego_ref: float = 0.0) -> YellowLineResult:
        result = YellowLineResult()
        frame = rgb_frame[:, :, :3]
        if frame.shape[1] != self.img_w or frame.shape[0] != self.img_h:
            frame = cv2.resize(frame, (self.img_w, self.img_h))

        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        mask_y = cv2.inRange(hsv, self._lower_y, self._upper_y)
        # Trắng: sáng hơn hẳn nhựa đường (ngưỡng thích ứng theo median vùng đường), ít màu, không phải vàng
        road_v = float(np.median(hsv[int(self.img_h * 0.75):, int(self.img_w * 0.35):int(self.img_w * 0.65), 2]))
        v_th = max(WHITE_V_LOW, road_v + 45.0)
        mask_w = ((hsv[:, :, 2] >= v_th) & (hsv[:, :, 1] <= WHITE_S_HIGH) & (mask_y == 0)).astype(np.uint8) * 255
        # Chỉ xét phần dưới đường chân trời
        horizon = int(self._ground_to_pixel(self.BEV_X_MAX, 0.0)[1]) - 2
        mask_y[:max(0, horizon)] = 0
        mask_w[:max(0, horizon)] = 0

        bev_y = cv2.warpPerspective(mask_y, self._M, (self.bev_w, self.bev_h), flags=cv2.INTER_NEAREST)
        bev_w = cv2.warpPerspective(mask_w, self._M, (self.bev_w, self.bev_h), flags=cv2.INTER_NEAREST)

        lines = self._find_lines(bev_y, 'yellow') + self._find_lines(bev_w, 'white')
        # Gộp vạch trùng vị trí (vạch đôi / vàng chồng trắng) — ưu tiên vàng, rồi liền
        lines.sort(key=lambda l: l.d_ego_m)
        merged: List[LaneLine] = []
        for ln in lines:
            if merged and abs(ln.d_ego_m - merged[-1].d_ego_m) < 0.55:
                a = merged[-1]
                keep = a if (a.color == 'yellow') > (ln.color == 'yellow') or \
                    ((a.color == ln.color) and a.fill_ratio >= ln.fill_ratio) else ln
                merged[-1] = keep
            else:
                merged.append(ln)
        result.lines = merged

        # Phát hiện lưới vạch vàng ô mắt cáo trong giao lộ (nhiều vạch vàng song song hoặc vạch vàng bên phải)
        n_yellow = sum(1 for l in merged if l.color == 'yellow')
        if n_yellow >= 2 or any(l.color == 'yellow' and l.d_ego_m > 0.5 for l in merged):
            result.confidence = 0.0
            return result

        # Phát hiện vạch kẻ người đi bộ (Zebra Crosswalk): nhiều vạch trắng song song nằm sát nhau (< 1.8m)
        white_lines = [l for l in merged if l.color == 'white']
        close_white_pairs = sum(
            1 for i in range(1, len(white_lines))
            if abs(white_lines[i].d_ego_m - white_lines[i - 1].d_ego_m) < 1.8
        )
        if len(white_lines) >= 5 or close_white_pairs >= 2:
            result.confidence = 0.0
            return result

        left = sorted([l for l in merged if l.d_ego_m < 0], key=lambda l: -l.d_ego_m)   # gần → xa
        right = sorted([l for l in merged if l.d_ego_m > 0], key=lambda l: l.d_ego_m)

        # Vạch vàng gần nhất bên trái
        yl = next((l for l in left if l.color == 'yellow'), None)
        if yl is not None:
            result.line_type = yl.line_type if yl.line_type != LineType.NONE else LineType.SOLID
            result.d_yellow_line_m = float(-yl.d_ego_m)

        # Biên trái: đi ra ngoài qua vạch trắng ĐỨT; dừng ở vạch vàng hoặc vạch LIỀN
        left_bound, left_kind = self._walk_out(left, sign=-1, has_yellow=(yl is not None))
        right_bound, right_kind = self._walk_out(right, sign=+1, has_yellow=False)
        result.left_line_type = left[0].line_type if left else LineType.NONE
        result.right_line_type = LineType.SOLID if right_kind in ('solid', 'yellow') else (
            LineType.DASHED if right_kind == 'assumed' else LineType.NONE)

        # Độ tin cậy: thấy được vạch biên làn hiện tại ở cả 2 phía (kể cả khi xe đang đè giữa 2 làn ~3.5 m)
        has_l = bool(left) and -left[0].d_ego_m < 3.8
        has_r = bool(right) and right[0].d_ego_m < 3.8
        result.confidence = 1.0 if (has_l and has_r) else (0.4 if (has_l or has_r) else 0.0)

        if result.confidence >= 0.5 and left_bound is not None and right_bound is not None:
            clr_l = self.r_clear_yellow if left_kind == 'yellow' else self.r_clearance
            # Quy đổi từ hệ thân xe → hệ đường tham chiếu: d_ref = d_ego + d_ego_ref
            result.road_min_m = float(left_bound + clr_l + d_ego_ref)
            result.road_max_m = float(right_bound - self.r_clearance + d_ego_ref)
            result.d_right_line_m = float(right_bound)
            result.source = 'CV'

        if self.debug_draw:
            dbg = np.zeros((self.bev_h, self.bev_w, 3), np.uint8)
            dbg[bev_w > 0] = (255, 255, 255)
            dbg[bev_y > 0] = (255, 220, 0)
            cx = int(self._ground_to_bev(0, 0)[0])
            cv2.line(dbg, (cx, 0), (cx, self.bev_h), (255, 0, 0), 1)
            for ln in merged:
                c = int(round((ln.d_ego_m + self.BEV_LAT) / self.BEV_RES))
                col = (255, 160, 0) if ln.color == 'yellow' else (0, 200, 255)
                cv2.line(dbg, (c, self.bev_h - 12), (c, self.bev_h - 1), col, 3)
            if result.road_min_m is not None:
                for d in (result.road_min_m - d_ego_ref, result.road_max_m - d_ego_ref):
                    c = int(round((d + self.BEV_LAT) / self.BEV_RES))
                    cv2.line(dbg, (c, 0), (c, self.bev_h), (0, 255, 0), 1)
            result.debug_bev = dbg
        return result

    def _walk_out(self, side: List[LaneLine], sign: int, has_yellow: bool = False) -> Tuple[Optional[float], str]:
        """Đi từ tim xe ra ngoài; trả về (vị trí biên hợp pháp theo hệ thân xe, loại biên)."""
        if not side:
            return None, 'none'
        prev = 0.0
        crossed_dashed = 0
        for ln in side:
            if prev != 0.0 and abs(ln.d_ego_m - prev) < 2.0:
                # Hai vạch quá sát nhau (< 2.0m, không phải bề rộng làn chuẩn) → bỏ qua vạch nhiễu
                continue
            if abs(ln.d_ego_m - prev) > self.LANE_WIDTH_DEFAULT + 1.5 and prev != 0.0:
                # Khoảng trống quá lớn giữa 2 vạch (mất vạch) → dừng ở 1 làn chuẩn sau vạch đứt cuối
                return prev + sign * self.LANE_WIDTH_DEFAULT, 'assumed'
            if ln.color == 'yellow':
                return ln.d_ego_m, 'yellow'
            if ln.line_type == LineType.SOLID:
                return ln.d_ego_m, 'solid'
            prev = ln.d_ego_m
            crossed_dashed += 1
            if crossed_dashed >= 1:
                # Chỉ mở rộng tối đa 1 làn kế bên qua vạch trắng đứt để tránh cộng dồn nhiều làn giả trong giao lộ
                break
        if prev == 0.0:
            return None, 'none'
        # Vạch ngoài cùng nhìn thấy là vạch ĐỨT → cộng tối đa 1 làn cùng chiều phía sau
        far = prev + sign * self.LANE_WIDTH_DEFAULT
        max_lat = 5.25 if (sign < 0 and not has_yellow) else (self.BEV_LAT - 0.5)
        if abs(far) > max_lat:
            far = sign * max_lat
        return far, 'assumed'

    def _find_lines(self, bev: np.ndarray, color: str) -> List[LaneLine]:
        """Histogram vùng gần → các chân vạch; sliding window có bám độ dốc → vị trí + tỷ lệ lấp đầy theo hàng."""
        H, W = bev.shape
        # Loại bỏ vạch dừng ngang / tường trắng rộng (>= 0.8 m theo phương ngang)
        wide = cv2.dilate(cv2.morphologyEx(bev, cv2.MORPH_OPEN, self._k_wide), self._k_dil)
        bev_clean = cv2.subtract(bev, wide)

        r_near0 = int((self.BEV_X_MAX - 10.5) / self.BEV_RES)   # dải 2.5..10.5 m trước camera
        r_near1 = int((self.BEV_X_MAX - 2.5) / self.BEV_RES)
        hist = np.count_nonzero(bev_clean[r_near0:r_near1], axis=0).astype(np.float32)
        hist = np.convolve(hist, np.ones(7, np.float32) / 7, mode='same')
        min_peak = 6.0
        peaks = []
        sep = int(0.8 / self.BEV_RES)
        h2 = hist.copy()
        while True:
            c = int(np.argmax(h2))
            if h2[c] < min_peak:
                break
            peaks.append(c)
            h2[max(0, c - sep):c + sep] = 0

        nz_r, nz_c = np.nonzero(bev_clean)
        n_win = 18
        win_h = H // n_win
        margin = int(0.65 / self.BEV_RES)   # Mở rộng margin 0.65m để bắt liền mạch các vệt vạch đứt cách nhau
        out: List[LaneLine] = []
        for base in peaks:
            cur = float(base)
            step = 0.0
            xs, ys = [], []
            for k in range(n_win):
                r1 = H - k * win_h
                r0 = r1 - win_h
                m = (nz_r >= r0) & (nz_r < r1) & (np.abs(nz_c - cur) <= margin)
                if np.count_nonzero(m) >= 3:
                    mc = float(np.mean(nz_c[m]))
                    if xs:
                        step = 0.4 * step + 0.6 * float(np.clip(mc - (cur - step), -5.0, 5.0))
                    cur = mc + step
                    xs.append(nz_c[m]); ys.append(nz_r[m])
                else:
                    cur += step
            if len(xs) < 2:
                continue
            xs_a, ys_a = np.concatenate(xs), np.concatenate(ys)
            x_min = self._bev_row_to_x(float(np.max(ys_a)))
            x_max = self._bev_row_to_x(float(np.min(ys_a)))
            # Vạch đứt có thể chỉ xuất hiện 1 nét ~2.0m trong tầm quét gần
            if x_min > 8.5 or (x_max - x_min) < 1.8:
                continue
            poly = np.poly1d(np.polyfit(ys_a, xs_a, 1))
            if np.std(xs_a - poly(ys_a)) > 0.35 / self.BEV_RES:
                continue
            # Tỷ lệ lấp đầy theo từng hàng dọc theo vạch để phân biệt chuẩn xác vạch LIỀN và vạch ĐỨT
            r_span = np.arange(int(np.min(ys_a)), int(np.max(ys_a)) + 1)
            c_pred = np.clip(np.round(poly(r_span)).astype(int), 6, W - 7)
            row_hit = [bool(np.any(bev_clean[rr, cc - 6:cc + 7] > 0)) for rr, cc in zip(r_span, c_pred)]
            fill = float(np.sum(row_hit)) / float(H)
            if fill < 0.10:
                continue
            # Ngoại suy tuyến tính vùng gần về vị trí tâm xe (sau camera FRONT_CAM_X_M)
            near = self._bev_row_to_x(ys_a) < 11.0
            fit_c = np.polyfit(self._bev_row_to_x(ys_a[near] if near.sum() > 10 else ys_a),
                               xs_a[near] if near.sum() > 10 else xs_a, 1)
            col_at_ego = np.polyval(fit_c, -FRONT_CAM_X_M)
            d = float(self._bev_col_to_lat(col_at_ego))
            ltype = LineType.SOLID if fill >= 0.68 else LineType.DASHED
            out.append(LaneLine(d, color, ltype, fill))
        return out

    # ------------------------------------------------------------------ MAP FALLBACK
    def _map_fallback(self, result: YellowLineResult, ego_vehicle, carla_map, ref_xy, ref_theta=None):
        try:
            import carla
            ego_loc = ego_vehicle.get_location()
            q = carla.Location(x=float(ref_xy[0]), y=float(ref_xy[1]), z=ego_loc.z) if ref_xy is not None else ego_loc
            wp0 = carla_map.get_waypoint(q, project_to_road=True, lane_type=carla.LaneType.Driving)
            if wp0 is None:
                return
            solid = (carla.LaneMarkingType.Solid, carla.LaneMarkingType.SolidSolid, carla.LaneMarkingType.SolidBroken)

            if ref_xy is not None:
                th_ref = float(ref_theta) if ref_theta is not None else math.radians(wp0.transform.rotation.yaw)
                nx_ref, ny_ref = -math.sin(th_ref), math.cos(th_ref)
                loc0 = wp0.transform.location
                d_center = (loc0.x - float(ref_xy[0])) * nx_ref + (loc0.y - float(ref_xy[1])) * ny_ref
            else:
                d_center = 0.0

            d_left = d_center - wp0.lane_width / 2.0
            wp, is_yel, yel_solid = wp0, False, False
            while True:
                lm = wp.left_lane_marking
                if lm and lm.color == carla.LaneMarkingColor.Yellow:
                    is_yel, yel_solid = True, lm.type in solid
                    break
                nl = wp.get_left_lane()
                if nl is not None and nl.lane_type == carla.LaneType.Driving and nl.lane_id * wp0.lane_id > 0:
                    d_left -= nl.lane_width
                    wp = nl
                else:
                    break
            d_right = d_center + wp0.lane_width / 2.0
            wp = wp0
            while True:
                rm = wp.right_lane_marking
                if rm and rm.type in solid:
                    break
                nr = wp.get_right_lane()
                if nr is not None and nr.lane_type == carla.LaneType.Driving and nr.lane_id * wp0.lane_id > 0:
                    d_right += nr.lane_width
                    wp = nr
                else:
                    break

            if getattr(wp0, 'is_junction', False):
                d_left = min(d_left, -5.25)
                d_right = max(d_right, 4.50)
                is_yel, yel_solid = False, False

            yaw = math.radians(wp0.transform.rotation.yaw)
            loc = wp0.transform.location
            d_lat_ego = d_center + (-math.sin(yaw) * (ego_loc.x - loc.x) + math.cos(yaw) * (ego_loc.y - loc.y))

            clr_l = self.r_clear_yellow if (is_yel and yel_solid) else self.r_clearance
            result.road_min_m = float(min(d_left + clr_l, -0.45))
            result.road_max_m = float(max(d_right - self.r_clearance, 0.45))
            result.d_right_line_m = float(d_right - d_lat_ego)
            if is_yel:
                result.line_type = LineType.SOLID if yel_solid else LineType.DASHED
                result.d_yellow_line_m = float(max(0.0, d_lat_ego - d_left))
            result.source = 'MAP'
        except Exception:
            pass

    # ------------------------------------------------------------------ HUD
    def draw_debug_overlay(self, rgb_frame: np.ndarray, result: YellowLineResult) -> np.ndarray:
        out = rgb_frame.copy()
        h, w = out.shape[:2]
        rm = f"{result.road_min_m:+.2f}" if result.road_min_m is not None else "N/A"
        rx = f"{result.road_max_m:+.2f}" if result.road_max_m is not None else "N/A"
        txt = f"{result.source} CORRIDOR [{rm};{rx}] Y={result.line_type.value} conf={int(result.confidence*100)}%"
        cv2.rectangle(out, (8, h - 30), (w - 8, h - 8), (15, 22, 32), -1)
        cv2.putText(out, txt, (14, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (230, 255, 240), 1, cv2.LINE_AA)
        return out
