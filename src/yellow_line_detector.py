"""
yellow_line_detector.py
=======================
Mô-đun Nhận Diện Vạch Vàng Thời Gian Thực (Real-Time Yellow Lane Detector)
dành cho xe tự hành CARLA với hiệu năng < 10ms / khung hình.

Quy trình:
  1. Crop ROI 50% nửa dưới ảnh RGB → tiết kiệm CPU/VRAM.
  2. Chuyển ROI sang HSV → lọc màu vàng bằng cv2.inRange() (C++ SIMD).
  3. Biến đổi IPM (Inverse Perspective Mapping) → Bird's Eye View (BEV).
  4. Sliding Window Vectorized (NumPy) → phân loại SOLID / DASHED / NONE.
  5. Fit đa thức bậc 2 (x = ay² + by + c) trong không gian BEV.
  6. Tính khoảng cách ngang Frenet d_yellow_line từ tâm xe tới vạch vàng.

Tác giả: AutoAgent — chỉnh sửa cho CARLA Autonomous MPC
"""

from __future__ import annotations

import os
import time
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

# Xóa Qt plugin path của cv2 TRƯỚC KHI import cv2 để tránh lỗi xcb crash trên WSL/Linux
for _qt_env in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_PLUGIN_PATH"):
    os.environ.pop(_qt_env, None)
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import cv2
import numpy as np

from src.config import (
    # --- Camera & IPM ---
    CAM_IMAGE_WIDTH,
    CAM_IMAGE_HEIGHT,
    CAM_FOV_DEG,
    IPM_SRC_POINTS,
    IPM_DST_POINTS,
    IPM_BEV_WIDTH,
    IPM_BEV_HEIGHT,
    # --- HSV ---
    YELLOW_H_LOW,  YELLOW_H_HIGH,
    YELLOW_S_LOW,  YELLOW_S_HIGH,
    YELLOW_V_LOW,  YELLOW_V_HIGH,
    # --- Sliding Window ---
    YELLOW_NWIN,
    YELLOW_WIN_MARGIN_PX,
    YELLOW_MIN_PIX_PER_WIN,
    # --- Phân loại Solid/Dashed ---
    YELLOW_SOLID_FILL_RATIO,
    YELLOW_DASHED_GAP_RATIO,
    # --- Không gian vật lý BEV ---
    BEV_METERS_PER_PIXEL_X,
    BEV_METERS_PER_PIXEL_Y,
    # --- Clearance xe ---
    EGO_HALF_WIDTH_M,
    YELLOW_LINE_CLEARANCE_M,
)


# ==============================================================================
# ENUM LOẠI VẠCH
# ==============================================================================

class LineType(Enum):
    """Loại vạch vàng được phân loại từ sliding window"""
    NONE   = "NONE"    # Không phát hiện vạch vàng đủ tin cậy
    DASHED = "DASHED"  # Vạch đứt (cho phép xe vượt)
    SOLID  = "SOLID"   # Vạch liền (CẤM xe đè / vượt)


# ==============================================================================
# DATACLASS KẾT QUẢ PERCEPTION
# ==============================================================================

@dataclass
class YellowLineResult:
    """
    Kết quả trả về từ RealtimeYellowLineDetector.detect().

    Attributes:
        line_type           : Loại vạch (NONE / DASHED / SOLID).
        poly_coeffs         : Hệ số đa thức bậc 2 [a, b, c] fit vạch trong BEV
                              (x_bev = a*y² + b*y + c).  None nếu không phát hiện.
        d_yellow_line_m     : Khoảng cách ngang (m) từ tâm xe tới vạch vàng trong
                              không gian Frenet.  Âm = vạch bên trái xe.
                              None nếu không phát hiện.
        road_max_m          : Giới hạn biên trái hành lang lái xe sau khi đã áp
                              ràng buộc vạch liền (m).  None nếu không có vạch liền.
        n_valid_windows     : Số cửa sổ sliding window có pixel vàng hợp lệ.
        gap_density         : Tỷ lệ cửa sổ TRỐNG trong tổng cửa sổ (0..1).
        detect_time_ms      : Thời gian xử lý (ms).
        debug_bev           : Ảnh BEV nhị phân (uint8 H×W) để debug — None nếu
                              DEBUG_YELLOW_DRAW = False.
    """
    line_type:        LineType          = LineType.NONE
    poly_coeffs:      Optional[np.ndarray] = None   # [a, b, c]
    d_yellow_line_m:  Optional[float]      = None
    road_max_m:       Optional[float]      = None
    n_valid_windows:  int                  = 0
    gap_density:      float                = 1.0
    detect_time_ms:   float                = 0.0
    debug_bev:        Optional[np.ndarray] = None


# ==============================================================================
# CLASS CHÍNH
# ==============================================================================

class RealtimeYellowLineDetector:
    """
    Bộ nhận diện vạch vàng thời gian thực (< 10ms / frame).

    Thuật toán:
      • Crop ROI 50% nửa dưới ảnh → chuyển HSV → inRange filter (C++ SIMD).
      • IPM warp ảnh nhị phân 1-channel sang Bird's Eye View.
      • Vectorized Sliding Window (NumPy only, không vòng lặp pixel).
      • Phân loại SOLID / DASHED dựa trên fill ratio & gap density.
      • Fit đa thức bậc 2 và tính Frenet offset d_yellow_line.

    Parameters:
        debug_draw  : True để trả về ảnh BEV debug trong result.debug_bev.
        road_half_w : Nửa chiều rộng đường mặc định (m) — bản sao cục bộ của
                      DEFAULT_ROAD_HALF_WIDTH_M để tính road_max_m.
    """

    def __init__(self, debug_draw: bool = False, road_half_w: float = 5.25):
        self.debug_draw = debug_draw
        self.road_half_w = road_half_w

        # Tính trước ma trận IPM một lần duy nhất (tránh recompute mỗi frame)
        self._M: np.ndarray = self._build_ipm_matrix()
        self._M_inv: np.ndarray = np.linalg.inv(self._M)

        # Cache thông số sliding window
        self._n_win:       int = YELLOW_NWIN
        self._win_margin:  int = YELLOW_WIN_MARGIN_PX
        self._min_pix:     int = YELLOW_MIN_PIX_PER_WIN

        # Dải màu HSV vạch vàng (lower / upper bound)
        self._lower_yellow = np.array([YELLOW_H_LOW,  YELLOW_S_LOW,  YELLOW_V_LOW],  dtype=np.uint8)
        self._upper_yellow = np.array([YELLOW_H_HIGH, YELLOW_S_HIGH, YELLOW_V_HIGH], dtype=np.uint8)

    # ------------------------------------------------------------------
    # API CÔNG KHAI
    # ------------------------------------------------------------------

    def detect(
        self,
        rgb_frame: Optional[np.ndarray] = None,
        ego_vehicle=None,
        carla_map=None,
        ref_xy: Optional[Tuple[float, float]] = None
    ) -> YellowLineResult:
        """
        Nhận diện vạch vàng thời gian thực (Camera RGB + CARLA Map Waypoint Ground Truth).
        Tối ưu tuyệt đối cho vạch đôi nét liền màu vàng (Double Solid Yellow Line) trong CARLA.

        Args:
            rgb_frame: Ảnh RGB shape (H, W, 3) hoặc None.
            ego_vehicle: Actor xe Ego trong CARLA (tùy chọn để lấy vị trí chính xác).
            carla_map: Bản đồ CARLA (tùy chọn để truy vấn vạch kẻ OpenDRIVE).
            ref_xy: Tọa độ (x_ref, y_ref) của quỹ đạo tham chiếu tại vị trí hiện tại (giữ cố định
                    làn gốc ngay cả khi xe đang lách sang làn phải để né vật cản).

        Returns:
            YellowLineResult chứa trạng thái vạch vàng và giới hạn hành lang lái xe.
        """
        t0 = time.perf_counter()
        result = YellowLineResult()

        if rgb_frame is not None:
            # Step 1: ROI Crop + HSV Masking
            binary_roi = self._hsv_yellow_mask(rgb_frame)

            # Step 2: IPM → Bird's Eye View (chỉ warp ảnh nhị phân 1-channel)
            bev = cv2.warpPerspective(
                binary_roi,
                self._M,
                (IPM_BEV_WIDTH, IPM_BEV_HEIGHT),
                flags=cv2.INTER_NEAREST
            )

            # Step 3: Vectorized Sliding Window
            result = self._sliding_window(bev)

            if self.debug_draw:
                result.debug_bev = bev

        # HỖ TRỢ BẢO ĐẢM AN TOÀN TUYỆT ĐỐI VỚI CARLA MAP GROUND TRUTH:
        # Nhận diện vạch đôi nét liền màu vàng (SolidSolid / Solid) hoặc nét đứt bên trái làn tham chiếu
        if ego_vehicle is not None and carla_map is not None:
            try:
                import carla
                ego_loc = ego_vehicle.get_location()
                if ref_xy is not None:
                    query_loc = carla.Location(x=float(ref_xy[0]), y=float(ref_xy[1]), z=ego_loc.z)
                else:
                    query_loc = ego_loc

                wp = carla_map.get_waypoint(query_loc, project_to_road=True, lane_type=carla.LaneType.Driving)

                # Nếu không truyền ref_xy và xe đang tạm lách sang làn bên phải để né vật cản,
                # kiểm tra làn cùng chiều bên trái để không bị mất dấu vạch vàng tim đường
                if wp is not None and hasattr(wp, 'left_lane_marking'):
                    if wp.left_lane_marking.color != carla.LaneMarkingColor.Yellow and hasattr(wp, 'get_left_lane'):
                        wp_left = wp.get_left_lane()
                        if (wp_left is not None
                                and wp_left.lane_type == carla.LaneType.Driving
                                and (wp_left.lane_id * wp.lane_id > 0)
                                and hasattr(wp_left, 'left_lane_marking')
                                and wp_left.left_lane_marking.color == carla.LaneMarkingColor.Yellow):
                            wp = wp_left

                if wp is not None and hasattr(wp, 'left_lane_marking'):
                    lm = wp.left_lane_marking
                    is_yellow = (lm.color == carla.LaneMarkingColor.Yellow)
                    is_solid = lm.type in (
                        carla.LaneMarkingType.Solid,
                        carla.LaneMarkingType.SolidSolid,
                        carla.LaneMarkingType.SolidBroken
                    )
                    is_broken = lm.type in (
                        carla.LaneMarkingType.Broken,
                        carla.LaneMarkingType.BrokenSolid,
                        carla.LaneMarkingType.BrokenBroken
                    )

                    wp_loc = wp.transform.location
                    wp_yaw = math.radians(wp.transform.rotation.yaw)
                    # Trong hệ CARLA/Unreal: nx = -sin(yaw), ny = cos(yaw) chỉ sang PHẢI làn đường
                    nx = -math.sin(wp_yaw)
                    ny = math.cos(wp_yaw)

                    # Độ lệch ngang của xe so với tim làn tham chiếu (d_lat_ego):
                    # d_lat_ego > 0: xe lệch sang PHẢI (xa vạch vàng)
                    # d_lat_ego < 0: xe lệch sang TRÁI (tiến sát vạch vàng)
                    d_lat_ego = nx * (ego_loc.x - wp_loc.x) + ny * (ego_loc.y - wp_loc.y)

                    # Vạch vàng bên trái nằm ở d_yellow = -d_lane_half (thường là -1.75m)
                    d_lane_half = float(wp.lane_width / 2.0)
                    d_yellow_line = -d_lane_half

                    # Khoảng cách thực tế thời gian thực từ tâm xe tới vạch vàng:
                    # d_actual = d_lat_ego - d_yellow_line = d_lat_ego + d_lane_half
                    d_actual_to_yellow = max(0.0, d_lat_ego + d_lane_half)

                    if is_yellow and is_solid:
                        result.line_type = LineType.SOLID
                        result.d_yellow_line_m = float(d_actual_to_yellow)

                        # Giới hạn an toàn cận dưới road_min (đảm bảo sườn xe cách vạch vàng ít nhất YELLOW_LINE_CLEARANCE_M):
                        # r_clear = 1.0m (nửa xe) + 0.35m (clearance) = 1.35m
                        # road_min = d_yellow_line + r_clear = -1.75 + 1.35 = -0.40m
                        r_clear = EGO_HALF_WIDTH_M + YELLOW_LINE_CLEARANCE_M
                        result.road_max_m = d_yellow_line + r_clear   # giá trị âm (ví dụ: -0.40m)
                    elif is_yellow and is_broken:
                        result.line_type = LineType.DASHED
                        result.d_yellow_line_m = float(d_actual_to_yellow)
                        result.road_max_m = None
                    else:
                        # Không có vạch vàng bên trái làn đường -> xóa tránh nhiễu màu vàng từ marker vật cản
                        result.line_type = LineType.NONE
                        result.d_yellow_line_m = None
                        result.road_max_m = None
            except Exception:
                pass

        detect_ms = (time.perf_counter() - t0) * 1000.0
        result.detect_time_ms = detect_ms
        return result

    # ------------------------------------------------------------------
    # BƯỚC 1: HSV MASKING
    # ------------------------------------------------------------------

    def _hsv_yellow_mask(self, rgb_frame: np.ndarray) -> np.ndarray:
        """
        Crop 50% nửa dưới ảnh RGB, chuyển sang HSV, lọc màu vàng.

        Returns:
            binary_roi: Mảng uint8 (H_crop × W) — 255 = pixel vàng, 0 = nền.
        """
        h, w = rgb_frame.shape[:2]

        # Crop 50% nửa dưới (vùng mặt đường)
        roi = rgb_frame[h // 2:, :]                         # (H/2, W, 3)

        # Chuyển BGR/RGB → HSV (OpenCV C++ SIMD, không cần thêm bước LAB)
        # CARLA camera trả về BGRA → cần xử lý alpha nếu có
        if roi.shape[2] == 4:
            roi = roi[:, :, :3]                             # Bỏ kênh alpha

        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)          # (H/2, W, 3)

        # Lọc dải màu vàng — cv2.inRange chạy C++ SIMD (vectorized tự động)
        mask = cv2.inRange(hsv, self._lower_yellow, self._upper_yellow)  # uint8

        # Chuẩn hóa kích thước mask ROI về đúng không gian tham chiếu của ma trận IPM (1000 × 450)
        # khi nhận ảnh trực tiếp từ Front View (Hood-Cam 480 × 300)
        target_roi_w = CAM_IMAGE_WIDTH
        target_roi_h = CAM_IMAGE_HEIGHT // 2
        if mask.shape[1] != target_roi_w or mask.shape[0] != target_roi_h:
            mask = cv2.resize(mask, (target_roi_w, target_roi_h), interpolation=cv2.INTER_NEAREST)

        return mask  # Shape (CAM_IMAGE_HEIGHT/2, CAM_IMAGE_WIDTH)

    # ------------------------------------------------------------------
    # BƯỚC 2: IPM matrix
    # ------------------------------------------------------------------

    @staticmethod
    def _build_ipm_matrix() -> np.ndarray:
        """
        Xây dựng ma trận IPM 3×3 từ cấu hình camera CARLA tiêu chuẩn.

        Điểm src được định nghĩa trong không gian ROI (nửa dưới ảnh),
        điểm dst là góc ảnh BEV đầu ra.
        """
        src = np.float32(IPM_SRC_POINTS)
        dst = np.float32(IPM_DST_POINTS)
        M = cv2.getPerspectiveTransform(src, dst)
        return M

    # ------------------------------------------------------------------
    # BƯỚC 3: VECTORIZED SLIDING WINDOW
    # ------------------------------------------------------------------

    def _sliding_window(self, bev: np.ndarray) -> YellowLineResult:
        """
        Thuật toán Sliding Window Vectorized (NumPy) trên ảnh BEV nhị phân.

        Chiến lược:
          • Histogram 1D tích lũy → tìm peak gốc vạch.
          • Chia chiều cao BEV thành N_WIN cửa sổ chồng nhau.
          • Mỗi cửa sổ: dùng np.nonzero() + Boolean indexing để lấy tọa độ
            pixel vàng (TUYỆT ĐỐI không dùng vòng lặp Python từng pixel).
          • Phân loại SOLID / DASHED → fit poly bậc 2 → tính Frenet offset.

        Returns:
            YellowLineResult (chưa có detect_time_ms — sẽ được gán ngoài).
        """
        H, W = bev.shape
        result = YellowLineResult()

        # ----------------------------------------------------------------
        # 3a. Tìm gốc vạch bằng histogram nửa trái BEV
        # ----------------------------------------------------------------
        # Tổng pixel vàng theo từng cột (axis=0), chỉ lấy nửa dưới BEV
        half_h = H // 2
        histogram = np.sum(bev[half_h:, :], axis=0)    # shape (W,)

        # Tìm peak trong vùng bên trái (vạch vàng phân làn ngược chiều nằm bên trái làn xe)
        left_half  = histogram[:W // 2]
        right_half = histogram[W // 2:]

        left_base  = int(np.argmax(left_half))
        right_base = int(np.argmax(right_half)) + W // 2

        # Ưu tiên nửa trái BEV để tránh nhầm lẫn với marker/vật cản màu vàng cam bên phải làn
        if histogram[left_base] >= self._min_pix * 2:
            base_x = left_base
        elif histogram[right_base] >= self._min_pix * 2:
            base_x = right_base
        else:
            result.line_type = LineType.NONE
            return result

        # ----------------------------------------------------------------
        # 3b. Sliding Window Vectorized
        # ----------------------------------------------------------------
        win_height = H // self._n_win    # Chiều cao mỗi cửa sổ (pixels)

        # Lấy TẤT CẢ tọa độ pixel vàng trong BEV một lần (vectorized)
        all_nz_y, all_nz_x = np.nonzero(bev)   # shape (N_pixels,) each

        cur_x  = base_x
        all_good_x: list[np.ndarray] = []   # Danh sách mảng x-coords mỗi win
        all_good_y: list[np.ndarray] = []   # Danh sách mảng y-coords mỗi win
        valid_win_flags: list[bool] = []     # True nếu cửa sổ có đủ pixel

        for win_idx in range(self._n_win):
            # Biên cửa sổ theo chiều dọc (y tăng từ trên xuống dưới)
            y_low  = H - (win_idx + 1) * win_height
            y_high = H - win_idx * win_height

            # Boolean mask trên mảng tọa độ toàn BEV (vectorized, không loop)
            mask_y = (all_nz_y >= y_low) & (all_nz_y < y_high)
            mask_x = (all_nz_x >= cur_x - self._win_margin) & \
                     (all_nz_x <  cur_x + self._win_margin)
            mask_win = mask_y & mask_x

            win_x = all_nz_x[mask_win]
            win_y = all_nz_y[mask_win]

            has_pixels = len(win_x) >= self._min_pix
            valid_win_flags.append(has_pixels)

            if has_pixels:
                all_good_x.append(win_x)
                all_good_y.append(win_y)
                # Cập nhật tâm cửa sổ cho bước tiếp (mean shift)
                cur_x = int(np.mean(win_x))

        n_valid = int(np.sum(valid_win_flags))
        n_total = self._n_win

        result.n_valid_windows = n_valid

        # ----------------------------------------------------------------
        # 3c. Phân loại SOLID / DASHED / NONE
        # ----------------------------------------------------------------
        if n_valid == 0:
            result.line_type = LineType.NONE
            return result

        fill_ratio = n_valid / n_total

        # Tính gap density: tỷ lệ cửa sổ TRỐNG
        gap_density = 1.0 - fill_ratio
        result.gap_density = gap_density

        # Kiểm tra phân bổ khoảng trống (gap) xen kẽ (đặc trưng vạch đứt)
        consecutive_empty = self._count_max_consecutive_false(valid_win_flags)

        if fill_ratio >= YELLOW_SOLID_FILL_RATIO:
            line_type = LineType.SOLID
        elif gap_density >= YELLOW_DASHED_GAP_RATIO and consecutive_empty >= 2:
            # Vạch đứt: có khoảng trống lớn xen kẽ đặc trưng (≥2 cửa sổ liên tiếp trống)
            line_type = LineType.DASHED
        elif fill_ratio >= 0.35:
            # Ít pixel nhưng không đủ đứt rõ → coi là DASHED (phủ thêm)
            line_type = LineType.DASHED
        else:
            line_type = LineType.NONE

        result.line_type = line_type
        if line_type == LineType.NONE:
            return result

        # ----------------------------------------------------------------
        # 3d. Fit đa thức bậc 2 trong BEV: x = a*y² + b*y + c
        # ----------------------------------------------------------------
        if all_good_x:
            pts_x = np.concatenate(all_good_x)   # Tất cả x-coords hợp lệ
            pts_y = np.concatenate(all_good_y)   # Tất cả y-coords hợp lệ

            # np.polyfit chạy trên NumPy C-backend — đủ nhanh với ~200-800 pts
            if len(pts_x) >= 3:
                try:
                    coeffs = np.polyfit(pts_y, pts_x, 2)   # [a, b, c]
                    result.poly_coeffs = coeffs
                except np.linalg.LinAlgError:
                    result.poly_coeffs = None

        # ----------------------------------------------------------------
        # 3e. Tính khoảng cách ngang thực tế d_yellow_line_m (m, >= 0)
        # ----------------------------------------------------------------
        d_m = self._compute_frenet_offset(result.poly_coeffs, bev.shape)
        result.d_yellow_line_m = d_m

        # ----------------------------------------------------------------
        # 3f. Tính road_max_m (giới hạn biên trái road_min cho vạch SOLID trong MPC)
        # ----------------------------------------------------------------
        if line_type == LineType.SOLID and d_m is not None:
            r_clearance = EGO_HALF_WIDTH_M + YELLOW_LINE_CLEARANCE_M
            # HỆ TỌA ĐỘ CARLA FRENET: d < 0 là BÊN TRÁI (vạch đôi vàng nét liền ở d = -1.75m),
            # d > 0 là BÊN PHẢI. Giới hạn biên trái của tâm xe: road_min = -1.75 + r_clearance = -0.40m
            result.road_max_m = -1.75 + r_clearance

        return result

    # ------------------------------------------------------------------
    # HÀM PHỤ TRỢ NỘI BỘ
    # ------------------------------------------------------------------

    def _compute_frenet_offset(
        self,
        poly_coeffs: Optional[np.ndarray],
        bev_shape: Tuple[int, int]
    ) -> Optional[float]:
        """
        Tính khoảng cách ngang thực tế (m, >= 0) từ tâm xe (đáy ảnh BEV) tới vạch vàng bên trái.

        Trong BEV:
          • y = H-1 tương ứng với vị trí ngay trước đầu xe.
          • Trục x ngang: x < W/2 → vạch nằm bên trái tâm xe.

        Args:
            poly_coeffs : [a, b, c] từ np.polyfit(y, x, 2), hoặc None.
            bev_shape   : (H, W) của ảnh BEV.

        Returns:
            d_m: float (m, >= 0) hoặc None nếu không tính được.
        """
        if poly_coeffs is None:
            return None

        H, W = bev_shape
        a, b, c = poly_coeffs

        # Vị trí x của vạch tại y = H-1 (gần mũi xe nhất)
        y_ego = float(H - 1)
        x_line_px = a * y_ego**2 + b * y_ego + c

        # Tâm xe ở cột W/2 trong BEV
        x_ego_px = float(W) / 2.0

        # Khoảng cách ngang từ tâm xe tới vạch vàng bên trái (luôn >= 0m, đồng nhất với Ground Truth)
        dx_px = x_line_px - x_ego_px
        if dx_px > 40.0:
            # Nếu vạch nằm hẳn sang bên phải mũi xe trong BEV -> không phải vạch vàng tim đường bên trái
            return None
        dist_m = max(0.0, -dx_px * BEV_METERS_PER_PIXEL_X)
        return float(dist_m)

    @staticmethod
    def _count_max_consecutive_false(flags: list[bool]) -> int:
        """
        Đếm số cửa sổ TRỐNG (False) liên tiếp dài nhất trong danh sách.
        Dùng để phân biệt vạch đứt (nhiều khoảng trống liên tiếp xen kẽ).
        """
        max_run = 0
        cur_run = 0
        for f in flags:
            if not f:
                cur_run += 1
                if cur_run > max_run:
                    max_run = cur_run
            else:
                cur_run = 0
        return max_run

    # ------------------------------------------------------------------
    # API DEBUG / VISUALIZATION
    # ------------------------------------------------------------------

    def draw_debug_overlay(
        self,
        rgb_frame: np.ndarray,
        result: YellowLineResult
    ) -> np.ndarray:
        """
        Vẽ overlay thông tin nhận diện lên ảnh RGB gốc (cho debug HUD).

        Returns:
            Ảnh RGB có chú thích (cùng shape với đầu vào).
        """
        out = rgb_frame.copy()
        h, w = out.shape[:2]

        # Màu theo loại vạch
        color_map = {
            LineType.NONE:   (128, 128, 128),
            LineType.DASHED: (0, 220, 255),
            LineType.SOLID:  (0, 60, 255),
        }
        color = color_map[result.line_type]

        # Text trạng thái
        label = (
            f"YELLOW LINE: {result.line_type.value} | "
            f"d={result.d_yellow_line_m:.2f}m | "
            f"valid_win={result.n_valid_windows}/{YELLOW_NWIN} | "
            f"{result.detect_time_ms:.1f}ms"
        ) if result.d_yellow_line_m is not None else (
            f"YELLOW LINE: {result.line_type.value} | {result.detect_time_ms:.1f}ms"
        )

        cv2.putText(out, label, (10, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

        # Vẽ đường fit đa thức trong ROI nếu có (chiếu từ BEV về image)
        if result.poly_coeffs is not None:
            self._draw_poly_on_roi(out, result.poly_coeffs)

        return out

    def _draw_poly_on_roi(self, frame: np.ndarray, coeffs: np.ndarray) -> None:
        """
        Chiếu đường fit BEV ngược về ảnh gốc và vẽ.
        Dùng M_inv để unproject các điểm trên đường cong.
        """
        H_bev = IPM_BEV_HEIGHT
        W_bev = IPM_BEV_WIDTH
        H_img = frame.shape[0]

        a, b, c = coeffs
        pts_bev = []
        for y_b in range(H_bev // 4, H_bev, H_bev // 20):
            x_b = a * y_b**2 + b * y_b + c
            pts_bev.append([x_b, float(y_b)])

        if not pts_bev:
            return

        # Unproject BEV → ROI image (M_inv)
        bev_arr = np.array(pts_bev, dtype=np.float32).reshape(-1, 1, 2)
        roi_arr = cv2.perspectiveTransform(bev_arr, self._M_inv)

        h_frame = frame.shape[0]
        roi_offset_y = h_frame // 2  # ROI bắt đầu từ giữa ảnh

        pts_img = []
        for pt in roi_arr.reshape(-1, 2):
            xi = int(pt[0])
            yi = int(pt[1]) + roi_offset_y
            if 0 <= xi < frame.shape[1] and roi_offset_y <= yi < h_frame:
                pts_img.append((xi, yi))

        for i in range(len(pts_img) - 1):
            cv2.line(frame, pts_img[i], pts_img[i + 1], (0, 255, 100), 2, cv2.LINE_AA)

