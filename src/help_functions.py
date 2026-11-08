import os
import csv
import numpy as np
import math
from src.config import N, dt, TRAJECTORY_CSV_PATH, TARGET_SPEED_MS, OVERRIDE_V_REF

def normalize_angle(angle):
    """Chuẩn hóa góc về dải [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))

def load_csv_trajectory(filepath=None):
    """
    Đọc file CSV chứa quỹ đạo tham chiếu (x, y, theta_rad, v_m_s).
    Áp dụng np.unwrap để làm mịn góc hướng theta.
    """
    if filepath is None:
        filepath = TRAJECTORY_CSV_PATH

    if not os.path.exists(filepath):
        alt_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../001/reference_trajectory.csv"))
        if os.path.exists(alt_path):
            filepath = alt_path
        else:
            raise FileNotFoundError(f"Không tìm thấy file quỹ đạo tại {filepath}. "
                                    f"Hãy chạy script python src/generate_path.py để chọn và tạo quỹ đạo mới!")

    x_traj, y_traj, theta_traj, v_ref = [], [], [], []
    with open(filepath, mode='r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            x_traj.append(float(row['x']))
            y_traj.append(float(row['y']))
            theta_traj.append(float(row['theta_rad']))
            v_ref.append(float(row['v_m_s']))

    x_traj = np.array(x_traj)
    y_traj = np.array(y_traj)
    theta_traj = np.unwrap(np.array(theta_traj))  # Khắc phục lỗi nhảy góc +-pi
    v_ref = np.array(v_ref)
    
    # Nâng tốc độ tham chiếu lên TARGET_SPEED_MS nếu OVERRIDE_V_REF được bật
    if OVERRIDE_V_REF and len(v_ref) > 0:
        max_v = np.max(v_ref)
        if max_v > 0:
            scale_factor = TARGET_SPEED_MS / max_v
            v_ref = v_ref * scale_factor
            print(f"[SPEED UP] Đã tự động nâng vận tốc tối đa trong file CSV lên {TARGET_SPEED_MS:.1f} m/s ({TARGET_SPEED_MS * 3.6:.1f} km/h).")

    # Làm mượt vận tốc tham chiếu tại các khúc cua (khử nhiễu "hố kép" do đạo hàm bậc 2 tại điểm nối Bezier):
    # Giữ vận tốc vào cua ổn định (~5.2 m/s) xuyên suốt đỉnh cua và chuyển tiếp gia/giảm tốc êm ái
    if len(x_traj) > 50:
        ds_arr = np.hypot(np.diff(x_traj), np.diff(y_traj))
        s_cum = np.concatenate(([0.0], np.cumsum(ds_arr)))
        ds_step = np.append(ds_arr, ds_arr[-1])

        win_k = 35  # Cửa sổ ~3.5m tính độ cong dây cung hiệu dụng, triệt tiêu gai độ cong tại mút nối thẳng-cong
        pad_k = win_k // 2
        th_pad = np.pad(theta_traj, pad_k, mode='edge')
        s_pad = np.pad(s_cum, pad_k, mode='edge')
        dth_win = np.abs(th_pad[2 * pad_k:] - th_pad[:-2 * pad_k])
        ds_win = np.maximum(s_pad[2 * pad_k:] - s_pad[:-2 * pad_k], 1.0)
        kappa_eff = dth_win / ds_win

        a_lat_comfort = 4.2  # Gia tốc hướng tâm êm ái khi vào cua (m/s^2)
        v_max_target = float(np.max(v_ref)) if len(v_ref) > 0 else TARGET_SPEED_MS
        v_min_corner = min(6.0, v_max_target)
        v_curve = np.sqrt(a_lat_comfort / np.maximum(kappa_eff, 1e-4))
        v_smooth = np.clip(v_curve, v_min_corner, v_max_target)

        # Chỉ làm phẳng "hố kép" (valley fill) bên trong phạm vi thực của khúc cua (kappa_eff > 0.03)
        # để không làm lan vùng tốc độ thấp ra đoạn đường thẳng trước và sau khúc cua
        pad_m = 30
        vp_pad = np.pad(v_smooth, pad_m, mode='edge')
        v_valley = np.array([np.min(vp_pad[i:i + 2 * pad_m + 1]) for i in range(len(v_smooth))])
        in_curve_mask = kappa_eff > 0.03
        v_smooth = np.where(in_curve_mask, v_valley, v_smooth)

        # Quy hoạch Forward-Backward giới hạn gia tốc dọc mượt mà ngay sát cửa vào/ra cua
        a_dec_smooth = 3.5  # m/s^2
        a_acc_smooth = 4.5  # m/s^2
        for i in range(len(v_smooth) - 2, -1, -1):
            v_smooth[i] = min(v_smooth[i], math.sqrt(v_smooth[i + 1] ** 2 + 2.0 * a_dec_smooth * float(ds_step[i])))
        for i in range(1, len(v_smooth)):
            v_smooth[i] = min(v_smooth[i], math.sqrt(v_smooth[i - 1] ** 2 + 2.0 * a_acc_smooth * float(ds_step[i - 1])))

        v_ref = v_smooth

    print(f"[LOAD TRAJECTORY] Đã tải thành công {len(x_traj)} waypoints từ: {filepath}")
    return x_traj, y_traj, theta_traj, v_ref

def update_reference_point(x0, y0, current_idx, x_traj, y_traj, search_range=150):
    """
    Tìm chỉ số Waypoint gần nhất trên đường đi xung quanh vị trí hiện tại của xe.
    Mở rộng search_range=150 (~15m) để không bao giờ bị lạc waypoint khi xe né lệch ngang lớn (d >= 3.5m).
    """
    total_len = len(x_traj)
    if current_idx == 0:
        start_search = 0
        end_search = min(total_len, 400)
    else:
        start_search = max(0, current_idx - 30)
        end_search = min(total_len, current_idx + search_range)

    min_dist = float('inf')
    best_idx = current_idx

    for i in range(start_search, end_search):
        dist = (x_traj[i] - x0) ** 2 + (y_traj[i] - y0) ** 2
        if dist < min_dist:
            min_dist = dist
            best_idx = i

    return best_idx

def get_ref_trajectory(x_traj, y_traj, theta_traj, v_traj, current_idx, s_traj=None, x0=None, y0=None, horizon=N, sample_time=dt):
    """
    Trích xuất N waypoint dự báo phía trước bằng phương pháp Nội suy Cung đường Động
    (Dynamic Arc-length Parameterized Interpolation) theo thời gian thực (t_k = k * dt).

    - Đồng bộ chính xác 100% khoảng cách không gian (delta_s = v_ref * dt) với vận tốc di chuyển.
    - Độc lập hoàn toàn với mật độ và độ phân giải của file CSV quỹ đạo.
    - Triệt tiêu lỗi lượng tử hóa (quantization error), loại bỏ hiện tượng dao động hình sin
      và hiện tượng giật cục khi tăng/giảm tốc độ.
    - Hỗ trợ quỹ đạo khép kín (closed-loop) mượt mà không bị ngắt quãng tại vạch xuất phát/đích.
    """
    total_points = len(x_traj)
    if total_points == 0:
        return np.zeros(horizon), np.zeros(horizon), np.zeros(horizon), np.zeros(horizon)

    # 1. Khởi tạo mảng cung tích lũy s_traj nếu chưa truyền vào (tự động cache)
    if s_traj is None:
        if not hasattr(get_ref_trajectory, "_cached_s_traj") or getattr(get_ref_trajectory, "_cached_len", 0) != total_points:
            dx = np.diff(x_traj)
            dy = np.diff(y_traj)
            get_ref_trajectory._cached_s_traj = np.concatenate(([0.0], np.cumsum(np.hypot(dx, dy))))
            get_ref_trajectory._cached_len = total_points
        s_traj = get_ref_trajectory._cached_s_traj

    s_total = float(s_traj[-1])
    is_closed = math.hypot(x_traj[-1] - x_traj[0], y_traj[-1] - y_traj[0]) < 2.0

    # Cache mảng mở rộng nối đầu-đuôi cho quỹ đạo khép kín để nội suy mượt qua vạch đích
    if not hasattr(get_ref_trajectory, "_cached_ext") or getattr(get_ref_trajectory, "_cached_ext_len", 0) != total_points:
        if is_closed and total_points > 50:
            pad_n = min(100, total_points // 4)
            s_ext = np.concatenate([s_traj, s_total + s_traj[1:pad_n]])
            x_ext = np.concatenate([x_traj, x_traj[1:pad_n]])
            y_ext = np.concatenate([y_traj, y_traj[1:pad_n]])
            th_ext = np.concatenate([theta_traj, theta_traj[1:pad_n]])
            v_ext = np.concatenate([v_traj, v_traj[1:pad_n]])
        else:
            s_ext = s_traj
            x_ext = x_traj
            y_ext = y_traj
            th_ext = theta_traj
            v_ext = v_traj
        get_ref_trajectory._cached_ext = (s_ext, x_ext, y_ext, th_ext, v_ext)
        get_ref_trajectory._cached_ext_len = total_points

    s_ext, x_ext, y_ext, th_ext, v_ext = get_ref_trajectory._cached_ext

    # 2. Xác định điểm cung s0 liên tục xuất phát từ vị trí xe
    current_idx = min(max(0, current_idx), total_points - 1)
    s0 = float(s_traj[current_idx])
    if x0 is not None and y0 is not None:
        xr = x_traj[current_idx]
        yr = y_traj[current_idx]
        thr = theta_traj[current_idx]
        tx, ty = math.cos(thr), math.sin(thr)
        ds_tangent = tx * (x0 - xr) + ty * (y0 - yr)
        if abs(ds_tangent) < 3.0:
            s0 = max(0.0, s0 + ds_tangent)

    # 3. Tính toán chính xác tọa độ cung s_k cho từng bước thời gian trong chân trời N
    s_queries = []
    curr_s = s0
    s_limit = float(s_ext[-1])
    for k in range(horizon):
        s_queries.append(curr_s)
        vk = max(float(np.interp(curr_s, s_ext, v_ext)), 1.5)
        curr_s = min(curr_s + vk * sample_time, s_limit)

    s_queries = np.array(s_queries, dtype=float)

    # 4. Nội suy 1 chiều liên tục các biến trạng thái tham chiếu
    x_ref = np.interp(s_queries, s_ext, x_ext)
    y_ref = np.interp(s_queries, s_ext, y_ext)
    theta_ref = np.interp(s_queries, s_ext, th_ext)
    v_ref = np.interp(s_queries, s_ext, v_ext)

    return x_ref, y_ref, theta_ref, v_ref

def calculate_lateral_deviation(x, y, x_ref1, y_ref1, x_ref2, y_ref2):
    """
    Tính khoảng cách vuông góc từ xe (x, y) đến đoạn thẳng nối 2 waypoint tham chiếu.
    """
    num = (y_ref2 - y_ref1) * x - (x_ref2 - x_ref1) * y + x_ref2 * y_ref1 - y_ref2 * x_ref1
    denom = np.sqrt((y_ref2 - y_ref1) ** 2 + (x_ref2 - x_ref1) ** 2)
    if denom > 1e-4:
        return num / denom
    else:
        return 0.0
