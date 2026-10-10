import os
import csv
import numpy as np
from src.config import N, TRAJECTORY_CSV_PATH

def load_csv_trajectory(filepath=None):
    """
    Đọc file CSV chứa quỹ đạo tham chiếu (x, y, theta_rad, v_m_s).
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

    print(f"[LOAD TRAJECTORY] Đã tải thành công {len(x_traj)} waypoints từ: {filepath}")
    return np.array(x_traj), np.array(y_traj), np.array(theta_traj), np.array(v_ref)

def update_reference_point(x0, y0, current_idx, x_traj, y_traj, search_range=50):
    """
    Tìm chỉ số Waypoint gần nhất trên đường đi xung quanh vị trí hiện tại của xe.
    """
    total_len = len(x_traj)
    if current_idx == 0:
        start_search = 0
        end_search = min(total_len, 300)
    else:
        start_search = max(0, current_idx - 10)
        end_search = min(total_len, current_idx + search_range)

    min_dist = float('inf')
    best_idx = current_idx

    for i in range(start_search, end_search):
        dist = (x_traj[i] - x0) ** 2 + (y_traj[i] - y0) ** 2
        if dist < min_dist:
            min_dist = dist
            best_idx = i

    return best_idx

def get_ref_trajectory(x_traj, y_traj, theta_traj, v_traj, current_idx):
    """
    Trích xuất N waypoint dự báo phía trước tính từ current_idx cho bộ điều khiển MPC.
    """
    total_points = len(x_traj)
    x_ref, y_ref, theta_ref, v_ref = [], [], [], []

    for i in range(N):
        idx = min(current_idx + i, total_points - 1)
        x_ref.append(x_traj[idx])
        y_ref.append(y_traj[idx])
        theta_ref.append(theta_traj[idx])
        v_ref.append(v_traj[idx])

    return np.array(x_ref), np.array(y_ref), np.array(theta_ref), np.array(v_ref)

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
