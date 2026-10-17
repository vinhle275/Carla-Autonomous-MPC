import os
import csv
import numpy as np
import math
from src.config import N, dt, TRAJECTORY_CSV_PATH, OBSTACLES_CSV_PATH, TARGET_SPEED_MS, OVERRIDE_V_REF

def normalize_angle(angle):
    """Chuẩn hóa góc về dải [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))

def load_csv_obstacles(filepath=None):
    """
    Đọc file CSV chứa danh sách vật cản (x, y, size).
    """
    if filepath is None:
        filepath = OBSTACLES_CSV_PATH

    obstacles = []
    if not os.path.exists(filepath):
        return obstacles

    with open(filepath, mode='r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            obstacles.append({
                'x': float(row['x']),
                'y': float(row['y']),
                'size': float(row.get('size', 0.5))
            })
    return obstacles

def cluster_obstacles(obstacles, threshold=2.5):
    """
    Gom cụm các vật cản gần nhau (khoảng cách <= threshold) thành một khối vật cản đại diện.
    Trả về danh sách các vật cản đã được gộp với tâm (x, y) và kích thước bao ngoài.
    """
    if not obstacles or len(obstacles) <= 1:
        return obstacles

    clusters = []
    visited = [False] * len(obstacles)

    for i in range(len(obstacles)):
        if visited[i]:
            continue

        current_cluster = [obstacles[i]]
        visited[i] = True
        queue = [obstacles[i]]

        while queue:
            cur = queue.pop(0)
            for j in range(len(obstacles)):
                if not visited[j]:
                    dist = np.sqrt((cur['x'] - obstacles[j]['x'])**2 + (cur['y'] - obstacles[j]['y'])**2)
                    if dist <= threshold:
                        visited[j] = True
                        current_cluster.append(obstacles[j])
                        queue.append(obstacles[j])

        xs = [o['x'] for o in current_cluster]
        ys = [o['y'] for o in current_cluster]
        center_x = float(np.mean(xs))
        center_y = float(np.mean(ys))

        # Bán kính bao trọn tất cả các vật thể trong cụm
        max_dist_from_center = max(
            np.sqrt((o['x'] - center_x)**2 + (o['y'] - center_y)**2) + (o.get('size', 0.5) / 2.0)
            for o in current_cluster
        )
        cluster_size = float(max_dist_from_center * 2.0)

        # Hỗ trợ cả vật cản di chuyển nếu có vx, vy
        avg_vx = float(np.mean([o.get('vx', 0.0) for o in current_cluster]))
        avg_vy = float(np.mean([o.get('vy', 0.0) for o in current_cluster]))

        clusters.append({
            'x': center_x,
            'y': center_y,
            'size': cluster_size,
            'vx': avg_vx,
            'vy': avg_vy,
            'count': len(current_cluster)
        })

    return clusters

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

    print(f"[LOAD TRAJECTORY] Đã tải thành công {len(x_traj)} waypoints từ: {filepath}")
    return x_traj, y_traj, theta_traj, v_ref

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
    Trích xuất N waypoint dự báo phía trước tính từ current_idx dựa trên khoảng cách di chuyển không gian thực tế (v * dt).
    Khớp tầm nhìn MPC với tốc độ di chuyển để triệt tiêu chao đảo lắc trái/phải.
    """
    total_points = len(x_traj)
    x_ref, y_ref, theta_ref, v_ref = [], [], [], []

    # Tốc độ tham chiếu hiện tại tại vị trí xe
    current_v = max(float(v_traj[current_idx]), 2.0)
    
    # Khoảng cách xe đi được trong 1 bước dt = 0.1s
    step_dist = current_v * dt
    # Mỗi waypoint trong CSV cách nhau khoảng DS = 0.1m
    idx_step = max(1, int(round(step_dist / 0.1)))

    for k in range(N):
        idx = min(current_idx + k * idx_step, total_points - 1)
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
