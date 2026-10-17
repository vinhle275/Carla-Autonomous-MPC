import os
import sys
import csv
import math

# Thêm đường dẫn src vào sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.trajectory_generator.waypoint_reader import get_path_anchors, pick_obstacles_on_trajectory
from src.trajectory_generator.spline_gen import generate_trajectory
from src.carla_simulator import CarlaSimulator
from src.config import TRAJECTORY_CSV_PATH, OBSTACLES_CSV_PATH, TARGET_SPEED_MS, OBSTACLE_SIZE

def save_waypoints_to_csv(waypoints, filename):
    """Ghi danh sách waypoint quỹ đạo ra file CSV"""
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'theta_rad', 'v_m_s'])
        for wp in waypoints:
            writer.writerow([round(wp[0], 4), round(wp[1], 4), round(wp[2], 4), round(wp[3], 4)])
    print(f"\n[THÀNH CÔNG] Đã lưu {len(waypoints)} waypoints vào file: {filename}")

def save_obstacles_to_csv(obstacles, filename):
    """Ghi danh sách vật cản cố định (x, y, size) ra file CSV"""
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'size'])
        for obs in obstacles:
            writer.writerow([round(obs['x'], 4), round(obs['y'], 4), OBSTACLE_SIZE])
    print(f"[THÀNH CÔNG] Đã lưu {len(obstacles)} vật cản cố định vào file: {filename}")

def main():
    print("==========================================================")
    print("   TẠO ĐƯỜNG ĐI & ĐẶT VẬT CẢN CỐ ĐỊNH CHO CARLA MPC       ")
    print("==========================================================")

    # 1. BƯỚC 1: Chọn điểm neo đường đi (Xanh lá) từ Cửa sổ Matplotlib thứ nhất
    x_anchors, y_anchors = get_path_anchors()

    if not x_anchors or len(x_anchors) < 2:
        print("[HỦY] Bạn chưa chọn đủ ít nhất 2 điểm. Thao tác bị hủy!")
        return

    # 2. Nội suy quỹ đạo PCHIP mượt mà với vận tốc mục tiêu TARGET_SPEED_MS
    print(f"\n--- ĐANG NỘI SUY QUỸ ĐẠO PCHIP (V_MAX = {TARGET_SPEED_MS:.1f} m/s) ---")
    trajectory = generate_trajectory(x_anchors, y_anchors, ds=0.1, v_max=TARGET_SPEED_MS, a_lat_max=3.0)

    x_traj = [wp[0] for wp in trajectory]
    y_traj = [wp[1] for wp in trajectory]
    theta_traj = [wp[2] for wp in trajectory]

    # 3. BƯỚC 2: Mở CỬA SỔ MATPLOTLIB MỚI với quỹ đạo đã chọn để click đặt vật cản
    obstacles = pick_obstacles_on_trajectory(x_traj, y_traj, theta_traj)

    # 4. Ghi dữ liệu ra CSV trong thư mục data/
    save_waypoints_to_csv(trajectory, TRAJECTORY_CSV_PATH)
    save_obstacles_to_csv(obstacles, OBSTACLES_CSV_PATH)

    # 5. Hiển thị 3D trên bản đồ CARLA
    print("\n--- HIỂN THỊ ĐƯỜNG ĐI VÀ VẬT CẢN TRÊN CARLA 3D ---")
    try:
        sim = CarlaSimulator()
        sim.clear_debug_drawings()

        yaw_deg = math.degrees(theta_traj[0])
        sim.set_spectator(x=x_traj[0], y=y_traj[0], z=35.0, pitch=-75.0, yaw=yaw_deg)

        # Vẽ đường đi màu xanh lá + mũi tên đỏ chỉ hướng
        sim.draw_full_trajectory_3d(x_traj, y_traj, theta_traj, life_time=300.0)

        # Vẽ các khối hộp vật cản 3D màu đỏ
        for obs in obstacles:
            sim.draw_obstacle_box_3d(obs['x'], obs['y'], size=OBSTACLE_SIZE, life_time=300.0)

        print(f"[HOÀN THÀNH] Đã vẽ đường đi ({len(trajectory)} waypoints) và {len(obstacles)} vật cản lên CARLA 3D!")
    except Exception as e:
        print(f"[CẢNH BÁO] Không thể vẽ 3D trên CARLA (Lỗi: {e}). File CSV đã được lưu thành công.")

if __name__ == '__main__':
    main()
