import os
import sys
import csv

# Thêm đường dẫn src vào sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.trajectory_generator.waypoint_reader import get_2d_map_and_ping_waypoints
from src.trajectory_generator.spline_gen import generate_trajectory
from src.carla_simulator import CarlaSimulator
from src.config import TRAJECTORY_CSV_PATH

def save_waypoints_to_csv(waypoints, filename):
    """Ghi danh sách waypoint ra file CSV"""
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'theta_rad', 'v_m_s'])
        for wp in waypoints:
            writer.writerow([round(wp[0], 4), round(wp[1], 4), round(wp[2], 4), round(wp[3], 4)])
    print(f"\n[THÀNH CÔNG] Đã lưu {len(waypoints)} waypoints vào file: {filename}")

def main():
    print("==========================================================")
    print("      TẠO QUỸ ĐẠO THAM CHIẾU CHO CARLA AUTONOMOUS MPC     ")
    print("==========================================================")

    # 1. Chọn điểm neo từ GUI Matplotlib
    x_anchors, y_anchors = get_2d_map_and_ping_waypoints()

    if not x_anchors or len(x_anchors) < 2:
        print("Bạn chưa chọn đủ điểm. Thao tác bị hủy!")
        return

    # 2. Nội suy quỹ đạo PCHIP
    print("\n--- TÍNH TOÁN QUỸ ĐẠO PCHIP ---")
    trajectory = generate_trajectory(x_anchors, y_anchors, ds=0.1, v_max=2.0, a_lat_max=1.5)

    # 3. Ghi dữ liệu ra CSV trong thư mục data/
    save_waypoints_to_csv(trajectory, TRAJECTORY_CSV_PATH)

    # 4. Hiển thị 3D trên bản đồ CARLA
    print("\n--- HIỂN THỊ 3D TRÊN CARLA MAP ---")
    try:
        carla = CarlaSimulator()
        x_traj = [wp[0] for wp in trajectory]
        y_traj = [wp[1] for wp in trajectory]
        theta_traj = [wp[2] for wp in trajectory]
        carla.draw_full_trajectory_3d(x_traj, y_traj, theta_traj, life_time=180.0)
        print("[HOÀN THÀNH] Đã vẽ xong quỹ đạo lên CARLA 3D!")
    except Exception as e:
        print(f"[CẢNH BÁO] Không thể vẽ 3D trên CARLA (Lỗi: {e}). File CSV đã được lưu thành công.")

if __name__ == '__main__':
    main()
