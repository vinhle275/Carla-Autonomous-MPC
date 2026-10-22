import os
import sys
import csv
import math

# Thêm đường dẫn src vào sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.trajectory_generator.waypoint_reader import (
    get_path_anchors,
    pick_obstacles_on_trajectory,
    pick_moving_obstacles_on_trajectory,
)
from src.trajectory_generator.spline_gen import generate_trajectory
from src.carla_simulator import CarlaSimulator
from src.config import (
    TRAJECTORY_CSV_PATH,
    OBSTACLES_CSV_PATH,
    MOVING_OBSTACLES_CSV_PATH,
    TARGET_SPEED_MS,
    OBSTACLE_SIZE,
)


def save_waypoints_to_csv(waypoints, filename):
    """Ghi danh sach waypoint quy dao ra file CSV"""
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'theta_rad', 'v_m_s'])
        for wp in waypoints:
            writer.writerow([round(wp[0], 4), round(wp[1], 4),
                             round(wp[2], 4), round(wp[3], 4)])
    print(f"\n[THANH CONG] Da luu {len(waypoints)} waypoints vao file: {filename}")


def save_obstacles_to_csv(obstacles, filename):
    """Ghi danh sach vat can co dinh (x, y, size) ra file CSV"""
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'size'])
        for obs in obstacles:
            writer.writerow([round(obs['x'], 4), round(obs['y'], 4), OBSTACLE_SIZE])
    print(f"[THANH CONG] Da luu {len(obstacles)} vat can co dinh vao file: {filename}")


def save_moving_obstacles_to_csv(moving_obstacles, filename):
    """
    Ghi danh sach vat can di dong ra file CSV.
    Format: obstacle_id, wp_index, x, y, speed_ms
    Moi vat can co nhieu dong (moi waypoint 1 dong).
    """
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['obstacle_id', 'wp_index', 'x', 'y', 'speed_ms'])
        for obs in moving_obstacles:
            obs_id = obs['id']
            speed = obs['speed_ms']
            for wp_idx, wp in enumerate(obs['waypoints']):
                writer.writerow([obs_id, wp_idx,
                                 round(wp['x'], 4), round(wp['y'], 4),
                                 round(speed, 4)])
    total_wps = sum(len(o['waypoints']) for o in moving_obstacles)
    print(f"[THANH CONG] Da luu {len(moving_obstacles)} vat can di dong "
          f"({total_wps} waypoints) vao file: {filename}")


def main():
    print("==========================================================")
    print("   TAO DUONG DI, VAT CAN CO DINH & VAT CAN DI DONG       ")
    print("==========================================================")

    # ---------------------------------------------------------------
    # BUOC 1: Chon diem neo duong di (Xanh la)
    # ---------------------------------------------------------------
    x_anchors, y_anchors = get_path_anchors()

    if not x_anchors or len(x_anchors) < 2:
        print("[HUY] Ban chua chon du it nhat 2 diem. Thao tac bi huy!")
        return

    # Noi suy quy dao PCHIP muot ma
    print(f"\n--- DANG NOI SUY QUY DAO PCHIP (V_MAX = {TARGET_SPEED_MS:.1f} m/s) ---")
    trajectory = generate_trajectory(
        x_anchors, y_anchors, ds=0.1, v_max=TARGET_SPEED_MS, a_lat_max=3.0
    )

    x_traj = [wp[0] for wp in trajectory]
    y_traj = [wp[1] for wp in trajectory]
    theta_traj = [wp[2] for wp in trajectory]

    # ---------------------------------------------------------------
    # BUOC 2: Chon vat can co dinh (mau do, hinh vuong)
    # ---------------------------------------------------------------
    obstacles = pick_obstacles_on_trajectory(x_traj, y_traj, theta_traj)

    # ---------------------------------------------------------------
    # BUOC 3: Ve duong di vat can di dong (mau cam)
    # ---------------------------------------------------------------
    moving_obstacles = pick_moving_obstacles_on_trajectory(
        x_traj, y_traj,
        static_obstacles=obstacles   # hien thi vat can co dinh lam nen tham chieu
    )

    # ---------------------------------------------------------------
    # Ghi du lieu ra CSV
    # ---------------------------------------------------------------
    save_waypoints_to_csv(trajectory, TRAJECTORY_CSV_PATH)
    save_obstacles_to_csv(obstacles, OBSTACLES_CSV_PATH)
    if moving_obstacles:
        save_moving_obstacles_to_csv(moving_obstacles, MOVING_OBSTACLES_CSV_PATH)
    else:
        print("[THONG BAO] Khong co vat can di dong nao duoc chon. Bo qua luu file.")

    # ---------------------------------------------------------------
    # Hien thi 3D tren ban do CARLA
    # ---------------------------------------------------------------
    print("\n--- HIEN THI DUONG DI VA VAT CAN TREN CARLA 3D ---")
    try:
        sim = CarlaSimulator()
        sim.clear_debug_drawings()

        yaw_deg = math.degrees(theta_traj[0])
        sim.set_spectator(x=x_traj[0], y=y_traj[0], z=35.0, pitch=-75.0, yaw=yaw_deg)

        # Ve duong di mau xanh la + mui ten do chi huong
        sim.draw_full_trajectory_3d(x_traj, y_traj, theta_traj, life_time=300.0)

        # Ve cac khoi hop vat can co dinh 3D mau do
        for obs in obstacles:
            sim.draw_obstacle_box_3d(obs['x'], obs['y'],
                                     size=OBSTACLE_SIZE, life_time=300.0)

        # Ve duong di cua vat can di dong (mau vang cam)
        for mov in moving_obstacles:
            wps = mov['waypoints']
            mov_id = mov['id']
            for i in range(len(wps) - 1):
                x1, y1 = wps[i]['x'], wps[i]['y']
                x2, y2 = wps[i + 1]['x'], wps[i + 1]['y']
                sim.draw_moving_obstacle_path(x1, y1, x2, y2, life_time=300.0)
            # Danh dau diem xuat phat
            sim.draw_moving_obstacle_start(
                wps[0]['x'], wps[0]['y'],
                label=f"MOV#{mov_id}", life_time=300.0
            )

        print(
            f"[HOAN THANH] Da ve duong di ({len(trajectory)} waypoints), "
            f"{len(obstacles)} vat can co dinh va "
            f"{len(moving_obstacles)} vat can di dong len CARLA 3D!"
        )
    except Exception as e:
        print(f"[CANH BAO] Khong the ve 3D tren CARLA (Loi: {e}). File CSV da duoc luu thanh cong.")


if __name__ == '__main__':
    main()
