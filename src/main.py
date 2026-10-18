import time
import os
import sys
import traceback
import numpy as np

# Thêm thư mục hiện tại vào sys.path để import mô-đun src dễ dàng
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.carla_simulator import CarlaSimulator
from src.config import (
    N, dt, USE_LINEARIZED_QP_MPC, ENABLE_OBSTACLES,
    ENABLE_CHASE_CAMERA_WINDOW, CAMERA_WINDOW_WIDTH, CAMERA_WINDOW_HEIGHT,
    ENABLE_SPECTATOR_FOLLOW
)
from src.help_functions import load_csv_trajectory, update_reference_point, get_ref_trajectory, normalize_angle
from src.logger import Logger
from src.mpc_controller import MpcController
from src.linearized_mpc_controller import LinearizedMpcController
from src.obstacle_detector import ObstacleDetector
from src.create_obstacles.obstacle_generator import ObstacleManager
from src.camera_viewer import CameraViewer

def main():
    import argparse
    parser = argparse.ArgumentParser(description="CARLA Autonomous MPC Path Tracking")
    parser.add_argument('--no-obstacles', action='store_true', help="Chạy chế độ thuần bám quỹ đạo (tắt sinh vật cản)")
    parser.add_argument('--with-obstacles', action='store_true', help="Bật sinh vật cản né (ghi đè config.py)")
    parser.add_argument('--no-camera-window', action='store_true', help="Tắt cửa sổ camera theo dõi xe")
    parser.add_argument('--camera-window', action='store_true', help="Bật cửa sổ camera theo dõi xe")
    args, unknown = parser.parse_known_args()

    enable_obstacles = ENABLE_OBSTACLES
    if args.no_obstacles:
        enable_obstacles = False
    elif args.with_obstacles:
        enable_obstacles = True

    print("==========================================================")
    print("   CARLA AUTONOMOUS MPC - PATH TRACKING & OBSTACLES       ")
    print("==========================================================")

    # 1. Tải quỹ đạo từ file CSV (đã áp dụng np.unwrap cho góc theta)
    x_traj, y_traj, theta_traj, v_ref_traj = load_csv_trajectory()
    total_waypoints = len(x_traj)

    # 2. Khởi tạo CARLA Simulator & Spawn Xe tại điểm xuất phát (đọc host từ config.py)
    carla = CarlaSimulator()
    
    x_init = x_traj[0]
    y_init = y_traj[0]
    yaw_init = theta_traj[0]

    carla.spawn_ego_vehicle('vehicle.tesla.model3', x=x_init, y=y_init, yaw_rad=yaw_init)
    carla.set_spectator(x=x_init, y=y_init, z=35.0, pitch=-75.0, yaw=0.0)

    # Khởi tạo Cửa sổ Camera theo dõi bám theo xe (Chase Camera Viewer)
    camera_viewer = None
    use_camera_window = ENABLE_CHASE_CAMERA_WINDOW
    if args.no_camera_window:
        use_camera_window = False
    elif args.camera_window:
        use_camera_window = True

    if use_camera_window:
        try:
            camera_viewer = CameraViewer(
                carla.world,
                carla.ego_vehicle,
                width=CAMERA_WINDOW_WIDTH,
                height=CAMERA_WINDOW_HEIGHT
            )
        except Exception as e:
            print(f"[CAMERA VIEWER] Cảnh báo: Không thể khởi tạo cửa sổ camera: {e}")
            camera_viewer = None

    # 3. Sinh các vật cản tĩnh trên môi trường (nếu được kích hoạt)
    obstacle_manager = None
    if enable_obstacles:
        print("[CHẾ ĐỘ] Kích hoạt sinh vật cản né (Obstacle Avoidance Mode).")
        obstacle_manager = ObstacleManager(carla_sim=carla)
        obstacle_manager.generate_obstacles(x_traj, y_traj, theta_traj)
    else:
        print("[CHẾ ĐỘ] Chạy THUẦN BÁM QUỸ ĐẠO (Pure Path Tracking - Không sinh vật cản).")

    # 4. Khởi tạo Cảm biến nhận thức ảo (Obstacle Detector) gắn trên xe Ego
    obstacle_detector = ObstacleDetector(carla.world, carla.ego_vehicle)

    # 5. Vẽ toàn bộ quỹ đạo 3D lên CARLA (điểm xanh lá + mũi tên đỏ)
    print("[CARLA] Đang hiển thị quỹ đạo 3D lên bản đồ...")
    carla.draw_full_trajectory_3d(x_traj, y_traj, theta_traj, life_time=300.0)

    # 6. Khởi tạo MPC Controller (Hỗ trợ Tuyến tính hóa lặp QP-MPC với Slack) & Logger
    if USE_LINEARIZED_QP_MPC:
        print("[THUẬT TOÁN] Sử dụng Bộ điều khiển Tuyến tính hóa lặp (Slack QP-MPC).")
        mpc_controller = LinearizedMpcController(horizon=N, sample_time=dt)
        mpc_controller.set_global_trajectory(x_traj, y_traj, theta_traj)
    else:
        print("[THUẬT TOÁN] Sử dụng Bộ điều khiển NMPC phi tuyến truyền thống.")
        mpc_controller = MpcController(horizon=N, dt=dt)

    logger = Logger()

    current_idx = 0
    print("\n--- BẮT ĐẦU VÒNG LẶP ĐIỀU KHIỂN MPC BÁM ĐƯỜNG & NÉ VẬT CẢN ---")

    try:
        while current_idx < total_waypoints - 1:
            start_time = time.time()

            # Lấy trạng thái hiện tại của xe từ CARLA
            x0, y0, theta0_raw, v0 = carla.get_main_ego_vehicle_state()

            # Quét cảm biến nhận thức vật cản theo thời gian thực (Perception)
            detected_obstacles = obstacle_detector.scan(draw_debug=True, life_time=dt * 2)

            # Cập nhật chỉ số waypoint gần nhất
            current_idx = update_reference_point(x0, y0, current_idx, x_traj, y_traj)

            # Trích xuất đoạn quỹ đạo tham chiếu N bước phía trước
            x_ref, y_ref, theta_ref, v_ref = get_ref_trajectory(x_traj, y_traj, theta_traj, v_ref_traj, current_idx)

            # Khắc phục lỗi nhảy góc +-pi: Chuẩn hóa theta0 trùng nhánh góc liên tục với theta_ref[0]
            theta0 = theta_ref[0] + normalize_angle(theta0_raw - theta_ref[0])

            # Ghi Log đầu vào
            logger.log_controller_input(x0, y0, v0, theta0, x_ref[0], y_ref[0], v_ref[0], theta_ref[0])

            # Giải bài toán tối ưu MPC (tùy theo loại bộ điều khiển)
            t_solve_start = time.perf_counter()
            if USE_LINEARIZED_QP_MPC:
                x_state = np.array([x0, y0, theta0, v0])
                wheel_angle_rad, acceleration_m_s_2 = mpc_controller.solve(
                    x_state, x_ref, y_ref, theta_ref, v_ref,
                    detected_obstacles=detected_obstacles
                )
                cost_val = mpc_controller.get_optimized_cost()

                # Vẽ đường tham chiếu N bước (hiển thị đường cong né lách thời gian thực trên CARLA)
                if hasattr(mpc_controller, 'x_ref_mod') and mpc_controller.x_ref_mod is not None:
                    carla.draw_horizon_path(mpc_controller.x_ref_mod, mpc_controller.y_ref_mod, life_time=dt * 2)
                else:
                    carla.draw_horizon_path(x_ref, y_ref, life_time=dt * 2)
            else:
                carla.draw_horizon_path(x_ref, y_ref, life_time=dt * 2)
                mpc_controller.reset_solver()
                mpc_controller.set_init_vehicle_state(x0, y0, theta0, v0)
                mpc_controller.update_cost_function(x_ref, y_ref, theta_ref, v_ref)
                mpc_controller.solve()
                wheel_angle_rad, acceleration_m_s_2 = mpc_controller.get_controls_value()
                cost_val = mpc_controller.get_optimized_cost()
            solve_time_sec = time.perf_counter() - t_solve_start

            # Chuyển đổi tín hiệu điều khiển góc lái và gia tốc
            throttle, brake, steer = CarlaSimulator.process_control_inputs(
                wheel_angle_rad, acceleration_m_s_2, current_speed=v0
            )

            logger.log_controller_output(steer, throttle, brake, cost=cost_val)
            carla.apply_control(steer, throttle, brake)

            # Cập nhật Spectator trong cửa sổ CARLA bay theo sau xe mượt mà
            if ENABLE_SPECTATOR_FOLLOW:
                carla.follow_vehicle(carla.ego_vehicle)

            # Cập nhật khung hình và hiển thị thông số HUD lên cửa sổ Camera riêng
            if camera_viewer is not None:
                side_tag = ""
                if hasattr(mpc_controller, 'active_obstacle') and mpc_controller.active_obstacle is not None:
                    side_tag = "TRÁI" if mpc_controller.active_obstacle['side_sign'] < 0 else "PHẢI"
                    bypass_txt = f"NÉ {side_tag} (#{mpc_controller.active_obstacle['id']})"
                else:
                    bypass_txt = "BÁM ĐƯỜNG"

                hud_info = {
                    'speed_kmh': v0 * 3.6,
                    'target_kmh': v_ref[0] * 3.6,
                    'steer_deg': float(np.rad2deg(wheel_angle_rad)),
                    'bypass_text': bypass_txt,
                    'obs_count': len(detected_obstacles),
                    'current_idx': current_idx,
                    'total_idx': total_waypoints
                }
                is_viewer_alive = camera_viewer.render(hud_info)
                if not is_viewer_alive:
                    print("\n[CAMERA VIEWER] Người dùng đã đóng cửa sổ camera.")
                    camera_viewer.destroy()
                    camera_viewer = None

            # Đồng bộ thời gian thực bước sample dt
            calc_time = time.time() - start_time
            sleep_time = max(dt - calc_time, 0.0)

            # Định dạng chuỗi trạng thái cảm biến và điều khiển né
            bypass_info = ""
            if hasattr(mpc_controller, 'active_obstacle') and mpc_controller.active_obstacle is not None:
                side_tag = "TRÁI" if mpc_controller.active_obstacle['side_sign'] < 0 else "PHẢI"
                bypass_info = f" [NÉ {side_tag}]"

            if detected_obstacles:
                nearest = detected_obstacles[0]
                src_tag = nearest.get('source', 'obs')[:3].upper()
                obs_status = f" | [!] OBS: {len(detected_obstacles)} [{src_tag}] (D:{nearest['dist']:.1f}m, MũiXe:{nearest['d_lat']:+.1f}m){bypass_info}"
            else:
                obs_status = f" | OBS: 0{bypass_info}"

            print(f"Index: {current_idx}/{total_waypoints} | Pos: ({x0:.1f}, {y0:.1f}) | "
                  f"Speed: {v0:.2f}/{v_ref[0]:.2f} m/s | Steer: {steer:.2f} | Solve: {solve_time_sec:.3f}s{obs_status}", end="\r")

            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[THÔNG BÁO] Người dùng ngắt chương trình bằng Ctrl+C.")
    except Exception as e:
        print(f"\n[LỖI THỰC THI] Xảy ra ngoại lệ: {e}")
        traceback.print_exc()
    finally:
        print("\n[HOÀN THÀNH] Kết thúc mô phỏng.")
        if camera_viewer is not None:
            camera_viewer.destroy()
        if obstacle_manager is not None:
            obstacle_manager.destroy_all()
        carla.clean()
        logger.show_plots()

if __name__ == '__main__':
    main()
