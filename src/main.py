import time
import os
import sys
import traceback
import numpy as np

# Thêm thư mục hiện tại vào sys.path để import mô-đun src dễ dàng
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.carla_simulator import CarlaSimulator
from src.config import N, dt, USE_LINEARIZED_QP_MPC
from src.help_functions import load_csv_trajectory, update_reference_point, get_ref_trajectory, normalize_angle
from src.logger import Logger
from src.mpc_controller import MpcController
from src.linearized_mpc_controller import LinearizedMpcController
from src.obstacle_detector import ObstacleDetector
from src.create_obstacles.obstacle_generator import ObstacleManager

def main():
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

    # 3. Sinh các vật cản tĩnh trên môi trường (từ 1/3 quỹ đạo trở đi)
    obstacle_manager = ObstacleManager(carla_sim=carla)
    obstacle_manager.generate_obstacles(x_traj, y_traj, theta_traj)

    # 4. Khởi tạo Cảm biến nhận thức ảo (Obstacle Detector) gắn trên xe Ego
    obstacle_detector = ObstacleDetector(carla.world, carla.ego_vehicle)

    # 5. Vẽ toàn bộ quỹ đạo 3D lên CARLA (điểm xanh lá + mũi tên đỏ)
    print("[CARLA] Đang hiển thị quỹ đạo 3D lên bản đồ...")
    carla.draw_full_trajectory_3d(x_traj, y_traj, theta_traj, life_time=300.0)

    # 6. Khởi tạo MPC Controller (Hỗ trợ Tuyến tính hóa lặp QP-MPC với Slack) & Logger
    if USE_LINEARIZED_QP_MPC:
        print("[THUẬT TOÁN] Sử dụng Bộ điều khiển Tuyến tính hóa lặp (Slack QP-MPC).")
        mpc_controller = LinearizedMpcController(horizon=N, sample_time=dt)
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

            # Chuyển đổi tín hiệu điều khiển góc lái và gia tốc
            throttle, brake, steer = CarlaSimulator.process_control_inputs(
                wheel_angle_rad, acceleration_m_s_2, current_speed=v0
            )

            logger.log_controller_output(steer, throttle, brake, cost=cost_val)
            carla.apply_control(steer, throttle, brake)

            # Đồng bộ thời gian thực bước sample dt
            calc_time = time.time() - start_time
            sleep_time = max(dt - calc_time, 0.0)

            # Định dạng chuỗi trạng thái cảm biến
            if detected_obstacles:
                nearest = detected_obstacles[0]
                src_tag = nearest.get('source', 'obs')[:3].upper()
                obs_status = f" | [!] OBS: {len(detected_obstacles)} [{src_tag}] (D:{nearest['dist']:.1f}m, Lat:{nearest['d_lat']:+.1f}m)"
            else:
                obs_status = " | OBS: 0"

            print(f"Index: {current_idx}/{total_waypoints} | Pos: ({x0:.1f}, {y0:.1f}) | "
                  f"Speed: {v0:.2f}/{v_ref[0]:.2f} m/s | Steer: {steer:.2f}{obs_status}", end="\r")

            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[THÔNG BÁO] Người dùng ngắt chương trình bằng Ctrl+C.")
    except Exception as e:
        print(f"\n[LỖI THỰC THI] Xảy ra ngoại lệ: {e}")
        traceback.print_exc()
    finally:
        print("\n[HOÀN THÀNH] Kết thúc mô phỏng.")
        obstacle_manager.destroy_all()
        carla.clean()
        logger.show_plots()

if __name__ == '__main__':
    main()
