import time
import os
import sys
import traceback
import numpy as np

# ==============================================================================
# PHẢI XỬ LÝ TRƯỚC KHI IMPORT BẤT KỲ MODULE NÀO KHÁC (đặc biệt là cv2/pygame)
# ==============================================================================
# Lỗi "Aborted (core dumped)" trên WSL/Linux xảy ra vì opencv-python (cv2)
# tự động đặt QT_QPA_PLATFORM_PLUGIN_PATH trỏ vào thư mục Qt plugin của chính nó.
# Khi process kết thúc, Qt cố khởi tạo backend xcb nhưng thất bại → Abort.
# Fix: xóa biến môi trường trước khi cv2 có cơ hội inject, và bắt buộc dùng
# backend offscreen để Qt không thử mở cửa sổ thực.
for _env_var in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_PLUGIN_PATH"):
    os.environ.pop(_env_var, None)
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# Bắt buộc matplotlib dùng backend Agg (không cần display) — PHẢI ĐẶT TRƯỚC import matplotlib
import matplotlib
matplotlib.use("Agg")

# Thêm thư mục hiện tại vào sys.path để import mô-đun src dễ dàng
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.carla_simulator import CarlaSimulator
from src.config import (
    N, dt, USE_LINEARIZED_QP_MPC, ENABLE_OBSTACLES,
    ENABLE_CHASE_CAMERA_WINDOW, CAMERA_WINDOW_WIDTH, CAMERA_WINDOW_HEIGHT,
    ENABLE_SPECTATOR_FOLLOW,
    ENABLE_MOVING_OBSTACLES, MOVING_OBSTACLES_CSV_PATH,
    MAX_CONTROL_BRAKING_M_S_2, MAX_WHEEL_ANGLE_RAD,
    ENABLE_YELLOW_LINE_DETECTOR, DEBUG_YELLOW_DRAW,
    CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT, CAM_FOV_DEG,
    ENABLE_DYNAMIC_ROAD_CORRIDOR, DEBUG_DRAW_ROAD_CORRIDOR
)
from src.help_functions import load_csv_trajectory, update_reference_point, get_ref_trajectory, normalize_angle
from src.logger import Logger
from src.mpc_controller import MpcController
from src.linearized_mpc_controller import LinearizedMpcController
from src.obstacle_detector import ObstacleDetector
from src.create_obstacles.obstacle_generator import ObstacleManager, MovingObstacleManager
from src.camera_viewer import CameraViewer
from src.emergency_safety import EmergencySafetyManager, VehicleMode
from src.yellow_line_detector import RealtimeYellowLineDetector, LineType
from src.road_corridor import RoadCorridorTracker


def main():
    import argparse
    parser = argparse.ArgumentParser(description="CARLA Autonomous MPC Path Tracking")
    parser.add_argument('--no-obstacles', action='store_true', help="Chạy chế độ thuần bám quỹ đạo (tắt sinh vật cản)")
    parser.add_argument('--with-obstacles', action='store_true', help="Bật sinh vật cản né (ghi đè config.py)")
    parser.add_argument('--no-camera-window', action='store_true', help="Tắt cửa sổ camera theo dõi xe")
    parser.add_argument('--camera-window', action='store_true', help="Bật cửa sổ camera theo dõi xe")
    parser.add_argument('--max-steps', type=int, default=None, help="Số bước mô phỏng tối đa")
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
        print("[CHE DO] Kich hoat sinh vat can ne (Obstacle Avoidance Mode).")
        obstacle_manager = ObstacleManager(carla_sim=carla)
        obstacle_manager.generate_obstacles(x_traj, y_traj, theta_traj)
    else:
        print("[CHE DO] Chay THUAN BAM QUY DAO (Pure Path Tracking - Khong sinh vat can tinh).")

    # 3b. Spawn vat can di dong tu file CSV (neu co va ENABLE_MOVING_OBSTACLES = True)
    moving_obstacle_manager = None
    if ENABLE_MOVING_OBSTACLES:
        if os.path.exists(MOVING_OBSTACLES_CSV_PATH):
            moving_obstacle_manager = MovingObstacleManager(carla_sim=carla, dt=dt)
            n_mov = moving_obstacle_manager.spawn_from_csv(MOVING_OBSTACLES_CSV_PATH)
            if n_mov > 0:
                print(f"[MOVING OBS] Da spawn {n_mov} vat can di dong. Cap nhat vi tri moi buoc MPC.")
            else:
                print("[MOVING OBS] Khong co vat can di dong nao duoc spawn (file trong hoac loi).")
                moving_obstacle_manager = None
        else:
            print("[MOVING OBS] Khong tim thay file moving_obstacles.csv. Bo qua.")

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

    # Khởi tạo Bộ Nhận diện Hành lang mặt đường động (Dynamic Road Corridor) từ CARLA OpenDRIVE Map
    corridor_tracker = None
    if ENABLE_DYNAMIC_ROAD_CORRIDOR and hasattr(carla, 'carla_map') and carla.carla_map is not None:
        try:
            corridor_tracker = RoadCorridorTracker(carla_map=carla.carla_map)
            corridor_tracker.build_trajectory_corridor(x_traj, y_traj, theta_traj)
            if USE_LINEARIZED_QP_MPC and hasattr(mpc_controller, 'set_corridor_tracker'):
                mpc_controller.set_corridor_tracker(corridor_tracker)
        except Exception as _corr_err:
            print(f"[ROAD CORRIDOR] Lỗi khởi tạo hành lang động: {_corr_err}")

    # 7. Khởi tạo Bộ Quản Lý An Toàn & 2 Chế Độ Điều Khiển
    safety_manager = EmergencySafetyManager()

    # --- Khởi tạo Bộ Nhận Diện Vạch Vàng (Yellow Lane Line Detector) ---
    yellow_detector = None
    if ENABLE_YELLOW_LINE_DETECTOR:
        yellow_detector = RealtimeYellowLineDetector(
            debug_draw=DEBUG_YELLOW_DRAW
        )
        print("[YELLOW LINE] Đã khởi tạo RealtimeYellowLineDetector (HSV+IPM+Sliding Window + CARLA Map Ground Truth).")

    # Tính mảng chiều dài cung tích lũy s_traj (m) dọc theo quỹ đạo để hiển thị tiến độ trên Cockpit Dashboard
    dx_traj = np.diff(x_traj)
    dy_traj = np.diff(y_traj)
    s_traj = np.concatenate(([0.0], np.cumsum(np.hypot(dx_traj, dy_traj))))
    s_total_m = float(s_traj[-1]) if len(s_traj) > 0 else 0.0

    # Biến trạng thái tính gia tốc tức thời a_k và độ giật Jerk j_k (lọc thông thấp EMA alpha = 0.35)
    prev_v0 = None
    prev_accel = 0.0
    jerk_filtered = 0.0
    jerk_ema_alpha = 0.35

    current_idx = 0
    step_count = 0
    print("\n--- BẮT ĐẦU VÒNG LẶP ĐIỀU KHIỂN MPC BÁM ĐƯỜNG & NÉ VẬT CẢN ---")

    try:
        while current_idx < total_waypoints - 1:
            if args.max_steps is not None and step_count >= args.max_steps:
                print(f"[TEST COMPLETED] Đã đạt giới hạn max-steps={args.max_steps}.")
                break
            step_count += 1
            start_time = time.time()

            # Cap nhat vi tri vat can di dong (truoc khi scan cam bien)
            if moving_obstacle_manager is not None:
                moving_obstacle_manager.update(dt=dt)

            # Lấy trạng thái hiện tại của xe từ CARLA
            x0, y0, theta0_raw, v0 = carla.get_main_ego_vehicle_state()

            # Tính toán gia tốc tức thời a_ego (m/s^2) và độ giật dọc Jerk j_ego (m/s^3) có lọc thông thấp EMA
            if prev_v0 is None:
                accel_ego = 0.0
                jerk_raw = 0.0
            else:
                accel_ego = (v0 - prev_v0) / max(dt, 1e-3)
                jerk_raw = (accel_ego - prev_accel) / max(dt, 1e-3)
            jerk_filtered = jerk_ema_alpha * jerk_raw + (1.0 - jerk_ema_alpha) * jerk_filtered
            prev_v0 = v0
            prev_accel = accel_ego

            # Quét cảm biến nhận thức vật cản theo thời gian thực (Perception)
            detected_obstacles = obstacle_detector.scan(draw_debug=True, life_time=dt * 2)

            # Cập nhật chỉ số waypoint gần nhất
            current_idx = update_reference_point(x0, y0, current_idx, x_traj, y_traj)

            # Trích xuất đoạn quỹ đạo tham chiếu N bước phía trước bằng nội suy cung đường theo vận tốc thực tế
            x_ref, y_ref, theta_ref, v_ref = get_ref_trajectory(
                x_traj, y_traj, theta_traj, v_ref_traj, current_idx, s_traj=s_traj, x0=x0, y0=y0
            )

            # Khắc phục lỗi nhảy góc +-pi: Chuẩn hóa theta0 trùng nhánh góc liên tục với theta_ref[0]
            theta0 = theta_ref[0] + normalize_angle(theta0_raw - theta_ref[0])

            # --- NHẬN DIỆN VẠCH VÀNG (Yellow Line Perception - lấy trực tiếp từ Front Hood-Cam) ---
            yellow_result = None
            if yellow_detector is not None:
                rgb_frame = None
                if camera_viewer is not None:
                    if hasattr(camera_viewer, 'get_front_frame'):
                        rgb_frame = camera_viewer.get_front_frame()
                    elif camera_viewer.surface is not None:
                        try:
                            import pygame
                            raw_surf = camera_viewer.surface
                            rgb_array = pygame.surfarray.array3d(raw_surf)
                            rgb_frame = np.transpose(rgb_array, (1, 0, 2))
                        except Exception:
                            pass

                try:
                    yellow_result = yellow_detector.detect(
                        rgb_frame,
                        ego_vehicle=carla.ego_vehicle,
                        carla_map=carla.carla_map,
                        ref_xy=(float(x_ref[0]), float(y_ref[0])),
                        ref_theta=float(theta_ref[0]),
                    )
                    # Cập nhật ràng buộc vạch vàng vào MPC controller
                    if USE_LINEARIZED_QP_MPC and hasattr(mpc_controller, 'update_yellow_line_constraint'):
                        mpc_controller.update_yellow_line_constraint(yellow_result)
                except Exception as _yl_err:
                    pass

            # Ghi Log đầu vào
            logger.log_controller_input(x0, y0, v0, theta0, x_ref[0], y_ref[0], v_ref[0], theta_ref[0])

            # Cập nhật trạng thái an toàn & kiểm tra 2 chế độ điều khiển
            moving_ids = moving_obstacle_manager.get_moving_actor_ids() if moving_obstacle_manager is not None else None
            current_mode, is_emergency, danger_obs, status_info = safety_manager.update(
                x0, y0, theta0_raw, v0, detected_obstacles, dt=dt, moving_actor_ids=moving_ids
            )

            # Vẽ trực quan hóa hộp 3D vùng nguy hiểm va chạm trước đầu xe trên CARLA
            carla.draw_emergency_danger_zone(x0, y0, theta0_raw, z_ego=0.5, is_triggered=is_emergency, life_time=dt * 2)

            # Vẽ trực quan hóa hành lang mặt đường động (viền cyan/magenta)
            if DEBUG_DRAW_ROAD_CORRIDOR and corridor_tracker is not None:
                corridor_tracker.draw_debug_corridor(carla, current_idx, lookahead_pts=30, life_time=dt * 2)

            # Phân nhánh thực thi theo 2 chế độ điều khiển:
            t_solve_start = time.perf_counter()
            if current_mode == VehicleMode.MODE_2_EMERGENCY_BRAKE:
                # =============================================================
                # --- CHẾ ĐỘ 2: PHANH GẤP TỐI ĐA, BỎ QUA MỌI RÀNG BUỘC ---
                # =============================================================
                wheel_angle_rad = 0.0
                acceleration_m_s_2 = MAX_CONTROL_BRAKING_M_S_2
                steer = 0.0
                throttle = 0.0
                brake = 1.0   # Phanh gấp tối đa 100% kịch sàn (mặc kệ mọi ràng buộc)
                cost_val = 0.0
                solve_time_sec = 0.0

                # Xóa Warm-start để khi trở về Chế độ 1 xe quy hoạch lại quỹ đạo danh định từ trạng thái mới
                if hasattr(mpc_controller, 'reset_nominal'):
                    mpc_controller.reset_nominal()
            else:
                # =============================================================
                # --- CHẾ ĐỘ 1: BÁM ĐƯỜNG NÉ VẬT CẢN (MPC CONTROLLER) ---
                # =============================================================
                if status_info.get('mode_changed', False):
                    if hasattr(mpc_controller, 'reset_nominal'):
                        mpc_controller.reset_nominal()

                if USE_LINEARIZED_QP_MPC:
                    x_state = np.array([x0, y0, theta0, v0])
                    try:
                        wheel_angle_rad, acceleration_m_s_2 = mpc_controller.solve(
                            x_state, x_ref, y_ref, theta_ref, v_ref,
                            detected_obstacles=detected_obstacles
                        )
                        cost_val = mpc_controller.get_optimized_cost()
                    except Exception as _solve_err:
                        print(f"\n[MPC PHANH AN TOÀN] Không tìm được đường né: {_solve_err}. Dừng xe an toàn tại chỗ!")
                        wheel_angle_rad = 0.0
                        acceleration_m_s_2 = -3.5
                        cost_val = 0.0

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

                # Chuyển đổi tín hiệu điều khiển góc lái và gia tốc cho Chế độ 1
                throttle, brake, steer = CarlaSimulator.process_control_inputs(
                    wheel_angle_rad, acceleration_m_s_2, current_speed=v0
                )

            # BẢO VỆ TUYỆT ĐỐI KHÔNG CHẠM VẠCH VÀNG:
            # Can thiệp an toàn nếu khoảng cách tâm xe tới vạch vàng <= 1.05m (mép xe <= 5cm):
            # - Nếu bị roadblock: Phanh dừng khẩn cấp.
            # - Chỉ mớm lái nhẹ sang phải nếu xe đang di chuyển lệch tiếp sang trái (v_lat_ego < -0.02)
            #   để tránh tạo dao động chattering (lắc lái trái/phải liên tục khi bám đường).
            if (yellow_result is not None
                    and yellow_result.line_type == LineType.SOLID
                    and yellow_result.d_yellow_line_m is not None
                    and yellow_result.d_yellow_line_m <= 1.05):
                v_lat_ego = v0 * np.sin(theta0 - theta_ref[0])
                is_rb = hasattr(mpc_controller, 'is_roadblock') and mpc_controller.is_roadblock
                if is_rb or (yellow_result.d_yellow_line_m <= 1.00 and v_lat_ego < -0.05 and v0 > 0.5):
                    throttle = 0.0
                    brake = 1.0
                    steer = max(0.05, steer)
                    print(f"\n[DỪNG AN TOÀN TRƯỚC VẠCH VÀNG] Cự ly tới vạch vàng chỉ còn {yellow_result.d_yellow_line_m:.2f}m (mép xe <= 5cm)! Phanh an toàn!")
                elif v_lat_ego < -0.02:
                    push_right = float(np.clip(0.02 + (1.05 - yellow_result.d_yellow_line_m) * 0.25, 0.02, 0.06))
                    steer = max(push_right, steer)

            logger.log_controller_output(steer, throttle, brake, cost=cost_val)
            carla.apply_control(steer, throttle, brake)

            # Cập nhật Spectator trong cửa sổ CARLA bay theo sau xe mượt mà
            if ENABLE_SPECTATOR_FOLLOW:
                carla.follow_vehicle(carla.ego_vehicle)

            # Lấy thông số biên hành lang mặt đường hiện tại [road_min, road_max] cho tâm xe
            road_min_curr = getattr(mpc_controller, 'current_road_min', None)
            road_max_curr = getattr(mpc_controller, 'current_road_max', None)
            if (road_min_curr is None or road_max_curr is None) and corridor_tracker is not None:
                road_min_curr, road_max_curr, _ = corridor_tracker.get_corridor_at_idx(current_idx)
            if road_min_curr is None or road_max_curr is None:
                road_min_curr, road_max_curr = -3.90, 3.90

            # Cập nhật khung hình và hiển thị thông số HUD lên cửa sổ Camera riêng
            if camera_viewer is not None:
                if is_emergency:
                    bypass_txt = status_info['status_text']
                elif hasattr(mpc_controller, 'is_roadblock') and mpc_controller.is_roadblock:
                    bypass_txt = "DỪNG AN TOÀN (ROADBLOCK)"
                elif hasattr(mpc_controller, 'active_obstacle') and mpc_controller.active_obstacle is not None:
                    side_tag = "PHẢI" if mpc_controller.active_obstacle['side_sign'] > 0 else "TRÁI"
                    bypass_txt = f"NÉ {side_tag} (#{mpc_controller.active_obstacle['id']})"
                else:
                    bypass_txt = "CHẾ ĐỘ 1: BÁM ĐƯỜNG"

                # Thêm thông tin vạch vàng vào HUD
                yellow_hud = '---'
                if yellow_result is not None:
                    yt = yellow_result.line_type.value
                    yd = f'{yellow_result.d_yellow_line_m:.2f}m' if yellow_result.d_yellow_line_m is not None else 'N/A'
                    yellow_hud = f'{yt} | d={yd} | {yellow_result.detect_time_ms:.1f}ms'
                s_ego_curr = float(s_traj[min(current_idx, len(s_traj) - 1)]) if len(s_traj) > 0 else 0.0
                hud_info = {
                    'speed_kmh': v0 * 3.6,
                    'target_kmh': v_ref[0] * 3.6,
                    'steer_deg': float(np.rad2deg(wheel_angle_rad)),
                    'accel_ms2': float(accel_ego),
                    'jerk_ms3': float(jerk_filtered),
                    'bypass_text': bypass_txt,
                    'obs_count': len(detected_obstacles),
                    'current_idx': current_idx,
                    'total_idx': total_waypoints,
                    's_ego_m': s_ego_curr,
                    's_total_m': s_total_m,
                    'is_emergency': is_emergency,
                    'yellow_line': yellow_hud,
                    'road_min': float(road_min_curr),
                    'road_max': float(road_max_curr),
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
            if is_emergency:
                mode_info = f" | [!] {status_info['status_text']}"
            elif hasattr(mpc_controller, 'active_obstacle') and mpc_controller.active_obstacle is not None:
                side_tag = "PHẢI" if mpc_controller.active_obstacle['side_sign'] > 0 else "TRÁI"
                mode_info = f" | [CHẾ ĐỘ 1: NÉ {side_tag}]"
            else:
                mode_info = " | [CHẾ ĐỘ 1: BÁM ĐƯỜNG]"

            if detected_obstacles:
                nearest = detected_obstacles[0]
                src_tag = nearest.get('source', 'obs')[:3].upper()
                obs_status = f" | OBS: {len(detected_obstacles)} [{src_tag}] (D:{nearest['dist']:.1f}m, MũiXe:{nearest['d_lat']:+.1f}m){mode_info}"
            else:
                obs_status = f" | OBS: 0{mode_info}"

            yl_tag = ""
            if yellow_result is not None and yellow_result.line_type == LineType.SOLID:
                yd_str = f"{yellow_result.d_yellow_line_m:.2f}m" if yellow_result.d_yellow_line_m is not None else ""
                yl_tag = f" | [VẠCH VÀNG NÉT LIỀN ĐÔI d={yd_str}]"

            corridor_tag = f" | Road: [{road_min_curr:+.2f}m; {road_max_curr:+.2f}m]"

            print(f"Index: {current_idx}/{total_waypoints} | Pos: ({x0:.1f}, {y0:.1f}) | "
                  f"Speed: {v0:.2f}/{v_ref[0]:.2f} m/s | Steer: {steer:.2f} | Solve: {solve_time_sec:.3f}s{obs_status}{corridor_tag}{yl_tag}", end="\r", flush=True)

            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[THÔNG BÁO] Người dùng ngắt chương trình bằng Ctrl+C.")
    except Exception as e:
        print(f"\n[LỖI THỰC THI] Xảy ra ngoại lệ: {e}")
        traceback.print_exc()
    finally:
        print("\n[HOAN THANH] Ket thuc mo phong.")
        if camera_viewer is not None:
            camera_viewer.destroy()
        if obstacle_manager is not None:
            obstacle_manager.destroy_all()
        if moving_obstacle_manager is not None:
            moving_obstacle_manager.destroy_all()
        carla.clean()
        logger.show_plots()

if __name__ == '__main__':
    main()
