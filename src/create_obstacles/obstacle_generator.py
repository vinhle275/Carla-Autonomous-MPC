import os
import sys
import time
import math
import random
import numpy as np

# Thêm đường dẫn gốc của dự án vào sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import carla
from src.carla_simulator import CarlaSimulator
from src.help_functions import load_csv_trajectory
from src.config import (
    OBSTACLE_START_DISTANCE_M,
    OBSTACLE_START_TRAJECTORY_FRACTION,
    OBSTACLE_END_TRAJECTORY_FRACTION,
    OBSTACLE_MIN_DISTANCE_M,
    OBSTACLE_PREVIEW_DURATION_SEC,
    OBSTACLE_LATERAL_OFFSETS,
    ENABLE_OBSTACLE_CLUSTERS,
    OBSTACLE_CLUSTER_SPACING_M,
    OBSTACLE_CLUSTER_CHANCE,
    OBSTACLE_CLUSTER_SIZES,
    STATIC_OBSTACLE_BLUEPRINTS,
    TRAJECTORY_CSV_PATH,
    CARLA_HOST,
    CARLA_PORT
)

class ObstacleManager:
    """
    Quản lý sinh các vật cản tĩnh dọc theo quỹ đạo tham chiếu trong CARLA:
    - Bắt đầu từ 1/3 quỹ đạo trở đi.
    - Đặt trên quỹ đạo (offset = 0) và gần quỹ đạo (lệch trái / lệch phải).
    - Khoảng cách giữa các vật cản tối thiểu 50m.
    - Hỗ trợ xem trước trong thời gian quy định (mặc định 60s) rồi tự động dọn dẹp.
    - Tích hợp hàm dọn dẹp cho vòng đời mô phỏng.
    """
    def __init__(self, carla_sim=None):
        if carla_sim is not None:
            self.carla_sim = carla_sim
            self.world = carla_sim.world
            self.carla_map = carla_sim.carla_map
            self._owns_sim = False
        else:
            self.carla_sim = CarlaSimulator(host=CARLA_HOST, port=CARLA_PORT)
            self.world = self.carla_sim.world
            self.carla_map = self.carla_sim.carla_map
            self._owns_sim = True

        self.spawned_obstacles = []
        self.obstacle_records = []

    def get_valid_blueprint(self, bp_name):
        """Tìm blueprint trong thư viện CARLA, nếu không có thì chọn blueprint tương đương."""
        bp_lib = self.world.get_blueprint_library()
        matching = bp_lib.filter(bp_name)
        if len(matching) > 0:
            return matching[0]

        # Dự phòng (Fallback) nếu CARLA thiếu blueprint cụ thể
        if 'vehicle' in bp_name:
            fallback = bp_lib.filter('vehicle.*')
        else:
            fallback = bp_lib.filter('static.prop.*')

        if len(fallback) > 0:
            return fallback[0]
        return bp_lib.filter('vehicle.tesla.model3')[0]

    def generate_obstacles(self, x_traj, y_traj, theta_traj):
        """
        Sinh các vật cản tĩnh theo cấu hình định nghĩa trong config.py:
        - Xuất hiện từ mét thứ 25 của quỹ đạo (hoặc OBSTACLE_START_DISTANCE_M)
        - Hỗ trợ cụm 2 - 3 vật cản sát nhau cách nhau 2 - 3m (ENABLE_OBSTACLE_CLUSTERS)
        - Khoảng cách giữa các cụm tối thiểu OBSTACLE_MIN_DISTANCE_M
        - Đặt cả trên quỹ đạo và xung quanh quỹ đạo (OBSTACLE_LATERAL_OFFSETS)
        """
        print("\n--- BẮT ĐẦU SINH VẬT CẢN TĨNH DỌC THEO QUỸ ĐẠO ---")
        total_points = len(x_traj)
        if total_points < 10:
            print("[CẢNH BÁO] Quỹ đạo quá ngắn để sinh vật cản!")
            return []

        # 1. Tính quãng đường tích lũy (s) dọc theo toàn bộ quỹ đạo
        s_accumulated = [0.0]
        for i in range(1, total_points):
            dist = math.hypot(x_traj[i] - x_traj[i - 1], y_traj[i] - y_traj[i - 1])
            s_accumulated.append(s_accumulated[-1] + dist)
        s_accumulated = np.array(s_accumulated)
        total_distance = s_accumulated[-1]

        # 2. Xác định phạm vi quãng đường sinh vật cản
        if OBSTACLE_START_DISTANCE_M is not None:
            s_start = min(float(OBSTACLE_START_DISTANCE_M), total_distance * 0.9)
        elif OBSTACLE_START_TRAJECTORY_FRACTION is not None:
            s_start = total_distance * OBSTACLE_START_TRAJECTORY_FRACTION
        else:
            s_start = 25.0

        s_end = total_distance * OBSTACLE_END_TRAJECTORY_FRACTION

        print(f"[THÔNG TIN QUỸ ĐẠO] Tổng quãng đường: {total_distance:.1f}m")
        print(f"[VÙNG SINH VẬT CẢN] Bắt đầu từ mét thứ {s_start:.1f}m đến {s_end:.1f}m")
        print(f"[QUY TẮC] Cụm vật cản sát nhau (2-3m): {'BẬT' if ENABLE_OBSTACLE_CLUSTERS else 'TẮT'} | Khoảng cách giữa các cụm: {OBSTACLE_MIN_DISTANCE_M}m")

        # 3. Duyệt và tạo vật cản
        current_s = s_start
        offset_idx = 0
        obstacle_counter = 1

        while current_s <= s_end:
            # Quyết định kích thước cụm (2-3 vật cản gần nhau hoặc 1 vật cản đơn)
            if ENABLE_OBSTACLE_CLUSTERS and random.random() < OBSTACLE_CLUSTER_CHANCE:
                cluster_size = random.choice(OBSTACLE_CLUSTER_SIZES)
            else:
                cluster_size = 1

            # Chọn độ lệch ngang cơ sở cho cụm vật cản
            base_d_lat = OBSTACLE_LATERAL_OFFSETS[offset_idx % len(OBSTACLE_LATERAL_OFFSETS)]
            offset_idx += 1

            for c_i in range(cluster_size):
                obs_s = current_s + c_i * OBSTACLE_CLUSTER_SPACING_M
                if obs_s > s_end:
                    break

                # Tìm chỉ số waypoint gần nhất với khoảng cách obs_s
                idx = int(np.searchsorted(s_accumulated, obs_s))
                if idx >= total_points:
                    break

                x_ref = x_traj[idx]
                y_ref = y_traj[idx]
                theta_ref = theta_traj[idx]

                # Độ lệch ngang: các vật cản trong cùng cụm giữ cùng dải làn (xê dịch nhẹ 0.15m)
                if cluster_size > 1 and c_i > 0:
                    d_lat = base_d_lat + random.uniform(-0.15, 0.15)
                else:
                    d_lat = base_d_lat

                # Tọa độ vật cản:
                # Vector ngang vuông góc với tim đường: (-sin(theta), cos(theta))
                x_obs = x_ref - d_lat * math.sin(theta_ref)
                y_obs = y_ref + d_lat * math.cos(theta_ref)

                # Lấy độ cao z từ mặt đường CARLA tại vị trí đặt vật cản
                wp = self.carla_map.get_waypoint(
                    carla.Location(x=x_obs, y=y_obs, z=0.0),
                    project_to_road=True,
                    lane_type=carla.LaneType.Any
                )
                road_z = wp.transform.location.z if wp else 0.5

                # Chọn Blueprint ngẫu nhiên từ danh sách cấu hình
                bp_name = random.choice(STATIC_OBSTACLE_BLUEPRINTS)
                bp = self.get_valid_blueprint(bp_name)

                is_vehicle = 'vehicle' in bp.id
                if is_vehicle:
                    spawn_z = road_z + 0.4
                    yaw_deg = math.degrees(theta_ref) + random.uniform(-10.0, 10.0)
                else:
                    spawn_z = road_z + 0.15
                    yaw_deg = math.degrees(theta_ref)

                spawn_transform = carla.Transform(
                    carla.Location(x=x_obs, y=y_obs, z=spawn_z),
                    carla.Rotation(pitch=0.0, yaw=yaw_deg, roll=0.0)
                )

                # Spawn actor vào CARLA
                actor = self.world.try_spawn_actor(bp, spawn_transform)

                if actor is not None:
                    if is_vehicle:
                        actor.apply_control(carla.VehicleControl(hand_brake=True, throttle=0.0, brake=1.0))

                    self.spawned_obstacles.append(actor)

                    cluster_tag = f" [Cụm {cluster_size} vật cản, #{c_i + 1}]" if cluster_size > 1 else ""
                    pos_desc = "TRÊN QUỸ ĐẠO (tim đường)" if abs(d_lat) < 0.1 else (
                        f"GẦN QUỸ ĐẠO (lệch phải +{d_lat:.1f}m)" if d_lat > 0 else f"GẦN QUỸ ĐẠO (lệch trái {d_lat:.1f}m)"
                    )

                    record = {
                        'index': obstacle_counter,
                        'actor_id': actor.id,
                        'type': bp.id,
                        'is_vehicle': is_vehicle,
                        'x': round(x_obs, 2),
                        'y': round(y_obs, 2),
                        'z': round(spawn_z, 2),
                        's_distance': round(obs_s, 1),
                        'lateral_offset': d_lat,
                        'position_type': pos_desc + cluster_tag
                    }
                    self.obstacle_records.append(record)

                    print(f"  [{obstacle_counter:02d}] Đã tạo: {bp.id:<30} | Quãng đường: {obs_s:6.1f}m | "
                          f"Vị trí: ({x_obs:6.2f}, {y_obs:6.2f}) | {pos_desc}{cluster_tag}")

                    obstacle_counter += 1
                else:
                    print(f"  [!] Thử spawn tại s={obs_s:.1f}m bị trùng lặp/vướng địa hình, bỏ qua...")

            # Khoảng cách giữa các CỤM vật cản liên tiếp
            last_cluster_s = current_s + (cluster_size - 1) * OBSTACLE_CLUSTER_SPACING_M
            current_s = last_cluster_s + OBSTACLE_MIN_DISTANCE_M + random.uniform(2.0, 8.0)

        # 4. Vẽ Marker 3D cảnh báo trong CARLA (hiển thị trên đầu mỗi vật cản)
        self.draw_debug_markers(life_time=OBSTACLE_PREVIEW_DURATION_SEC + 10.0)

        print(f"\n[THÀNH CÔNG] Đã sinh tổng cộng {len(self.spawned_obstacles)} vật cản tĩnh trên và gần quỹ đạo.")
        return self.spawned_obstacles

    def draw_debug_markers(self, life_time=60.0):
        """Vẽ nhãn và hộp 3D đánh dấu vị trí các vật cản trên màn hình CARLA"""
        debug = self.world.debug
        for rec in self.obstacle_records:
            loc = carla.Location(x=rec['x'], y=rec['y'], z=rec['z'] + 1.8)
            text = f"OBS #{rec['index']}: {rec['position_type']}"
            color = carla.Color(255, 50, 50) if abs(rec['lateral_offset']) < 0.1 else carla.Color(255, 180, 0)
            
            # Vẽ chữ trên cao
            debug.draw_string(loc, text, draw_shadow=True, color=color, life_time=life_time)
            # Vẽ điểm tròn màu đỏ định vị
            base_loc = carla.Location(x=rec['x'], y=rec['y'], z=rec['z'])
            debug.draw_point(base_loc, size=0.2, color=carla.Color(255, 0, 0), life_time=life_time)

    def destroy_all(self):
        """Dọn dẹp sạch toàn bộ các vật cản đã sinh ra"""
        if not self.spawned_obstacles:
            return

        print(f"\n[DỌN DẸP] Đang xóa {len(self.spawned_obstacles)} vật cản khỏi CARLA...")
        destroyed_count = 0
        for actor in self.spawned_obstacles:
            try:
                if actor.is_alive:
                    actor.destroy()
                    destroyed_count += 1
            except Exception:
                pass

        self.spawned_obstacles.clear()
        self.obstacle_records.clear()
        print(f"[HOÀN TẤT] Đã dọn dẹp thành công {destroyed_count} vật cản.")

    def run_preview(self, duration_sec=OBSTACLE_PREVIEW_DURATION_SEC):
        """Giữ vật cản xuất hiện trong duration_sec (mặc định 60s) rồi tự động dọn dẹp"""
        print(f"\n[PREVIEW] Vật cản sẽ xuất hiện trong {int(duration_sec)} giây để bạn quan sát trên CARLA.")
        print("[HƯỚNG DẪN] Nhấn Ctrl+C bất kỳ lúc nào nếu muốn kết thúc và dọn dẹp sớm.\n")

        start_time = time.time()
        try:
            while True:
                elapsed = time.time() - start_time
                remaining = duration_sec - elapsed
                if remaining <= 0:
                    break

                print(f"Thời gian còn lại: {int(remaining):02d}s (Vật cản đang hiển thị trên CARLA)...", end="\r")
                time.sleep(1.0)
            print("\n[HẾT GIỜ] Đã hết thời gian xem trước 60 giây.")
        except KeyboardInterrupt:
            print("\n[NGẮT] Người dùng nhấn Ctrl+C để kết thúc sớm.")
        finally:
            self.destroy_all()

def main():
    print("==========================================================")
    print("      TẠO VẬT CẢN TĨNH TRÊN VÀ GẦN QUỸ ĐẠO CARLA MPC      ")
    print("==========================================================")

    # 1. Tải quỹ đạo tham chiếu từ file CSV
    x_traj, y_traj, theta_traj, _ = load_csv_trajectory(TRAJECTORY_CSV_PATH)

    # 2. Khởi tạo ObstacleManager và kết nối CARLA
    manager = ObstacleManager()

    # 3. Vẽ lại đường quỹ đạo 3D (màu xanh lá) để tiện so sánh trực quan
    print("[CARLA] Đang hiển thị quỹ đạo 3D tham chiếu...")
    manager.carla_sim.draw_full_trajectory_3d(x_traj, y_traj, theta_traj, life_time=OBSTACLE_PREVIEW_DURATION_SEC + 10.0)

    # Đặt góc nhìn quan sát Spectator nhìn bao quát từ trên cao tại vị trí bắt đầu có vật cản
    s_accum = [0.0]
    for i in range(1, len(x_traj)):
        s_accum.append(s_accum[-1] + math.hypot(x_traj[i] - x_traj[i - 1], y_traj[i] - y_traj[i - 1]))
    
    if OBSTACLE_START_DISTANCE_M is not None:
        start_s = min(float(OBSTACLE_START_DISTANCE_M), s_accum[-1] * 0.9)
    elif OBSTACLE_START_TRAJECTORY_FRACTION is not None:
        start_s = s_accum[-1] * OBSTACLE_START_TRAJECTORY_FRACTION
    else:
        start_s = 25.0
    idx_start = int(np.searchsorted(s_accum, start_s))
    manager.carla_sim.set_spectator(
        x=x_traj[idx_start],
        y=y_traj[idx_start],
        z=45.0,
        pitch=-75.0,
        yaw=math.degrees(theta_traj[idx_start])
    )

    # 4. Sinh các vật cản tĩnh theo cấu hình
    manager.generate_obstacles(x_traj, y_traj, theta_traj)

    # 5. Duy trì vật cản trong 1 phút rồi tự dọn dẹp
    manager.run_preview(duration_sec=OBSTACLE_PREVIEW_DURATION_SEC)

if __name__ == '__main__':
    main()
