import glob
import os
import sys
import numpy as np

# Thêm đường dẫn thư viện carla egg nếu cần
try:
    sys.path.append(glob.glob(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + '/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from src.config import (
    MAX_BRAKING_M_S_2, MAX_WHEEL_ANGLE_RAD, MAX_ACCELERATION_M_S_2, CARLA_HOST, CARLA_PORT,
    DEBUG_DRAW_FULL_TRAJECTORY, DEBUG_DRAW_TRAJECTORY_ARROWS, DEBUG_DRAW_HORIZON_PATH, DEBUG_DRAW_OBSTACLE_BOX,
    DEBUG_DRAW_EMERGENCY_ZONE, EMERGENCY_ZONE_WIDTH_M, EMERGENCY_ZONE_LENGTH_M, EGO_FRONT_OVERHANG_M
)

class CarlaSimulator:
    def __init__(self, host=CARLA_HOST, port=CARLA_PORT):
        hosts_to_try = [host, 'localhost', '127.0.0.1']
        self.client = None
        self.world = None

        for h in hosts_to_try:
            try:
                print(f"[CARLA] Thử kết nối tới CARLA Server tại {h}:{port}...")
                c = carla.Client(h, port)
                c.set_timeout(10.0)
                w = c.get_world()
                self.client = c
                self.world = w
                self.carla_map = self.world.get_map()
                print(f"[CARLA] Kết nối thành công tới CARLA Server ({h}:{port})!")
                break
            except Exception:
                continue

        if self.client is None:
            raise ConnectionError(f"Không thể kết nối tới CARLA Server tại các địa chỉ {hosts_to_try}:{port}!")

        self.ego_vehicle = None
        
        # Dọn dẹp các xe cũ nếu có
        self.clean()

    def load_world(self, map_name):
        current_map = self.world.get_map().name
        if map_name not in current_map:
            print(f"[CARLA] Đang tải bản đồ: {map_name}...")
            self.client.load_world(map_name)
            self.world = self.client.get_world()
            self.carla_map = self.world.get_map()

    def spawn_ego_vehicle(self, vehicle_name='vehicle.tesla.model3', x=0.0, y=0.0, z=None, yaw_rad=0.0):
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter(vehicle_name)[0]

        # Đặt xe CHÍNH XÁC tại vị trí waypoint xuất phát (x, y), lấy độ cao Z từ mặt đường
        wp = self.carla_map.get_waypoint(carla.Location(x=x, y=y, z=0.0), project_to_road=True, lane_type=carla.LaneType.Any)
        spawn_x = x
        spawn_y = y
        spawn_z = (wp.transform.location.z + 0.5) if wp else (0.5 if z is None else z + 0.5)
        yaw_deg = np.rad2deg(yaw_rad)

        spawn_location = carla.Location(x=spawn_x, y=spawn_y, z=spawn_z)
        spawn_rotation = carla.Rotation(pitch=0.0, yaw=yaw_deg, roll=0.0)
        spawn_transform = carla.Transform(location=spawn_location, rotation=spawn_rotation)

        self.ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_transform)
        print(f"[CARLA] Đã tạo xe {vehicle_name} tại đúng điểm bắt đầu x={spawn_x:.2f}, y={spawn_y:.2f}, z={spawn_z:.2f}, yaw={yaw_deg:.1f}°")
        return self.ego_vehicle

    def set_spectator(self, x=0.0, y=0.0, z=30.0, pitch=-60.0, yaw=0.0):
        spectator = self.world.get_spectator()
        location = carla.Location(x=x, y=y, z=z)
        rotation = carla.Rotation(pitch=pitch, yaw=yaw, roll=0.0)
        spectator.set_transform(carla.Transform(location, rotation))

    def follow_vehicle(self, ego_vehicle=None, distance=7.5, height=3.5, pitch=-15.0):
        """Cập nhật Spectator của CARLA bám theo sau xe Ego (Chase Cam) mượt mà, không tốn thêm tài nguyên render"""
        target = ego_vehicle if ego_vehicle is not None else self.ego_vehicle
        if not target or not target.is_alive:
            return
        try:
            tf = target.get_transform()
            yaw_rad = math.radians(tf.rotation.yaw)
            cam_x = tf.location.x - distance * math.cos(yaw_rad)
            cam_y = tf.location.y - distance * math.sin(yaw_rad)
            cam_z = tf.location.z + height
            spectator = self.world.get_spectator()
            spectator.set_transform(carla.Transform(
                carla.Location(x=cam_x, y=cam_y, z=cam_z),
                carla.Rotation(pitch=pitch, yaw=tf.rotation.yaw, roll=0.0)
            ))
        except Exception:
            pass

    def clean(self):
        print("[CARLA] Đang dọn dẹp các actor xe và vật cản...")
        for actor in self.world.get_actors().filter('*vehicle*'):
            try:
                actor.destroy()
            except Exception:
                pass
        for prop in self.world.get_actors().filter('*static.prop*'):
            try:
                prop.destroy()
            except Exception:
                pass

    def clear_debug_drawings(self):
        """Xóa toàn bộ các nét vẽ debug (đường quỹ đạo cũ) bằng cách reload world nhẹ"""
        print("[CARLA] Đang làm sạch toàn bộ nét vẽ quỹ đạo và đối tượng cũ trên CARLA 3D...")
        try:
            self.clean()
            # Tăng timeout lên 25s vì CARLA Server cần thời gian nạp lại bản đồ khi reload_world
            self.client.set_timeout(25.0)
            self.world = self.client.reload_world(False)
            self.carla_map = self.world.get_map()
            self.client.set_timeout(10.0)
            print("[CARLA] Đã làm sạch bản đồ 3D thành công!")
        except Exception as e:
            print(f"[CARLA] Cảnh báo khi làm sạch bản đồ: {e}")
            try:
                self.client.set_timeout(10.0)
                self.world = self.client.get_world()
                self.carla_map = self.world.get_map()
            except Exception:
                pass

    def draw_full_trajectory_3d(self, x_traj, y_traj, theta_traj=None, life_time=300.0):
        """Vẽ toàn bộ quỹ đạo lên không gian 3D của CARLA."""
        if not DEBUG_DRAW_FULL_TRAJECTORY:
            return
        debug = self.world.debug
        last_z = 0.5
        for i in range(len(x_traj)):
            x_c, y_c = x_traj[i], y_traj[i]
            wp = self.carla_map.get_waypoint(carla.Location(x=x_c, y=y_c, z=0.0), project_to_road=True, lane_type=carla.LaneType.Any)
            z_c = wp.transform.location.z + 0.3 if wp else last_z
            last_z = z_c

            loc = carla.Location(x=x_c, y=y_c, z=z_c)
            debug.draw_point(loc, size=0.10, color=carla.Color(0, 255, 0), life_time=life_time)

            if DEBUG_DRAW_TRAJECTORY_ARROWS and theta_traj is not None and i % 10 == 0:
                th = theta_traj[i]
                end_loc = carla.Location(x=x_c + 1.2 * np.cos(th), y=y_c + 1.2 * np.sin(th), z=z_c)
                debug.draw_arrow(loc, end_loc, thickness=0.05, arrow_size=0.15, color=carla.Color(255, 0, 0), life_time=life_time)

    def draw_horizon_path(self, x_ref, y_ref, life_time=0.2):
        """Vẽ đoạn quỹ đạo N bước MPC đang nhìn thấy phía trước (đường đỏ)"""
        if not DEBUG_DRAW_HORIZON_PATH:
            return
        debug = self.world.debug
        for i in range(len(x_ref) - 1):
            p1 = carla.Location(x=x_ref[i], y=y_ref[i], z=0.5)
            p2 = carla.Location(x=x_ref[i + 1], y=y_ref[i + 1], z=0.5)
            debug.draw_line(p1, p2, thickness=0.1, color=carla.Color(255, 0, 0), life_time=life_time)

    def draw_obstacle_box_3d(self, x, y, size=0.5, life_time=300.0):
        """Ve khoi hop 3D mau do va nhan danh dau vat can tren CARLA 3D"""
        if not DEBUG_DRAW_OBSTACLE_BOX:
            return
        debug = self.world.debug
        wp = self.carla_map.get_waypoint(carla.Location(x=x, y=y, z=0.0), project_to_road=True, lane_type=carla.LaneType.Any)
        z = wp.transform.location.z + size / 2.0 if wp else 0.5
        box_loc = carla.Location(x=x, y=y, z=z)
        extent = carla.Vector3D(x=size / 2.0, y=size / 2.0, z=size / 2.0)
        box = carla.BoundingBox(box_loc, extent)
        debug.draw_box(box, carla.Rotation(0, 0, 0), thickness=0.1, color=carla.Color(255, 0, 0), life_time=life_time)
        debug.draw_string(carla.Location(x=x, y=y, z=z + size + 0.3), "OBS", color=carla.Color(255, 50, 50), life_time=life_time)

    def draw_moving_obstacle_path(self, x1, y1, x2, y2, life_time=300.0):
        """Ve doan thang duong di cua vat can di dong (mau vang cam) len CARLA 3D"""
        import math as _math
        debug = self.world.debug
        wp1 = self.carla_map.get_waypoint(carla.Location(x=x1, y=y1, z=0.0),
                                          project_to_road=True, lane_type=carla.LaneType.Any)
        wp2 = self.carla_map.get_waypoint(carla.Location(x=x2, y=y2, z=0.0),
                                          project_to_road=True, lane_type=carla.LaneType.Any)
        z1 = (wp1.transform.location.z + 0.4) if wp1 else 0.5
        z2 = (wp2.transform.location.z + 0.4) if wp2 else 0.5
        p1 = carla.Location(x=x1, y=y1, z=z1)
        p2 = carla.Location(x=x2, y=y2, z=z2)
        debug.draw_line(p1, p2, thickness=0.12,
                        color=carla.Color(255, 165, 0), life_time=life_time)
        # Ve mui ten chi chieu di chuyen
        mid_x = (x1 + x2) / 2.0
        mid_y = (y1 + y2) / 2.0
        mid_z = (z1 + z2) / 2.0
        angle = _math.atan2(y2 - y1, x2 - x1)
        arr_len = 0.8
        end_x = mid_x + arr_len * _math.cos(angle)
        end_y = mid_y + arr_len * _math.sin(angle)
        debug.draw_arrow(
            carla.Location(x=mid_x, y=mid_y, z=mid_z),
            carla.Location(x=end_x, y=end_y, z=mid_z),
            thickness=0.06, arrow_size=0.2,
            color=carla.Color(255, 140, 0), life_time=life_time
        )

    def draw_moving_obstacle_start(self, x, y, label="MOV", life_time=300.0):
        """Ve diem xuat phat va nhan cua vat can di dong len CARLA 3D"""
        debug = self.world.debug
        wp = self.carla_map.get_waypoint(carla.Location(x=x, y=y, z=0.0),
                                         project_to_road=True, lane_type=carla.LaneType.Any)
        z = (wp.transform.location.z + 0.5) if wp else 0.5
        loc = carla.Location(x=x, y=y, z=z)
        debug.draw_point(loc, size=0.25, color=carla.Color(255, 165, 0), life_time=life_time)
        debug.draw_string(
            carla.Location(x=x, y=y, z=z + 1.5),
            label, draw_shadow=True,
            color=carla.Color(255, 200, 0), life_time=life_time
        )

    def draw_emergency_danger_zone(self, x_ego, y_ego, yaw_rad, z_ego=0.5, is_triggered=False,
                                   zone_width=EMERGENCY_ZONE_WIDTH_M,
                                   zone_length=EMERGENCY_ZONE_LENGTH_M,
                                   front_overhang=EGO_FRONT_OVERHANG_M,
                                   life_time=0.2):
        """
        Vẽ hộp 3D hiển thị vùng nguy hiểm va chạm trước đầu xe trên môi trường CARLA:
        - Chế độ 1 (is_triggered = False): Khung màu xanh lá thể hiện vùng giám sát an toàn.
        - Chế độ 2 (is_triggered = True): Khung viền đỏ rực + nhãn cảnh báo PHANH GẤP!
        """
        if not DEBUG_DRAW_EMERGENCY_ZONE:
            return
        import math as _math
        debug = self.world.debug

        d_center = front_overhang + zone_length / 2.0
        box_x = x_ego + d_center * _math.cos(yaw_rad)
        box_y = y_ego + d_center * _math.sin(yaw_rad)
        box_z = z_ego + 0.4

        box_loc = carla.Location(x=box_x, y=box_y, z=box_z)
        extent = carla.Vector3D(x=zone_length / 2.0, y=zone_width / 2.0, z=0.4)
        box = carla.BoundingBox(box_loc, extent)
        rotation = carla.Rotation(pitch=0.0, yaw=_math.degrees(yaw_rad), roll=0.0)

        if is_triggered:
            box_color = carla.Color(255, 0, 0, 220)
            thickness = 0.08
            debug.draw_box(box, rotation, thickness=thickness, color=box_color, life_time=life_time)
            txt_loc = carla.Location(x=box_x, y=box_y, z=box_z + 1.2)
            debug.draw_string(txt_loc, "[!] VÙNG NGUY HIỂM - PHANH GẤP!", draw_shadow=True,
                              color=carla.Color(255, 50, 50), life_time=life_time)
        else:
            box_color = carla.Color(0, 255, 120, 70)
            thickness = 0.03
            debug.draw_box(box, rotation, thickness=thickness, color=box_color, life_time=life_time)

    def get_main_ego_vehicle_state(self):
        if not self.ego_vehicle or not self.ego_vehicle.is_alive:
            raise RuntimeError("Xe Ego trong CARLA đã bị phá hủy hoặc rơi khỏi bản đồ!")
        transform = self.ego_vehicle.get_transform()
        x = transform.location.x
        y = transform.location.y
        theta = np.deg2rad(transform.rotation.yaw)
        vel = self.ego_vehicle.get_velocity()
        v = np.sqrt(vel.x**2 + vel.y**2)
        return x, y, theta, v

    def apply_control(self, steer, throttle, brake):
        if self.ego_vehicle and self.ego_vehicle.is_alive:
            self.ego_vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake))

    @staticmethod
    def process_control_inputs(wheel_angle_rad, acceleration_m_s_2, current_speed=0.0):
        # Chống trượt bánh (Traction Control) khi vừa khởi động từ tốc độ 0
        if current_speed < 1.5:
            acceleration_m_s_2 = min(acceleration_m_s_2, 2.5)  # Khởi động mềm gia tốc max 2.5 m/s^2

        if acceleration_m_s_2 <= 0:
            throttle = 0.0
            # Khi xe đang đi chậm (current_speed < 2.5 m/s):
            # Nếu chỉ rà phanh nhẹ (acc >= -1.5), giữ brake tối đa 0.15 để không khựng dính bánh
            # Nếu phanh dừng hẳn (acc < -1.5 hoặc dừng khẩn cấp), cho phép phanh dứt khoát để xe dừng hẳn về 0
            if current_speed < 2.5 and acceleration_m_s_2 >= -1.5:
                brake = min(abs(acceleration_m_s_2 / MAX_BRAKING_M_S_2), 0.15)
            else:
                brake = min(abs(acceleration_m_s_2 / MAX_BRAKING_M_S_2), 1.0)
        else:
            throttle = min(acceleration_m_s_2 / MAX_ACCELERATION_M_S_2, 1.0)
            brake = 0.0

        steer = np.clip(wheel_angle_rad / MAX_WHEEL_ANGLE_RAD, -1.0, 1.0)
        return throttle, brake, steer
