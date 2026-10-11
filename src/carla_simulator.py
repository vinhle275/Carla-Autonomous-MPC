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
from src.config import MAX_BRAKING_M_S_2, MAX_WHEEL_ANGLE_RAD, MAX_ACCELERATION_M_S_2

class CarlaSimulator:
    def __init__(self, host='172.20.240.1', port=2000):
        print(f"[CARLA] Kết nối tới CARLA Server tại {host}:{port}...")
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()
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

    def clean(self):
        print("[CARLA] Đang dọn dẹp các actor xe...")
        for vehicle in self.world.get_actors().filter('*vehicle*'):
            try:
                vehicle.destroy()
            except Exception:
                pass

    def draw_full_trajectory_3d(self, x_traj, y_traj, theta_traj=None, life_time=300.0):
        """Vẽ toàn bộ quỹ đạo lên không gian 3D của CARLA."""
        debug = self.world.debug
        last_z = 0.5
        for i in range(len(x_traj)):
            x_c, y_c = x_traj[i], y_traj[i]
            wp = self.carla_map.get_waypoint(carla.Location(x=x_c, y=y_c, z=0.0), project_to_road=True, lane_type=carla.LaneType.Any)
            z_c = wp.transform.location.z + 0.3 if wp else last_z
            last_z = z_c

            loc = carla.Location(x=x_c, y=y_c, z=z_c)
            debug.draw_point(loc, size=0.10, color=carla.Color(0, 255, 0), life_time=life_time)

            if theta_traj is not None and i % 10 == 0:
                th = theta_traj[i]
                end_loc = carla.Location(x=x_c + 1.2 * np.cos(th), y=y_c + 1.2 * np.sin(th), z=z_c)
                debug.draw_arrow(loc, end_loc, thickness=0.05, arrow_size=0.15, color=carla.Color(255, 0, 0), life_time=life_time)

    def draw_horizon_path(self, x_ref, y_ref, life_time=0.2):
        """Vẽ đoạn quỹ đạo N bước MPC đang nhìn thấy phía trước (đường đỏ)"""
        debug = self.world.debug
        for i in range(len(x_ref) - 1):
            p1 = carla.Location(x=x_ref[i], y=y_ref[i], z=0.5)
            p2 = carla.Location(x=x_ref[i + 1], y=y_ref[i + 1], z=0.5)
            debug.draw_line(p1, p2, thickness=0.1, color=carla.Color(255, 0, 0), life_time=life_time)

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
            brake = min(abs(acceleration_m_s_2 / MAX_BRAKING_M_S_2), 1.0)
        else:
            throttle = min(acceleration_m_s_2 / MAX_ACCELERATION_M_S_2, 1.0)
            brake = 0.0

        steer = np.clip(wheel_angle_rad / MAX_WHEEL_ANGLE_RAD, -1.0, 1.0)
        return throttle, brake, steer
