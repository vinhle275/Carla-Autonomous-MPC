import math
import numpy as np
import carla

from src.config import (
    SENSOR_DETECTION_RANGE_M,
    SENSOR_FOV_DEG,
    SENSOR_SURROUND_RANGE_M,
    SENSOR_TRACK_DROP_BEHIND_M,
    OBSTACLE_AVOIDANCE_MARGIN_M,
    DEBUG_DRAW_SENSOR_BEAMS,
    DEBUG_DRAW_SENSOR_FOV,
    DEBUG_DRAW_SURROUND_CIRCLE,
    DEBUG_DRAW_OBSTACLE_BOX
)
from src.help_functions import normalize_angle

class ObstacleDetector:
    """
    Hệ thống Cảm biến & Nhận thức Đa vùng (Dual-Zone Perception & Persistent Local Object Map):
    1. Vùng 1 - Tầm xa phía trước (Forward Camera/Radar): Quét hình quạt 120 độ, tầm xa tới 25m.
    2. Vùng 2 - Vòng quét bảo vệ cự ly gần 360 độ (Surround Ultrasonic & Blind-spot Radar):
       Quét 360 độ quanh thân xe trong cự ly 10m, triệt tiêu hoàn toàn góc chết khi xe ngoặt mũi né.
    3. Bộ nhớ theo dõi đối tượng (Local Object Map / Multi-Object Tracker - MOT):
       Lưu tọa độ thế giới của vật cản; khi xe đánh lái khiến vật cản tạm thời khuất góc,
       vật cản vẫn được lưu trong bộ nhớ và cập nhật vị trí tương đối cho đến khi xe vượt qua hẳn (> 5m sau đuôi xe).
    """
    def __init__(self, world, ego_vehicle=None,
                 detection_range=SENSOR_DETECTION_RANGE_M,
                 fov_deg=SENSOR_FOV_DEG,
                 surround_range=SENSOR_SURROUND_RANGE_M,
                 track_drop_behind=SENSOR_TRACK_DROP_BEHIND_M):
        self.world = world
        self.ego_vehicle = ego_vehicle
        self.detection_range = detection_range
        self.fov_deg = fov_deg
        self.half_fov_rad = math.radians(fov_deg / 2.0)
        self.surround_range = surround_range
        self.track_drop_behind = track_drop_behind

        # Bản đồ vật cản cục bộ có nhớ (Local Object Map)
        # Lưu trữ: actor_id -> obs_info dict
        self.tracked_obstacles = {}

    def set_ego_vehicle(self, ego_vehicle):
        """Cập nhật đối tượng xe Ego khi được spawn"""
        self.ego_vehicle = ego_vehicle

    def clear_memory(self):
        """Xóa bộ nhớ theo dõi vật cản (khi reset mô phỏng)"""
        self.tracked_obstacles.clear()

    @staticmethod
    def get_closest_point_on_obb(px, py, box_center_x, box_center_y, box_yaw_rad, half_len, half_wid):
        """
        Tính tọa độ điểm gần nhất trên hình chữ nhật định hướng 2D (OBB) đến điểm (px, py):
        1. Chuyển (px, py) sang hệ tọa độ cục bộ của Bounding Box.
        2. Kẹp (clamp/clip) tọa độ theo nửa chiều dài và nửa chiều rộng [-half_len, half_len] x [-half_wid, half_wid].
        3. Chuyển ngược lại hệ tọa độ toàn cục.
        """
        dx = px - box_center_x
        dy = py - box_center_y

        cos_b = math.cos(box_yaw_rad)
        sin_b = math.sin(box_yaw_rad)

        # Chuyển sang hệ tọa độ cục bộ của hộp bao
        x_local = dx * cos_b + dy * sin_b
        y_local = -dx * sin_b + dy * cos_b

        # Kẹp giá trị vào cạnh của hình chữ nhật AABB cục bộ
        x_cl_local = float(np.clip(x_local, -half_len, half_len))
        y_cl_local = float(np.clip(y_local, -half_wid, half_wid))

        # Chuyển ngược lại tọa độ thế giới (Global Frame)
        closest_x = box_center_x + x_cl_local * cos_b - y_cl_local * sin_b
        closest_y = box_center_y + x_cl_local * sin_b + y_cl_local * cos_b

        return closest_x, closest_y

    def scan(self, draw_debug=True, life_time=0.15):
        """
        Quét và trả về danh sách vật cản kết hợp Cảm biến Đa vùng (Dual-Zone) và Local Object Map:
        - Vùng 1: Quét xa 120 độ phía trước (tới 25m).
        - Vùng 2: Quét 360 độ quanh thân xe (tới 10m).
        - Vùng 3: Duy trì đối tượng trong bộ nhớ cho đến khi vượt qua hẳn (sau lưng > 5m).
        """
        if not self.ego_vehicle or not self.ego_vehicle.is_alive:
            return []

        # 1. Trạng thái hiện tại của xe Ego
        ego_tf = self.ego_vehicle.get_transform()
        x_ego = ego_tf.location.x
        y_ego = ego_tf.location.y
        z_ego = ego_tf.location.z
        yaw_ego = math.radians(ego_tf.rotation.yaw)

        # Tọa độ mũi xe (Front Sensor Origin)
        sensor_x = x_ego + 2.0 * math.cos(yaw_ego)
        sensor_y = y_ego + 2.0 * math.sin(yaw_ego)
        sensor_z = z_ego + 0.5
        sensor_origin = carla.Location(x=sensor_x, y=sensor_y, z=sensor_z)
        vehicle_center = carla.Location(x=x_ego, y=y_ego, z=z_ego + 0.5)

        # 2. Lấy các actor tiềm năng trong thế giới CARLA
        actors_to_check = []
        try:
            vehicles = self.world.get_actors().filter('*vehicle*')
            props = self.world.get_actors().filter('*static.prop*')
            actors_to_check = list(vehicles) + list(props)
        except Exception:
            return list(self.tracked_obstacles.values())

        current_seen_ids = set()

        # 3. Quét phát hiện đa vùng (Dual-Zone Sensing)
        for actor in actors_to_check:
            if actor.id == self.ego_vehicle.id:
                continue

            try:
                tf = actor.get_transform()
                box_center_x = tf.location.x
                box_center_y = tf.location.y
                box_center_z = tf.location.z
                box_yaw = math.radians(tf.rotation.yaw)

                # Lấy kích thước Bounding Box thực tế
                if hasattr(actor, 'bounding_box') and actor.bounding_box is not None:
                    half_len = max(actor.bounding_box.extent.x, 0.4)
                    half_wid = max(actor.bounding_box.extent.y, 0.4)
                else:
                    is_v = 'vehicle' in actor.type_id
                    half_len = 2.4 if is_v else 0.6
                    half_wid = 1.0 if is_v else 0.6

            except Exception:
                continue

            # Khoảng cách thô tới tâm
            dist_to_center = math.hypot(box_center_x - x_ego, box_center_y - y_ego)
            if dist_to_center > (self.detection_range + half_len + 3.0):
                continue

            # Tính tọa độ ĐIỂM GẦN NHẤT trên vỏ ngoài của Bounding Box
            closest_x, closest_y = self.get_closest_point_on_obb(
                sensor_x, sensor_y,
                box_center_x, box_center_y, box_yaw,
                half_len, half_wid
            )

            # Khoảng cách từ mũi xe và từ tâm xe tới bề mặt vật cản
            dist_surface_front = math.hypot(closest_x - sensor_x, closest_y - sensor_y)
            dist_surface_ego = math.hypot(closest_x - x_ego, closest_y - y_ego)

            if dist_surface_front < 0.1:
                continue

            # Góc phương vị từ mũi xe tới điểm bề mặt gần nhất
            angle_to_surface = math.atan2(closest_y - sensor_y, closest_x - sensor_x)
            rel_angle = normalize_angle(angle_to_surface - yaw_ego)

            # =================================================================
            # LOGIC CẢM BIẾN ĐA VÙNG (DUAL-ZONE DETECTION LOGIC)
            # =================================================================
            # Vùng 1: Quét xa phía trước (tới 25m) trong góc mở 120 độ
            is_in_front_fov = (dist_surface_front <= self.detection_range) and (abs(rel_angle) <= self.half_fov_rad)
            # Vùng 2: Quét bảo vệ cự ly gần 360 độ quanh thân xe (tới 10m - siêu âm & radar góc mù)
            is_in_surround_360 = (dist_surface_ego <= self.surround_range)

            if is_in_front_fov or is_in_surround_360:
                current_seen_ids.add(actor.id)

                # Tọa độ tương đối dọc và ngang so với trục xe Ego
                d_long = dist_surface_front * math.cos(rel_angle)
                d_lat = dist_surface_front * math.sin(rel_angle)
                r_safe_surface = 1.0 + OBSTACLE_AVOIDANCE_MARGIN_M

                obs_info = {
                    'id': actor.id,
                    'type': actor.type_id,
                    'x': closest_x,
                    'y': closest_y,
                    'z': box_center_z,
                    'center_x': box_center_x,
                    'center_y': box_center_y,
                    'dist': dist_surface_front,
                    'dist_center': dist_to_center,
                    'd_long': d_long,
                    'd_lat': d_lat,
                    'radius': r_safe_surface,
                    'extent_x': half_len,
                    'extent_y': half_wid,
                    'box_yaw': box_yaw,
                    'is_vehicle': 'vehicle' in actor.type_id,
                    'actor': actor,
                    'source': 'front_fov' if is_in_front_fov else 'surround_360'
                }

                # Cập nhật vào Bản đồ vật cản cục bộ (Local Object Map)
                self.tracked_obstacles[actor.id] = obs_info

        # =====================================================================
        # 4. DUY TRÌ BẢN ĐỒ VẬT CẢN CÓ NHỚ BẰNG DEAD-RECKONING (LOCAL OBJECT MAP)
        # =====================================================================
        ids_to_remove = []
        for track_id, obs_info in self.tracked_obstacles.items():
            if track_id not in current_seen_ids:
                # Kiểm tra actor còn tồn tại trong CARLA không
                actor = obs_info.get('actor')
                if actor is None or not actor.is_alive:
                    ids_to_remove.append(track_id)
                    continue

                # Cập nhật vị trí bề mặt gần nhất theo actor transform thực tế
                try:
                    tf_act = actor.get_transform()
                    box_center_x = tf_act.location.x
                    box_center_y = tf_act.location.y
                    box_center_z = tf_act.location.z
                    box_yaw = math.radians(tf_act.rotation.yaw)
                    half_len = obs_info.get('extent_x', 2.0)
                    half_wid = obs_info.get('extent_y', 1.0)
                    closest_x, closest_y = self.get_closest_point_on_obb(
                        sensor_x, sensor_y,
                        box_center_x, box_center_y, box_yaw,
                        half_len, half_wid
                    )
                    obs_x, obs_y = closest_x, closest_y
                    obs_info['x'] = closest_x
                    obs_info['y'] = closest_y
                    obs_info['center_x'] = box_center_x
                    obs_info['center_y'] = box_center_y
                except Exception:
                    obs_x = obs_info['x']
                    obs_y = obs_info['y']

                dx = obs_x - sensor_x
                dy = obs_y - sensor_y
                dist_now = math.hypot(dx, dy)
                angle_now = math.atan2(dy, dx)
                rel_angle_now = normalize_angle(angle_now - yaw_ego)
                d_long_now = dist_now * math.cos(rel_angle_now)
                d_lat_now = dist_now * math.sin(rel_angle_now)

                # Điều kiện giải phóng track: xe đã vượt qua hẳn (sau lưng > 5.0m) hoặc quá xa (> 30m)
                if d_long_now < -self.track_drop_behind or dist_now > (self.detection_range + 5.0):
                    ids_to_remove.append(track_id)
                else:
                    # Vẫn duy trì trong Local Object Map!
                    obs_info['dist'] = dist_now
                    obs_info['d_long'] = d_long_now
                    obs_info['d_lat'] = d_lat_now
                    obs_info['source'] = 'memory_map'

        # Xóa các vật cản đã hoàn toàn qua khỏi xe
        for track_id in ids_to_remove:
            if track_id in self.tracked_obstacles:
                del self.tracked_obstacles[track_id]

        # Danh sách kết quả cuối cùng lấy từ Local Object Map
        final_obstacles = list(self.tracked_obstacles.values())
        final_obstacles.sort(key=lambda o: o['dist'])

        # 5. Trực quan hóa 3D trên CARLA
        if draw_debug:
            self._draw_sensor_debug(sensor_origin, vehicle_center, yaw_ego, final_obstacles, life_time)

        return final_obstacles

    def _draw_sensor_debug(self, sensor_origin, vehicle_center, yaw_ego, detected_obstacles, life_time):
        """Vẽ tia quét FOV 120°, vòng quét 360° cự ly gần và các tia nhận diện trên màn hình CARLA theo cấu hình"""
        debug = self.world.debug

        # 1. Vẽ 2 tia biên hình quạt FOV 120° phía trước (màu xanh lá) - Tắt nếu DEBUG_DRAW_SENSOR_FOV = False
        if DEBUG_DRAW_SENSOR_FOV:
            left_angle = yaw_ego + self.half_fov_rad
            right_angle = yaw_ego - self.half_fov_rad

            left_end = carla.Location(
                x=sensor_origin.x + self.detection_range * math.cos(left_angle),
                y=sensor_origin.y + self.detection_range * math.sin(left_angle),
                z=sensor_origin.z
            )
            right_end = carla.Location(
                x=sensor_origin.x + self.detection_range * math.cos(right_angle),
                y=sensor_origin.y + self.detection_range * math.sin(right_angle),
                z=sensor_origin.z
            )

            fov_color = carla.Color(0, 220, 100, 100)
            debug.draw_line(sensor_origin, left_end, thickness=0.03, color=fov_color, life_time=life_time)
            debug.draw_line(sensor_origin, right_end, thickness=0.03, color=fov_color, life_time=life_time)

        # 2. Vẽ vòng bảo vệ cự ly gần 360° quanh thân xe - Tắt nếu DEBUG_DRAW_SURROUND_CIRCLE = False
        if DEBUG_DRAW_SURROUND_CIRCLE:
            num_circle_pts = 12
            surround_pts = []
            for i in range(num_circle_pts):
                ang = 2.0 * math.pi * i / num_circle_pts
                pt = carla.Location(
                    x=vehicle_center.x + self.surround_range * math.cos(ang),
                    y=vehicle_center.y + self.surround_range * math.sin(ang),
                    z=vehicle_center.z
                )
                surround_pts.append(pt)
            for i in range(num_circle_pts):
                p1 = surround_pts[i]
                p2 = surround_pts[(i + 1) % num_circle_pts]
                debug.draw_line(p1, p2, thickness=0.02, color=carla.Color(0, 150, 255, 60), life_time=life_time)

        # 3. Vẽ tia cảnh báo và hộp bao cho từng vật cản
        for obs in detected_obstacles:
            closest_loc = carla.Location(x=obs['x'], y=obs['y'], z=obs['z'] + 0.3)
            src = obs.get('source', 'front_fov')

            if src == 'front_fov':
                beam_color = carla.Color(255, 0, 0)       # Đỏ: Nhìn thấy trực diện từ camera trước
                point_color = carla.Color(255, 60, 0)
                status_txt = f"[FRONT FOV: {obs['dist']:.1f}m]"
            elif src == 'surround_360':
                beam_color = carla.Color(255, 140, 0)     # Cam: Cảm biến 360° sườn xe
                point_color = carla.Color(255, 180, 0)
                status_txt = f"[360° SURROUND: {obs['dist']:.1f}m]"
            else:
                beam_color = carla.Color(200, 0, 255)     # Tím: Duy trì trong Local Object Map có nhớ
                point_color = carla.Color(255, 0, 255)
                status_txt = f"[MAP MEMORY: {obs['dist']:.1f}m]"

            # Tia nối từ xe tới điểm bề mặt gần nhất - Chỉ vẽ nếu DEBUG_DRAW_SENSOR_BEAMS = True
            if DEBUG_DRAW_SENSOR_BEAMS:
                debug.draw_line(sensor_origin, closest_loc, thickness=0.06, color=beam_color, life_time=life_time)
                debug.draw_point(closest_loc, size=0.15, color=point_color, life_time=life_time)
                # Nhãn chữ hiển thị thông tin
                text_loc = carla.Location(x=obs['x'], y=obs['y'], z=obs['z'] + 1.8)
                info_str = f"{status_txt} | Lat: {obs['d_lat']:+.1f}m"
                debug.draw_string(text_loc, info_str, draw_shadow=True, color=carla.Color(255, 255, 0), life_time=life_time)

            # Vẽ khung Bounding Box 3D nhẹ nhàng để biết vị trí vật cản
            if DEBUG_DRAW_OBSTACLE_BOX:
                try:
                    actor = obs['actor']
                    if hasattr(actor, 'bounding_box') and actor.bounding_box is not None:
                        bb = actor.bounding_box
                        bb_tf = actor.get_transform()
                        debug.draw_box(
                            carla.BoundingBox(bb_tf.location + bb.location, bb.extent),
                            bb_tf.rotation,
                            thickness=0.04,
                            color=carla.Color(0, 180, 255, 100),
                            life_time=life_time
                        )
                except Exception:
                    pass

