#!/usr/bin/env python3
"""
==============================================================================
CARLA SIMULATION BRIDGE DAEMON FOR AUTONOMOUS C++ STACK
High-Performance, Lockstep Synchronized Socket Server via Carla WorldSnapshot.
Zero-Spam RPC Architecture: uses wait_for_tick() + in-memory snapshot lookups.
CARLA runs at full 30-60 FPS with zero freezing.
==============================================================================
"""

import sys
import os
import time
import math
import socket
import struct
import threading
import argparse

# Thêm đường dẫn carla egg hoặc wheel nếu chưa có trong sys.path
import glob
for base_search in [
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../Carla-Autonomous-MPC/carla/dist"),
    "D:/Appfile/Carla/CARLA_0.9.15/WindowsNoEditor/PythonAPI/carla/dist",
    "/mnt/d/Appfile/Carla/CARLA_0.9.15/WindowsNoEditor/PythonAPI/carla/dist"
]:
    eggs = glob.glob(os.path.join(base_search, 'carla-*.egg'))
    for egg in eggs:
        if egg not in sys.path:
            sys.path.append(egg)

try:
    import carla
except ImportError:
    print("[ERROR] Thư viện CARLA Python API không tìm thấy! Hãy đảm bảo đã cài đặt carla package.")
    sys.exit(1)


def clip(v, lo, hi):
    return max(lo, min(v, hi))


def load_csv(filepath):
    data = []
    if not os.path.exists(filepath):
        return data
    with open(filepath, 'r') as f:
        lines = f.readlines()
        if not lines:
            return data
        header = [h.strip() for h in lines[0].split(',')]
        for l in lines[1:]:
            parts = [p.strip() for p in l.split(',') if p.strip()]
            if len(parts) >= len(header):
                row = {}
                for idx, h in enumerate(header):
                    try:
                        row[h] = float(parts[idx])
                    except ValueError:
                        row[h] = parts[idx]
                data.append(row)
    return data


class CarlaBridgeDaemon:
    def __init__(self, host='127.0.0.1', port=2000, bridge_port=2002, with_obstacles=True):
        self.host = host
        self.port = port
        self.bridge_port = bridge_port
        self.with_obstacles = with_obstacles

        self.client = None
        self.world = None
        self.carla_map = None
        self.spectator = None
        self.ego_vehicle = None

        self.spawned_actors = []
        self.moving_obstacle_entries = []

        # Điều khiển từ C++ (bảo vệ thread-safe)
        self.cmd_lock = threading.Lock()
        self.latest_throttle = 0.0
        self.latest_brake = 0.18
        self.latest_steer = 0.0

        self.running = threading.Event()
        self.running.set()

        self.connect_carla()

    def connect_carla(self):
        hosts = [self.host]
        # Tự phát hiện IP host Windows khi chạy trong WSL2
        if os.path.exists('/etc/resolv.conf'):
            try:
                with open('/etc/resolv.conf', 'r') as f:
                    for line in f:
                        if line.strip().startswith('nameserver'):
                            ns = line.split()[1].strip()
                            if ns not in hosts:
                                hosts.append(ns)
            except Exception:
                pass
        for h in ['127.0.0.1', 'localhost', '172.20.240.1']:
            if h not in hosts:
                hosts.append(h)

        for h in hosts:
            try:
                print(f"[BRIDGE] Kết nối tới CARLA Server tại {h}:{self.port}...")
                c = carla.Client(h, self.port)
                c.set_timeout(10.0)
                w = c.get_world()
                self.client = c
                self.world = w
                self.carla_map = self.world.get_map()
                self.spectator = self.world.get_spectator()

                # BẢO VỆ CHỐNG ĐƠ: Đảm bảo CARLA chạy ở Asynchronous Mode (mặc định mượt mà)
                # Nếu phiên chạy trước bị kẹt ở Synchronous Mode, dòng này sẽ giải phóng CARLA ngay lập tức
                try:
                    settings = self.world.get_settings()
                    if settings.synchronous_mode:
                        print("[BRIDGE] Phát hiện CARLA đang bị kẹt Synchronous Mode. Đang chuyển về Asynchronous Mode...")
                        settings.synchronous_mode = False
                        settings.fixed_delta_seconds = None
                        self.world.apply_settings(settings)
                except Exception as e:
                    print(f"[BRIDGE] Cảnh báo cấu hình settings CARLA: {e}")

                print(f"[BRIDGE] Kết nối CARLA Server thành công ({h}:{self.port})!")
                break
            except Exception:
                continue

        if self.client is None:
            raise ConnectionError("Không thể kết nối đến CARLA Server! Hãy đảm bảo CarlaUE4.exe đang chạy.")

    def clean_old_actors(self):
        """Xóa toàn bộ actors cũ bằng batch sync đơn lẻ - không làm đơ game thread"""
        print("[BRIDGE] Đang làm sạch các actor cũ trên bản đồ...")
        try:
            actors = self.world.get_actors()
            destroy_cmds = []
            for v in actors.filter('*vehicle*'):
                if v.is_alive:
                    destroy_cmds.append(carla.command.DestroyActor(v.id))
            for p in actors.filter('*static.prop*'):
                if p.is_alive:
                    destroy_cmds.append(carla.command.DestroyActor(p.id))
            if destroy_cmds:
                self.client.apply_batch_sync(destroy_cmds, False)
        except Exception as e:
            print(f"[BRIDGE] Cảnh báo dọn dẹp actors: {e}")

    def setup_simulation(self, traj_path, obs_path, moving_obs_path):
        self.clean_old_actors()

        # 1. Nạp quỹ đạo
        traj_data = load_csv(traj_path)
        if not traj_data:
            raise FileNotFoundError(f"Không tìm thấy quỹ đạo tại {traj_path}")

        x0 = traj_data[0]['x']
        y0 = traj_data[0]['y']
        th0 = traj_data[0]['theta_rad']

        # 2. Tạo xe Ego Tesla Model 3
        bp_lib = self.world.get_blueprint_library()
        ego_bp = bp_lib.filter('vehicle.tesla.model3')[0]

        wp = self.carla_map.get_waypoint(carla.Location(x=x0, y=y0, z=0.0), project_to_road=True, lane_type=carla.LaneType.Any)
        z0 = (wp.transform.location.z + 0.5) if wp else 0.5
        yaw_deg = math.degrees(th0)

        tf = carla.Transform(carla.Location(x=x0, y=y0, z=z0), carla.Rotation(pitch=0.0, yaw=yaw_deg, roll=0.0))
        self.ego_vehicle = self.world.try_spawn_actor(ego_bp, tf)
        if self.ego_vehicle is None:
            tf.location.z += 0.3
            self.ego_vehicle = self.world.spawn_actor(ego_bp, tf)

        try:
            self.ego_vehicle.set_autopilot(False)
        except Exception:
            pass

        self.spawned_actors.append(self.ego_vehicle)
        print(f"[BRIDGE] Đã tạo xe Ego tại ({x0:.2f}, {y0:.2f}, z={z0:.2f}, yaw={yaw_deg:.1f}°)")

        # Đặt spectator ban đầu bao quát điểm xuất phát
        if self.spectator:
            cam_x = x0 - 7.5 * math.cos(th0)
            cam_y = y0 - 7.5 * math.sin(th0)
            cam_z = z0 + 3.5
            self.spectator.set_transform(carla.Transform(
                carla.Location(x=cam_x, y=cam_y, z=cam_z),
                carla.Rotation(pitch=-15.0, yaw=yaw_deg, roll=0.0)
            ))

        # 3. Tạo vật cản tĩnh (giữ phanh tay tĩnh chống trôi)
        if self.with_obstacles and os.path.exists(obs_path):
            obs_data = load_csv(obs_path)
            obs_bp = bp_lib.filter('vehicle.toyota.prius')[0]
            for ob in obs_data:
                ox = ob['x']
                oy = ob['y']
                owp = self.carla_map.get_waypoint(carla.Location(x=ox, y=oy, z=0.0), project_to_road=True)
                oz = (owp.transform.location.z + 0.3) if owp else 0.5
                oyaw = owp.transform.rotation.yaw if owp else 0.0
                otf = carla.Transform(carla.Location(x=ox, y=oy, z=oz), carla.Rotation(yaw=oyaw))
                try:
                    act = self.world.try_spawn_actor(obs_bp, otf)
                    if act is not None:
                        act.apply_control(carla.VehicleControl(hand_brake=True, throttle=0.0, brake=1.0))
                        self.spawned_actors.append(act)
                except Exception:
                    pass
            print(f"[BRIDGE] Đã tạo {len(self.spawned_actors) - 1} vật cản tĩnh.")

        # 4. Tạo vật cản di động
        if self.with_obstacles and os.path.exists(moving_obs_path):
            moving_data = load_csv(moving_obs_path)
            groups = {}
            for row in moving_data:
                oid = int(row['obstacle_id'])
                if oid not in groups:
                    groups[oid] = []
                groups[oid].append(row)

            m_bp = bp_lib.filter('vehicle.audi.etron')[0]
            for oid, wps in groups.items():
                wps.sort(key=lambda r: int(r['wp_index']))
                start_w = wps[0]
                mwp = self.carla_map.get_waypoint(carla.Location(x=start_w['x'], y=start_w['y'], z=0.0), project_to_road=True)
                mz = (mwp.transform.location.z + 0.3) if mwp else 0.5
                mtf = carla.Transform(carla.Location(x=start_w['x'], y=start_w['y'], z=mz), carla.Rotation(yaw=0.0))
                try:
                    m_act = self.world.try_spawn_actor(m_bp, mtf)
                    if m_act is not None:
                        try:
                            m_act.set_autopilot(False)
                        except Exception:
                            pass
                        self.spawned_actors.append(m_act)
                        # Tính tổng s và tạo path trong bộ nhớ
                        path_xy = [(float(w['x']), float(w['y'])) for w in wps]
                        s_accum = [0.0]
                        for idx_p in range(1, len(path_xy)):
                            dx = path_xy[idx_p][0] - path_xy[idx_p - 1][0]
                            dy = path_xy[idx_p][1] - path_xy[idx_p - 1][1]
                            s_accum.append(s_accum[-1] + math.hypot(dx, dy))

                        self.moving_obstacle_entries.append({
                            'actor': m_act,
                            'path_xy': path_xy,
                            's_accum': s_accum,
                            'total_s': s_accum[-1],
                            'current_s': 0.0,
                            'speed_ms': float(start_w.get('speed_ms', 9.0)),
                            'road_z': mz
                        })
                except Exception:
                    pass
            print(f"[BRIDGE] Đã tạo {len(self.moving_obstacle_entries)} xe vật cản di động.")

    def update_moving_obstacles_in_memory(self, dt=0.02):
        """Cập nhật vị trí vật cản di động hoàn toàn tính toán nội suy trong RAM, chỉ gọi 1 RPC set_transform"""
        for mob in self.moving_obstacle_entries:
            act = mob['actor']
            if not act.is_alive:
                continue

            mob['current_s'] += mob['speed_ms'] * dt
            if mob['current_s'] >= mob['total_s']:
                mob['current_s'] = mob['current_s'] % mob['total_s']

            cur_s = mob['current_s']
            path_xy = mob['path_xy']
            s_accum = mob['s_accum']

            # Tìm đoạn thẳng tương ứng
            seg_idx = 0
            for idx_s in range(len(s_accum) - 1):
                if s_accum[idx_s] <= cur_s <= s_accum[idx_s + 1]:
                    seg_idx = idx_s
                    break

            seg_len = max(s_accum[seg_idx + 1] - s_accum[seg_idx], 1e-6)
            ratio = (cur_s - s_accum[seg_idx]) / seg_len

            cur_x = path_xy[seg_idx][0] + ratio * (path_xy[seg_idx + 1][0] - path_xy[seg_idx][0])
            cur_y = path_xy[seg_idx][1] + ratio * (path_xy[seg_idx + 1][1] - path_xy[seg_idx][1])
            yaw_rad = math.atan2(path_xy[seg_idx + 1][1] - path_xy[seg_idx][1],
                                 path_xy[seg_idx + 1][0] - path_xy[seg_idx][0])

            try:
                act.set_transform(carla.Transform(
                    carla.Location(x=cur_x, y=cur_y, z=mob['road_z']),
                    carla.Rotation(pitch=0.0, yaw=math.degrees(yaw_rad), roll=0.0)
                ))
            except Exception:
                pass

    def update_spectator(self, ego_tf):
        """Cập nhật góc nhìn camera theo sau xe Ego (Chase Cam) mượt mà"""
        if not self.spectator:
            return
        try:
            yaw_rad = math.radians(ego_tf.rotation.yaw)
            cam_x = ego_tf.location.x - 7.5 * math.cos(yaw_rad)
            cam_y = ego_tf.location.y - 7.5 * math.sin(yaw_rad)
            cam_z = ego_tf.location.z + 3.5
            self.spectator.set_transform(carla.Transform(
                carla.Location(x=cam_x, y=cam_y, z=cam_z),
                carla.Rotation(pitch=-15.0, yaw=ego_tf.rotation.yaw, roll=0.0)
            ))
        except Exception:
            pass

    def _receiver_loop(self, client_sock):
        """Thread riêng đọc lệnh điều khiển ACTR 28 bytes liên tục từ C++ mà không block CARLA tick"""
        while self.running.is_set():
            try:
                cmd_buf = client_sock.recv(28)
                if len(cmd_buf) == 28:
                    magic, thr, brk, str_val = struct.unpack('<4s3d', cmd_buf)
                    if magic == b'ACTR':
                        with self.cmd_lock:
                            self.latest_throttle = float(clip(thr, 0.0, 1.0))
                            self.latest_brake = float(clip(brk, 0.0, 1.0))
                            self.latest_steer = float(clip(str_val, -1.0, 1.0))
                elif len(cmd_buf) == 0:
                    break
            except (socket.timeout, BlockingIOError):
                continue
            except Exception:
                break

    def run_bridge_server(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('0.0.0.0', self.bridge_port))
        server_sock.listen(1)
        print(f"[BRIDGE] Lắng nghe kết nối từ C++ App tại cổng {self.bridge_port}...")

        client_sock, addr = server_sock.accept()
        print(f"[BRIDGE] C++ Autonomous Runtime đã kết nối từ {addr}!")

        # Khởi động thread nhận lệnh không chặn
        client_sock.settimeout(0.5)
        recv_thread = threading.Thread(target=self._receiver_loop, args=(client_sock,), daemon=True)
        recv_thread.start()

        prev_v = 0.0
        prev_a = 0.0
        cycle = 0
        last_tick_time = time.time()

        try:
            while self.running.is_set():
                # 1. Chờ CARLA sinh frame mới - tự động đồng bộ theo FPS thực tế của CARLA (30-60Hz)
                # Tuyệt đối không bao giờ spam RPC làm đơ CARLA!
                try:
                    snapshot = self.world.wait_for_tick(seconds=1.5)
                except RuntimeError:
                    # Timeout chờ tick (server bận)
                    continue

                now_time = time.time()
                dt = max(now_time - last_tick_time, 0.001)
                last_tick_time = now_time
                cycle += 1

                # 2. Đọc trạng thái Ego từ WorldSnapshot trong RAM (0 RPC call!)
                ego_snap = snapshot.find(self.ego_vehicle.id)
                if ego_snap is None:
                    print("[BRIDGE] Xe Ego không tồn tại trong snapshot.")
                    break

                tf = ego_snap.get_transform()
                vel = ego_snap.get_velocity()
                v = math.hypot(vel.x, vel.y)
                yaw_rad = math.radians(tf.rotation.yaw)
                accel = (v - prev_v) / dt
                jerk = (accel - prev_a) / dt
                prev_v = v
                prev_a = accel

                # 3. Lấy vị trí các vật cản từ WorldSnapshot trong RAM (0 RPC call!)
                actors_to_send = []
                for act in self.spawned_actors:
                    if act.id == self.ego_vehicle.id or not act.is_alive:
                        continue
                    act_snap = snapshot.find(act.id)
                    if act_snap is None:
                        continue
                    atf = act_snap.get_transform()
                    avel = act_snap.get_velocity()
                    d = math.hypot(atf.location.x - tf.location.x, atf.location.y - tf.location.y)
                    if d < 35.0:
                        actors_to_send.append((
                            act.id, atf.location.x, atf.location.y, atf.location.z,
                            math.radians(atf.rotation.yaw),
                            2.4, 1.0,
                            avel.x, avel.y,
                            1
                        ))

                # 4. Đóng gói binary packet chuẩn POD C++:
                # VehicleStateData (69 bytes): <QI7dB
                # RawActorData (69 bytes): <i8dB
                now_ns = int(time.time() * 1e9)
                state_bytes = struct.pack(
                    '<QIdddddddB',
                    now_ns, cycle,
                    tf.location.x, tf.location.y, yaw_rad,
                    v, self.latest_steer, accel, jerk, 1
                )

                actors_bytes = struct.pack('<I', len(actors_to_send))
                for a in actors_to_send:
                    actors_bytes += struct.pack(
                        '<iddddddddB',
                        a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9]
                    )

                payload = state_bytes + actors_bytes
                header = struct.pack('<4sI', b'CARL', len(payload))

                try:
                    client_sock.sendall(header + payload)
                except Exception:
                    print("[BRIDGE] C++ App đã ngắt kết nối socket.")
                    break

                # 5. Áp dụng lệnh điều khiển nhận từ C++ vào xe Ego (chỉ 1 RPC)
                with self.cmd_lock:
                    thr = self.latest_throttle
                    brk = self.latest_brake
                    steer = self.latest_steer

                ctrl = carla.VehicleControl(throttle=thr, brake=brk, steer=steer)
                self.ego_vehicle.apply_control(ctrl)

                # 6. Cập nhật vị trí vật cản di động (1 RPC)
                self.update_moving_obstacles_in_memory(dt=dt)

                # 7. Cập nhật spectator bám theo sau xe Ego (1 RPC mỗi 2 frame)
                if cycle % 2 == 0:
                    self.update_spectator(tf)

        except KeyboardInterrupt:
            print("\n[BRIDGE] Dừng theo yêu cầu người dùng (Ctrl+C).")
        finally:
            self.running.clear()
            try:
                client_sock.close()
                server_sock.close()
            except Exception:
                pass
            self.clean_up()

    def clean_up(self):
        print("[BRIDGE] Đang dọn dẹp các actor xe đã spawn...")
        try:
            destroy_cmds = [carla.command.DestroyActor(a.id) for a in self.spawned_actors if a.is_alive]
            if destroy_cmds and self.client:
                self.client.apply_batch_sync(destroy_cmds, False)
        except Exception:
            pass
        print("[BRIDGE] Dọn dẹp hoàn tất. Daemon thoát.")


def main():
    parser = argparse.ArgumentParser(description="CARLA Simulation Bridge Daemon for C++ MPC")
    parser.add_argument('--with-obstacles', action='store_true', default=True, help="Spawn vật cản tĩnh và động")
    parser.add_argument('--no-obstacles', action='store_true', help="Tắt vật cản")
    parser.add_argument('--host', type=str, default='127.0.0.1', help="CARLA host")
    parser.add_argument('--port', type=int, default=2000, help="CARLA port")
    parser.add_argument('--bridge-port', type=int, default=2002, help="TCP bridge port for C++")
    args = parser.parse_args()

    with_obs = not args.no_obstacles

    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
    traj_path = os.path.join(base_dir, "configs/trajectories/reference_trajectory.csv")
    obs_path = os.path.join(base_dir, "configs/trajectories/obstacles.csv")
    mobs_path = os.path.join(base_dir, "configs/trajectories/moving_obstacles.csv")

    server = CarlaBridgeDaemon(host=args.host, port=args.port, bridge_port=args.bridge_port, with_obstacles=with_obs)
    server.setup_simulation(traj_path, obs_path, mobs_path)
    server.run_bridge_server()


if __name__ == '__main__':
    main()
