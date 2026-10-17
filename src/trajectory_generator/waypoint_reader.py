import glob
import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

# Thêm thư viện carla nếu có file .egg
try:
    sys.path.append(glob.glob(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + '/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from src.config import CARLA_HOST, CARLA_PORT, OBSTACLE_SIZE

_cached_carla_road_data = None

def get_carla_road_network(host=CARLA_HOST, port=CARLA_PORT):
    """Trích xuất và cache mạng lưới tâm đường từ CARLA Server"""
    global _cached_carla_road_data
    if _cached_carla_road_data is not None:
        return _cached_carla_road_data

    hosts_to_try = [host, 'localhost', '127.0.0.1']
    carla_map = None

    for h in hosts_to_try:
        try:
            print(f"[CARLA] Đang kết nối CARLA Server tại {h}:{port}...")
            c = carla.Client(h, port)
            c.set_timeout(10.0)
            world = c.get_world()
            carla_map = world.get_map()
            print(f"[CARLA] Kết nối thành công tới {h}:{port}!")
            break
        except Exception:
            continue

    if carla_map is None:
        raise ConnectionError(f"Không thể kết nối đến CARLA Server tại các địa chỉ {hosts_to_try}:{port}!")

    print("[CARLA] Đang trích xuất dữ liệu bản đồ 2D (Tâm các làn đường)...")
    waypoints = carla_map.generate_waypoints(distance=2.0)
    carla_y_data = [w.transform.location.y for w in waypoints]
    carla_x_data = [w.transform.location.x for w in waypoints]

    _cached_carla_road_data = (carla_y_data, carla_x_data)
    return _cached_carla_road_data


class PathAnchorPicker:
    """
    CỬA SỔ 1: Chọn các điểm neo đường đi (Snap tự động vào tâm đường ô xám)
    """
    def __init__(self, carla_y_data, carla_x_data):
        self.clicked_points = []
        self.carla_y_data = carla_y_data
        self.carla_x_data = carla_x_data

        self.fig, self.ax = plt.subplots(figsize=(11, 8.5))
        self.ax.scatter(carla_y_data, carla_x_data, s=1.5, c='lightgray', label='Road Network (Bản đồ đường)')
        self.ax.set_title("BƯỚC 1: CHỌN ĐƯỜNG ĐI (Click Chuột Trái = Chấm điểm Xanh lá | Chuột Phải = Undo | ENTER = Chốt đường)",
                          fontsize=11, fontweight='bold', color='darkgreen')
        self.ax.set_xlabel("Trục Ngang (CARLA Y)")
        self.ax.set_ylabel("Trục Dọc (CARLA X)")
        self.ax.axis('equal')
        self.ax.grid(True, linestyle='--', alpha=0.6)

        self.path_line, = self.ax.plot([], [], 'go-', markersize=6, linewidth=2.0, label='Đường xe đi (Path Anchors)')
        self.ax.legend(loc='upper right')

        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

    def on_click(self, event):
        if event.inaxes != self.ax or self.fig.canvas.toolbar.mode != "":
            return

        click_y, click_x = event.xdata, event.ydata

        # Chuột trái: Chọn điểm neo (Snap vào điểm xám gần nhất)
        if event.button == 1:
            min_dist = float('inf')
            closest_y, closest_x = click_y, click_x
            for y, x in zip(self.carla_y_data, self.carla_x_data):
                dist = math.hypot(y - click_y, x - click_x)
                if dist < min_dist:
                    min_dist = dist
                    closest_y = y
                    closest_x = x

            if self.clicked_points and self.clicked_points[-1] == (closest_y, closest_x):
                return
            self.clicked_points.append((closest_y, closest_x))
            self.update_plot()

        # Chuột phải: Xóa điểm vừa chọn
        elif event.button == 3:
            if self.clicked_points:
                self.clicked_points.pop()
                self.update_plot()

    def on_key(self, event):
        if event.key == 'enter':
            if len(self.clicked_points) < 2:
                print("[CẢNH BÁO] Cần chọn ít nhất 2 điểm trước khi chốt đường đi!")
                return
            plt.close(self.fig)

    def update_plot(self):
        if self.clicked_points:
            ys, xs = zip(*self.clicked_points)
            self.path_line.set_data(ys, xs)
        else:
            self.path_line.set_data([], [])
        self.fig.canvas.draw_idle()

    def get_anchors(self):
        plt.show()
        x_anchors = [round(float(pt[1]), 2) for pt in self.clicked_points]
        y_anchors = [round(float(pt[0]), 2) for pt in self.clicked_points]
        return x_anchors, y_anchors


class ObstaclePicker:
    """
    CỬA SỔ 2: Hiển thị toàn bộ Quỹ đạo PCHIP mượt mà và cho phép click đặt vật cản cố định
    """
    def __init__(self, carla_y_data, carla_x_data, x_traj, y_traj, theta_traj=None):
        self.carla_y_data = carla_y_data
        self.carla_x_data = carla_x_data
        self.x_traj = x_traj
        self.y_traj = y_traj
        self.theta_traj = theta_traj
        self.clicked_obstacles = []

        self.fig, self.ax = plt.subplots(figsize=(11, 8.5))
        # 1. Vẽ mạng lưới đường nền
        self.ax.scatter(carla_y_data, carla_x_data, s=1.5, c='lightgray', label='Road Network')

        # 2. Vẽ toàn bộ đường cong quỹ đạo PCHIP mịn màng (màu xanh lá)
        self.ax.plot(y_traj, x_traj, 'g-', linewidth=2.5, label='Quỹ đạo xe chạy (PCHIP Reference)')

        # Đánh dấu điểm bắt đầu (Start Point)
        self.ax.scatter([y_traj[0]], [x_traj[0]], s=120, c='blue', marker='o', edgecolors='black', linewidth=1.5,
                        label=f'Điểm bắt đầu (Start: {x_traj[0]:.1f}, {y_traj[0]:.1f})', zorder=5)

        # Vẽ một số mũi tên chỉ hướng dọc theo quỹ đạo
        step = max(len(x_traj) // 15, 1)
        for i in range(0, len(x_traj) - step // 2, step):
            dy = y_traj[i + step // 2] - y_traj[i]
            dx = x_traj[i + step // 2] - x_traj[i]
            self.ax.arrow(y_traj[i], x_traj[i], dy * 0.7, dx * 0.7,
                          head_width=1.0, head_length=1.5, fc='darkgreen', ec='darkgreen', zorder=4)

        # 3. Layer hiển thị các vật cản (khối vuông màu đỏ)
        self.obstacle_scatter = self.ax.scatter([], [], s=100, c='red', marker='s', edgecolors='black',
                                                linewidth=1.5, label=f'Vật cản ({OBSTACLE_SIZE}x{OBSTACLE_SIZE}m)', zorder=6)

        self.ax.set_title("BƯỚC 2: CHỌN VẬT CẢN TRÊN QUỸ ĐẠO (Click Trái = Đặt vật cản Đỏ | Click Phải = Xóa | ENTER = Hoàn tất)",
                          fontsize=11, fontweight='bold', color='darkred')
        self.ax.set_xlabel("Trục Ngang (CARLA Y)")
        self.ax.set_ylabel("Trục Dọc (CARLA X)")
        self.ax.axis('equal')
        self.ax.grid(True, linestyle='--', alpha=0.6)
        self.ax.legend(loc='upper right')

        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

    def on_click(self, event):
        if event.inaxes != self.ax or self.fig.canvas.toolbar.mode != "":
            return

        click_y, click_x = event.xdata, event.ydata

        # Chuột trái: Đặt vật cản tại đúng vị trí click
        if event.button == 1:
            self.clicked_obstacles.append((click_y, click_x))
            self.update_plot()
            print(f"  [+] Đã chọn vật cản #{len(self.clicked_obstacles)} tại CARLA Pos: (x={click_x:.2f}, y={click_y:.2f})")

        # Chuột phải: Xóa vật cản vừa đặt (Undo)
        elif event.button == 3:
            if self.clicked_obstacles:
                removed = self.clicked_obstacles.pop()
                self.update_plot()
                print(f"  [-] Đã xóa vật cản gần nhất tại (x={removed[1]:.2f}, y={removed[0]:.2f})")

    def on_key(self, event):
        if event.key == 'enter':
            plt.close(self.fig)

    def update_plot(self):
        if self.clicked_obstacles:
            oys, oxs = zip(*self.clicked_obstacles)
            self.obstacle_scatter.set_offsets(list(zip(oys, oxs)))
        else:
            self.obstacle_scatter.set_offsets(np.empty((0, 2)))
        self.fig.canvas.draw_idle()

    def get_obstacles(self):
        plt.show()
        obstacles = [{'x': round(float(pt[1]), 2), 'y': round(float(pt[0]), 2)} for pt in self.clicked_obstacles]
        return obstacles


def get_path_anchors(host=CARLA_HOST, port=CARLA_PORT):
    """
    BƯỚC 1: Mở Cửa sổ 1 chọn các điểm neo đường đi (Xanh lá)
    """
    carla_y_data, carla_x_data = get_carla_road_network(host=host, port=port)

    print("\n================ [BƯỚC 1: CHỌN ĐƯỜNG ĐI TRÊN BẢN ĐỒ 2D] ================")
    print("  - Click chuột trái: Chọn các điểm xe sẽ đi (Tự động bắt vào tâm đường ô xám).")
    print("  - Click chuột phải: Xóa điểm vừa chọn (Undo).")
    print("  - Nhấn phím ENTER: Chốt đường đi -> Hệ thống sẽ tính quỹ đạo mượt và mở Cửa sổ 2.")
    print("========================================================================\n")

    picker = PathAnchorPicker(carla_y_data, carla_x_data)
    x_anchors, y_anchors = picker.get_anchors()
    return x_anchors, y_anchors


def pick_obstacles_on_trajectory(x_traj, y_traj, theta_traj=None, host=CARLA_HOST, port=CARLA_PORT):
    """
    BƯỚC 2: Mở Cửa sổ 2 MỚI hiển thị toàn bộ quỹ đạo PCHIP mịn màng để chọn vật cản
    """
    carla_y_data, carla_x_data = get_carla_road_network(host=host, port=port)

    print("\n================ [BƯỚC 2: CHỌN VẬT CẢN TRÊN QUỸ ĐẠO MƯỢT] ================")
    print("  - Quỹ đạo PCHIP mượt mà (màu xanh lá) đã được tính toán và hiển thị trên bản đồ.")
    print("  - Click chuột trái: Đặt một hoặc nhiều vật cản (màu đỏ) trên hoặc cạnh đường đi.")
    print("  - Click chuột phải: Xóa vật cản vừa đặt (Undo).")
    print("  - Nhấn phím ENTER: Hoàn tất toàn bộ và xuất dữ liệu.")
    print("=========================================================================\n")

    picker = ObstaclePicker(carla_y_data, carla_x_data, x_traj, y_traj, theta_traj)
    obstacles = picker.get_obstacles()
    return obstacles


def get_2d_map_and_ping_waypoints(host=CARLA_HOST, port=CARLA_PORT):
    """Tương thích ngược với các file cũ"""
    return get_path_anchors(host=host, port=port)
