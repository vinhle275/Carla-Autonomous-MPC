import glob
import os
import sys
import math
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
from src.config import CARLA_HOST, CARLA_PORT

class WaypointPicker:
    def __init__(self, carla_y_data, carla_x_data, invert_y=False, invert_x=False):
        self.clicked_points = []
        self.carla_y_data = carla_y_data
        self.carla_x_data = carla_x_data
        
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.scatter(carla_y_data, carla_x_data, s=1, c='gray', label='Road Network (Tâm đường)')
        self.ax.set_title("Bản đồ 2D MPC - Chọn điểm neo (Tự động bắt vào tâm đường ô xám)")
        self.ax.set_xlabel("Trục Ngang (CARLA Y)")
        self.ax.set_ylabel("Trục Dọc (CARLA X)")
        self.ax.axis('equal')
        self.ax.grid(True)
        
        if invert_y:
            self.ax.invert_yaxis()
        if invert_x:
            self.ax.invert_xaxis()
        
        self.line, = self.ax.plot([], [], 'ro-', markersize=5, linewidth=1.5, label='Selected Path')
        self.ax.legend(loc='upper right')

        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

    def on_click(self, event):
        if event.inaxes != self.ax:
            return
            
        if self.fig.canvas.toolbar.mode != "":
            return
            
        # Chuột trái (Button 1): Chấm điểm neo (TỰ ĐỘNG BẮT ĐIỂM VÀO TÂM ĐƯỜNG Ô XÁM GẦN NHẤT)
        if event.button == 1:
            click_y, click_x = event.xdata, event.ydata
            
            # Quét tìm điểm xám gần vị trí click nhất
            min_dist = float('inf')
            closest_y, closest_x = click_y, click_x
            for y, x in zip(self.carla_y_data, self.carla_x_data):
                dist = math.hypot(y - click_y, x - click_x)
                if dist < min_dist:
                    min_dist = dist
                    closest_y = y
                    closest_x = x
                    
            # Tránh double-click tạo ra 2 điểm trùng nhau liên tiếp
            if self.clicked_points and self.clicked_points[-1] == (closest_y, closest_x):
                return
            
            self.clicked_points.append((closest_y, closest_x))
            self.update_plot()
            
        # Chuột phải (Button 3): Xóa điểm neo vừa chọn (Undo)
        elif event.button == 3:
            if self.clicked_points:
                self.clicked_points.pop()
                self.update_plot()

    def on_key(self, event):
        if event.key == 'enter':
            plt.close(self.fig)

    def update_plot(self):
        if self.clicked_points:
            xs, ys = zip(*self.clicked_points)
            self.line.set_data(xs, ys)
        else:
            self.line.set_data([], [])
        self.fig.canvas.draw_idle()

    def get_points(self):
        plt.show()
        return self.clicked_points

def get_2d_map_and_ping_waypoints(host=CARLA_HOST, port=CARLA_PORT):
    client = None
    hosts_to_try = [host, 'localhost', '127.0.0.1']
    carla_map = None

    for h in hosts_to_try:
        try:
            print(f"[CARLA] Đang thử kết nối CARLA Server tại {h}:{port}...")
            c = carla.Client(h, port)
            c.set_timeout(10.0)
            world = c.get_world()
            carla_map = world.get_map()
            client = c
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

    print("\n================ [HƯỚNG DẪN BẢN ĐỒ 2D MPC] ================")
    print("- Click chuột trái: Điểm neo TỰ ĐỘNG BẮT VÀO TÂM ĐƯỜNG (Ô XÁM).")
    print("- Click chuột phải: Xóa điểm vừa chọn (Undo).")
    print("- Bấm phím ENTER: Hoàn tất và xuất tọa độ.")
    print("============================================================\n")

    picker = WaypointPicker(carla_y_data, carla_x_data)
    clicked_points = picker.get_points()

    x_anchors = [round(float(y), 2) for (x, y) in clicked_points]
    y_anchors = [round(float(x), 2) for (x, y) in clicked_points]
    
    print(f"[XONG] Đã chọn {len(x_anchors)} điểm neo (Đã chuẩn hóa về tâm đường).")
    return x_anchors, y_anchors
