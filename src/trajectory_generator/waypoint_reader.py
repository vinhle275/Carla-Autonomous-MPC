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

class PathAndObstaclePicker:
    def __init__(self, carla_y_data, carla_x_data, invert_y=False, invert_x=False):
        self.clicked_path_points = []
        self.clicked_obstacle_points = []
        self.carla_y_data = carla_y_data
        self.carla_x_data = carla_x_data
        
        # Mode: 'path' (giai đoạn 1) hoặc 'obstacle' (giai đoạn 2)
        self.mode = 'path'
        
        self.fig, self.ax = plt.subplots(figsize=(11, 8.5))
        self.ax.scatter(carla_y_data, carla_x_data, s=1, c='lightgray', label='Road Network (Bản đồ đường)')
        self.ax.set_xlabel("Trục Ngang (CARLA Y)")
        self.ax.set_ylabel("Trục Dọc (CARLA X)")
        self.ax.axis('equal')
        self.ax.grid(True, linestyle='--', alpha=0.6)
        
        if invert_y:
            self.ax.invert_yaxis()
        if invert_x:
            self.ax.invert_xaxis()
        
        # Đường xe đi: Xanh lá (Green)
        self.path_line, = self.ax.plot([], [], 'go-', markersize=5, linewidth=2.0, label='Đường xe đi (Path - Xanh lá)')
        # Điểm vật cản: Đỏ (Red)
        self.obstacle_scatter = self.ax.scatter([], [], s=80, c='red', marker='s', edgecolors='black', linewidth=1.5, label='Vật cản (Obstacles - 0.5x0.5m)')
        
        self.ax.legend(loc='upper right')
        self.update_title()

        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

    def update_title(self):
        if self.mode == 'path':
            self.ax.set_title("BƯỚC 1: CHỌN ĐƯỜNG ĐI (Click Chuột Trái = Chấm đường Xanh lá | Chuột Phải = Xóa | ENTER = Chốt đường)", 
                              fontsize=11, fontweight='bold', color='darkgreen')
        else:
            self.ax.set_title("BƯỚC 2: CHỌN VẬT CẢN (Click Chuột Trái = Đặt vật cản Đỏ 0.5x0.5m | Chuột Phải = Xóa | ENTER = Hoàn tất)", 
                              fontsize=11, fontweight='bold', color='darkred')
        self.fig.canvas.draw_idle()

    def on_click(self, event):
        if event.inaxes != self.ax:
            return
            
        if self.fig.canvas.toolbar.mode != "":
            return
            
        click_y, click_x = event.xdata, event.ydata

        # Chuột trái: Thêm điểm
        if event.button == 1:
            if self.mode == 'path':
                # Chế độ chọn đường: Snap vào tâm đường gần nhất
                min_dist = float('inf')
                closest_y, closest_x = click_y, click_x
                for y, x in zip(self.carla_y_data, self.carla_x_data):
                    dist = math.hypot(y - click_y, x - click_x)
                    if dist < min_dist:
                        min_dist = dist
                        closest_y = y
                        closest_x = x
                        
                if self.clicked_path_points and self.clicked_path_points[-1] == (closest_y, closest_x):
                    return
                self.clicked_path_points.append((closest_y, closest_x))
            else:
                # Chế độ chọn vật cản: Tọa độ tự do chính xác tại vị trí click (hoặc trên đường)
                self.clicked_obstacle_points.append((click_y, click_x))

            self.update_plot()
            
        # Chuột phải: Xóa điểm vừa chọn (Undo)
        elif event.button == 3:
            if self.mode == 'path':
                if self.clicked_path_points:
                    self.clicked_path_points.pop()
            else:
                if self.clicked_obstacle_points:
                    self.clicked_obstacle_points.pop()
            self.update_plot()

    def on_key(self, event):
        if event.key == 'enter':
            if self.mode == 'path':
                if len(self.clicked_path_points) < 2:
                    print("[CẢNH BÁO] Cần chọn ít nhất 2 điểm đường đi trước khi chuyển sang chọn vật cản!")
                    return
                # Chuyển từ Bước 1 sang Bước 2
                self.mode = 'obstacle'
                print("\n>>> ĐÃ CHỐT ĐƯỜNG ĐI! CHUYỂN SANG BƯỚC 2: CHỌN VẬT CẢN (MÀU ĐỎ) <<<")
                print("- Click chuột trái vào bản đồ để đặt một hoặc nhiều vật cản (màu đỏ).")
                print("- Bấm ENTER lần nữa để hoàn tất toàn bộ.\n")
                self.update_title()
            else:
                # Hoàn tất cả 2 bước
                plt.close(self.fig)

    def update_plot(self):
        # 1. Cập nhật đường xe đi (xanh lá): Nối liền mạch ngay lập tức giữa các điểm
        if self.clicked_path_points:
            xs, ys = zip(*self.clicked_path_points)
            self.path_line.set_data(xs, ys)
        else:
            self.path_line.set_data([], [])

        # 2. Cập nhật vật cản (đỏ): Hiển thị ngay lập tức các khối đỏ
        if self.clicked_obstacle_points:
            oxs, oys = zip(*self.clicked_obstacle_points)
            self.obstacle_scatter.set_offsets(list(zip(oxs, oys)))
        else:
            self.obstacle_scatter.set_offsets(np.empty((0, 2)))

        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def get_data(self):
        plt.show()
        return self.clicked_path_points, self.clicked_obstacle_points

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

    print("\n================ [HƯỚNG DẪN CHỌN ĐƯỜNG ĐI & VẬT CẢN 2D] ================")
    print("▶ BƯỚC 1 - CHỌN ĐƯỜNG ĐI (MÀU XANH LÁ):")
    print("  - Click chuột trái: Chọn các điểm xe sẽ đi (Tự động bắt vào tâm đường ô xám).")
    print("  - Click chuột phải: Xóa điểm vừa chọn (Undo).")
    print("  - Nhấn phím ENTER: Chốt đường đi và chuyển sang chọn vật cản.")
    print("▶ BƯỚC 2 - CHỌN VẬT CẢN (MÀU ĐỎ):")
    print("  - Click chuột trái: Chọn một hoặc nhiều vị trí vật cản (kích thước 0.5x0.5m).")
    print("  - Click chuột phải: Xóa vật cản vừa đặt (Undo).")
    print("  - Nhấn phím ENTER: Hoàn tất và lưu toàn bộ dữ liệu.")
    print("========================================================================\n")

    picker = PathAndObstaclePicker(carla_y_data, carla_x_data)
    path_points, obstacle_points = picker.get_data()

    # Chuyển đổi hệ tọa độ hiển thị (Y, X) về hệ tọa độ CARLA (X, Y)
    x_anchors = [round(float(pt[1]), 2) for pt in path_points]
    y_anchors = [round(float(pt[0]), 2) for pt in path_points]

    obstacles = [{'x': round(float(pt[1]), 2), 'y': round(float(pt[0]), 2)} for pt in obstacle_points]

    print(f"[XONG] Đã chọn {len(x_anchors)} điểm đường đi và {len(obstacles)} vật cản.")
    return (x_anchors, y_anchors), obstacles

def get_2d_map_and_ping_path_and_obstacles(host=CARLA_HOST, port=CARLA_PORT):
    return get_2d_map_and_ping_waypoints(host=host, port=port)
