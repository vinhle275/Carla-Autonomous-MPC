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

class WaypointPicker:
    def __init__(self, carla_y_data, carla_x_data, invert_y=False, invert_x=False):
        self.clicked_points = []
        self.carla_y_data = carla_y_data
        self.carla_x_data = carla_x_data
        
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.scatter(carla_y_data, carla_x_data, s=1, c='gray', label='Road Network')
        self.ax.set_title("Bản đồ 2D MPC - Chọn điểm neo (Anchor Points)")
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
            
        # Chuột trái (Button 1): Chấm điểm neo
        if event.button == 1:
            click_y, click_x = event.xdata, event.ydata
            
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

def get_2d_map_and_ping_waypoints(host='172.20.240.1', port=2000):
    print("[CARLA] Đang kết nối với CARLA Server để đọc bản đồ...")
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    print("[CARLA] Đang trích xuất dữ liệu bản đồ 2D...")
    waypoints = carla_map.generate_waypoints(distance=2.0)
    
    carla_y_data = [w.transform.location.y for w in waypoints]
    carla_x_data = [w.transform.location.x for w in waypoints]

    print("\n================ [HƯỚNG DẪN BẢN ĐỒ 2D] ================")
    print("- Click chuột trái để chấm điểm neo.")
    print("- Click chuột phải để xóa điểm neo (Undo).")
    print("- Bấm phím ENTER để kết thúc chọn và xuất tọa độ.")
    print("========================================================\n")

    picker = WaypointPicker(carla_y_data, carla_x_data)
    clicked_points = picker.get_points()

    x_anchors = [round(float(y), 2) for (x, y) in clicked_points]
    y_anchors = [round(float(x), 2) for (x, y) in clicked_points]
    
    print(f"[XONG] Đã chọn {len(x_anchors)} điểm neo.")
    return x_anchors, y_anchors
