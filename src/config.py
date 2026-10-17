import os
import numpy as np

# Thư mục lưu trữ quỹ đạo và biểu đồ kết quả
TRAJECTORY_CSV_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../data/reference_trajectory.csv")
)
OBSTACLES_CSV_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../data/obstacles.csv")
)
PLOTS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../plots")
)

# Kích thước vật cản (0.5m x 0.5m)
OBSTACLE_SIZE = 0.5

# --- CẤU HÌNH KẾT NỐI CARLA SERVER ---
CARLA_HOST = '172.20.240.1'    # Đổi thành '127.0.0.1' hoặc IP máy chủ CARLA nếu chạy WSL2/máy khác (vd: '172.20.240.1')
CARLA_PORT = 2000

# --- VẬN TỐC THAM CHIẾU MONG MUỐN ---
TARGET_SPEED_MS = 10.0        # Vận tốc mục tiêu đường thẳng (m/s) (~28.8 km/h)
OVERRIDE_V_REF = True        # Tự động nâng tốc độ trong CSV lên TARGET_SPEED_MS nhưng vẫn giữ giảm tốc an toàn khi vào cua

# --- CẤU HÌNH TỰ ĐỘNG GIẢM TỐC KHI GẶP VẬT CẢN ---
ENABLE_OBSTACLE_SLOWDOWN = True  # Bật tính năng chủ động giảm tốc khi gặp vật cản
OBSTACLE_SLOWDOWN_DIST = 10.0    # Bán kính phát hiện vật cản để giảm tốc (mét)
OBSTACLE_SLOWDOWN_SPEED = 4.0    # Vận tốc an toàn khi né vật cản (m/s) (~12.6 km/h)

# --- CẤU HÌNH GOM CỤM VẬT CẢN (CLUSTERING) ---
ENABLE_OBSTACLE_CLUSTERING = True   # Bật/tắt tính năng gom cụm các vật cản gần nhau
CLUSTER_DISTANCE_THRESHOLD = 1.5    # Khoảng cách tối đa giữa các vật cản để gộp thành 1 khối (mét)

# Vehicle characteristics (from CARLA simulator)
MAX_ACCELERATION_M_S_2 = 4.0
MAX_BRAKING_M_S_2 = -4.1
MAX_WHEEL_ANGLE_RAD = 70.0 / 180.0 * np.pi
L = 2.875  # Wheelbase length of vehicle in meters

### MPC Settings
N = 18      # Horizon length (tăng lên 18 bước để xe nhìn thấy vật cản và lập kế hoạch từ sớm hơn)
dt = 0.1    # Sample time in seconds (10Hz)

MAX_CONTROL_WHEEL_ANGLE_RAD = 70.0 / 180.0 * np.pi
MAX_CONTROL_ACCELERATION_M_S_2 = 4.0   # Giảm từ 10.0 xuống 4.0 để tránh thốc ga mạnh
MAX_CONTROL_BRAKING_M_S_2 = -4.1

# Trọng số tối ưu MPC
FINE_X_COEF = 8.0            # Phạt sai số X (giảm nhẹ để cho phép xe lệch tâm tránh vật cản)
FINE_Y_COEF = 8.0            # Phạt sai số Y (giảm nhẹ để cho phép xe lệch tâm tránh vật cản)
FINE_V_COEF = 10.0           # Phạt sai số vận tốc V
FINE_THETA_COEF = 120.0      # Phạt sai số góc hướng Theta (giảm bớt để xe đổi hướng lách vật cản)
FINE_LATERAL_COEF = 20.0     # Phạt độ lệch ngang (giảm từ 25 xuống 20 để xe lách làn né êm hơn)
FINE_STEER_COEF = 12.0       # Phạt sử dụng góc đánh lái lớn
FINE_ACC_COEF = 2.0          # Phạt sử dụng gia tốc quá lớn
FINE_STEER_DOT_COEF = 220.0  # Tăng từ 100 lên 220: Phạt tốc độ bẻ tay lái dồn dập, giúp xe lượn mượt mà
FINE_ACC_DOT_COEF = 25.0     # Tăng từ 10 lên 25: Phạt giật ga/phanh

# Hệ số né vật cản hàm mũ (Exponential Obstacle Avoidance Penalty)
FINE_OBS_COEF = 7000.0       # Trọng số phạt vật cản W_obs
OBS_SAFETY_MARGIN_X = 5.0    # TĂNG TỪ 2.0 LÊN 5.0m: Bắt đầu tạo lực đẩy né vật cản từ khoảng cách xa
OBS_SAFETY_MARGIN_Y = 1.9    # Bán kính an toàn theo phương ngang (m)
OBS_EXP_SCALE = 0.6          # GIẢM TỪ 1.0 XUỐNG 0.6: Giúp độ dốc lực né thoai thoải, xe né nhẹ nhàng không bị giật

