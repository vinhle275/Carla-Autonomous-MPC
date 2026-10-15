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
TARGET_SPEED_MS = 10.0       # Vận tốc mục tiêu (m/s) -> 2 m/s
OVERRIDE_V_REF = True        # Tự động nâng tốc độ trong CSV lên TARGET_SPEED_MS nhưng vẫn giữ giảm tốc an toàn khi vào cua

# Vehicle characteristics (from CARLA simulator)
MAX_ACCELERATION_M_S_2 = 10.0
MAX_BRAKING_M_S_2 = -4.1
MAX_WHEEL_ANGLE_RAD = 70.0 / 180.0 * np.pi
L = 2.875  # Wheelbase length of vehicle in meters

### MPC Settings
N = 15      # Horizon length (tăng lên 15 bước để xe nhìn thấy vật cản từ sớm hơn)
dt = 0.1    # Sample time in seconds (10Hz)

MAX_CONTROL_WHEEL_ANGLE_RAD = 70.0 / 180.0 * np.pi
MAX_CONTROL_ACCELERATION_M_S_2 = 10.0
MAX_CONTROL_BRAKING_M_S_2 = -4.1

# Trọng số tối ưu MPC
FINE_X_COEF = 10.0           # Phạt sai số X (giảm nhẹ để cho phép xe lệch tâm tránh vật cản)
FINE_Y_COEF = 10.0           # Phạt sai số Y (giảm nhẹ để cho phép xe lệch tâm tránh vật cản)
FINE_V_COEF = 10.0           # Phạt sai số vận tốc V
FINE_THETA_COEF = 150.0      # Phạt sai số góc hướng Theta (giảm bớt để xe đổi hướng lách vật cản)
FINE_LATERAL_COEF = 25.0     # Phạt độ lệch ngang (giảm từ 80 xuống 25 để không cản trở xe lách làn né)
FINE_STEER_COEF = 10.0       # Phạt sử dụng góc đánh lái lớn
FINE_ACC_COEF = 1.0          # Phạt sử dụng gia tốc quá lớn
FINE_STEER_DOT_COEF = 100.0  # Phạt tốc độ bẻ tay lái dồn dập
FINE_ACC_DOT_COEF = 10.0     # Phạt giật ga/phanh

# Hệ số né vật cản hàm mũ (Exponential Obstacle Avoidance Penalty)
FINE_OBS_COEF = 8000.0       # Trọng số phạt vật cản W_obs (đủ lớn để lực đẩy thắng lực kéo bám đường)
OBS_SAFETY_MARGIN_X = 2.0    # Bán kính an toàn theo phương dọc (m)
OBS_SAFETY_MARGIN_Y = 1.8    # Bán kính an toàn theo phương ngang (m)
OBS_EXP_SCALE = 1.0          # Hệ số suy giảm c trong hàm mũ exp(-c * d^2)
