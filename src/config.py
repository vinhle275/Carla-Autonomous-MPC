import os
import numpy as np

# Thư mục lưu trữ quỹ đạo và biểu đồ kết quả
TRAJECTORY_CSV_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../data/reference_trajectory.csv")
)
PLOTS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../plots")
)

# --- VẬN TỐC THAM CHIẾU MONG MUỐN ---
TARGET_SPEED_MS = 10.0       # Vận tốc mục tiêu (m/s) -> 2 m/s
OVERRIDE_V_REF = True        # Tự động nâng tốc độ trong CSV lên TARGET_SPEED_MS nhưng vẫn giữ giảm tốc an toàn khi vào cua

# Vehicle characteristics (from CARLA simulator)
MAX_ACCELERATION_M_S_2 = 10.0
MAX_BRAKING_M_S_2 = -4.1
MAX_WHEEL_ANGLE_RAD = 70.0 / 180.0 * np.pi
L = 2.875  # Wheelbase length of vehicle in meters

### MPC Settings
N = 10      # Horizon length
dt = 0.1    # Sample time in seconds (10Hz)

MAX_CONTROL_WHEEL_ANGLE_RAD = 70.0 / 180.0 * np.pi
MAX_CONTROL_ACCELERATION_M_S_2 = 10.0
MAX_CONTROL_BRAKING_M_S_2 = -4.1

# Trọng số tối ưu MPC (Triệt tiêu chao đảo lắc trái phải)
FINE_X_COEF = 20.0           # Phạt sai số X
FINE_Y_COEF = 20.0           # Phạt sai số Y
FINE_V_COEF = 15.0           # Phạt sai số vận tốc V
FINE_THETA_COEF = 30.0       # Phạt sai số góc hướng Theta
FINE_LATERAL_COEF = 80.0     # Phạt độ lệch ngang vuông góc với đường
FINE_STEER_COEF = 20.0       # Phạt sử dụng góc đánh lái lớn (Tránh đánh vô lăng quá đà)
FINE_ACC_COEF = 1.0          # Phạt sử dụng gia tốc quá lớn
FINE_STEER_DOT_COEF = 300.0  # Phạt tốc độ bẻ tay lái dồn dập (Triệt tiêu lắc trái/phải)
FINE_ACC_DOT_COEF = 10.0     # Phạt giật ga/phanh
