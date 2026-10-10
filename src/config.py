import os
import numpy as np

# Đường dẫn đến file CSV lưu trữ quỹ đạo tham chiếu trong thư mục data/ của dự án
TRAJECTORY_CSV_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../data/reference_trajectory.csv")
)

# --- VẬN TỐC THAM CHIẾU MONG MUỐN ---
TARGET_SPEED_MS = 8.0        # Vận tốc mục tiêu (m/s) -> 8.0 m/s ≈ 28.8 km/h
OVERRIDE_V_REF = True        # Tự động nâng tốc độ trong CSV nhưng vẫn giữ giảm tốc an toàn khi vào cua

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

# Trọng số tối ưu MPC (Smooth Quadratic Penalties)
FINE_X_COEF = 30.0           # Phạt sai số X
FINE_Y_COEF = 30.0           # Phạt sai số Y
FINE_V_COEF = 20.0           # Phạt sai số vận tốc V
FINE_THETA_COEF = 25.0       # Phạt sai số góc hướng Theta
FINE_LATERAL_COEF = 100.0    # Phạt độ lệch ngang vuông góc với đường
FINE_STEER_COEF = 1.0        # Phạt sử dụng góc đánh lái quá lớn
FINE_ACC_COEF = 0.5          # Phạt sử dụng gia tốc quá lớn
FINE_STEER_DOT_COEF = 50.0   # Phạt đánh lái gắt (giúp lái xe êm mượt)
FINE_ACC_DOT_COEF = 5.0      # Phạt giật ga/phanh
