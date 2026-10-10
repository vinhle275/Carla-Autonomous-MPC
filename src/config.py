import os
import numpy as np

# Đường dẫn đến file CSV lưu trữ quỹ đạo tham chiếu trong thư mục data/ của dự án
TRAJECTORY_CSV_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../data/reference_trajectory.csv")
)

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
PATH_TOLERANCE_M = 0.5

# MPC Penalty Weights (Cost function)
FINE_X_COEF = 15.0
FINE_Y_COEF = 15.0
FINE_STEER_COEF = 1.0
FINE_ACC_COEF = 0.5
FINE_STEER_DOT_COEF = 100.0
FINE_ACC_DOT_COEF = 5.0
FINE_V_COEF = 25.0
FINE_THETA_COEF = 15.0
FINE_LATERAL_COEF = 50.0
