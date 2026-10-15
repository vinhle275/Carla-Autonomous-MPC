import casadi as ca
import numpy as np
from src.help_functions import calculate_lateral_deviation
from src.config import (
    MAX_CONTROL_WHEEL_ANGLE_RAD, MAX_CONTROL_ACCELERATION_M_S_2, MAX_CONTROL_BRAKING_M_S_2,
    FINE_X_COEF, FINE_Y_COEF, FINE_V_COEF, FINE_STEER_COEF, FINE_ACC_COEF,
    FINE_STEER_DOT_COEF, FINE_ACC_DOT_COEF, FINE_THETA_COEF, FINE_LATERAL_COEF,
    FINE_OBS_COEF, OBS_SAFETY_MARGIN_X, OBS_SAFETY_MARGIN_Y, OBS_EXP_SCALE
)
from src.vehicle_model import vehicle_model

class MpcController:
    def __init__(self, horizon=10, dt=0.1):
        self.horizon = horizon
        self.dt = dt
        self.opti = None
        self.sol = None
        self.cost = None
        self.is_success = False
        self.X = None
        self.U = None
        self.prev_X = None
        self.prev_U = None
        self.control_buffer = {"acceleration": [0.0] * self.horizon, "wheel_angle": [0.0] * self.horizon}
        self.buffer_index = 0

    def reset_solver(self):
        self.opti = ca.Opti()
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_cpu_time': self.dt
        }
        self.opti.solver('ipopt', opts)

        self.X = self.opti.variable(4, self.horizon + 1)  # State: [x, y, theta, v]
        self.U = self.opti.variable(2, self.horizon)      # Control: [delta, a]

        # System dynamics and state/control constraints
        for k in range(self.horizon):
            self.opti.subject_to(self.X[:, k + 1] == vehicle_model(self.X[:, k], self.U[:, k], self.dt))
            self.opti.subject_to(self.U[0, k] <= MAX_CONTROL_WHEEL_ANGLE_RAD)
            self.opti.subject_to(self.U[0, k] >= -MAX_CONTROL_WHEEL_ANGLE_RAD)
            self.opti.subject_to(self.U[1, k] <= MAX_CONTROL_ACCELERATION_M_S_2)
            self.opti.subject_to(self.U[1, k] >= MAX_CONTROL_BRAKING_M_S_2)

    def set_init_vehicle_state(self, x, y, theta, v):
        self.opti.subject_to(self.X[:, 0] == ca.vertcat(x, y, theta, v))

    def update_cost_function(self, x_ref, y_ref, theta_ref, v_ref, obstacles=None):
        self.cost = 0
        if obstacles is None:
            obstacles = []
        
        # Khởi tạo nghiệm ban đầu (Warm Start):
        if self.prev_X is not None and self.is_success:
            for k in range(self.horizon):
                next_k = min(k + 1, self.horizon)
                self.opti.set_initial(self.X[:, k], self.prev_X[:, next_k])
                self.opti.set_initial(self.U[:, k], self.prev_U[:, min(k + 1, self.horizon - 1)])
            v_end = v_ref[-1] if hasattr(v_ref, '__len__') else v_ref
            self.opti.set_initial(self.X[0, self.horizon], x_ref[-1])
            self.opti.set_initial(self.X[1, self.horizon], y_ref[-1])
            self.opti.set_initial(self.X[2, self.horizon], theta_ref[-1])
            self.opti.set_initial(self.X[3, self.horizon], v_end)
        else:
            for k in range(self.horizon + 1):
                idx = min(k, len(x_ref) - 1)
                v_k = v_ref[idx] if hasattr(v_ref, '__len__') else v_ref
                self.opti.set_initial(self.X[0, k], x_ref[idx])
                self.opti.set_initial(self.X[1, k], y_ref[idx])
                self.opti.set_initial(self.X[2, k], theta_ref[idx])
                self.opti.set_initial(self.X[3, k], v_k)

        for k in range(self.horizon):
            v_k = v_ref[k] if hasattr(v_ref, '__len__') else v_ref

            # 1. Sai số vị trí X và Y (Hàm trơn nhẵn Quadratic)
            fine_x = FINE_X_COEF * (self.X[0, k] - x_ref[k]) ** 2
            fine_y = FINE_Y_COEF * (self.X[1, k] - y_ref[k]) ** 2

            # 2. Sai số vận tốc V
            fine_v = FINE_V_COEF * (self.X[3, k] - v_k) ** 2

            # 3. Sai số góc hướng Theta (Dùng trực tiếp góc unwrapped continuous)
            fine_theta = FINE_THETA_COEF * (self.X[2, k] - theta_ref[k]) ** 2

            # 4. Độ lệch ngang vuông góc với quỹ đạo (Lateral Deviation)
            if k < self.horizon - 1:
                lateral_dev = calculate_lateral_deviation(
                    self.X[0, k], self.X[1, k],
                    x_ref[k], y_ref[k],
                    x_ref[k + 1], y_ref[k + 1]
                )
            else:
                lateral_dev = 0.0

            fine_lateral = FINE_LATERAL_COEF * (lateral_dev ** 2)

            # 5. Phạt biên độ điều khiển
            fine_steer = FINE_STEER_COEF * (self.U[0, k] ** 2)
            fine_acc = FINE_ACC_COEF * (self.U[1, k] ** 2)

            # 6. Phạt tốc độ thay đổi góc lái & gia tốc (Smooth control action)
            fine_steer_dot = 0
            fine_acc_dot = 0
            if k > 0:
                fine_steer_dot = FINE_STEER_DOT_COEF * ((self.U[0, k] - self.U[0, k - 1]) ** 2)
                fine_acc_dot = FINE_ACC_DOT_COEF * ((self.U[1, k] - self.U[1, k - 1]) ** 2)

            # 7. Hàm phạt né vật cản hàm mũ (Exponential Penalty Obstacle Avoidance)
            # Dạng hàm: W_obs * exp(-c * d_norm^2)
            fine_obs = 0
            for obs in obstacles:
                # Dự báo vị trí vật cản tại bước k (hỗ trợ cả vật cản di chuyển vx, vy)
                obs_vx = obs.get('vx', 0.0)
                obs_vy = obs.get('vy', 0.0)
                obs_x_k = obs['x'] + k * self.dt * obs_vx
                obs_y_k = obs['y'] + k * self.dt * obs_vy

                dx = self.X[0, k] - obs_x_k
                dy = self.X[1, k] - obs_y_k

                # Nếu vật cản có góc hướng (dynamic vehicle), xoay theo hệ tọa độ của vật cản
                if 'yaw_rad' in obs and abs(obs.get('speed', 0.0)) > 0.1:
                    cos_th = np.cos(obs['yaw_rad'])
                    sin_th = np.sin(obs['yaw_rad'])
                    dx_local = cos_th * dx + sin_th * dy
                    dy_local = -sin_th * dx + cos_th * dy
                else:
                    dx_local = dx
                    dy_local = dy

                # Bán kính an toàn Elip (kích thước vật cản + khoảng đệm an toàn)
                obs_size = obs.get('size', 0.5)
                a = (obs_size / 2.0) + OBS_SAFETY_MARGIN_X
                b = (obs_size / 2.0) + OBS_SAFETY_MARGIN_Y

                # Khoảng cách chuẩn hóa dạng Elip
                d_norm_sq = (dx_local / a) ** 2 + (dy_local / b) ** 2

                # Hàm phạt hàm mũ (Exponential/Gaussian Potential Field)
                fine_obs += FINE_OBS_COEF * ca.exp(-OBS_EXP_SCALE * d_norm_sq)

            self.cost += (fine_x + fine_y + fine_v + fine_steer + fine_acc +
                          fine_steer_dot + fine_acc_dot + fine_theta + fine_lateral + fine_obs)

        self.opti.minimize(self.cost)

    def solve(self):
        try:
            self.sol = self.opti.solve()
            self.is_success = True

            wheel_angle_rad, acceleration_m_s_2 = float(self.sol.value(self.U[0, 0])), float(self.sol.value(self.U[1, 0]))
            self.prev_X = np.array(self.sol.value(self.X))
            self.prev_U = np.array(self.sol.value(self.U))

            self.control_buffer["acceleration"] = self.control_buffer["acceleration"][1:] + [acceleration_m_s_2]
            self.control_buffer["wheel_angle"] = self.control_buffer["wheel_angle"][1:] + [wheel_angle_rad]
            self.buffer_index = 0
        except Exception as e:
            try:
                wheel_angle_rad = float(self.opti.debug.value(self.U[0, 0]))
                acceleration_m_s_2 = float(self.opti.debug.value(self.U[1, 0]))
                self.prev_X = np.array(self.opti.debug.value(self.X))
                self.prev_U = np.array(self.opti.debug.value(self.U))

                self.control_buffer["acceleration"] = self.control_buffer["acceleration"][1:] + [acceleration_m_s_2]
                self.control_buffer["wheel_angle"] = self.control_buffer["wheel_angle"][1:] + [wheel_angle_rad]
                self.buffer_index = 0
                self.is_success = True
            except Exception:
                print(f"[MPC WARNING] IPOPT calculation failed: {e}. Using buffered control step.")
                self.is_success = False

    def get_controls_value(self):
        if self.is_success and len(self.control_buffer["wheel_angle"]) > 0:
            wheel_angle_rad = self.control_buffer["wheel_angle"][-1]
            acceleration_m_s_2 = self.control_buffer["acceleration"][-1]
        else:
            if self.buffer_index < self.horizon:
                acceleration_m_s_2 = self.control_buffer["acceleration"][self.buffer_index]
                wheel_angle_rad = self.control_buffer["wheel_angle"][self.buffer_index]
                self.buffer_index += 1
            else:
                acceleration_m_s_2 = 0.0
                wheel_angle_rad = 0.0

        return wheel_angle_rad, acceleration_m_s_2

    def get_optimized_cost(self):
        if self.is_success and self.sol:
            try:
                return float(self.sol.value(self.cost))
            except Exception:
                return 0.0
        return 0.0
