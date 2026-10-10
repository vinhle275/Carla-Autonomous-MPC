import numpy as np
from matplotlib import pyplot as plt
from src.config import dt

class Logger:
    def __init__(self):
        self.x_data = []
        self.y_data = []
        self.v_data = []
        self.theta_data = []
        self.x_ref_data = []
        self.y_ref_data = []
        self.v_ref_data = []
        self.theta_ref_data = []
        self.steering_data = []
        self.throttle_data = []
        self.brake_data = []

    def log_controller_input(self, x, y, v, theta, x_ref, y_ref, v_ref, theta_ref):
        self.x_data.append(x)
        self.y_data.append(y)
        self.v_data.append(v)
        self.theta_data.append(np.rad2deg(theta))
        self.x_ref_data.append(x_ref)
        self.y_ref_data.append(y_ref)
        self.v_ref_data.append(v_ref)
        self.theta_ref_data.append(np.rad2deg(theta_ref))

    def log_controller_output(self, steering, throttle, brake):
        self.steering_data.append(steering)
        self.throttle_data.append(throttle)
        self.brake_data.append(brake)

    def show_plots(self):
        min_length = min(
            len(self.x_data), len(self.y_data), len(self.v_data), len(self.theta_data),
            len(self.x_ref_data), len(self.y_ref_data), len(self.v_ref_data), len(self.theta_ref_data),
            len(self.steering_data), len(self.throttle_data), len(self.brake_data)
        )

        if min_length == 0:
            print("[LOGGER] Không có dữ liệu để hiển thị đồ thị.")
            return

        x_data = self.x_data[:min_length]
        y_data = self.y_data[:min_length]
        v_data = self.v_data[:min_length]
        theta_data = self.theta_data[:min_length]
        x_ref_data = self.x_ref_data[:min_length]
        y_ref_data = self.y_ref_data[:min_length]
        v_ref_data = self.v_ref_data[:min_length]
        theta_ref_data = self.theta_ref_data[:min_length]
        steering_data = self.steering_data[:min_length]
        throttle_data = self.throttle_data[:min_length]
        brake_data = self.brake_data[:min_length]

        time_axis = np.linspace(0, min_length * dt, min_length)

        # 1. Biểu đồ vị trí 2D (Path tracking evaluation)
        plt.figure(figsize=(10, 6))
        plt.plot(x_ref_data, y_ref_data, 'r--', label='Reference Path (001)')
        plt.plot(x_data, y_data, 'b-', label='Actual Vehicle Path')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.title('2D Path Tracking Performance (MPC vs 001 Path)')
        plt.grid(True)

        # 2. Vận tốc theo thời gian
        plt.figure(figsize=(10, 4))
        plt.plot(time_axis, v_data, label='Car Speed (v)')
        plt.plot(time_axis, v_ref_data, label='Reference Speed (v_ref)', linestyle='--')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.legend()
        plt.title('Speed Tracking')
        plt.grid(True)

        # 3. Tín hiệu điều khiển (Steering, Throttle, Brake)
        plt.figure(figsize=(10, 4))
        plt.plot(time_axis, steering_data, label='Steer Control')
        plt.plot(time_axis, throttle_data, label='Throttle')
        plt.plot(time_axis, brake_data, label='Brake', linestyle='--')
        plt.xlabel('Time (s)')
        plt.ylabel('Normalized Control Input [-1, 1]')
        plt.legend()
        plt.title('MPC Control Output')
        plt.grid(True)

        plt.show()
