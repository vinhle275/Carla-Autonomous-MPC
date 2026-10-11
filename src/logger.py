import os
import numpy as np
from matplotlib import pyplot as plt
from src.config import dt, PLOTS_DIR

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

    def show_plots(self, save_dir=None):
        if save_dir is None:
            save_dir = PLOTS_DIR

        os.makedirs(save_dir, exist_ok=True)

        min_length = min(
            len(self.x_data), len(self.y_data), len(self.v_data), len(self.theta_data),
            len(self.x_ref_data), len(self.y_ref_data), len(self.v_ref_data), len(self.theta_ref_data),
            len(self.steering_data), len(self.throttle_data), len(self.brake_data)
        )

        if min_length == 0:
            print("[LOGGER] Không có dữ liệu để hiển thị hoặc lưu biểu đồ.")
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
        fig1 = plt.figure(figsize=(10, 6))
        plt.plot(x_ref_data, y_ref_data, 'r--', label='Reference Path (001)')
        plt.plot(x_data, y_data, 'b-', label='Actual Vehicle Path')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.title('2D Path Tracking Performance (MPC vs Reference Path)')
        plt.grid(True)
        fig1_path = os.path.join(save_dir, 'path_tracking.png')
        plt.savefig(fig1_path, dpi=300, bbox_inches='tight')

        # 2. Vận tốc theo thời gian
        fig2 = plt.figure(figsize=(10, 4))
        plt.plot(time_axis, v_data, label='Car Speed (v)')
        plt.plot(time_axis, v_ref_data, label='Reference Speed (v_ref)', linestyle='--')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.legend()
        plt.title('Speed Tracking Performance')
        plt.grid(True)
        fig2_path = os.path.join(save_dir, 'speed_tracking.png')
        plt.savefig(fig2_path, dpi=300, bbox_inches='tight')

        # 3. Tín hiệu điều khiển (Steering, Throttle, Brake)
        fig3 = plt.figure(figsize=(10, 4))
        plt.plot(time_axis, steering_data, label='Steer Control')
        plt.plot(time_axis, throttle_data, label='Throttle')
        plt.plot(time_axis, brake_data, label='Brake', linestyle='--')
        plt.xlabel('Time (s)')
        plt.ylabel('Normalized Control Input [-1, 1]')
        plt.legend()
        plt.title('MPC Control Signals (Steer, Throttle, Brake)')
        plt.grid(True)
        fig3_path = os.path.join(save_dir, 'control_signals.png')
        plt.savefig(fig3_path, dpi=300, bbox_inches='tight')

        # 4. Biểu đồ tổng hợp (Combined Summary Report 3-in-1)
        fig_summary, axes = plt.subplots(3, 1, figsize=(12, 12))
        
        # Subplot 1: Path tracking
        axes[0].plot(x_ref_data, y_ref_data, 'r--', label='Reference Path')
        axes[0].plot(x_data, y_data, 'b-', label='Actual Path')
        axes[0].set_xlabel('X (m)')
        axes[0].set_ylabel('Y (m)')
        axes[0].set_title('2D Path Tracking')
        axes[0].legend()
        axes[0].grid(True)

        # Subplot 2: Speed tracking
        axes[1].plot(time_axis, v_data, 'b-', label='Actual Speed (v)')
        axes[1].plot(time_axis, v_ref_data, 'r--', label='Ref Speed (v_ref)')
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Speed (m/s)')
        axes[1].set_title('Speed Tracking')
        axes[1].legend()
        axes[1].grid(True)

        # Subplot 3: Controls
        axes[2].plot(time_axis, steering_data, label='Steer')
        axes[2].plot(time_axis, throttle_data, label='Throttle')
        axes[2].plot(time_axis, brake_data, label='Brake', linestyle='--')
        axes[2].set_xlabel('Time (s)')
        axes[2].set_ylabel('Control [-1, 1]')
        axes[2].set_title('Control Signals')
        axes[2].legend()
        axes[2].grid(True)

        plt.tight_layout()
        summary_path = os.path.join(save_dir, 'summary_report.png')
        fig_summary.savefig(summary_path, dpi=300, bbox_inches='tight')

        print(f"\n[LOGGER] Đã tự động lưu thành công toàn bộ biểu đồ vào thư mục:")
        print(f"         📁 {save_dir}")
        print(f"         - {os.path.basename(fig1_path)}")
        print(f"         - {os.path.basename(fig2_path)}")
        print(f"         - {os.path.basename(fig3_path)}")
        print(f"         - {os.path.basename(summary_path)}")

        plt.show()
