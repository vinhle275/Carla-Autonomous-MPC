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
        self.cost_data = []

    def log_controller_input(self, x, y, v, theta, x_ref, y_ref, v_ref, theta_ref):
        self.x_data.append(x)
        self.y_data.append(y)
        self.v_data.append(v)
        self.theta_data.append(np.rad2deg(theta))
        self.x_ref_data.append(x_ref)
        self.y_ref_data.append(y_ref)
        self.v_ref_data.append(v_ref)
        self.theta_ref_data.append(np.rad2deg(theta_ref))

    def log_controller_output(self, steering, throttle, brake, cost=0.0):
        self.steering_data.append(steering)
        self.throttle_data.append(throttle)
        self.brake_data.append(brake)
        self.cost_data.append(cost)

    def show_plots(self, save_dir=None):
        if save_dir is None:
            save_dir = PLOTS_DIR

        os.makedirs(save_dir, exist_ok=True)

        min_length = min(
            len(self.x_data), len(self.y_data), len(self.v_data), len(self.theta_data),
            len(self.x_ref_data), len(self.y_ref_data), len(self.v_ref_data), len(self.theta_ref_data),
            len(self.steering_data), len(self.throttle_data), len(self.brake_data),
            len(self.cost_data)
        )

        if min_length == 0:
            print("[LOGGER] Không có dữ liệu để hiển thị hoặc lưu biểu đồ.")
            return

        x_data = np.array(self.x_data[:min_length])
        y_data = np.array(self.y_data[:min_length])
        v_data = np.array(self.v_data[:min_length])
        theta_data = np.array(self.theta_data[:min_length])
        x_ref_data = np.array(self.x_ref_data[:min_length])
        y_ref_data = np.array(self.y_ref_data[:min_length])
        v_ref_data = np.array(self.v_ref_data[:min_length])
        theta_ref_data = np.array(self.theta_ref_data[:min_length])
        steering_data = np.array(self.steering_data[:min_length])
        throttle_data = np.array(self.throttle_data[:min_length])
        brake_data = np.array(self.brake_data[:min_length])
        cost_data = np.array(self.cost_data[:min_length])

        time_axis = np.linspace(0, min_length * dt, min_length)

        # ---------------------------------------------------------
        # TÍNH TOÁN CÁC SAI SỐ ĐIỀU KHIỂN (TRACKING ERRORS)
        # ---------------------------------------------------------
        # 1. Sai số vị trí Euclid (Euclidean Position Error)
        pos_errors = np.sqrt((x_data - x_ref_data)**2 + (y_data - y_ref_data)**2)

        # 2. Sai số ngang (Cross-Track / Lateral Error)
        theta_rad_ref = np.deg2rad(theta_ref_data)
        dx = x_data - x_ref_data
        dy = y_data - y_ref_data
        lat_errors = -dx * np.sin(theta_rad_ref) + dy * np.cos(theta_rad_ref)

        # 3. Sai số góc lệch hướng (Heading Error quy đổi về [-180, 180] độ)
        theta_rad_actual = np.deg2rad(theta_data)
        heading_errors_rad = np.arctan2(np.sin(theta_rad_actual - theta_rad_ref), np.cos(theta_rad_actual - theta_rad_ref))
        heading_errors_deg = np.rad2deg(heading_errors_rad)

        # ---------------------------------------------------------
        # 1. Biểu đồ vị trí 2D (Path tracking evaluation)
        # ---------------------------------------------------------
        fig1 = plt.figure(figsize=(10, 6))
        plt.plot(x_ref_data, y_ref_data, 'r--', label='Reference Path')
        plt.plot(x_data, y_data, 'b-', label='Actual Vehicle Path')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.title('2D Path Tracking Performance (MPC vs Reference Path)')
        plt.grid(True)
        fig1_path = os.path.join(save_dir, 'path_tracking.png')
        plt.savefig(fig1_path, dpi=300, bbox_inches='tight')
        plt.close(fig1)

        # ---------------------------------------------------------
        # 2. Vận tốc theo thời gian (Speed tracking)
        # ---------------------------------------------------------
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
        plt.close(fig2)

        # ---------------------------------------------------------
        # 3. Tín hiệu điều khiển (Steering, Throttle, Brake)
        # ---------------------------------------------------------
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
        plt.close(fig3)

        # ---------------------------------------------------------
        # 4. BIỂU ĐỒ SAI SỐ ĐƯỜNG ĐI SO VỚI THỜI GIAN
        # ---------------------------------------------------------
        fig4, (ax_err1, ax_err2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

        ax_err1.plot(time_axis, lat_errors, 'b-', label='Lateral Error $e_{lat}$ (m)')
        ax_err1.plot(time_axis, pos_errors, 'r--', label='Euclidean Position Error $e_{pos}$ (m)', alpha=0.7)
        ax_err1.axhline(0, color='gray', linestyle=':', linewidth=0.8)
        ax_err1.set_ylabel('Position Error (m)')
        ax_err1.set_title('Path Tracking Errors vs Time')
        ax_err1.legend(loc='upper right')
        ax_err1.grid(True)

        ax_err2.plot(time_axis, heading_errors_deg, 'g-', label='Heading Error $e_{\\theta}$ (deg)')
        ax_err2.axhline(0, color='gray', linestyle=':', linewidth=0.8)
        ax_err2.set_xlabel('Time (s)')
        ax_err2.set_ylabel('Heading Error (deg)')
        ax_err2.legend(loc='upper right')
        ax_err2.grid(True)

        plt.tight_layout()
        fig4_path = os.path.join(save_dir, 'tracking_errors.png')
        fig4.savefig(fig4_path, dpi=300, bbox_inches='tight')
        plt.close(fig4)

        # ---------------------------------------------------------
        # 5. BIỂU ĐỒ GIÁ TRỊ HÀM CHI PHÍ SO VỚI THỜI GIAN
        # ---------------------------------------------------------
        fig5 = plt.figure(figsize=(10, 4))
        plt.plot(time_axis, cost_data, 'm-', linewidth=1.5, label='MPC Optimal Cost Value ($J^*$)')
        plt.xlabel('Time (s)')
        plt.ylabel('Cost Value $J$')
        plt.title('MPC Cost Function Value vs Time')
        plt.legend(loc='upper right')
        plt.grid(True)
        fig5_path = os.path.join(save_dir, 'cost_vs_time.png')
        plt.savefig(fig5_path, dpi=300, bbox_inches='tight')
        plt.close(fig5)

        # ---------------------------------------------------------
        # 6. BIỂU ĐỒ BÁO CÁO TỔNG HỢP (SUMMARY REPORT - LƯỚI 3x2)
        # ---------------------------------------------------------
        fig_summary, axes = plt.subplots(3, 2, figsize=(16, 12))

        # (0, 0): Quỹ đạo 2D
        axes[0, 0].plot(x_ref_data, y_ref_data, 'r--', label='Reference')
        axes[0, 0].plot(x_data, y_data, 'b-', label='Actual')
        axes[0, 0].set_xlabel('X (m)')
        axes[0, 0].set_ylabel('Y (m)')
        axes[0, 0].set_title('1. 2D Path Tracking')
        axes[0, 0].legend()
        axes[0, 0].grid(True)

        # (0, 1): Vận tốc
        axes[0, 1].plot(time_axis, v_data, 'b-', label='Actual $v$')
        axes[0, 1].plot(time_axis, v_ref_data, 'r--', label='Reference $v_{ref}$')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Speed (m/s)')
        axes[0, 1].set_title('2. Speed Tracking vs Time')
        axes[0, 1].legend()
        axes[0, 1].grid(True)

        # (1, 0): Tín hiệu điều khiển
        axes[1, 0].plot(time_axis, steering_data, label='Steer')
        axes[1, 0].plot(time_axis, throttle_data, label='Throttle')
        axes[1, 0].plot(time_axis, brake_data, label='Brake', linestyle='--')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Control Input [-1, 1]')
        axes[1, 0].set_title('3. Control Signals vs Time')
        axes[1, 0].legend()
        axes[1, 0].grid(True)

        # (1, 1): Sai số vị trí & sai số ngang
        axes[1, 1].plot(time_axis, lat_errors, 'b-', label='Lateral Error $e_{lat}$ (m)')
        axes[1, 1].plot(time_axis, pos_errors, 'r--', label='Euclidean Error $e_{pos}$ (m)', alpha=0.7)
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Position Error (m)')
        axes[1, 1].set_title('4. Position & Lateral Error vs Time')
        axes[1, 1].legend()
        axes[1, 1].grid(True)

        # (2, 0): Sai số góc hướng
        axes[2, 0].plot(time_axis, heading_errors_deg, 'g-', label='Heading Error $e_{\\theta}$ (deg)')
        axes[2, 0].set_xlabel('Time (s)')
        axes[2, 0].set_ylabel('Heading Error (deg)')
        axes[2, 0].set_title('5. Heading Error vs Time')
        axes[2, 0].legend()
        axes[2, 0].grid(True)

        # (2, 1): Giá trị hàm chi phí tối ưu
        axes[2, 1].plot(time_axis, cost_data, 'm-', label='Optimal Cost $J^*$')
        axes[2, 1].set_xlabel('Time (s)')
        axes[2, 1].set_ylabel('Cost $J$')
        axes[2, 1].set_title('6. Cost Function Value vs Time')
        axes[2, 1].legend()
        axes[2, 1].grid(True)

        plt.tight_layout()
        summary_path = os.path.join(save_dir, 'summary_report.png')
        fig_summary.savefig(summary_path, dpi=300, bbox_inches='tight')

        print(f"\n[LOGGER] Đã tự động lưu thành công toàn bộ {6} biểu đồ vào thư mục:")
        print(f"         📁 {save_dir}")
        print(f"         1. {os.path.basename(fig1_path)}     (Quỹ đạo 2D)")
        print(f"         2. {os.path.basename(fig2_path)}    (Bám vận tốc)")
        print(f"         3. {os.path.basename(fig3_path)}   (Tín hiệu điều khiển)")
        print(f"         4. {os.path.basename(fig4_path)}   (Sai số bám đường theo thời gian)")
        print(f"         5. {os.path.basename(fig5_path)}     (Giá trị hàm chi phí theo thời gian)")
        print(f"         6. {os.path.basename(summary_path)}   (Báo cáo tổng hợp 6-trong-1)")

        plt.show()
