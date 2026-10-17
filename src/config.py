import os
import numpy as np

# Thư mục lưu trữ quỹ đạo, vật cản và biểu đồ kết quả
TRAJECTORY_CSV_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../data/reference_trajectory.csv")
)
OBSTACLES_CSV_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../data/obstacles.csv")
)
OBSTACLE_SIZE = 0.5                   # Kích thước khối hộp vật cản (m)
USE_FIXED_OBSTACLES_CSV = True        # True: Ưu tiên spawn vật cản cố định từ data/obstacles.csv nếu có
PLOTS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../plots")
)

# --- CẤU HÌNH KẾT NỐI CARLA SERVER ---
# CARLA_HOST = '172.20.240.1'    # Đổi thành '127.0.0.1' hoặc IP máy chủ CARLA nếu chạy WSL2/máy khác (vd: '172.20.240.1')
CARLA_HOST = '127.0.0.1' 
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
N = 10      # Horizon length
dt = 0.1    # Sample time in seconds (10Hz)

MAX_CONTROL_WHEEL_ANGLE_RAD = 70.0 / 180.0 * np.pi
MAX_CONTROL_ACCELERATION_M_S_2 = 10.0
MAX_CONTROL_BRAKING_M_S_2 = -4.1

# Trọng số tối ưu MPC (Triệt tiêu chao đảo lắc trái phải)
FINE_X_COEF = 20.0           # Phạt sai số X
FINE_Y_COEF = 20.0           # Phạt sai số Y
FINE_V_COEF = 20.0           # Phạt sai số vận tốc V
FINE_THETA_COEF = 30.0       # Phạt sai số góc hướng Theta
FINE_LATERAL_COEF = 40.0     # Phạt độ lệch ngang vừa phải (nhập làn êm, không giật)
FINE_STEER_COEF = 25.0       # Phạt góc lái (cho phép mở góc lái linh hoạt 15-35 độ khi né vật cản)
FINE_ACC_COEF = 1.0          # Phạt sử dụng gia tốc quá lớn
FINE_STEER_DOT_COEF = 500.0  # Phạt tốc độ bẻ lái (đủ đầm chắc để không lắc, nhưng bẻ lái dứt khoát kịp thời)
FINE_ACC_DOT_COEF = 20.0     # Phạt giật ga/phanh

# ==============================================================================
# --- CẤU HÌNH SINH VẬT CẢN (OBSTACLE GENERATION) ---
# ==============================================================================
ENABLE_OBSTACLES = False              # True: Tự động sinh vật cản; False: Chế độ thuần bám quỹ đạo (không sinh vật cản)
OBSTACLE_START_DISTANCE_M = 25.0      # Bắt đầu xuất hiện từ mét thứ 25 của quỹ đạo (có thể thay đổi số này)
OBSTACLE_START_TRAJECTORY_FRACTION = None  # (Tùy chọn fallback) Đặt None để ưu tiên dùng OBSTACLE_START_DISTANCE_M
OBSTACLE_END_TRAJECTORY_FRACTION = 0.95    # Giới hạn kết thúc sinh vật cản (trước 5% cuối)
OBSTACLE_MIN_DISTANCE_M = 50.0        # Khoảng cách tối thiểu giữa các CỤM vật cản liên tiếp (m)
OBSTACLE_PREVIEW_DURATION_SEC = 60.0  # Thời gian vật cản tồn tại trước khi tự dọn dẹp (giây)

# Cấu hình cụm vật cản gần nhau (Clustered Obstacles):
ENABLE_OBSTACLE_CLUSTERS = True       # Bật sinh cụm vật cản gần nhau (2-3 vật cản chỉ cách nhau 2-3m)
OBSTACLE_CLUSTER_SPACING_M = 2.5      # Khoảng cách giữa các vật cản trong CÙNG MỘT CỤM (2 - 3m)
OBSTACLE_CLUSTER_CHANCE = 0.6         # Tỷ lệ sinh cụm nhiều vật cản (60% sinh cụm 2-3 vật cản, 40% sinh vật cản đơn)
OBSTACLE_CLUSTER_SIZES = [2, 3]       # Kích thước cụm khi kích hoạt (ngẫu nhiên 2 hoặc 3 vật cản gần nhau)

# Độ lệch ngang so với tim đường (m):
# 0.0: Nằm trực tiếp TRÊN quỹ đạo; dương (> 0): lệch phải (+Y); âm (< 0): lệch trái (-Y) (GẦN quỹ đạo)
OBSTACLE_LATERAL_OFFSETS = [0.0, -1.5, 1.5, -0.8, 0.8]

# Danh sách Blueprint các vật cản tĩnh phong phú (Cọc tiêu, rào chắn, thùng hàng, xe đỗ tĩnh)
# STATIC_OBSTACLE_BLUEPRINTS = [
#     'static.prop.constructioncone',       # Cọc tiêu giao thông phản quang
#     'static.prop.streetbarrier',          # Rào chắn công trường
#     'static.prop.trafficwarning',         # Biển cảnh báo tam giác
#     'static.prop.box01',                  # Thùng hàng / hộp xốp lớn
#     'static.prop.box02',                  # Thùng hàng kích thước vừa
#     'vehicle.audi.etron',                 # Xe Audi đỗ dừng tĩnh
#     'vehicle.toyota.prius',               # Xe Toyota đỗ dừng tĩnh
#     'vehicle.mini.cooperst',              # Xe Mini Cooper đỗ dừng tĩnh
# ]
STATIC_OBSTACLE_BLUEPRINTS = [
    'static.prop.box01',
    'static.prop.box02',
]

# ==============================================================================
# --- CẤU HÌNH CẢM BIẾN NHẬN THỨC VẬT CẢN (PERCEPTION / SENSOR) ---
# ==============================================================================
SENSOR_DETECTION_RANGE_M = 28.0       # Tầm quét xa tối đa của cảm biến trước xe (m)
SENSOR_FOV_DEG = 120.0                 # Góc mở trường nhìn (Field of View - FOV) phía trước mũi xe (độ)
SENSOR_SURROUND_RANGE_M = 10.0        # Tầm quét 360 độ của cảm biến cự ly gần (Ultrasonic / Blind-spot Radar quanh thân xe)
SENSOR_TRACK_DROP_BEHIND_M = 3.0      # Khoảng cách sau lưng xe (m) để xóa vật cản khỏi Local Object Map
OBSTACLE_DEFAULT_SAFETY_RADIUS_M = 2.0 # Bán kính an toàn mặc định bao quanh vật cản (m)
OBSTACLE_PROP_SAFETY_RADIUS_M = 1.4    # Bán kính an toàn cho cọc tiêu, rào chắn, thùng hàng (m)
OBSTACLE_VEHICLE_SAFETY_RADIUS_M = 2.3 # Bán kính an toàn cho xe ô tô đỗ tĩnh (m)

# ==============================================================================
# --- CẤU HÌNH TUYẾN TÍNH HÓA LẶP & RÀNG BUỘC MỀM (SLACK QP-MPC) ---
# ==============================================================================
USE_LINEARIZED_QP_MPC = True          # Bật chế độ điều khiển MPC Tuyến tính hóa lặp (QP-MPC)
MPC_MAX_LINEARIZATION_ITER = 2        # Số vòng lặp tuyến tính hóa tại mỗi bước (1-2 vòng lặp)
MPC_W_SLACK = 20000.0                 # Trọng số phạt biến bù nới lỏng (Slack Variable)
MPC_W_SLACK_LINEAR = 500.0            # Trọng số phạt bậc 1 cho biến bù (Exact Penalty)
OBSTACLE_AVOIDANCE_MARGIN_M = 0.35    # Khoảng đệm an toàn dự phòng (m)

# ==============================================================================
# --- CẤU HÌNH THÍCH ỨNG TỐC ĐỘ KHI GẶP VẬT CẢN (ADAPTIVE SPEED GOVERNOR) ---
# ==============================================================================
OBSTACLE_CAUTION_SPEED_MS = 5.0       # Vận tốc an toàn khi lách qua vật cản (m/s) (~23.4 km/h - lướt êm, thoát nhanh, không quá chậm)
OBSTACLE_SLOWDOWN_DISTANCE_M = 15.0   # Khoảng cách bắt đầu giảm tốc êm (m)
OBSTACLE_SLOWDOWN_LATERAL_CORRIDOR_M = 0.5  # Chỉ giảm tốc nếu vật cản nằm trong hành lang tim đường +-2m
OBSTACLE_PREFER_BYPASS_SIDE = 'right'  # Hướng né ưu tiên khi vật cản nằm chính giữa tim đường ('left' hoặc 'right')




