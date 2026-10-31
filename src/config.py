import os
import numpy as np

# Thư mục lưu trữ quỹ đạo, vật cản và biểu đồ kết quả
TRAJECTORY_CSV_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../data/reference_trajectory.csv")
)
OBSTACLES_CSV_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../data/obstacles.csv")
)
MOVING_OBSTACLES_CSV_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../data/moving_obstacles.csv")
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
N = 10      # Horizon length (10 bước ~ 1.0s khớp tối ưu với chu kỳ 10Hz, tránh nhìn quá xa gây cắt cua và dao động)
dt = 0.1    # Sample time in seconds (10Hz)

MAX_CONTROL_WHEEL_ANGLE_RAD = 70.0 / 180.0 * np.pi
MAX_CONTROL_ACCELERATION_M_S_2 = 10.0
MAX_CONTROL_BRAKING_M_S_2 = -4.1

# Trọng số tối ưu MPC (Cân bằng biên độ - pha, triệt tiêu hoàn toàn dao động hình sin khi bám đường & nhập làn)
FINE_X_COEF = 20.0           # Phạt sai số X
FINE_Y_COEF = 20.0           # Phạt sai số Y
FINE_V_COEF = 25.0           # Phạt sai số vận tốc V
FINE_THETA_COEF = 35.0       # Phạt sai số góc hướng Theta (đủ lớn để giữ mũi xe ổn định khi nhập làn)
FINE_LATERAL_COEF = 40.0     # Phạt độ lệch ngang vuông góc vừa phải (không quá gắt gây vọt lố qua lại tim đường)
FINE_STEER_COEF = 35.0       # Phạt biên độ góc lái
FINE_ACC_COEF = 1.0          # Phạt gia tốc
FINE_STEER_DOT_COEF = 1000.0 # Phạt tốc độ bẻ lái nội bộ chân trời (k = 1..N-1)
FINE_ACC_DOT_COEF = 30.0     # Phạt giật ga/phanh

# ==============================================================================
# --- CẤU HÌNH SINH VẬT CẢN (OBSTACLE GENERATION) ---
# ==============================================================================
ENABLE_OBSTACLES = True               # True: Tự động sinh vật cản; False: Chế độ thuần bám quỹ đạo (không sinh vật cản)
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
'vehicle.toyota.prius',
]

# ==============================================================================
# --- CẤU HÌNH VẬT CẢN DI ĐỘNG (MOVING OBSTACLES) ---
# ==============================================================================
ENABLE_MOVING_OBSTACLES = True         # True: Kích hoạt vật cản di động; False: Bỏ qua
MOVING_OBSTACLE_SPEED_MS = 9.0        # Vận tốc cố định của vật cản di động (m/s) ~ 10.8 km/h
MOVING_OBSTACLE_LOOP = False          # True: Vật cản đi lặp vô hạn theo vòng; False: Dừng ở điểm cuối
MOVING_OBSTACLE_BLUEPRINTS = [
    'vehicle.toyota.prius',            # Xe sedan
    'vehicle.audi.etron',              # Xe SUV
    'vehicle.mini.cooperst',           # Xe nhỏ
    'vehicle.tesla.model3',            # Xe Tesla
]
MOVING_OBSTACLE_MARKER_COLOR = (255, 165, 0)  # Màu vàng cam cho debug marker (R, G, B)

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
OBSTACLE_SPATIAL_CLUSTER_RANGE_M = 6.5  # Khoảng cách dọc tối đa (m) để gộp chung vào 1 mặt cắt ngang

# Thông số hình học thân xe & lề đường mặc định
DEFAULT_ROAD_HALF_WIDTH_M = 5.25      # Giới hạn biên dịch tâm xe tối đa (m) (A_shift_max ~ 3.20m; nếu vật cản ép vượt quá biên này sẽ coi là chắn kín làn né)
MAX_LATERAL_ACCEL_M_S_2 = 3.2           # Gia tốc ngang tối đa cho phép khi vừa cua vừa né (m/s^2)
EGO_FRONT_OVERHANG_M = 2.4              # Khoảng cách từ tâm xe Ego tới cản trước (m)
EGO_REAR_OVERHANG_M = 2.4               # Khoảng cách từ tâm xe Ego tới cản sau (m)
EGO_HALF_WIDTH_M = 1.0                  # Nửa chiều rộng thân xe Ego (m)

# ==============================================================================
# --- CẤU HÌNH TUYẾN TÍNH HÓA LẶP & RÀNG BUỘC MỀM (SLACK QP-MPC) ---
# ==============================================================================
USE_LINEARIZED_QP_MPC = True          # Bật chế độ điều khiển MPC Tuyến tính hóa lặp (QP-MPC)
MPC_MAX_LINEARIZATION_ITER = 2        # Số vòng lặp tuyến tính hóa tại mỗi bước (1-2 vòng lặp)
MPC_W_SLACK = 20000.0                 # Trọng số phạt biến bù nới lỏng (Slack Variable)
MPC_W_SLACK_LINEAR = 500.0            # Trọng số phạt bậc 1 cho biến bù (Exact Penalty)
OBSTACLE_AVOIDANCE_MARGIN_M = 0.35    # Khoảng đệm an toàn dự phòng (m) (45cm cách mép ngoài vật cản giúp giữ làn ổn định)

# ==============================================================================
# --- CẤU HÌNH THÍCH ỨNG TỐC ĐỘ KHI GẶP VẬT CẢN (ADAPTIVE SPEED GOVERNOR) ---
# ==============================================================================
OBSTACLE_CAUTION_SPEED_MS = 6.0       # Vận tốc an toàn khi lách qua vật cản (m/s) (~18.0 km/h - lướt êm, thoát nhanh, không quá chậm)
OBSTACLE_SLOWDOWN_DISTANCE_M = 15.0   # Khoảng cách bắt đầu giảm tốc êm (m)
OBSTACLE_TRIGGER_LATERAL_CORRIDOR_M = 2.0  # Hành lang khoảng cách lệch ngang tối đa so với tim đường (|d_lat| <= 2.0m) để kích hoạt né (bỏ qua xe làn bên cạnh)
OBSTACLE_PREFER_BYPASS_SIDE = 'left'  # Hướng né ưu tiên khi vật cản nằm chính giữa tim đường ('left' hoặc 'right')

# ==============================================================================
# --- CẤU HÌNH 2 CHẾ ĐỘ ĐIỀU KHIỂN & PHANH GẤP VÙNG NGUY HIỂM VA CHẠM ---
# ==============================================================================
# Chế độ 1: Bám đường né vật cản theo MPC (Normal Path Tracking & Obstacle Avoidance)
# Chế độ 2: Phanh gấp tối đa, bỏ qua mọi ràng buộc khi có vật cản di động xâm nhập vùng nguy hiểm
ENABLE_EMERGENCY_BRAKE_MODE = True        # True: Kích hoạt cơ chế 2 chế độ điều khiển
EMERGENCY_ZONE_WIDTH_M = 2.4              # Độ rộng vùng nguy hiểm va chạm trước đầu xe (m) (đối xứng qua trục dọc xe)
EMERGENCY_ZONE_LENGTH_M = 6.0             # Chiều dài vùng nguy hiểm tính từ cản trước xe về phía trước (m)
EMERGENCY_CLEAR_HOLD_TIME_SEC = 2.0       # Thời gian giữ (giây) liên tục không có vật cản trước khi chuyển từ Chế độ 2 về Chế độ 1
EMERGENCY_TRIGGER_ONLY_MOVING = True      # True: Chỉ kích hoạt Chế độ 2 với vật cản di động (vật cản tĩnh do MPC tự né hoặc dừng Roadblock)
EMERGENCY_MOVING_SPEED_THRESHOLD_MS = 0.2 # Ngưỡng vận tốc (m/s) để xác định vật cản đang di chuyển

# ==============================================================================
# --- CẤU HÌNH PHANH DỪNG KHẨN CẤP KHI PHONG TỎA ĐƯỜNG (SAFE EMERGENCY STOP) ---
# ==============================================================================
ENABLE_ROADBLOCK_EMERGENCY_STOP = True  # True: Tự động phanh dừng an toàn nếu cả 2 bên làn đều bị chặn, không có lối né
ROADBLOCK_SAFE_STOP_DISTANCE_M = 3.5   # Khoảng cách dừng an toàn trước hàng rào vật cản (m)
ROADBLOCK_DECELERATION_M_S_2 = 3.0     # Gia tốc hãm phanh êm ái khi dừng khẩn cấp (m/s^2)

# ==============================================================================
# --- CẤU HÌNH HÌNH HỌC ĐƯỜNG CONG NÉ VẬT CẢN (S-CURVE MANEUVER GEOMETRY) ---
# ==============================================================================
OBSTACLE_APPROACH_RAMP_M = 12.0         # Chiều dài dốc mở lái trước mũi xe - đuôi vật cản (m)
OBSTACLE_APPROACH_MARGIN_M = 2.5       # Khoảng đệm hoàn tất 100% mở lái trước đuôi vật cản (m)
OBSTACLE_HOLD_DISTANCE_M = 3.5        # Khoảng đệm giữ làn sau khi đuôi xe vượt qua mũi vật cản (m)
OBSTACLE_RETURN_RAMP_M = 10.0         # Chiều dài dốc trả lái êm về tim đường (15m giúp góc nhập làn nhỏ, không vọt lố)

# ==============================================================================
# --- CẤU HÌNH COCKPIT DASHBOARD ĐA MÀN HÌNH (FOLLOW CAMERA VIEWER) ---
# ==============================================================================
ENABLE_CHASE_CAMERA_WINDOW = True      # True: Mở cửa sổ Pygame Cockpit Dashboard đa màn hình theo dõi xe
COCKPIT_WINDOW_WIDTH = 1440            # Chiều rộng tổng cửa sổ Cockpit Dashboard (px) (tỷ lệ 16:10)
COCKPIT_WINDOW_HEIGHT = 900            # Chiều cao tổng cửa sổ Cockpit Dashboard (px)
TOP_BAR_HEIGHT = 60                    # Chiều cao thanh thông số hợp nhất trên cùng (px)
MAIN_VIEW_WIDTH = 960                  # Chiều rộng khung nhìn chính Chase Cam (px)
MAIN_VIEW_HEIGHT = 600                 # Chiều cao khung nhìn chính Chase Cam (px)
SUB_VIEW_WIDTH = 480                   # Chiều rộng mỗi khung nhìn camera phụ (Bird-Eye & Front) (px)
SUB_VIEW_HEIGHT = 300                  # Chiều cao mỗi khung nhìn camera phụ (Bird-Eye & Front) (px)
PLOT_WIDTH = 720                       # Chiều rộng mỗi đồ thị Realtime (px)
PLOT_HEIGHT = 240                      # Chiều cao khu vực 2 đồ thị Realtime (px)
TELEMETRY_HISTORY_LEN = 100            # Độ dài cửa sổ trượt đồ thị (100 mẫu ~ 10 giây tại dt=0.1s)

# Tương thích ngược với các module tham chiếu tên hằng số cũ
CAMERA_WINDOW_WIDTH = COCKPIT_WINDOW_WIDTH
CAMERA_WINDOW_HEIGHT = COCKPIT_WINDOW_HEIGHT
ENABLE_SPECTATOR_FOLLOW = True         # True: Cho phép góc nhìn Spectator của CARLA tự động bay theo sau xe

# ==============================================================================
# --- CẤU HÌNH HIỂN THỊ NÉT VẼ DEBUG (DEBUG DRAWINGS SETTINGS) ---
# ==============================================================================
DEBUG_DRAW_SENSOR_BEAMS = False        # False: Tắt các tia laser đỏ/tím bắn từ xe tới vật cản để đỡ rối mắt
DEBUG_DRAW_SENSOR_FOV = False          # False: Tắt 2 tia xanh lá quạt 120 độ phía trước cho màn hình sạch sẽ
DEBUG_DRAW_SURROUND_CIRCLE = False     # False: Tắt vòng tròn 360 độ quanh thân xe
DEBUG_DRAW_OBSTACLE_BOX = False         # True: Giữ lại viền hộp Bounding Box nhẹ quanh vật cản để dễ nhìn vị trí
DEBUG_DRAW_FULL_TRAJECTORY = True      # True: Vẽ đường tim đường toàn cục; False: Tắt hoàn toàn
DEBUG_DRAW_TRAJECTORY_ARROWS = False   # False: Tắt các mũi tên đỏ dày đặc trên mặt đường để thoáng mắt
DEBUG_DRAW_HORIZON_PATH = True         # True: Giữ lại vệt đường đỏ N bước dự đoán MPC né vật cản
DEBUG_DRAW_EMERGENCY_ZONE = True       # True: Hiển thị hộp 3D vùng nguy hiểm trước đầu xe (Xanh: An toàn, Đỏ: Phanh gấp)

# ==============================================================================
# --- CẤU HÌNH NHẬN DIỆN VẠCH VÀNG (YELLOW LINE DETECTOR) ---
# ==============================================================================

# --- Camera RGB gắn trên xe (dùng cho IPM) ---
CAM_IMAGE_WIDTH  = 1000   # Chiều rộng ảnh camera CARLA (px) — khớp CAMERA_WINDOW_WIDTH
CAM_IMAGE_HEIGHT = 900    # Chiều cao ảnh camera CARLA (px) — khớp CAMERA_WINDOW_HEIGHT
CAM_FOV_DEG      = 90.0   # Field of View camera (độ) — khớp FOV trong camera_viewer.py

# --- Tham số IPM (Inverse Perspective Mapping) ---
# Điểm nguồn (src) trong không gian ROI = nửa dưới ảnh (y tính từ 0 = đầu ROI)
# Điều chỉnh 4 điểm này khi đổi độ cao / pitch camera trong CARLA
IPM_SRC_POINTS = [
    [200,  350],   # Dưới-trái  (gần xe, bên trái)
    [800,  350],   # Dưới-phải  (gần xe, bên phải)
    [650,    0],   # Trên-phải  (xa xe, bên phải)
    [350,    0],   # Trên-trái  (xa xe, bên trái)
]
# Điểm đích (dst) trong không gian BEV đầu ra
IPM_DST_POINTS = [
    [200, 480],    # Dưới-trái BEV
    [600, 480],    # Dưới-phải BEV
    [600,   0],    # Trên-phải BEV
    [200,   0],    # Trên-trái BEV
]
IPM_BEV_WIDTH  = 800   # Chiều rộng ảnh BEV đầu ra (px)
IPM_BEV_HEIGHT = 480   # Chiều cao ảnh BEV đầu ra (px)

# Tỷ lệ quy đổi pixel → mét trong không gian BEV
# (Hiệu chỉnh thực nghiệm theo camera mount; giá trị ước lượng cho Tesla Model3 CARLA)
BEV_METERS_PER_PIXEL_X = 0.035   # ~3.5 cm/px theo chiều ngang
BEV_METERS_PER_PIXEL_Y = 0.05    # ~5.0 cm/px theo chiều dọc (xa-gần)

# --- Bộ lọc màu vàng (HSV) ---
YELLOW_H_LOW  = 12    # Hue thấp nhất của vàng
YELLOW_H_HIGH = 38    # Hue cao nhất của vàng
YELLOW_S_LOW  = 40    # Saturation tối thiểu
YELLOW_S_HIGH = 255
YELLOW_V_LOW  = 70   # Value (độ sáng) tối thiểu
YELLOW_V_HIGH = 255

# --- Sliding Window ---
YELLOW_NWIN          = 10    # Số cửa sổ chia chiều cao BEV (9–12)
YELLOW_WIN_MARGIN_PX = 60    # Bán kính tìm kiếm ngang mỗi cửa sổ (px)
YELLOW_MIN_PIX_PER_WIN = 40  # Số pixel vàng tối thiểu để cửa sổ hợp lệ

# --- Ngưỡng phân loại vạch Solid / Dashed ---
# fill_ratio = n_valid_windows / YELLOW_NWIN
YELLOW_SOLID_FILL_RATIO  = 0.75   # fill ≥ 75% → SOLID (vạch liền)
YELLOW_DASHED_GAP_RATIO  = 0.30   # gap ≥ 30% và có ≥2 cửa sổ trống liên tiếp → DASHED

# --- Khoảng cách an toàn vạch liền (SOLID line clearance) ---
# road_max = d_yellow_line - r_clearance  (r = EGO_HALF_WIDTH_M + YELLOW_LINE_CLEARANCE_M)
YELLOW_LINE_CLEARANCE_M  = 0.35   # Khoảng đệm tối thiểu giữa thân xe và vạch liền (m)

# --- Bật / Tắt module nhận diện vạch vàng ---
ENABLE_YELLOW_LINE_DETECTOR = True   # True: Kích hoạt; False: Bỏ qua (tiết kiệm CPU nếu không cần)
DEBUG_YELLOW_DRAW = False             # True: Vẽ overlay vạch vàng lên HUD camera (chỉ dùng khi debug)
