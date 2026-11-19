// ==============================================================================
// CẤU HÌNH HẰNG SỐ BẤT BIẾN HỆ THỐNG (SYSTEM CONFIGURATION CONSTANTS)
// Chuẩn hóa đồng bộ 100% với config.py và SKILL.md trong Python
// ==============================================================================

#pragma once

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <cstdint>
#include <numbers>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace av::config {

// Tần số và chu kỳ thời gian (đồng bộ 100% chân trời dự đoán N=10, dt=0.10s với config.py)
constexpr double DT_CORE = 0.02;                   // Chu kỳ lấy mẫu Process 2 Core: 50Hz (0.02s = 20ms)
constexpr double DT_MPC = 0.10;                    // Bước tích phân nội bộ chân trời MPC (0.10s x 10 = 1.0s chuẩn config.py)
constexpr double DT_PAIO = 0.01;                   // Chu kỳ Process 1 PAIO: 100Hz (0.01s = 10ms)
constexpr int MPC_N = 10;                          // Chân trời dự đoán N = 10 bước (1.0s chuẩn config.py)
constexpr int MPC_MAX_LINEARIZATION_ITER = 1;      // Số vòng lặp RTI QP-MPC mỗi chu kỳ
constexpr uint64_t WATCHDOG_TIMEOUT_MS = 100;      // Ngưỡng Watchdog Timeout P1: 100ms

// Vận tốc và giới hạn chấp hành
constexpr double TARGET_SPEED_MS = 10.0;           // Vận tốc mục tiêu (m/s) ~ 36.0 km/h
constexpr double CAUTION_SPEED_MS = 6.0;           // Vận tốc an toàn khi lách qua vật cản (m/s)
constexpr double MAX_ACCELERATION_MS2 = 10.0;      // Gia tốc tiến tối đa (m/s^2)
constexpr double THROTTLE_ACCEL_SCALE_MS2 = 6.2;   // Hệ số quy đổi gia tốc -> bướm ga trên CARLA (m/s^2)
constexpr double MAX_BRAKING_MS2 = -4.1;           // Gia tốc hãm phanh tối đa (m/s^2)
constexpr double MAX_WHEEL_ANGLE_RAD = 70.0 * M_PI / 180.0; // Góc lái tối đa bánh xe (~1.2217 rad)
constexpr double WHEELBASE_L = 2.875;              // Chiều dài cơ sở L (m)

// Kích thước thân xe Ego & Bán kính an toàn bất biến (Core Invariants)
constexpr double EGO_FRONT_OVERHANG_M = 2.40;      // Từ tâm xe tới cản trước L_f (m)
constexpr double EGO_REAR_OVERHANG_M = 2.40;       // Từ tâm xe tới cản sau L_r (m)
constexpr double EGO_HALF_WIDTH_M = 1.00;          // Nửa chiều rộng xe (m)
constexpr double OBSTACLE_AVOIDANCE_MARGIN_M = 0.35;// Đệm an toàn cách mép vật cản (m)
constexpr double YELLOW_LINE_CLEARANCE_M = 0.35;   // Đệm an toàn vạch vàng liền (m)

// Bán kính đệm an toàn tổng hợp chuẩn: 1.00 + 0.35 = 1.35m (cấm cộng số lẻ tùy tiện)
constexpr double R_CLEARANCE_EDGE = EGO_HALF_WIDTH_M + OBSTACLE_AVOIDANCE_MARGIN_M; // 1.35m
constexpr double LATERAL_BYPASS_PAD_M = 0.15;      // Đệm tâm khe trống -> d_target = d_max + 1.50m

// Cảm biến nhận thức (Perception)
constexpr double SENSOR_DETECTION_RANGE_M = 28.0;   // Tầm quét xa tối đa (m)
constexpr double SENSOR_FOV_DEG = 120.0;            // Góc mở trường nhìn phía trước (độ)
constexpr double SENSOR_SURROUND_RANGE_M = 10.0;   // Vòng quét 360 độ cự ly gần (m)
constexpr double SENSOR_TRACK_DROP_BEHIND_M = 3.0; // Khoảng cách sau lưng xe để xóa track (m)
constexpr double OBSTACLE_SLOWDOWN_DISTANCE_M = 15.0; // Bắt đầu giảm tốc êm khi tới gần (m)
constexpr double LATERAL_CROSSING_HOLD_TIME_S = 2.5;  // Thời gian giữ trễ vật cản cắt ngang (2.5s ~ 25 bước @ 10Hz)
constexpr double LATERAL_CROSSING_CORRIDOR_MARGIN_M = 4.50; // Biên theo dõi mở rộng cho vật cản cắt ngang (+-4.50m)

// SAT Danger Zone va chạm khẩn cấp
constexpr double DANGER_ZONE_WIDTH_M = 2.4;        // Chiều rộng hộp nguy hiểm trước xe (m)
constexpr double DANGER_ZONE_LENGTH_M = 6.0;       // Chiều dài hộp nguy hiểm tính từ cản trước (m)
constexpr double DANGER_CLEAR_HOLD_TIME_S = 2.0;   // Giữ an toàn liên tục trước khi nhả phanh (s)
constexpr double DANGER_MOVING_SPEED_THRESHOLD_MS = 0.2; // Ngưỡng vận tốc vật cản di động (m/s)

// Roadblock & Dừng an toàn
constexpr bool ENABLE_ROADBLOCK_EMERGENCY_STOP = true;
constexpr double ROADBLOCK_SAFE_STOP_DIST_M = 5.5; // Khoảng cách dừng trước rào chắn (m)
constexpr double ROADBLOCK_DECELERATION_MS2 = 1.6; // Gia tốc hãm phanh êm ái Roadblock (m/s^2)
constexpr double MAX_ACCEL_DELTA_PER_100MS = 0.45; // Giới hạn biến thiên gia tốc |da| <= 0.45 m/s^2 mỗi 0.1s (Jerk <= 4.5 m/s^3)

// Hình học S-Curve né vật cản
constexpr double S_CURVE_APPROACH_RAMP_M = 18.0;   // Chiều dài dốc mở lái tiếp cận (m)
constexpr double S_CURVE_APPROACH_MARGIN_M = 2.5;  // Khoảng đệm trước đuôi vật cản (m)
constexpr double S_CURVE_HOLD_DIST_M = 4.0;        // Khoảng đệm giữ làn sau mũi vật cản (m)
constexpr double S_CURVE_RETURN_RAMP_M = 16.0;     // Chiều dài dốc trả lái về tim đường (m)

// Mặc định mặt đường & Hành lang pháp lý chuẩn (Đường 4 làn, Ego ở Làn 3)
constexpr double DEFAULT_ROAD_HALF_WIDTH_M = 5.25;
constexpr double STANDARD_ROAD_MIN_M = -0.45;      // Vạch vàng liền tại -1.75m -> road_min = -0.45m
constexpr double STANDARD_ROAD_MAX_M =  3.90;      // Vạch trắng liền mép phải tại +5.25m -> road_max = +3.90m

// Trọng số QP-MPC
constexpr double FINE_X_COEF = 2.0;
constexpr double FINE_Y_COEF = 2.0;
constexpr double FINE_V_COEF = 25.0;
constexpr double FINE_THETA_COEF = 120.0;
constexpr double FINE_LATERAL_COEF = 30.0;
constexpr double FINE_STEER_COEF = 25.0;
constexpr double FINE_ACC_COEF = 2.0;
constexpr double FINE_STEER_DOT_COEF = 300.0;
constexpr double FINE_ACC_DOT_COEF = 50.0;

constexpr double MPC_W_SLACK = 20000.0;
constexpr double MPC_W_SLACK_LINEAR = 500.0;
constexpr double MPC_W_ROAD_RIGHT = 8000.0;
constexpr double MPC_W_ROAD_RIGHT_LINEAR = 800.0;
constexpr double MPC_W_YELLOW_QUAD = 25000.0;
constexpr double MPC_W_YELLOW_LINEAR = 2500.0;

// Tham số camera Front Hood-Cam & Nhận diện vạch kẻ đường (Analytic IPM + HSV + Sliding Window)
constexpr double FRONT_CAM_X_M = 2.2;
constexpr double FRONT_CAM_Z_M = 1.2;
constexpr double FRONT_CAM_PITCH_DEG = -5.0;
constexpr double FRONT_CAM_FOV_DEG = 90.0;
constexpr int CAM_IMG_W = 480;
constexpr int CAM_IMG_H = 300;

constexpr int YELLOW_H_LOW = 22;
constexpr int YELLOW_H_HIGH = 34;
constexpr int YELLOW_S_LOW = 25;
constexpr int YELLOW_S_HIGH = 255;
constexpr int YELLOW_V_LOW = 195;
constexpr int YELLOW_V_HIGH = 255;

constexpr int WHITE_S_HIGH = 40;
constexpr int WHITE_V_LOW = 200;

constexpr int YELLOW_NWIN = 10;
constexpr double YELLOW_SOLID_FILL_RATIO = 0.75;
constexpr double YELLOW_DASHED_GAP_RATIO = 0.30;
constexpr double YELLOW_SOLID_HOLD_TIME_S = 1.2;   // Giữ ổn định trạng thái vạch vàng 1.2s khi qua giao lộ/bị che khuất

} // namespace av::config
