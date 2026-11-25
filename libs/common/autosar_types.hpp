// ==============================================================================
// AUTOSAR ADAPTIVE PLATFORM (PAIO) - DATA CONTRACT & INTERFACE TYPES
// Architecture: Modern C++20 | Plain Old Data (POD) for Zero-Copy Shared Memory
// ==============================================================================

#pragma once

#include <cstdint>
#include <array>
#include <string>

namespace av::types {

constexpr size_t MPC_HORIZON = 20;
constexpr size_t MAX_RAW_ACTORS = 32;
constexpr size_t CAM_SHM_W = 160;
constexpr size_t CAM_SHM_H = 100;
constexpr size_t CAM_SHM_BYTES = CAM_SHM_W * CAM_SHM_H * 3; // 48000 bytes (RGB24)

/// @brief Chế độ vận hành của xe tự hành
enum class VehicleMode : uint8_t {
    MODE_1_NORMAL = 1,          // Chế độ 1: Bám đường né vật cản theo MPC / Dừng Roadblock
    MODE_2_EMERGENCY_BRAKE = 2  // Chế độ 2: Phanh gấp tối đa khẩn cấp (SAT Danger Zone kích hoạt)
};

/// @brief Phân loại vạch kẻ đường nhận diện từ Camera / OpenDRIVE
enum class LineType : uint8_t {
    NONE = 0,
    DASHED = 1,                 // Vạch nét đứt (cho phép mượn làn né)
    SOLID = 2                   // Vạch nét liền (cấm lấn, phạt nghiêm ngặt)
};

#pragma pack(push, 1)
/// @brief Dữ liệu trạng thái xe từ Process 1 (PAIO) gửi sang Process 2 (Core) - Chu kỳ 50Hz
struct VehicleStateData {
    uint64_t timestamp_ns{0};
    uint32_t sequence_id{0};
    double x{0.0};              // Tọa độ toàn cục X (m)
    double y{0.0};              // Tọa độ toàn cục Y (m)
    double yaw_rad{0.0};        // Góc hướng xe Theta (rad)
    double velocity_m_s{0.0};   // Vận tốc dài thực tế (m/s)
    double current_steer{0.0};  // Tỉ lệ góc bẻ lái hiện tại [-1.0, 1.0]
    double acceleration_m_s2{0.0}; // Gia tốc dài hiện tại (m/s^2)
    double jerk_m_s3{0.0};       // Độ giật dọc Jerk (m/s^3)
    uint8_t is_valid{0};
};

/// @brief Dữ liệu thô của một Actor vật cản thu nhận từ CARLA / Sensor
struct RawActorPacket {
    int32_t id{0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw_rad{0.0};
    double extent_x{2.4};
    double extent_y{1.0};
    double vx{0.0};
    double vy{0.0};
    uint8_t is_vehicle{1};
};

/// @brief Dữ liệu cảm biến nhận thức (Actors + OpenDRIVE Map + Front Hood-Cam RGB) truyền qua Zero-Copy Shared Memory từ P1 sang P2
struct PerceptionSensorData {
    uint64_t timestamp_ns{0};
    uint32_t sequence_id{0};
    uint32_t num_actors{0};
    std::array<RawActorPacket, MAX_RAW_ACTORS> actors{};
    double map_road_min{-0.45};
    double map_road_max{3.90};
    uint8_t map_is_yellow_solid{1};
    uint8_t is_junction{0};
    uint8_t has_cam_frame{0};
    uint16_t cam_w{CAM_SHM_W};
    uint16_t cam_h{CAM_SHM_H};
    std::array<uint8_t, CAM_SHM_BYTES> cam_rgb{};
    uint8_t is_valid{0};
};
#pragma pack(pop)

/// @brief Lệnh điều khiển cơ cấu chấp hành & Telemetry HUD từ Process 2 (Core) gửi sang Process 1 (PAIO) - Chu kỳ 50Hz/100Hz
struct alignas(8) ActuatorCommandData {
    uint64_t timestamp_ns{0};
    uint32_t sequence_id{0};

    double throttle{0.0};          // Mức bướm ga [0.0, 1.0]
    double brake{0.0};             // Mức đạp phanh [0.0, 1.0]
    double steer{0.0};             // Tỉ lệ vô lăng [-1.0, 1.0]

    double wheel_angle_rad{0.0};   // Góc lái thực tế (rad)
    double acceleration_cmd{0.0};  // Gia tốc yêu cầu (m/s^2)

    // Thông số Telemetry phục vụ Cockpit Dashboard & 3D Visualization
    double v_ref_m_s{10.0};
    double jerk_filtered_m_s3{0.0};
    double road_min_m{-0.45};
    double road_max_m{3.90};
    double s_ego_m{0.0};
    double s_total_m{0.0};
    double d_yellow_line_m{1.75};
    double detect_time_ms{0.0};
    uint32_t current_idx{0};
    uint32_t total_idx{0};
    uint32_t obs_count{0};
    int32_t active_obs_id{-1};
    int8_t bypass_side{0};         // +1: Phải, -1: Trái, 0: Bám đường
    uint8_t yellow_line_type{2};   // 0: NONE, 1: DASHED, 2: SOLID
    uint8_t yellow_source{2};      // 0: NONE, 1: CV, 2: MAP
    std::array<double, MPC_HORIZON> horizon_x{};
    std::array<double, MPC_HORIZON> horizon_y{};

    VehicleMode mode{VehicleMode::MODE_1_NORMAL};
    bool is_emergency_stop{false}; // Kích hoạt phanh khẩn cấp Chế độ 2
    bool is_roadblock{false};      // Đang ở trạng thái Roadblock
    bool is_roadblock_latch{false};// Khóa dừng tĩnh chống trôi (brake = 0.18)
    bool is_valid{false};
};

/// @brief Thông số hình học OBB 2D của vật cản trong hệ tọa độ Frenet (s, d) & Trạng thái chuỗi né
struct ObstacleFrenetOBB {
    int32_t id{0};
    double s_rear{0.0};            // Tọa độ dọc mép sau (m)
    double s_front{0.0};           // Tọa độ dọc mép trước (m)
    double s_center{0.0};          // Tọa độ dọc tâm OBB (m)
    double d_min{0.0};             // Độ lệch ngang mép trái (m)
    double d_max{0.0};             // Độ lệch ngang mép phải (m)
    double d_center{0.0};          // Tâm ngang OBB (m)
    double center_x{0.0};          // Tọa độ thế giới X
    double center_y{0.0};          // Tọa độ thế giới Y
    double vx{0.0};                // Vận tốc thế giới Vx (m/s)
    double vy{0.0};                // Vận tốc thế giới Vy (m/s)
    double v_s{0.0};               // Vận tốc dọc Frenet (m/s)
    double v_d{0.0};               // Vận tốc ngang Frenet (m/s)
    double box_yaw{0.0};           // Góc hướng của hộp OBB (rad)
    double extent_x{0.6};          // Nửa chiều dài OBB (m)
    double extent_y{0.6};          // Nửa chiều rộng OBB (m)
    double side_sign{0.0};         // +1.0 né phải, -1.0 né trái
    double A_shift{0.0};           // Biên độ dịch ngang tuyệt đối |d_target| (m)
    double d_target{0.0};          // Độ lệch ngang mục tiêu có dấu (m)
    double group_s_rear{0.0};      // Tọa độ dọc đầu cụm liên thông (m)
    double group_s_front{0.0};     // Tọa độ dọc cuối cụm liên thông (m)
    double min_s_app_start{-1e9};  // Giới hạn kẹp điểm mở lái S-Curve chống đè xe trước (m)
    bool has_min_s_app_start{false};
    bool affects_lane{false};      // Vật cản có chắn tim làn hiện tại (<= 1.35m) hay không
    bool is_moving{false};         // Vật cản đang di chuyển
    bool is_lateral_crossing{false};// Đang di chuyển cắt ngang qua lại
    bool is_vehicle{true};         // Là xe ô tô hay cọc tiêu/chướng ngại nhỏ
};

/// @brief Dữ liệu hành lang pháp lý mặt đường dọc theo chân trời dự đoán N bước
struct CorridorBoundsData {
    std::array<double, MPC_HORIZON> road_min{};        // Giới hạn tâm xe bên trái (m)
    std::array<double, MPC_HORIZON> road_max{};        // Giới hạn tâm xe bên phải (m)
    std::array<bool, MPC_HORIZON> is_yellow_solid{};   // Cờ vạch vàng liền chống lấn
    bool is_junction{false};                           // Đang đi qua giao lộ (bảo toàn biên vạch liền)
    bool is_roadblock{false};                          // Cờ phát hiện phong tỏa đường
    bool is_lateral_crossing{false};                   // Cờ có vật cản cắt ngang liên tục
    double roadblock_barrier_dist{0.0};                // Cự ly tới rào chắn (m)
};

/// @brief Dữ liệu Telemetry phục vụ giám sát HMI và ghi hộp đen SSD
struct alignas(8) TelemetryData {
    uint64_t timestamp_ns{0};
    uint32_t step_count{0};
    double x{0.0};
    double y{0.0};
    double yaw_rad{0.0};
    double v_ego_m_s{0.0};
    double v_ref_m_s{0.0};
    double lateral_error_m{0.0};
    double heading_error_rad{0.0};
    double steer_cmd{0.0};
    double throttle_cmd{0.0};
    double brake_cmd{0.0};
    double accel_cmd{0.0};
    double jerk_filtered_m_s3{0.0};
    double qp_cost{0.0};
    double solve_time_ms{0.0};
    double road_min_m{-0.45};
    double road_max_m{3.90};
    VehicleMode mode{VehicleMode::MODE_1_NORMAL};
    bool is_roadblock{false};
    char status_text[96]{0};
};

} // namespace av::types
