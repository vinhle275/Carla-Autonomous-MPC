// ==============================================================================
// LANE LINE PERCEPTION & ANALYTIC IPM
// Modern C++20 | HSV Color Segmentation, Ray-Ground IPM & Sliding Window
// ==============================================================================

#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <cmath>
#include "common/autosar_types.hpp"
#include "common/config_constants.hpp"

namespace av::perception {

struct LaneDetectionResult {
    types::LineType line_type{types::LineType::NONE};
    double d_yellow_line_m{1.75};
    double road_min_m{config::STANDARD_ROAD_MIN_M};
    double road_max_m{config::STANDARD_ROAD_MAX_M};
    bool is_yellow_solid{true};
    double confidence{0.0};
    double detect_time_ms{0.0};
    std::string source{"NONE"}; // "CV" hoặc "MAP"
};

class YellowLineDetector {
public:
    YellowLineDetector(int img_w = config::CAM_IMG_W, int img_h = config::CAM_IMG_H);

    /// @brief Xử lý trực tiếp buffer ảnh RGB từ Front Hood-Cam bằng Analytic IPM + HSV + Sliding Window + Map Fallback
    LaneDetectionResult detect_from_rgb_image(const uint8_t* rgb_data, int width, int height,
                                              double ego_x, double ego_y, double ego_yaw_rad,
                                              double ref_x, double ref_y, double ref_theta,
                                              bool map_has_yellow_solid = true,
                                              bool is_roadblock = false,
                                              bool is_junction = false,
                                              double dt = config::DT_CORE,
                                              double map_road_min = config::STANDARD_ROAD_MIN_M,
                                              double map_road_max = config::STANDARD_ROAD_MAX_M);

    /// @brief Nhận diện kết hợp dữ liệu bản đồ OpenDRIVE động + Hysteresis bảo vệ khi qua giao lộ hoặc dừng xe
    LaneDetectionResult detect_from_features(double ego_x, double ego_y, double ego_yaw_rad,
                                            double ref_x, double ref_y, double ref_theta,
                                            bool map_has_yellow_solid = true,
                                            bool is_roadblock = false,
                                            bool is_junction = false,
                                            double dt = config::DT_CORE,
                                            double map_road_min = config::STANDARD_ROAD_MIN_M,
                                            double map_road_max = config::STANDARD_ROAD_MAX_M);

    void reset_hold() { hold_timer_s_ = 0.0; }

private:
    struct DetectedBeVLine {
        double d_ego_m{0.0};
        bool is_yellow{false};
        types::LineType line_type{types::LineType::NONE};
        double fill_ratio{0.0};
    };

    static void rgb_to_hsv_opencv(uint8_t r, uint8_t g, uint8_t b, int& out_h, int& out_s, int& out_v);

    bool pixel_to_ground_local(int u, int v, int width, int height, double& out_x_local, double& out_y_local) const;
    static void ground_to_pixel(double X, double Y, int width, int height, int& out_u, int& out_v);

    int img_w_;
    int img_h_;
    double hold_timer_s_{config::YELLOW_SOLID_HOLD_TIME_S};
    const double hold_max_time_s_{config::YELLOW_SOLID_HOLD_TIME_S};
    double last_valid_road_min_{config::STANDARD_ROAD_MIN_M};
    double last_valid_road_max_{config::STANDARD_ROAD_MAX_M};
    bool has_ema_{false};
    double ema_min_{config::STANDARD_ROAD_MIN_M};
    double ema_max_{config::STANDARD_ROAD_MAX_M};
    double prev_map_min_{config::STANDARD_ROAD_MIN_M};
    double prev_map_max_{config::STANDARD_ROAD_MAX_M};
};

} // namespace av::perception
