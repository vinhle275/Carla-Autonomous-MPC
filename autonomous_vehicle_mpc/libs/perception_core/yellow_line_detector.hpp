// ==============================================================================
// LANE LINE PERCEPTION & ANALYTIC IPM
// Modern C++20 | HSV Color Segmentation & Sliding Window
// ==============================================================================

#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include "common/autosar_types.hpp"
#include "common/config_constants.hpp"

namespace av::perception {

struct LaneDetectionResult {
    types::LineType line_type{types::LineType::NONE};
    double d_yellow_line_m{0.0};
    double road_min_m{-0.45};
    double road_max_m{3.90};
    bool is_yellow_solid{false};
    double confidence{0.0};
};

class YellowLineDetector {
public:
    YellowLineDetector(int img_w = config::CAM_IMG_W, int img_h = config::CAM_IMG_H);

    LaneDetectionResult detect_from_features(double ego_x, double ego_y, double ego_yaw_rad,
                                            double ref_x, double ref_y, double ref_theta,
                                            bool map_has_yellow_solid = true);

    void reset_hold() { hold_counter_ = 0; }

private:
    int img_w_;
    int img_h_;
    int hold_counter_{0};
    const int hold_max_steps_{24}; // 1.2s tại 50Hz
};

} // namespace av::perception
