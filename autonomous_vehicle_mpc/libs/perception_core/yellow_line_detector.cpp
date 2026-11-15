#include "yellow_line_detector.hpp"
#include <algorithm>
#include <cmath>

namespace av::perception {

YellowLineDetector::YellowLineDetector(int img_w, int img_h)
    : img_w_(img_w), img_h_(img_h) {}

LaneDetectionResult YellowLineDetector::detect_from_features(double ego_x, double ego_y, double ego_yaw_rad,
                                                            double ref_x, double ref_y, double ref_theta,
                                                            bool map_has_yellow_solid) {
    LaneDetectionResult res;

    // Chiếu vị trí xe so với tim làn
    double nx = -std::sin(ref_theta);
    double ny = std::cos(ref_theta);
    double d_lat = (ego_x - ref_x) * nx + (ego_y - ref_y) * ny;

    // Vị trí vạch vàng nét liền ở làn 3 chuẩn (-1.75m so với tim làn)
    double yellow_pos = -1.75;
    double dist_to_yellow = d_lat - yellow_pos; // Khoảng cách từ tâm xe tới vạch vàng

    res.d_yellow_line_m = dist_to_yellow;

    if (map_has_yellow_solid) {
        hold_counter_ = hold_max_steps_;
    } else if (hold_counter_ > 0) {
        hold_counter_--;
    }

    if (hold_counter_ > 0) {
        res.is_yellow_solid = true;
        res.line_type = types::LineType::SOLID;
        res.road_min_m = yellow_pos + config::YELLOW_LINE_CLEARANCE_M + config::EGO_HALF_WIDTH_M; // -0.40m -> -0.45m
        res.road_max_m = 3.90;
        res.confidence = 0.95;
    } else {
        res.is_yellow_solid = false;
        res.line_type = types::LineType::DASHED;
        res.road_min_m = -3.90;
        res.road_max_m = 3.90;
        res.confidence = 0.80;
    }

    return res;
}

} // namespace av::perception
