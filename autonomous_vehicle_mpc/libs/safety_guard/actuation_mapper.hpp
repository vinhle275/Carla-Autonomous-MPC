// ==============================================================================
// ACTUATION OUTPUT MAPPER (THROTTLE, BRAKE, STEER)
// Modern C++20 | Traction Control, Anti-Lock Cornering, Static Holding
// ==============================================================================

#pragma once

#include <tuple>
#include <algorithm>
#include <cmath>
#include "common/config_constants.hpp"

namespace av::safety {

struct ActuatorOutput {
    double throttle{0.0}; // [0.0, 1.0]
    double brake{0.0};    // [0.0, 1.0]
    double steer{0.0};    // [-1.0, 1.0]
};

class ActuationMapper {
public:
    static ActuatorOutput map_controls(double wheel_angle_rad,
                                      double acceleration_m_s2,
                                      double current_speed_m_s) {
        ActuatorOutput out;

        // 1. Chống trượt bánh (Traction Control) khi vừa khởi động từ tốc độ 0
        if (current_speed_m_s < 0.8) {
            acceleration_m_s2 = std::min(acceleration_m_s2, 2.5);
        }

        // 2. Phân nhánh Ga / Phanh
        if (acceleration_m_s2 <= 0.0) {
            out.throttle = 0.0;
            bool is_turning = std::abs(wheel_angle_rad) > 0.08;

            if (is_turning && current_speed_m_s > 0.8 && acceleration_m_s2 >= -0.35) {
                // Giảm tốc nhẹ khi ôm cua: Nhả ga tự nhiên (coasting), không đạp phanh
                out.brake = 0.0;
            } else if (is_turning && current_speed_m_s > 0.8 && acceleration_m_s2 >= -1.8) {
                // Rà phanh điều tốc vừa phải trong cua: Đệm phanh êm chống khóa bánh trước
                double b = (std::abs(acceleration_m_s2) - 0.35) / (std::abs(config::MAX_BRAKING_MS2) * 2.2);
                out.brake = std::min(b, 0.12);
            } else if (current_speed_m_s < 2.5 && acceleration_m_s2 >= -1.2) {
                // Giữ phanh tĩnh khi dừng Roadblock / tốc độ rất thấp
                out.brake = std::min(std::abs(acceleration_m_s2 / config::MAX_BRAKING_MS2), 0.18);
            } else {
                // Phanh gắt hoặc dừng khẩn cấp
                out.brake = std::min(std::abs(acceleration_m_s2 / config::MAX_BRAKING_MS2), 1.0);
            }
        } else {
            out.throttle = std::min(acceleration_m_s2 / config::MAX_ACCELERATION_MS2, 1.0);
            out.brake = 0.0;
        }

        // 3. Chuẩn hóa góc lái [-1.0, 1.0]
        out.steer = std::clamp(wheel_angle_rad / config::MAX_WHEEL_ANGLE_RAD, -1.0, 1.0);
        return out;
    }
};

} // namespace av::safety
