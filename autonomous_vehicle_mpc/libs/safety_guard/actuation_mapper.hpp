// ==============================================================================
// ACTUATION OUTPUT MAPPER (THROTTLE, BRAKE, STEER)
// Modern C++20 | Traction Control, Anti-Lock Cornering, Static Holding
// Synchronized with CarlaSimulator.process_control_inputs in carla_simulator.py
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
                                      double current_speed_m_s,
                                      bool is_roadblock_latch = false,
                                      bool is_roadblock_braking = false) {
        ActuatorOutput out;

        // 0. Chốt dừng tĩnh Roadblock (Limousine Static Hold): khóa thẳng lái, giữ phanh tĩnh 0.18
        if (is_roadblock_latch || (current_speed_m_s < 0.20 && acceleration_m_s2 <= -1.0 && std::abs(wheel_angle_rad) < 0.02)) {
            out.throttle = 0.0;
            out.brake = 0.18;
            out.steer = 0.0;
            return out;
        }

        // 1. Chống trượt bánh (Traction Control) khi vừa khởi động từ tốc độ 0
        if (current_speed_m_s < 0.8) {
            acceleration_m_s2 = std::min(acceleration_m_s2, 2.5);
        }

        // Bù lực cản lăn + cản gió CARLA liên tục khi xe đang chạy (không áp dụng khi dừng Roadblock)
        double a_eff = acceleration_m_s2;
        if (!is_roadblock_braking && current_speed_m_s > 0.3 && acceleration_m_s2 > -0.35) {
            double a_drag_ff = 0.72 + 0.12 * current_speed_m_s;
            double ff_weight = std::clamp((acceleration_m_s2 + 0.35) / 0.35, 0.0, 1.0);
            a_eff = acceleration_m_s2 + ff_weight * a_drag_ff;
        }
        if (is_roadblock_braking) {
            a_eff = std::min(0.0, acceleration_m_s2);
        }

        // 2. Phân nhánh Ga / Phanh
        if (a_eff <= 0.0) {
            out.throttle = 0.0;
            bool is_turning = std::abs(wheel_angle_rad) > 0.08;

            if (is_turning && current_speed_m_s > 0.8 && acceleration_m_s2 >= -0.35) {
                // Giảm tốc nhẹ khi ôm cua: Nhả ga tự nhiên (coasting), không đạp phanh
                out.brake = 0.0;
            } else if (is_turning && current_speed_m_s > 0.8 && acceleration_m_s2 >= -1.8) {
                // Rà phanh điều tốc vừa phải trong cua: Giới hạn phanh êm <= 0.12 chống khóa bánh
                double b = (std::abs(acceleration_m_s2) - 0.35) / (std::abs(config::MAX_BRAKING_MS2) * 2.2);
                out.brake = std::min(b, 0.12);
            } else if (current_speed_m_s < 2.5 && acceleration_m_s2 >= -1.2) {
                // Đệm phanh êm tốc độ thấp (Limousine Stop Feathering chuẩn carla_simulator.py): giới hạn <= 0.18
                out.brake = std::min(std::abs(acceleration_m_s2 / config::MAX_BRAKING_MS2), 0.18);
            } else if (!is_roadblock_braking) {
                // Giảm tốc điều tốc thông thường (tránh phanh thủy lực CARLA khóa bánh dừng đột ngột khi xuống 6 m/s)
                double b = std::max(0.0, std::abs(acceleration_m_s2) - 0.25) / (std::abs(config::MAX_BRAKING_MS2) * 1.35);
                out.brake = std::min(b, 0.25);
            } else {
                // Khi dừng Roadblock: Áp dụng đầy đủ lực phanh tỉ lệ
                out.brake = std::min(std::abs(acceleration_m_s2 / config::MAX_BRAKING_MS2), 1.0);
            }
        } else {
            out.throttle = std::clamp(a_eff / config::THROTTLE_ACCEL_SCALE_MS2, 0.0, 1.0);
            out.brake = 0.0;
        }

        // 3. Chuẩn hóa góc lái [-1.0, 1.0]
        out.steer = std::clamp(wheel_angle_rad / config::MAX_WHEEL_ANGLE_RAD, -1.0, 1.0);
        return out;
    }
};

} // namespace av::safety
