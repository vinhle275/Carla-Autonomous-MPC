#include "emergency_safety_manager.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>

namespace av::safety {

EmergencySafetyManager::EmergencySafetyManager(double zone_width,
                                               double zone_length,
                                               double clear_hold_time_s,
                                               bool trigger_only_moving,
                                               double moving_speed_threshold_m_s)
    : zone_(zone_width, zone_length, av::config::EGO_FRONT_OVERHANG_M),
      clear_hold_time_s_(clear_hold_time_s),
      trigger_only_moving_(trigger_only_moving),
      moving_speed_threshold_m_s_(moving_speed_threshold_m_s) {}

void EmergencySafetyManager::reset() {
    current_mode_ = types::VehicleMode::MODE_1_NORMAL;
    clear_timer_ = 0.0;
    mode_switches_ = 0;
    total_emergency_time_ = 0.0;
}

SafetyStatusInfo EmergencySafetyManager::update(double ego_x, double ego_y, double ego_yaw_rad, double /*ego_v*/,
                                                const std::vector<types::ObstacleFrenetOBB>& detected_obstacles,
                                                double dt) {
    SafetyStatusInfo info;
    std::vector<types::ObstacleFrenetOBB> triggering_obstacles;

    for (const auto& obs : detected_obstacles) {
        ZoneCheckResult check = zone_.is_obstacle_in_zone(obs, ego_x, ego_y, ego_yaw_rad);
        if (check.is_inside) {
            bool is_moving = obs.is_moving || (std::hypot(obs.vx, obs.vy) >= moving_speed_threshold_m_s_);
            if (!trigger_only_moving_ || is_moving) {
                triggering_obstacles.push_back(obs);
            }
        }
    }

    info.danger_count = static_cast<int>(triggering_obstacles.size());
    bool mode_changed = false;

    if (current_mode_ == types::VehicleMode::MODE_1_NORMAL) {
        // Chuyển sang Chế độ 2 nếu có vật cản xâm phạm vùng nguy hiểm
        if (!triggering_obstacles.empty()) {
            current_mode_ = types::VehicleMode::MODE_2_EMERGENCY_BRAKE;
            clear_timer_ = 0.0;
            mode_switches_++;
            mode_changed = true;
            std::cout << "\n[CẢNH BÁO NGUY HIỂM] Vật cản #" << triggering_obstacles[0].id
                      << " xâm phạm Danger Zone trước đầu xe! -> CHUYỂN SANG CHẾ ĐỘ 2 (PHANH GẤP)!" << std::endl;
        }
    } else { // MODE_2_EMERGENCY_BRAKE
        total_emergency_time_ += dt;
        if (triggering_obstacles.empty()) {
            clear_timer_ += dt;
            if (clear_timer_ + 1e-4 >= clear_hold_time_s_) {
                current_mode_ = types::VehicleMode::MODE_1_NORMAL;
                clear_timer_ = 0.0;
                mode_switches_++;
                mode_changed = true;
                std::cout << "\n[AN TOÀN TRỞ LẠI] Vùng nguy hiểm thông thoáng liên tục "
                          << clear_hold_time_s_ << "s -> CHUYỂN VỀ CHẾ ĐỘ 1 (BÁM ĐƯỜNG NÉ VẬT CẢN)!" << std::endl;
            }
        } else {
            clear_timer_ = 0.0;
        }
    }

    info.mode = current_mode_;
    info.is_emergency = (current_mode_ == types::VehicleMode::MODE_2_EMERGENCY_BRAKE);
    info.mode_changed = mode_changed;
    info.clear_timer = clear_timer_;
    info.hold_time = clear_hold_time_s_;

    std::ostringstream ss;
    if (info.is_emergency) {
        if (!triggering_obstacles.empty()) {
            ss << "PHANH GẤP [CHẾ ĐỘ 2] - VẬT CẢN (#" << triggering_obstacles[0].id << ")";
        } else {
            ss << std::fixed << std::setprecision(1)
               << "PHANH GẤP [CHỜ AN TOÀN: " << std::max(0.0, clear_hold_time_s_ - clear_timer_) << "s]";
        }
    } else {
        ss << "CHẾ ĐỘ 1 (BÁM ĐƯỜNG NÉ VẬT CẢN)";
    }
    info.status_text = ss.str();

    return info;
}

} // namespace av::safety
