// ==============================================================================
// EMERGENCY SAFETY MANAGER & 2-MODE FINITE STATE MACHINE
// Modern C++20 | Safety Guard State Machine
// ==============================================================================

#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include "common/autosar_types.hpp"
#include "common/config_constants.hpp"
#include "collision_danger_zone.hpp"

namespace av::safety {

struct SafetyStatusInfo {
    types::VehicleMode mode{types::VehicleMode::MODE_1_NORMAL};
    bool is_emergency{false};
    bool mode_changed{false};
    int danger_count{0};
    double clear_timer{0.0};
    double hold_time{av::config::DANGER_CLEAR_HOLD_TIME_S};
    std::string status_text;
};

class EmergencySafetyManager {
public:
    EmergencySafetyManager(double zone_width = av::config::DANGER_ZONE_WIDTH_M,
                           double zone_length = av::config::DANGER_ZONE_LENGTH_M,
                           double clear_hold_time_s = av::config::DANGER_CLEAR_HOLD_TIME_S,
                           bool trigger_only_moving = true,
                           double moving_speed_threshold_m_s = av::config::DANGER_MOVING_SPEED_THRESHOLD_MS);

    void reset();

    SafetyStatusInfo update(double ego_x, double ego_y, double ego_yaw_rad, double ego_v,
                            const std::vector<types::ObstacleFrenetOBB>& detected_obstacles,
                            double dt);

    types::VehicleMode get_current_mode() const { return current_mode_; }
    bool is_emergency() const { return current_mode_ == types::VehicleMode::MODE_2_EMERGENCY_BRAKE; }

private:
    CollisionDangerZone zone_;
    double clear_hold_time_s_;
    bool trigger_only_moving_;
    double moving_speed_threshold_m_s_;

    types::VehicleMode current_mode_{types::VehicleMode::MODE_1_NORMAL};
    double clear_timer_{0.0};
    int mode_switches_{0};
    double total_emergency_time_{0.0};
};

} // namespace av::safety
