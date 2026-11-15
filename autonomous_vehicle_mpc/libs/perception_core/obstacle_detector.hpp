// ==============================================================================
// DUAL-ZONE PERCEPTION & PERSISTENT LOCAL OBJECT MAP (MOT)
// Modern C++20 | OBB 2D, Velocity Estimation & Dead-Reckoning Memory
// ==============================================================================

#pragma once

#include <vector>
#include <unordered_map>
#include <cmath>
#include "common/autosar_types.hpp"
#include "common/config_constants.hpp"

namespace av::perception {

#pragma pack(push, 1)
struct RawActorData {
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
#pragma pack(pop)

class ObstacleDetector {
public:
    ObstacleDetector(double detection_range = config::SENSOR_DETECTION_RANGE_M,
                     double fov_deg = config::SENSOR_FOV_DEG,
                     double surround_range = config::SENSOR_SURROUND_RANGE_M,
                     double track_drop_behind = config::SENSOR_TRACK_DROP_BEHIND_M);

    void clear_memory();

    static std::pair<double, double> get_closest_point_on_obb(double px, double py,
                                                             double box_cx, double box_cy,
                                                             double box_yaw_rad,
                                                             double half_len, double half_wid);

    std::vector<types::ObstacleFrenetOBB> process_scan(double ego_x, double ego_y, double ego_yaw_rad,
                                                      const std::vector<RawActorData>& raw_actors,
                                                      const std::vector<double>& x_traj,
                                                      const std::vector<double>& y_traj,
                                                      const std::vector<double>& theta_traj,
                                                      const std::vector<double>& s_traj,
                                                      size_t current_idx,
                                                      double dt);

private:
    double detection_range_;
    double fov_deg_;
    double half_fov_rad_;
    double surround_range_;
    double track_drop_behind_;

    std::unordered_map<int32_t, types::ObstacleFrenetOBB> tracked_obstacles_;
    std::unordered_map<int32_t, std::pair<double, double>> prev_positions_;
};

} // namespace av::perception
