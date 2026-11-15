// ==============================================================================
// ROAD CORRIDOR TRACKER & DYNAMIC DRIVABLE BOUNDS
// Modern C++20 | OpenDRIVE & Geometry-based Corridor Tracking
// ==============================================================================

#pragma once

#include <vector>
#include <array>
#include <algorithm>
#include "common/autosar_types.hpp"
#include "common/config_constants.hpp"

namespace av::perception {

struct CorridorResult {
    double road_min{-3.90};
    double road_max{3.90};
    double d_left_legal{-5.25};
    double d_right_legal{5.25};
    bool is_yellow_solid{false};
    double lane_width{3.5};
};

class RoadCorridorTracker {
public:
    RoadCorridorTracker(double road_half_width = config::DEFAULT_ROAD_HALF_WIDTH_M);

    void build_trajectory_corridor(const std::vector<double>& x_traj,
                                   const std::vector<double>& y_traj,
                                   const std::vector<double>& theta_traj);

    CorridorResult get_corridor_at_idx(size_t idx) const;

    types::CorridorBoundsData get_horizon_corridor(size_t start_idx, size_t horizon_len = config::MPC_N) const;

    static std::vector<std::pair<double, double>> find_free_gaps_in_corridor(
        const std::vector<std::pair<double, double>>& blocked_intervals,
        double road_min = -config::DEFAULT_ROAD_HALF_WIDTH_M,
        double road_max = config::DEFAULT_ROAD_HALF_WIDTH_M,
        double min_gap_width = 0.20
    );

private:
    double r_clearance_{config::R_CLEARANCE_EDGE};
    std::vector<double> road_min_traj_;
    std::vector<double> road_max_traj_;
    std::vector<bool> is_yellow_solid_traj_;
    size_t n_pts_{0};
};

} // namespace av::perception
