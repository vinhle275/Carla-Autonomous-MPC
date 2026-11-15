#include "road_corridor.hpp"
#include <algorithm>
#include <cmath>

namespace av::perception {

RoadCorridorTracker::RoadCorridorTracker(double road_half_width)
    : r_clearance_(config::EGO_HALF_WIDTH_M + config::OBSTACLE_AVOIDANCE_MARGIN_M) {}

void RoadCorridorTracker::build_trajectory_corridor(const std::vector<double>& x_traj,
                                                    const std::vector<double>& y_traj,
                                                    const std::vector<double>& theta_traj) {
    n_pts_ = x_traj.size();
    if (n_pts_ == 0) return;

    road_min_traj_.assign(n_pts_, -config::DEFAULT_ROAD_HALF_WIDTH_M + r_clearance_);
    road_max_traj_.assign(n_pts_,  config::DEFAULT_ROAD_HALF_WIDTH_M - r_clearance_);
    is_yellow_solid_traj_.assign(n_pts_, true); // Mặc định vạch vàng nét liền phân cách làn đối diện bên trái

    // Tiền tính toán biên pháp lý chuẩn: Với làn 3.5m (Làn 3 trong đường 4 làn):
    // Vạch vàng ở -1.75m -> road_min = -1.75 + 1.35 = -0.40m
    // Mép phải ở +5.25m -> road_max = 5.25 - 1.35 = +3.90m
    for (size_t i = 0; i < n_pts_; ++i) {
        road_min_traj_[i] = -0.45;
        road_max_traj_[i] =  3.90;
        is_yellow_solid_traj_[i] = true;
    }
}

CorridorResult RoadCorridorTracker::get_corridor_at_idx(size_t idx) const {
    CorridorResult res;
    if (idx < n_pts_) {
        res.road_min = road_min_traj_[idx];
        res.road_max = road_max_traj_[idx];
        res.is_yellow_solid = is_yellow_solid_traj_[idx];
    }
    return res;
}

types::CorridorBoundsData RoadCorridorTracker::get_horizon_corridor(size_t start_idx, size_t horizon_len) const {
    types::CorridorBoundsData data;
    for (size_t k = 0; k < horizon_len; ++k) {
        size_t idx = std::min(start_idx + k, n_pts_ > 0 ? (n_pts_ - 1) : 0);
        if (idx < n_pts_) {
            data.road_min[k] = road_min_traj_[idx];
            data.road_max[k] = road_max_traj_[idx];
            data.is_yellow_solid[k] = is_yellow_solid_traj_[idx];
        } else {
            data.road_min[k] = -0.45;
            data.road_max[k] = 3.90;
            data.is_yellow_solid[k] = true;
        }
    }
    return data;
}

std::vector<std::pair<double, double>> RoadCorridorTracker::find_free_gaps_in_corridor(
    const std::vector<std::pair<double, double>>& blocked_intervals,
    double road_min, double road_max, double min_gap_width) {
    if (road_min >= road_max) return {};
    if (blocked_intervals.empty()) {
        if (road_max - road_min >= min_gap_width) return {{road_min, road_max}};
        return {};
    }

    auto sorted_iv = blocked_intervals;
    std::sort(sorted_iv.begin(), sorted_iv.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });

    std::vector<std::pair<double, double>> merged;
    for (const auto& iv : sorted_iv) {
        if (merged.empty() || iv.first > merged.back().second) {
            merged.push_back(iv);
        } else {
            merged.back().second = std::max(merged.back().second, iv.second);
        }
    }

    std::vector<std::pair<double, double>> free_gaps;
    double cursor = road_min;
    for (const auto& b : merged) {
        if (b.first > cursor) {
            free_gaps.emplace_back(cursor, std::min(b.first, road_max));
        }
        cursor = std::max(cursor, b.second);
        if (cursor >= road_max) break;
    }
    if (cursor < road_max) {
        free_gaps.emplace_back(cursor, road_max);
    }

    std::vector<std::pair<double, double>> valid_gaps;
    for (const auto& g : free_gaps) {
        if (g.second - g.first >= min_gap_width) {
            valid_gaps.push_back(g);
        }
    }
    return valid_gaps;
}

} // namespace av::perception
