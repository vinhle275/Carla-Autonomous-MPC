#include "road_corridor.hpp"
#include "common/frenet_utils.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>

namespace av::perception {

RoadCorridorTracker::RoadCorridorTracker(double /*road_half_width*/)
    : r_clearance_(config::EGO_HALF_WIDTH_M + config::OBSTACLE_AVOIDANCE_MARGIN_M) {}

void RoadCorridorTracker::build_trajectory_corridor(const std::vector<double>& x_traj,
                                                    const std::vector<double>& y_traj,
                                                    const std::vector<double>& /*theta_traj*/,
                                                    const std::vector<double>& s_traj) {
    n_pts_ = x_traj.size();
    if (n_pts_ == 0) return;

    if (s_traj.size() == n_pts_) {
        s_traj_ = s_traj;
    } else {
        s_traj_.resize(n_pts_, 0.0);
        for (size_t i = 1; i < n_pts_; ++i) {
            s_traj_[i] = s_traj_[i - 1] + std::hypot(x_traj[i] - x_traj[i - 1], y_traj[i] - y_traj[i - 1]);
        }
    }

    // Mặc định khởi tạo theo hành lang chuẩn trước khi nạp bảng tiền tính toán OpenDRIVE
    road_min_traj_.assign(n_pts_, config::STANDARD_ROAD_MIN_M);
    road_max_traj_.assign(n_pts_, config::STANDARD_ROAD_MAX_M);
    is_yellow_solid_traj_.assign(n_pts_, true);
    has_precomputed_map_ = false;

    // Tự động nạp bảng tiền tính toán OpenDRIVE toàn cục (tương đương 100% build_trajectory_corridor trong Python)
    load_opendrive_corridor_cache();
}

bool RoadCorridorTracker::load_opendrive_corridor_cache(const std::string& cache_path) {
    if (n_pts_ == 0) return false;

    std::vector<std::string> candidates;
    if (!cache_path.empty()) {
        candidates.push_back(cache_path);
    }
    candidates.push_back("configs/trajectories/opendrive_corridor_cache.csv");
    candidates.push_back("../configs/trajectories/opendrive_corridor_cache.csv");
    candidates.push_back("configs\\trajectories\\opendrive_corridor_cache.csv");
    candidates.push_back("..\\configs\\trajectories\\opendrive_corridor_cache.csv");

    std::ifstream ifs;
    std::string loaded_file;
    for (const auto& c : candidates) {
        ifs.open(c);
        if (ifs.is_open()) {
            loaded_file = c;
            break;
        }
        ifs.clear();
    }
    if (!ifs.is_open()) return false;

    std::vector<double> rmin_vec;
    std::vector<double> rmax_vec;
    std::vector<bool> yl_vec;
    rmin_vec.reserve(n_pts_);
    rmax_vec.reserve(n_pts_);
    yl_vec.reserve(n_pts_);

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        if (line[0] == '#' || (line[0] >= 'a' && line[0] <= 'z') || (line[0] >= 'A' && line[0] <= 'Z')) {
            continue; // Bỏ qua dòng header
        }
        std::stringstream ss(line);
        std::string s_min, s_max, s_yl;
        if (std::getline(ss, s_min, ',') && std::getline(ss, s_max, ',')) {
            std::getline(ss, s_yl, ',');
            try {
                double rm = std::stod(s_min);
                double rx = std::stod(s_max);
                int yl = s_yl.empty() ? 1 : std::stoi(s_yl);
                rmin_vec.push_back(std::min(rm, config::STANDARD_ROAD_MIN_M));
                rmax_vec.push_back(std::max(rx, 0.45));
                yl_vec.push_back(yl != 0);
            } catch (...) {
                continue;
            }
        }
    }

    if (rmin_vec.size() == n_pts_) {
        road_min_traj_ = std::move(rmin_vec);
        road_max_traj_ = std::move(rmax_vec);
        is_yellow_solid_traj_ = std::move(yl_vec);
        has_precomputed_map_ = true;
        auto [min_it, max_it] = std::minmax_element(road_min_traj_.begin(), road_min_traj_.end());
        auto [min_rx, max_rx] = std::minmax_element(road_max_traj_.begin(), road_max_traj_.end());
        std::cout << "[ROAD CORRIDOR C++] Đã nạp hành lang động OpenDRIVE cho " << n_pts_
                  << " waypoints từ " << loaded_file
                  << " (road_min=[" << *min_it << "m.." << *max_it << "m], road_max=["
                  << *min_rx << "m.." << *max_rx << "m])" << std::endl;
        return true;
    }
    return false;
}

void RoadCorridorTracker::update_live_map_bounds(size_t current_idx,
                                                 double map_road_min,
                                                 double map_road_max,
                                                 bool map_is_yellow_solid,
                                                 bool /*is_junction*/) {
    if (n_pts_ == 0 || current_idx >= n_pts_) return;

    // Nếu chưa nạp được bảng OpenDRIVE toàn cục, thử nạp lại ngay khi Bridge vừa khởi tạo xong
    if (!has_precomputed_map_) {
        if (load_opendrive_corridor_cache()) {
            return;
        }
    } else {
        // Khi đã có bảng tiền tính toán OpenDRIVE toàn cục (giống 100% src/road_corridor.py),
        // TUYỆT ĐỐI KHÔNG ghi đè biên của waypoint hiện tại lên các waypoint phía trước,
        // để giữ nguyên độ dịch chuyển làn chéo (delta_min, delta_max) và biên tại vị trí vật cản!
        return;
    }

    double safe_min = std::min(map_road_min, config::STANDARD_ROAD_MIN_M);
    double safe_max = std::max(map_road_max, 0.45);
    road_min_traj_[current_idx] = safe_min;
    road_max_traj_[current_idx] = safe_max;
    is_yellow_solid_traj_[current_idx] = map_is_yellow_solid;
}

CorridorResult RoadCorridorTracker::get_corridor_at_idx(size_t idx) const {
    CorridorResult res;
    if (n_pts_ > 0) {
        size_t clamped_idx = std::min(idx, n_pts_ - 1);
        res.road_min = road_min_traj_[clamped_idx];
        res.road_max = road_max_traj_[clamped_idx];
        res.is_yellow_solid = is_yellow_solid_traj_[clamped_idx];
    }
    return res;
}

CorridorResult RoadCorridorTracker::get_corridor_at_s(double s_query) const {
    if (n_pts_ == 0 || s_traj_.empty()) return CorridorResult{};
    auto it = std::lower_bound(s_traj_.begin(), s_traj_.end(), s_query);
    size_t idx = std::min(static_cast<size_t>(std::distance(s_traj_.begin(), it)), n_pts_ - 1);
    return get_corridor_at_idx(idx);
}

types::CorridorBoundsData RoadCorridorTracker::get_horizon_corridor(size_t start_idx, size_t horizon_len) const {
    types::CorridorBoundsData data;
    size_t len = std::min(horizon_len, types::MPC_HORIZON);
    for (size_t k = 0; k < len; ++k) {
        size_t idx = std::min(start_idx + k, n_pts_ > 0 ? (n_pts_ - 1) : 0);
        if (idx < n_pts_) {
            data.road_min[k] = road_min_traj_[idx];
            data.road_max[k] = road_max_traj_[idx];
            data.is_yellow_solid[k] = is_yellow_solid_traj_[idx];
        } else {
            data.road_min[k] = config::STANDARD_ROAD_MIN_M;
            data.road_max[k] = config::STANDARD_ROAD_MAX_M;
            data.is_yellow_solid[k] = true;
        }
    }
    return data;
}

types::CorridorBoundsData RoadCorridorTracker::get_horizon_corridor_at_s(const std::vector<double>& s_queries) const {
    types::CorridorBoundsData data;
    for (size_t k = 0; k < types::MPC_HORIZON; ++k) {
        if (k < s_queries.size()) {
            auto c = get_corridor_at_s(s_queries[k]);
            data.road_min[k] = c.road_min;
            data.road_max[k] = c.road_max;
            data.is_yellow_solid[k] = c.is_yellow_solid;
        } else {
            data.road_min[k] = config::STANDARD_ROAD_MIN_M;
            data.road_max[k] = config::STANDARD_ROAD_MAX_M;
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
