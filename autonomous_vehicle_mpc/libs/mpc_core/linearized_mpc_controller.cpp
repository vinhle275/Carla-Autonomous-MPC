#include "linearized_mpc_controller.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace av::mpc {

LinearizedMpcController::LinearizedMpcController(int horizon, double dt_mpc)
    : horizon_(horizon), dt_(dt_mpc) {
    x_nom_.resize(horizon_ + 1, {0.0, 0.0, 0.0, 0.0});
    u_nom_.resize(horizon_, {0.0, 0.0});
    x_ref_mod_.resize(horizon_, 0.0);
    y_ref_mod_.resize(horizon_, 0.0);
    theta_ref_mod_.resize(horizon_, 0.0);
    s_queries_.resize(horizon_, 0.0);
    buffer_steer_.assign(horizon_, 0.0);
    buffer_accel_.assign(horizon_, 0.0);
}

void LinearizedMpcController::set_global_trajectory(const std::vector<double>& x,
                                                   const std::vector<double>& y,
                                                   const std::vector<double>& theta,
                                                   const std::vector<double>& v) {
    x_traj_ = x;
    y_traj_ = y;
    theta_traj_ = theta;
    v_traj_ = v;

    if (x_traj_.size() > 1) {
        s_traj_.resize(x_traj_.size(), 0.0);
        for (size_t i = 1; i < x_traj_.size(); ++i) {
            s_traj_[i] = s_traj_[i - 1] + std::hypot(x_traj_[i] - x_traj_[i - 1], y_traj_[i] - y_traj_[i - 1]);
        }
    } else {
        s_traj_.assign(x_traj_.size(), 0.0);
    }
}

void LinearizedMpcController::reset_nominal() {
    nom_initialized_ = false;
    for (auto& st : x_nom_) st = {0.0, 0.0, 0.0, 0.0};
    for (auto& ct : u_nom_) ct = {0.0, 0.0};
    u_prev_ = {0.0, 0.0};
    roadblock_latched_ids_.clear();
    roadblock_s_barrier_ = std::nullopt;
    roadblock_stopped_latch_ = false;
    is_roadblock_ = false;
    roadblock_reason_.clear();
    has_active_obs_ = false;
    active_chain_.clear();
    lateral_crossing_timer_s_.clear();
    std::fill(buffer_steer_.begin(), buffer_steer_.end(), 0.0);
    std::fill(buffer_accel_.begin(), buffer_accel_.end(), 0.0);
    buffer_idx_ = 0;
    qp_solver_.reset_warm_start();
}

void LinearizedMpcController::update_yellow_line_constraint(
    const perception::LaneDetectionResult& yellow_result,
    double dt_loop) {

    cv_source_ = yellow_result.source;
    cv_confidence_ = yellow_result.confidence;

    if (yellow_result.source == "CV" && yellow_result.confidence >= 0.80) {
        if (!cv_road_min_.has_value() || std::abs(yellow_result.road_min_m - *cv_road_min_) > 0.05) {
            cv_road_min_ = yellow_result.road_min_m;
        }
        if (!cv_road_max_.has_value() || std::abs(yellow_result.road_max_m - *cv_road_max_) > 0.05) {
            cv_road_max_ = yellow_result.road_max_m;
        }
    }

    if (yellow_result.line_type == types::LineType::NONE) {
        // Khi xe đang dừng Roadblock trước vật cản, vật cản che khuất camera mũi xe -> giữ nguyên trạng thái vạch vàng!
        if (yellow_solid_hold_timer_s_ > 0.0 && !is_roadblock_) {
            yellow_solid_hold_timer_s_ = std::max(0.0, yellow_solid_hold_timer_s_ - dt_loop);
        }
        if (yellow_solid_hold_timer_s_ <= 0.0 && !is_roadblock_) {
            yellow_solid_active_ = false;
            yellow_road_limit_ = std::nullopt;
            yellow_blocks_left_ = false;
        }
        return;
    }

    yellow_d_line_ = yellow_result.d_yellow_line_m;

    if (yellow_result.line_type == types::LineType::SOLID) {
        yellow_solid_active_ = true;
        yellow_blocks_left_ = true;
        yellow_solid_hold_timer_s_ = config::YELLOW_SOLID_HOLD_TIME_S;
        yellow_road_limit_ = yellow_result.road_min_m;
    } else if (yellow_result.line_type == types::LineType::DASHED) {
        yellow_solid_hold_timer_s_ = std::max(0.0, yellow_solid_hold_timer_s_ - 4.0 * dt_loop);
        if (yellow_solid_hold_timer_s_ <= 0.0) {
            yellow_solid_active_ = false;
            yellow_road_limit_ = std::nullopt;
            yellow_blocks_left_ = false;
        }
    }
}

void LinearizedMpcController::get_legal_corridor_bounds(
    std::optional<size_t> idx,
    std::optional<double> s,
    const types::CorridorBoundsData& corridor_data,
    double& out_road_min,
    double& out_road_max,
    bool& out_is_yellow_solid) {

    double road_min = config::STANDARD_ROAD_MIN_M;
    double road_max = config::STANDARD_ROAD_MAX_M;
    bool is_map_yl = true;

    if (corridor_tracker_ != nullptr) {
        perception::CorridorResult cr;
        if (idx.has_value()) {
            cr = corridor_tracker_->get_corridor_at_idx(*idx);
        } else if (s.has_value()) {
            cr = corridor_tracker_->get_corridor_at_s(*s);
        } else {
            cr = corridor_tracker_->get_corridor_at_idx(current_traj_idx_);
        }
        road_min = cr.road_min;
        road_max = cr.road_max;
        is_map_yl = cr.is_yellow_solid;
    } else {
        road_min = corridor_data.road_min[0];
        road_max = corridor_data.road_max[0];
        is_map_yl = corridor_data.is_yellow_solid[0];
    }

    bool in_cv_range = true;
    double delta_min = 0.0;
    double delta_max = 0.0;
    bool c0_yl = is_map_yl;

    if (corridor_tracker_ != nullptr && (idx.has_value() || s.has_value())) {
        auto c0 = corridor_tracker_->get_corridor_at_idx(current_traj_idx_);
        c0_yl = c0.is_yellow_solid;
        if (!c0_yl && is_map_yl) {
            delta_min = std::min(0.0, road_min - c0.road_min);
        } else {
            delta_min = road_min - c0.road_min;
        }
        delta_max = road_max - c0.road_max;

        if (idx.has_value() && std::abs(static_cast<long>(*idx) - static_cast<long>(current_traj_idx_)) > 55) {
            in_cv_range = false;
        } else if (s.has_value() && current_traj_idx_ < s_traj_.size()) {
            double s_curr = s_traj_[current_traj_idx_];
            if (*s < s_curr - 3.0 || *s > s_curr + 18.0) {
                in_cv_range = false;
            }
        }
    }

    if (in_cv_range && cv_source_ == "CV" && cv_confidence_ >= 0.80) {
        if (cv_road_min_.has_value()) road_min = *cv_road_min_ + delta_min;
        if (cv_road_max_.has_value()) road_max = *cv_road_max_ + delta_max;
    }

    bool is_cam_solid = (in_cv_range && yellow_solid_active_ && yellow_blocks_left_);
    out_is_yellow_solid = is_map_yl || is_cam_solid;

    if (in_cv_range && is_cam_solid && yellow_road_limit_.has_value()) {
        double yl_shift = (!c0_yl) ? std::min(0.0, delta_min) : delta_min;
        road_min = std::max(road_min, *yellow_road_limit_ + yl_shift);
    }

    // Luôn bảo đảm hành lang pháp lý chứa tối thiểu lân cận đường tham chiếu [-0.45m, +0.45m]
    out_road_min = std::min(road_min, config::STANDARD_ROAD_MIN_M);
    out_road_max = std::max(road_max, 0.45);

    if ((!idx.has_value() && !s.has_value()) ||
        (idx.has_value() && std::abs(static_cast<long>(*idx) - static_cast<long>(current_traj_idx_)) <= 5)) {
        current_road_min_ = out_road_min;
        current_road_max_ = out_road_max;
    }
}

std::array<double, 4> LinearizedMpcController::eval_discrete_dynamics(
    const std::array<double, 4>& x,
    const std::array<double, 2>& u) const {
    double pos_x = x[0], pos_y = x[1], theta = x[2], v = x[3];
    double delta = u[0], a = u[1];
    return {
        pos_x + v * std::cos(theta) * dt_,
        pos_y + v * std::sin(theta) * dt_,
        theta + (v / L_) * std::tan(delta) * dt_,
        v + a * dt_
    };
}

void LinearizedMpcController::get_linearized_matrices(
    const std::array<double, 4>& x_bar,
    const std::array<double, 2>& u_bar,
    std::array<std::array<double, 4>, 4>& A,
    std::array<std::array<double, 2>, 4>& B,
    std::array<double, 4>& d) const {

    double theta = x_bar[2];
    double v = x_bar[3];
    double delta = u_bar[0];

    for (int i = 0; i < 4; ++i) {
        A[i] = {0.0, 0.0, 0.0, 0.0};
        A[i][i] = 1.0;
        B[i] = {0.0, 0.0};
    }

    A[0][2] = -v * std::sin(theta) * dt_;
    A[0][3] =  std::cos(theta) * dt_;
    A[1][2] =  v * std::cos(theta) * dt_;
    A[1][3] =  std::sin(theta) * dt_;
    A[2][3] = (std::tan(delta) / L_) * dt_;

    double cos_d = std::cos(delta);
    double sec_d_sq = 1.0 / (cos_d * cos_d + 1e-6);
    B[2][0] = (v / L_) * sec_d_sq * dt_;
    B[3][1] = dt_;

    auto x_next_exact = eval_discrete_dynamics(x_bar, u_bar);
    for (int i = 0; i < 4; ++i) {
        double lin_val = B[i][0] * u_bar[0] + B[i][1] * u_bar[1];
        for (int j = 0; j < 4; ++j) {
            lin_val += A[i][j] * x_bar[j];
        }
        d[i] = x_next_exact[i] - lin_val;
    }
}

void LinearizedMpcController::init_nominal_trajectory(
    const std::array<double, 4>& x_init,
    const std::vector<double>& x_ref,
    const std::vector<double>& y_ref,
    const std::vector<double>& theta_ref,
    const std::vector<double>& v_ref) {

    x_nom_.assign(horizon_ + 1, {0.0, 0.0, 0.0, 0.0});
    u_nom_.assign(horizon_, {0.0, 0.0});
    x_nom_[0] = x_init;

    double curr_v = x_init[3];
    for (int k = 0; k < horizon_; ++k) {
        x_nom_[k + 1][0] = x_ref[k];
        x_nom_[k + 1][1] = y_ref[k];
        x_nom_[k + 1][2] = theta_ref[k];
        curr_v = std::min(curr_v + 2.5 * dt_, v_ref[k]);
        x_nom_[k + 1][3] = (x_init[3] > v_ref[k]) ? std::max(curr_v, v_ref[k]) : curr_v;
        u_nom_[k] = {0.0, 0.0};
    }
    nom_initialized_ = true;
}

void LinearizedMpcController::shift_nominal_trajectory(
    const std::vector<std::array<double, 4>>& x_opt,
    const std::vector<std::array<double, 2>>& u_opt,
    double dt_loop) {
    if (static_cast<int>(x_opt.size()) < horizon_ + 1 || static_cast<int>(u_opt.size()) < horizon_) {
        return;
    }
    double shift_frac = std::clamp(dt_loop / dt_, 0.20, 1.0);
    for (int k = 0; k < horizon_; ++k) {
        for (int i = 0; i < 4; ++i) {
            x_nom_[k][i] = (1.0 - shift_frac) * x_opt[k][i] + shift_frac * x_opt[k + 1][i];
        }
        x_nom_[k][2] = x_opt[k][2] + shift_frac * math::normalize_angle(x_opt[k + 1][2] - x_opt[k][2]);
    }
    x_nom_[horizon_] = x_opt[horizon_];
    for (int k = 0; k < horizon_ - 1; ++k) {
        for (int i = 0; i < 2; ++i) {
            u_nom_[k][i] = (1.0 - shift_frac) * u_opt[k][i] + shift_frac * u_opt[k + 1][i];
        }
    }
    u_nom_[horizon_ - 1] = u_opt[horizon_ - 1];
}

std::pair<std::optional<double>, std::pair<double, double>> LinearizedMpcController::select_optimal_shift_from_gaps(
    const std::vector<std::pair<double, double>>& valid_gaps,
    std::optional<double> preferred_sign,
    std::optional<double> enforce_sign,
    bool yellow_solid_active,
    double yellow_limit,
    bool has_oncoming_traffic,
    double min_gap_width) {

    if (valid_gaps.empty()) return {std::nullopt, {0.0, 0.0}};

    // 1. Lọc khe trống theo luật vạch vàng & xe ngược chiều
    std::vector<std::pair<double, double>> legal_gaps;
    for (const auto& [g_low, g_high] : valid_gaps) {
        if (yellow_solid_active || has_oncoming_traffic) {
            double sub_low = std::max(g_low, yellow_limit);
            if ((g_high - sub_low) >= min_gap_width) {
                legal_gaps.emplace_back(sub_low, g_high);
            }
        } else {
            legal_gaps.emplace_back(g_low, g_high);
        }
    }

    if (legal_gaps.empty()) return {std::nullopt, {0.0, 0.0}};

    std::vector<std::pair<double, double>> candidate_gaps = legal_gaps;
    if (enforce_sign.has_value()) {
        std::vector<std::pair<double, double>> same_side_gaps;
        for (const auto& [g_low, g_high] : legal_gaps) {
            if (*enforce_sign > 0.0 && g_high > 0.10) {
                double sub_low = std::max(g_low, -0.15);
                if ((g_high - sub_low) >= min_gap_width) {
                    same_side_gaps.emplace_back(sub_low, g_high);
                }
            } else if (*enforce_sign < 0.0 && g_low < -0.10) {
                double sub_high = std::min(g_high, 0.15);
                if ((sub_high - g_low) >= min_gap_width) {
                    same_side_gaps.emplace_back(g_low, sub_high);
                }
            }
        }
        if (same_side_gaps.empty()) return {std::nullopt, {0.0, 0.0}};
        candidate_gaps = same_side_gaps;
    }

    std::optional<double> best_d = std::nullopt;
    double best_cost = std::numeric_limits<double>::max();
    std::pair<double, double> best_gap{0.0, 0.0};

    for (const auto& [g_low, g_high] : candidate_gaps) {
        // QUY TẮC BẤT BIẾN (SKILL.md Mục 2.2):
        // pad = min(0.15, 0.25 * (g_high - g_low)) -> d_target cách mép vật cản đúng 1.50m (1.35m + 0.15m)
        double pad = std::min(config::LATERAL_BYPASS_PAD_M, 0.25 * (g_high - g_low));
        double d_cand = std::clamp(0.0, g_low + pad, g_high - pad);

        double cost = std::abs(d_cand);
        if (d_cand < yellow_limit) {
            cost += 0.85;
        } else if (d_cand < -0.05) {
            cost += 0.15;
        }

        if (preferred_sign.has_value() && std::abs(d_cand) > 0.05 && (d_cand * (*preferred_sign) < 0.0)) {
            cost += 1.50;
        }

        if (cost < best_cost) {
            best_cost = cost;
            best_d = d_cand;
            best_gap = {g_low, g_high};
        }
    }

    return {best_d, best_gap};
}

double LinearizedMpcController::calc_span_obstacle_weight(
    double s_query, double s_rear, double s_front,
    bool has_min_s_app, double min_s_app_start) const {

    double s_hold_start = s_rear - config::EGO_FRONT_OVERHANG_M - config::S_CURVE_APPROACH_MARGIN_M;
    double s_app_start = s_hold_start - config::S_CURVE_APPROACH_RAMP_M;
    if (has_min_s_app && min_s_app_start > s_app_start) {
        s_app_start = std::min(min_s_app_start, s_hold_start - 4.0);
    }
    double s_hold_end = s_front + config::S_CURVE_HOLD_DIST_M;
    double s_ret_end = s_hold_end + config::S_CURVE_RETURN_RAMP_M;

    if (s_query < s_app_start) {
        return 0.0;
    } else if (s_query < s_hold_start) {
        double ramp_len = std::max(s_hold_start - s_app_start, 1.0);
        double u = (s_query - s_app_start) / ramp_len;
        return 0.5 * (1.0 - std::cos(M_PI * u));
    } else if (s_query <= s_hold_end) {
        return 1.0;
    } else if (s_query <= s_ret_end) {
        double u = (s_query - s_hold_end) / std::max(config::S_CURVE_RETURN_RAMP_M, 1.0);
        return 0.5 * (1.0 + std::cos(M_PI * u));
    }
    return 0.0;
}

void LinearizedMpcController::resample_ref_by_speed(
    const std::array<double, 4>& x_init,
    const std::vector<double>& x_ref_in,
    const std::vector<double>& y_ref_in,
    const std::vector<double>& theta_ref_in,
    const std::vector<double>& v_ref_in,
    const std::vector<types::ObstacleFrenetOBB>& detected_obstacles,
    std::vector<double>& x_sync,
    std::vector<double>& y_sync,
    std::vector<double>& theta_sync,
    std::vector<double>& v_sync) {

    if (x_traj_.empty() || s_traj_.size() < 2) {
        x_sync = x_ref_in;
        y_sync = y_ref_in;
        theta_sync = theta_ref_in;
        v_sync = v_ref_in;
        return;
    }

    auto proj = math::project_to_frenet(x_init[0], x_init[1], x_traj_, y_traj_, theta_traj_, s_traj_, current_traj_idx_);
    current_traj_idx_ = proj.closest_idx;

    bool has_near_obs = (!active_chain_.empty()) || (std::abs(proj.d) > 0.35);
    if (!has_near_obs) {
        for (const auto& obs : detected_obstacles) {
            double dist = std::hypot(obs.center_x - x_init[0], obs.center_y - x_init[1]);
            if (dist <= config::OBSTACLE_SLOWDOWN_DISTANCE_M + 8.0 && std::abs(obs.d_center) <= 3.0) {
                has_near_obs = true;
                break;
            }
        }
    }

    double v_curr = std::max(x_init[3], 2.5);
    double s_acc = proj.s;
    double s_max = s_traj_.back();

    x_sync.resize(horizon_);
    y_sync.resize(horizon_);
    theta_sync.resize(horizon_);
    v_sync.resize(horizon_);
    s_queries_.resize(horizon_);

    for (int k = 0; k < horizon_; ++k) {
        double s_q = std::min(s_acc, s_max);
        s_queries_[k] = s_q;
        double v_tgt = has_near_obs ? std::min(v_ref_in[k], config::CAUTION_SPEED_MS) : v_ref_in[k];
        double dv = std::clamp(v_tgt - v_curr, -2.5 * dt_, 2.0 * dt_);
        v_curr = std::max(2.0, v_curr + dv);
        s_acc += v_curr * dt_;

        x_sync[k] = math::interp1d(s_q, s_traj_, x_traj_);
        y_sync[k] = math::interp1d(s_q, s_traj_, y_traj_);
        theta_sync[k] = math::interp1d(s_q, s_traj_, theta_traj_);
        if (v_traj_.size() == s_traj_.size()) {
            v_sync[k] = math::interp1d(s_q, s_traj_, v_traj_);
        } else {
            v_sync[k] = v_ref_in[k];
        }
    }
}

void LinearizedMpcController::compute_local_reference_shift(
    const std::array<double, 4>& x_init,
    const std::vector<double>& x_ref,
    const std::vector<double>& y_ref,
    const std::vector<double>& theta_ref,
    const std::vector<double>& /*v_ref*/,
    const std::vector<types::ObstacleFrenetOBB>& detected_obstacles,
    const types::CorridorBoundsData& corridor_data,
    double dt_loop,
    std::vector<double>& x_mod,
    std::vector<double>& y_mod,
    std::vector<double>& theta_mod,
    bool& has_shift) {

    x_mod = x_ref;
    y_mod = y_ref;
    theta_mod = theta_ref;
    has_shift = false;

    auto proj_ego = math::project_to_frenet(x_init[0], x_init[1], x_traj_, y_traj_, theta_traj_, s_traj_, current_traj_idx_);
    double s_ego = proj_ego.s;
    double d_ego = proj_ego.d;
    size_t idx_ego = proj_ego.closest_idx;
    current_traj_idx_ = idx_ego;

    double r_clearance = config::R_CLEARANCE_EDGE; // 1.35m
    double road_min = config::STANDARD_ROAD_MIN_M;
    double road_max = config::STANDARD_ROAD_MAX_M;
    bool is_yellow_solid = true;
    get_legal_corridor_bounds(idx_ego, std::nullopt, corridor_data, road_min, road_max, is_yellow_solid);
    double min_gap_width = 0.20;

    // 1. Giảm bộ đếm giữ trạng thái vật cản cắt ngang (Hysteresis 2.5s ~ 25 bước @ 10Hz)
    for (auto it = lateral_crossing_timer_s_.begin(); it != lateral_crossing_timer_s_.end();) {
        it->second -= dt_loop;
        if (it->second <= 0.0) {
            it = lateral_crossing_timer_s_.erase(it);
        } else {
            ++it;
        }
    }

    std::vector<types::ObstacleFrenetOBB> road_obs;
    std::unordered_map<int32_t, types::ObstacleFrenetOBB> road_obs_by_id;
    bool has_oncoming_traffic = false;

    for (const auto& obs_in : detected_obstacles) {
        int32_t oid = obs_in.id;
        double s_r = obs_in.s_rear, s_f = obs_in.s_front, d_min = obs_in.d_min, d_max = obs_in.d_max;
        if (!x_traj_.empty()) {
            math::compute_obb_frenet_bounds(obs_in.center_x, obs_in.center_y, obs_in.box_yaw,
                                            obs_in.extent_x, obs_in.extent_y,
                                            x_traj_, y_traj_, theta_traj_, s_traj_, idx_ego,
                                            s_r, s_f, d_min, d_max);
        }

        auto proj_o = math::project_to_frenet(obs_in.center_x, obs_in.center_y, x_traj_, y_traj_, theta_traj_, s_traj_, idx_ego);
        double v_s_obs = obs_in.vx * proj_o.tx + obs_in.vy * proj_o.ty;
        double v_d_obs = obs_in.vx * proj_o.nx + obs_in.vy * proj_o.ny;
        double obs_spd = std::hypot(obs_in.vx, obs_in.vy);
        bool is_oncoming = (v_s_obs < -0.5);

        if ((obs_spd > 0.5 || is_oncoming) && d_min < -1.20 && (s_f >= s_ego - 5.0) && (s_r <= s_ego + 35.0)) {
            if (is_oncoming || (d_min <= road_min + 0.5)) {
                has_oncoming_traffic = true;
            }
        }

        double obs_map_min = road_min, obs_map_max = road_max;
        bool obs_map_yl = is_yellow_solid;
        get_legal_corridor_bounds(std::nullopt, s_r, corridor_data, obs_map_min, obs_map_max, obs_map_yl);

        // Phát hiện vật cản cắt ngang đường: |v_d| >= 1.5, |v_d| >= 1.5 * |v_s|, |v_s| < 3.0 trong biên mở rộng +-4.5m
        if (std::abs(v_d_obs) >= 1.5 && std::abs(v_d_obs) >= 1.5 * std::abs(v_s_obs) && std::abs(v_s_obs) < 3.0) {
            if ((d_max + r_clearance) > (obs_map_min - config::LATERAL_CROSSING_CORRIDOR_MARGIN_M) &&
                (d_min - r_clearance) < (obs_map_max + config::LATERAL_CROSSING_CORRIDOR_MARGIN_M)) {
                lateral_crossing_timer_s_[oid] = config::LATERAL_CROSSING_HOLD_TIME_S; // 2.5s
            }
        } else if (lateral_crossing_timer_s_.count(oid) && std::abs(v_s_obs) >= 3.0 && std::abs(v_s_obs) > 1.3 * std::abs(v_d_obs)) {
            lateral_crossing_timer_s_.erase(oid);
        }

        bool is_lat_cross = (lateral_crossing_timer_s_.count(oid) > 0);

        if (!is_lat_cross) {
            if ((is_yellow_solid || obs_map_yl) && (d_max + r_clearance) <= (std::max(road_min, obs_map_min) + 0.05)) {
                continue;
            }
            if ((d_max + r_clearance) <= (obs_map_min + 0.15) || (d_min - r_clearance) >= (obs_map_max - 0.15)) {
                continue;
            }
        } else {
            if ((d_max + r_clearance) <= (obs_map_min - config::LATERAL_CROSSING_CORRIDOR_MARGIN_M) ||
                (d_min - r_clearance) >= (obs_map_max + config::LATERAL_CROSSING_CORRIDOR_MARGIN_M)) {
                lateral_crossing_timer_s_.erase(oid);
                continue;
            }
        }

        double d_lat_margin = is_lat_cross ? config::LATERAL_CROSSING_CORRIDOR_MARGIN_M : 1.0;
        if ((s_f >= s_ego - 12.0) && (s_r <= s_ego + config::SENSOR_DETECTION_RANGE_M) &&
            (d_max >= obs_map_min - d_lat_margin) && (d_min <= obs_map_max + d_lat_margin)) {
            types::ObstacleFrenetOBB obs_c = obs_in;
            obs_c.s_rear = s_r;
            obs_c.s_front = s_f;
            obs_c.s_center = 0.5 * (s_r + s_f);
            obs_c.d_min = d_min;
            obs_c.d_max = d_max;
            obs_c.d_center = 0.5 * (d_min + d_max);
            obs_c.v_s = v_s_obs;
            obs_c.v_d = v_d_obs;
            obs_c.is_lateral_crossing = is_lat_cross;
            road_obs.push_back(obs_c);
            road_obs_by_id[oid] = obs_c;
        }
    }

    // Dọn dẹp Ghost Obstacles khỏi active_chain_ (SKILL.md Mục 5.3)
    std::vector<int32_t> to_remove;
    for (auto& [obs_id, obs_item] : active_chain_) {
        if (road_obs_by_id.count(obs_id)) {
            const auto& ro = road_obs_by_id[obs_id];
            obs_item.center_x = ro.center_x;
            obs_item.center_y = ro.center_y;
            obs_item.s_rear = ro.s_rear;
            obs_item.s_front = ro.s_front;
            obs_item.d_min = ro.d_min;
            obs_item.d_max = ro.d_max;
            obs_item.d_center = ro.d_center;
            if (ro.is_lateral_crossing) {
                to_remove.push_back(obs_id);
                continue;
            }
        } else {
            if (obs_item.s_front >= s_ego - 1.0) {
                to_remove.push_back(obs_id);
                continue;
            }
        }

        double grp_front = (obs_item.group_s_front > obs_item.s_front) ? obs_item.group_s_front : obs_item.s_front;
        double s_ret_done = grp_front + config::S_CURVE_HOLD_DIST_M + config::S_CURVE_RETURN_RAMP_M + 2.0;
        if (s_ego > s_ret_done) {
            to_remove.push_back(obs_id);
        } else if (is_yellow_solid && (obs_item.d_max + r_clearance) <= (road_min + 0.05)) {
            to_remove.push_back(obs_id);
        }
    }
    for (int32_t rid : to_remove) {
        active_chain_.erase(rid);
    }

    // 2. Chia mặt cắt ngang (Cross-sectional Slices): gom các vật cản đứng ngang hàng (<= 2.0m)
    std::vector<types::ObstacleFrenetOBB> ahead_road_obs;
    for (const auto& o : road_obs) {
        if (o.s_front >= s_ego - config::EGO_REAR_OVERHANG_M - 0.5) {
            ahead_road_obs.push_back(o);
        }
    }
    std::sort(ahead_road_obs.begin(), ahead_road_obs.end(), [](const auto& a, const auto& b) {
        return a.s_rear < b.s_rear;
    });

    std::vector<CrossSlice> slices;
    for (const auto& o : ahead_road_obs) {
        if (slices.empty() || (o.s_rear - slices.back().s_front) > 2.0) {
            CrossSlice sl;
            sl.obs_list.push_back(o);
            sl.s_rear = o.s_rear;
            sl.s_front = o.s_front;
            sl.blocked.emplace_back(o.d_min - r_clearance, o.d_max + r_clearance);
            slices.push_back(sl);
        } else {
            slices.back().obs_list.push_back(o);
            slices.back().s_rear = std::min(slices.back().s_rear, o.s_rear);
            slices.back().s_front = std::max(slices.back().s_front, o.s_front);
            slices.back().blocked.emplace_back(o.d_min - r_clearance, o.d_max + r_clearance);
        }
    }

    // 3. Đánh giá tuần tự từng lát cắt dọc theo hành lang di chuyển
    is_roadblock_ = false;
    roadblock_barrier_dist_ = 99.0;
    double lock_dist = config::S_CURVE_APPROACH_RAMP_M + config::S_CURVE_APPROACH_MARGIN_M;

    std::optional<double> committed_sign = std::nullopt;
    double committed_d = 0.0;
    std::optional<double> prev_s_front = std::nullopt;

    for (const auto& [_, obs_item] : active_chain_) {
        if (obs_item.A_shift < 0.05 || std::abs(obs_item.side_sign) < 0.1) continue;
        double grp_front = (obs_item.group_s_front > obs_item.s_front) ? obs_item.group_s_front : obs_item.s_front;
        if (grp_front + config::S_CURVE_HOLD_DIST_M >= s_ego) {
            double dist_to_lock = (obs_item.s_rear - config::EGO_FRONT_OVERHANG_M) - s_ego;
            if (dist_to_lock <= lock_dist) {
                double target_d = obs_item.d_target;
                double obs_rmin = road_min, obs_rmax = road_max;
                bool obs_yl = is_yellow_solid;
                get_legal_corridor_bounds(std::nullopt, obs_item.s_rear, corridor_data, obs_rmin, obs_rmax, obs_yl);
                if ((obs_yl && target_d < (obs_rmin - 0.05)) || target_d < (obs_rmin - 0.15) || target_d > (obs_rmax + 0.15)) {
                    continue;
                }
                committed_sign = obs_item.side_sign;
                if (std::abs(obs_item.d_target) > std::abs(committed_d)) {
                    committed_d = obs_item.d_target;
                }
                if (!prev_s_front.has_value() || obs_item.s_front > *prev_s_front) {
                    prev_s_front = obs_item.s_front;
                }
            }
        }
    }

    if (!committed_sign.has_value() && std::abs(d_ego) >= 0.65) {
        if (!(is_yellow_solid && d_ego < (road_min - 0.05))) {
            committed_sign = (d_ego < 0.0) ? -1.0 : 1.0;
            committed_d = d_ego;
        }
    }

    std::optional<double> initial_pref_sign = committed_sign;
    if (!initial_pref_sign.has_value() && !slices.empty()) {
        bool has_first_crossing = false;
        for (const auto& o : slices[0].obs_list) {
            if (o.is_lateral_crossing) has_first_crossing = true;
        }
        if (!has_first_crossing) {
            double s0_rear = slices[0].s_rear;
            double s0_rmin = road_min, s0_rmax = road_max;
            bool s0_yl = is_yellow_solid;
            get_legal_corridor_bounds(std::nullopt, s0_rear, corridor_data, s0_rmin, s0_rmax, s0_yl);
            double s0_yellow_limit = s0_yl ? s0_rmin : (-1.75 + config::R_CLEARANCE_EDGE);

            std::vector<std::pair<double, double>> lookahead_blocked;
            for (const auto& sl : slices) {
                bool sl_cross = false;
                for (const auto& o : sl.obs_list) if (o.is_lateral_crossing) sl_cross = true;
                if (sl.s_rear <= s0_rear + 2.5 && !sl_cross) {
                    lookahead_blocked.insert(lookahead_blocked.end(), sl.blocked.begin(), sl.blocked.end());
                }
            }
            auto common_gaps = perception::RoadCorridorTracker::find_free_gaps_in_corridor(
                lookahead_blocked, s0_rmin, s0_rmax, min_gap_width);
            if (!common_gaps.empty()) {
                auto [d_init_common, _] = select_optimal_shift_from_gaps(
                    common_gaps, std::nullopt, std::nullopt, s0_yl, s0_yellow_limit, has_oncoming_traffic, min_gap_width);
                if (d_init_common.has_value() && std::abs(*d_init_common) > 0.05) {
                    initial_pref_sign = (*d_init_common < 0.0) ? -1.0 : 1.0;
                }
            }
        }
    }

    std::optional<double> curr_sign = initial_pref_sign;
    double curr_d_target = committed_d;
    std::vector<CrossSlice> slice_plans;
    std::optional<CrossSlice> blocking_slice = std::nullopt;

    struct PrevSliceInfo {
        double s_rear{0.0};
        double s_front{0.0};
        std::pair<double, double> gap{0.0, 0.0};
        double d_target{0.0};
        bool has_moving{false};
        bool is_ahead{false};
    };
    std::optional<PrevSliceInfo> prev_slice_info = std::nullopt;

    for (auto& sl : slices) {
        double sl_road_min = road_min, sl_road_max = road_max;
        bool sl_yl = is_yellow_solid;
        get_legal_corridor_bounds(std::nullopt, sl.s_rear, corridor_data, sl_road_min, sl_road_max, sl_yl);
        double sl_yellow_limit = sl_yl ? sl_road_min : (-1.75 + config::R_CLEARANCE_EDGE);

        bool sl_has_moving = false;
        for (const auto& o : sl.obs_list) {
            if (std::hypot(o.vx, o.vy) >= 0.5) {
                sl_has_moving = true;
                break;
            }
        }

        // QUY TẮC BẤT BIẾN (SKILL.md Mục 2.1):
        // Chỉ né khi affects_lane = true (vật cản cách tâm đường <= 1.35m,即 blocked interval đè lên [-0.05, +0.05])
        bool affects_lane = false;
        for (const auto& [b_l, b_h] : sl.blocked) {
            if (b_l < 0.05 && b_h > -0.05) {
                affects_lane = true;
                break;
            }
        }

        auto gaps = perception::RoadCorridorTracker::find_free_gaps_in_corridor(
            sl.blocked, sl_road_min, sl_road_max, min_gap_width);

        double dist_ego_front_to_sl = (sl.s_rear - config::EGO_FRONT_OVERHANG_M) - s_ego;
        bool is_moving_away_far = (dist_ego_front_to_sl > 12.0);
        for (const auto& o : sl.obs_list) {
            if (o.v_s < 2.0) is_moving_away_far = false;
        }
        bool is_ahead_of_ego = (sl.s_rear >= s_ego + 0.5) && (dist_ego_front_to_sl <= 19.5) && (!is_moving_away_far);

        bool has_lateral_crossing = false;
        for (const auto& o : sl.obs_list) {
            if (o.is_lateral_crossing) has_lateral_crossing = true;
        }

        // Xử lý vật cản di động cắt ngang đường (SKILL.md Mục 2.3):
        if (has_lateral_crossing && (sl.s_rear >= s_ego + 0.5)) {
            bool is_latched_cross = false;
            for (const auto& o : sl.obs_list) {
                if (roadblock_latched_ids_.count(o.id)) is_latched_cross = true;
            }
            if (config::ENABLE_ROADBLOCK_EMERGENCY_STOP && (dist_ego_front_to_sl <= 20.0 || is_latched_cross)) {
                is_roadblock_ = true;
                if (!roadblock_s_barrier_.has_value()) {
                    roadblock_s_barrier_ = sl.s_rear;
                } else {
                    roadblock_s_barrier_ = std::min(*roadblock_s_barrier_, sl.s_rear);
                }
                roadblock_barrier_dist_ = std::max(0.0, *roadblock_s_barrier_ - config::EGO_FRONT_OVERHANG_M - s_ego);
                for (const auto& o : sl.obs_list) {
                    roadblock_latched_ids_.insert(o.id);
                    active_chain_.erase(o.id);
                }
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(1)
                    << "VẬT CẢN DI ĐỘNG CẮT NGANG ĐƯỜNG tại s=" << *roadblock_s_barrier_
                    << "m: Giữ làn và dừng an toàn chờ vật cản đi qua!";
                roadblock_reason_ = oss.str();
                blocking_slice = sl;
                break;
            }
            continue;
        }

        // Nếu lát cắt đã chốt Staggered Roadblock từ trước, duy trì trạng thái dừng
        if (config::ENABLE_ROADBLOCK_EMERGENCY_STOP && is_ahead_of_ego && (affects_lane || gaps.empty())) {
            bool has_latched = false;
            for (const auto& o : sl.obs_list) {
                if (roadblock_latched_ids_.count(o.id)) has_latched = true;
            }
            if (has_latched) {
                is_roadblock_ = true;
                roadblock_barrier_dist_ = std::max(0.0, sl.s_rear - config::EGO_FRONT_OVERHANG_M - s_ego);
                if (roadblock_reason_.empty()) {
                    roadblock_reason_ = "PHONG TỎA ĐƯỜNG SO LE (STAGGERED ROADBLOCK): Giữ trạng thái phanh dừng an toàn.";
                }
                blocking_slice = sl;
                break;
            }
        }

        // Kiểm tra cam kết hướng né (must_keep_side) — đồng bộ 100% với src/linearized_mpc_controller.py
        bool must_keep_side = false;
        if (curr_sign.has_value()) {
            // (a) Bản thân lát cắt đã có vật cản nằm trong vùng khóa hướng của active_chain_
            for (const auto& o : sl.obs_list) {
                if (active_chain_.count(o.id) &&
                    (active_chain_[o.id].s_rear - config::EGO_FRONT_OVERHANG_M - s_ego) <= lock_dist) {
                    double target_d_o = active_chain_[o.id].d_target;
                    if ((sl_road_min - 0.05) <= target_d_o && target_d_o <= (sl_road_max + 0.05)) {
                        must_keep_side = true;
                        curr_sign = active_chain_[o.id].side_sign;
                        break;
                    }
                }
            }
            // (b) Khoảng trống dọc từ lát cắt trước đó đến lát cắt này không đủ để xe đảo từ bên này sang bên kia
            if (!must_keep_side && affects_lane && prev_s_front.has_value()) {
                double ds_gap = sl.s_rear - *prev_s_front;
                double dd_switch = std::abs(curr_d_target) + 1.85;
                double ds_req = (config::EGO_FRONT_OVERHANG_M + config::EGO_REAR_OVERHANG_M) + 2.0 * dd_switch;
                if (ds_gap < ds_req) {
                    must_keep_side = true;
                }
            }
            // (c) Thân xe thực tế đang lệch sang phía curr_sign và khoảng cách dọc tới lát cắt chặn làn không đủ để vắt chéo qua tim đường
            if (!must_keep_side && affects_lane && (d_ego * (*curr_sign) >= 0.65)) {
                double dist_ego_to_sl = (sl.s_rear - config::EGO_FRONT_OVERHANG_M) - s_ego;
                double ds_req_ego = 2.2 * (std::abs(d_ego) + 1.85) + 2.0;
                if (dist_ego_to_sl < ds_req_ego) {
                    must_keep_side = true;
                }
            }
        }

        if (gaps.empty()) {
            if (config::ENABLE_ROADBLOCK_EMERGENCY_STOP && is_ahead_of_ego) {
                is_roadblock_ = true;
                roadblock_barrier_dist_ = std::max(0.0, sl.s_rear - config::EGO_FRONT_OVERHANG_M - s_ego);
                if (!sl_has_moving) {
                    for (const auto& o : sl.obs_list) {
                        roadblock_latched_ids_.insert(o.id);
                    }
                }
                if (sl_yl) {
                    roadblock_reason_ = "VẠCH VÀNG NÉT LIỀN BÊN TRÁI CẤM VƯỢT + LÀN PHẢI BỊ CHẶN; dừng xe an toàn!";
                } else {
                    roadblock_reason_ = "BÍT KÍN TOÀN BỘ MẶT ĐƯỜNG; dừng xe an toàn!";
                }
                blocking_slice = sl;
                break;
            }
            continue;
        }

        std::optional<double> enforce_sign = must_keep_side ? curr_sign : std::nullopt;
        auto [d_target_opt, best_gap] = select_optimal_shift_from_gaps(
            gaps, curr_sign, enforce_sign, sl_yl, sl_yellow_limit, has_oncoming_traffic, min_gap_width);

        double dist_to_sl = (sl.s_rear - config::EGO_FRONT_OVERHANG_M) - s_ego;
        if (!d_target_opt.has_value() && enforce_sign.has_value() && dist_to_sl >= 4.5 && !prev_s_front.has_value()) {
            std::tie(d_target_opt, best_gap) = select_optimal_shift_from_gaps(
                gaps, std::nullopt, std::nullopt, sl_yl, sl_yellow_limit, has_oncoming_traffic, min_gap_width);
            if (d_target_opt.has_value()) {
                must_keep_side = false;
                enforce_sign = std::nullopt;
            }
        }

        if (!d_target_opt.has_value()) {
            if (config::ENABLE_ROADBLOCK_EMERGENCY_STOP && is_ahead_of_ego) {
                is_roadblock_ = true;
                roadblock_barrier_dist_ = std::max(0.0, sl.s_rear - config::EGO_FRONT_OVERHANG_M - s_ego);
                if (!sl_has_moving) {
                    for (const auto& o : sl.obs_list) {
                        roadblock_latched_ids_.insert(o.id);
                    }
                }
                if (sl_yl) {
                    roadblock_reason_ = "VẠCH VÀNG NÉT LIỀN BÊN TRÁI CẤM VƯỢT + LỀ PHẢI BỊ CHẶN; dừng xe an toàn!";
                } else {
                    roadblock_reason_ = "PHONG TỎA ĐƯỜNG (ROADBLOCK): Cả 2 làn đều bị chặn; dừng xe an toàn!";
                }
                blocking_slice = sl;
                break;
            }
            continue;
        }

        // Vật cản ở làn bên cạnh (affects_lane == false) phải giữ actual_d_target = 0.0m,
        // nhưng VẪN PHẢI LƯU vào prev_slice_info để kiểm tra Staggered Roadblock!
        double actual_d_target = (affects_lane || must_keep_side) ? (*d_target_opt) : 0.0;

        // Kiểm tra Phong tỏa đường so le (Staggered Roadblock - SKILL.md Mục 4.2):
        bool has_min_s_app = false;
        double min_s_app_start = -1e9;
        if (prev_slice_info.has_value()) {
            auto prev_gap = prev_slice_info->gap;
            double prev_d = prev_slice_info->d_target;
            double ov_low = std::max(prev_gap.first, best_gap.first);
            double ov_high = std::min(prev_gap.second, best_gap.second);
            bool has_common_corridor = (ov_high - ov_low) >= min_gap_width;
            double dd_between = std::abs(actual_d_target - prev_d);
            double ds_gap = sl.s_rear - prev_slice_info->s_front;
            // Công thức chuẩn: s_req = (2.4 + 2.4 + 2.5) + max(7.5, 4.0 * |dd|) = 17.3m khi dd = 2.50m
            double ds_min_weave = config::EGO_FRONT_OVERHANG_M + config::EGO_REAR_OVERHANG_M +
                                  config::S_CURVE_APPROACH_MARGIN_M + std::max(7.5, 4.0 * dd_between);

            if ((!has_common_corridor || dd_between > 0.85) && ds_gap < ds_min_weave) {
                bool staggered_has_moving = sl_has_moving || prev_slice_info->has_moving;
                bool staggered_ahead = is_ahead_of_ego || (prev_slice_info->is_ahead && !is_moving_away_far);
                if (config::ENABLE_ROADBLOCK_EMERGENCY_STOP && staggered_ahead) {
                    is_roadblock_ = true;
                    double barrier_s = is_ahead_of_ego ? sl.s_rear : prev_slice_info->s_rear;
                    roadblock_barrier_dist_ = std::max(0.0, barrier_s - config::EGO_FRONT_OVERHANG_M - s_ego);
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(1)
                        << "PHONG TỎA ĐƯỜNG SO LE (STAGGERED ROADBLOCK) tại s=" << barrier_s
                        << "m (ds=" << ds_gap << "m < " << ds_min_weave << "m); dừng xe an toàn!";
                    roadblock_reason_ = oss.str();
                    for (const auto& o : sl.obs_list) {
                        if (!staggered_has_moving) {
                            roadblock_latched_ids_.insert(o.id);
                        }
                        active_chain_.erase(o.id);
                    }
                    blocking_slice = sl;
                    break;
                }
            } else if (!has_common_corridor || dd_between > 0.85) {
                // Đủ cự ly lách qua (ds >= 17.3m): kẹp điểm mở lái S-Curve của xe sau >= s_front_1 + L_r
                has_min_s_app = true;
                min_s_app_start = prev_slice_info->s_front + config::EGO_REAR_OVERHANG_M;
            }
        }

        // Kiểm tra khả thi hình học dọc trực tiếp từ vị trí xe hiện tại tới lát cắt chặn làn
        double dd_ego = std::abs(actual_d_target - d_ego);
        double ds_min_ego = config::S_CURVE_APPROACH_MARGIN_M + 3.2 * dd_ego;
        bool in_best_gap = (d_ego >= best_gap.first - 0.15 && d_ego <= best_gap.second + 0.15);
        if ((affects_lane || !in_best_gap) && dd_ego > 1.10 && dist_to_sl < ds_min_ego) {
            if (config::ENABLE_ROADBLOCK_EMERGENCY_STOP && is_ahead_of_ego) {
                is_roadblock_ = true;
                roadblock_barrier_dist_ = std::max(0.0, dist_to_sl);
                roadblock_reason_ = "KHOẢNG CÁCH DỌC QUÁ NGẮN ĐỂ ĐẢO LÀN AN TOÀN; dừng xe an toàn!";
                for (const auto& o : sl.obs_list) {
                    if (!sl_has_moving) {
                        roadblock_latched_ids_.insert(o.id);
                    }
                    active_chain_.erase(o.id);
                }
                blocking_slice = sl;
                break;
            }
        }

        sl.gaps = gaps;
        sl.d_target = actual_d_target;
        sl.affects_lane = affects_lane || must_keep_side;
        sl.has_min_s_app_start = has_min_s_app;
        sl.min_s_app_start = min_s_app_start;

        prev_slice_info = PrevSliceInfo{sl.s_rear, sl.s_front, best_gap, actual_d_target, sl_has_moving, is_ahead_of_ego};
        if (std::abs(sl.d_target) > 0.05) {
            curr_sign = (sl.d_target < 0.0) ? -1.0 : 1.0;
            curr_d_target = sl.d_target;
            prev_s_front = sl.s_front;
        }
        slice_plans.push_back(sl);
    }

    // Giải phóng bộ nhớ chốt phanh nếu không còn Roadblock; hoặc xóa ghost obstacle khi dừng tại tim làn
    if (!is_roadblock_) {
        if (roadblock_stopped_latch_) {
            u_prev_ = {0.0, 0.0};
        }
        roadblock_latched_ids_.clear();
        roadblock_reason_.clear();
        roadblock_s_barrier_ = std::nullopt;
        roadblock_stopped_latch_ = false;
    } else if (std::abs(d_ego) <= 0.60) {
        std::vector<int32_t> ahead_ids;
        for (const auto& [oid_k, item_k] : active_chain_) {
            if (item_k.s_front >= s_ego - 1.0) ahead_ids.push_back(oid_k);
        }
        for (int32_t oid_k : ahead_ids) active_chain_.erase(oid_k);
    }

    // 4. Cập nhật các lát cắt cần né vào active_chain_
    for (const auto& sl : slice_plans) {
        if (std::abs(sl.d_target) < 0.05) {
            if (std::abs(d_ego) < 0.35) {
                for (const auto& o : sl.obs_list) {
                    active_chain_.erase(o.id);
                }
            }
            continue;
        }
        double d_target = sl.d_target;
        double side_sign = (d_target < 0.0) ? -1.0 : 1.0;
        double A_shift = std::abs(d_target);

        for (const auto& o : sl.obs_list) {
            int32_t oid = o.id;
            if (!active_chain_.count(oid)) {
                types::ObstacleFrenetOBB item = o;
                item.side_sign = side_sign;
                item.A_shift = A_shift;
                item.d_target = d_target;
                item.group_s_rear = o.s_rear;
                item.group_s_front = o.s_front;
                item.has_min_s_app_start = sl.has_min_s_app_start;
                item.min_s_app_start = sl.min_s_app_start;
                item.affects_lane = true;
                active_chain_[oid] = item;
            } else {
                auto& item = active_chain_[oid];
                item.center_x = o.center_x;
                item.center_y = o.center_y;
                item.s_rear = o.s_rear;
                item.s_front = o.s_front;
                item.d_min = o.d_min;
                item.d_max = o.d_max;
                item.d_center = o.d_center;
                item.has_min_s_app_start = sl.has_min_s_app_start;
                item.min_s_app_start = sl.min_s_app_start;

                if ((item.s_rear - config::EGO_FRONT_OVERHANG_M - s_ego) > lock_dist) {
                    item.side_sign = side_sign;
                    item.A_shift = A_shift;
                    item.d_target = d_target;
                } else {
                    double locked_sign = item.side_sign;
                    if (locked_sign * side_sign > 0.0) {
                        if (A_shift > item.A_shift) {
                            item.A_shift = 0.3 * A_shift + 0.7 * item.A_shift;
                            item.d_target = locked_sign * item.A_shift;
                        }
                    } else if (locked_sign * side_sign < 0.0) {
                        double old_d = item.d_target;
                        double obs_rmin = road_min, obs_rmax = road_max;
                        bool obs_yl = is_yellow_solid;
                        get_legal_corridor_bounds(std::nullopt, o.s_rear, corridor_data, obs_rmin, obs_rmax, obs_yl);
                        if (old_d < (obs_rmin - 0.05) || old_d > (obs_rmax + 0.05) || std::abs(d_ego) < 0.65) {
                            item.side_sign = side_sign;
                            item.A_shift = A_shift;
                            item.d_target = d_target;
                        }
                    }
                }
            }
        }
    }

    // 5. Hợp nhất các vật cản liên tiếp cùng phía thành Flat Outer Envelope (Corridor Groups)
    std::vector<types::ObstacleFrenetOBB> chain_list;
    chain_list.reserve(active_chain_.size());
    for (const auto& [_, item] : active_chain_) {
        chain_list.push_back(item);
    }
    std::sort(chain_list.begin(), chain_list.end(), [](const auto& a, const auto& b) {
        return a.s_rear < b.s_rear;
    });

    std::vector<CorridorGroup> corridor_groups;
    size_t i_start = 0;
    const double max_cluster_gap = config::S_CURVE_HOLD_DIST_M + config::S_CURVE_RETURN_RAMP_M + config::S_CURVE_APPROACH_RAMP_M;
    while (i_start < chain_list.size()) {
        size_t i_end = i_start;
        double group_sign = chain_list[i_start].side_sign;
        double max_A = chain_list[i_start].A_shift;
        bool grp_has_min_s = chain_list[i_start].has_min_s_app_start;
        double grp_min_s_app = chain_list[i_start].min_s_app_start;

        while (i_end + 1 < chain_list.size()) {
            const auto& nxt = chain_list[i_end + 1];
            double gap_s = nxt.s_rear - chain_list[i_end].s_front;
            if (nxt.side_sign == group_sign && gap_s <= max_cluster_gap) {
                max_A = std::max(max_A, nxt.A_shift);
                i_end++;
            } else {
                break;
            }
        }

        double grp_s_rear = chain_list[i_start].s_rear;
        double grp_s_front = chain_list[i_start].s_front;
        for (size_t idx_g = i_start; idx_g <= i_end; ++idx_g) {
            grp_s_front = std::max(grp_s_front, chain_list[idx_g].s_front);
        }
        for (size_t idx_g = i_start; idx_g <= i_end; ++idx_g) {
            chain_list[idx_g].A_shift = max_A;
            chain_list[idx_g].d_target = group_sign * max_A;
            chain_list[idx_g].group_s_rear = grp_s_rear;
            chain_list[idx_g].group_s_front = grp_s_front;
            active_chain_[chain_list[idx_g].id] = chain_list[idx_g];
        }

        corridor_groups.push_back(CorridorGroup{
            grp_s_rear, grp_s_front, group_sign, max_A, group_sign * max_A, grp_has_min_s, grp_min_s_app
        });
        i_start = i_end + 1;
    }

    if (active_chain_.empty() && !is_roadblock_) {
        has_active_obs_ = false;
        return;
    }

    has_active_obs_ = false;
    double min_ahead_s = std::numeric_limits<double>::max();
    for (const auto& o : chain_list) {
        if (o.s_front + config::S_CURVE_HOLD_DIST_M >= s_ego && o.s_rear < min_ahead_s) {
            min_ahead_s = o.s_rear;
            active_obs_ = o;
            has_active_obs_ = true;
        }
    }

    // 6. Xử lý hành lang ngang khi ROADBLOCK hoặc né vật cản
    double hold_corridor_d = 0.0;
    double hold_corridor_s_end = -1e9;
    if (is_roadblock_) {
        corridor_groups.clear();
        if (std::abs(d_ego) <= 0.60) {
            hold_corridor_d = 0.0;
            has_active_obs_ = false;
        } else {
            hold_corridor_d = d_ego;
            hold_corridor_s_end = blocking_slice.has_value() ? (blocking_slice->s_front + 15.0) : (s_ego + 30.0);
        }
    }

    for (int k = 0; k < horizon_; ++k) {
        auto proj_k = math::project_to_frenet(x_ref[k], y_ref[k], x_traj_, y_traj_, theta_traj_, s_traj_, idx_ego);
        double max_pos_shift = 0.0;
        double min_neg_shift = 0.0;

        for (const auto& grp : corridor_groups) {
            double w = calc_span_obstacle_weight(
                proj_k.s, grp.s_rear, grp.s_front, grp.has_min_s_app_start, grp.min_s_app_start);
            if (w > 1e-4) {
                double val = w * grp.d_target;
                if (val > max_pos_shift) max_pos_shift = val;
                else if (val < min_neg_shift) min_neg_shift = val;
            }
        }

        if (is_roadblock_ && std::abs(hold_corridor_d) > 0.20 && proj_k.s <= hold_corridor_s_end) {
            if (hold_corridor_d > 0.0) max_pos_shift = std::max(max_pos_shift, hold_corridor_d);
            else min_neg_shift = std::min(min_neg_shift, hold_corridor_d);
        }

        double d_shift_k = max_pos_shift + min_neg_shift;
        if (std::abs(d_shift_k) > 1e-4) {
            has_shift = true;
            x_mod[k] = x_ref[k] + d_shift_k * proj_k.nx;
            y_mod[k] = y_ref[k] + d_shift_k * proj_k.ny;
        }
    }

    // 7. Tính góc tiếp tuyến theta_mod đồng bộ với hướng đường cong S-Curve
    for (int k = 0; k < horizon_ - 1; ++k) {
        int k_next = std::min(horizon_ - 1, k + 2);
        double dx = x_mod[k_next] - x_mod[k];
        double dy = y_mod[k_next] - y_mod[k];
        if (std::hypot(dx, dy) > 1e-3) {
            double th_target = std::atan2(dy, dx);
            double d_th = math::normalize_angle(th_target - theta_ref[k]);
            theta_mod[k] = theta_ref[k] + 0.70 * d_th;
        }
    }
    if (horizon_ > 1) {
        theta_mod[horizon_ - 1] = theta_mod[horizon_ - 2];
    }

    // 8. In log quyết định khi có thay đổi trạng thái (chuẩn format Road: [-0.45m; +3.90m])
    std::string log_key;
    std::string current_decision;
    if (is_roadblock_) {
        log_key = "ROADBLOCK|" + roadblock_reason_;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1)
            << "ROADBLOCK_STOP | dist=" << roadblock_barrier_dist_ << "m | " << roadblock_reason_;
        current_decision = oss.str();
    } else if (has_active_obs_ && std::abs(active_obs_.d_target) > 0.05) {
        std::string side_str = (active_obs_.d_target > 0.0) ? "PHẢI (Right)" : "TRÁI (Left)";
        double dist_to_o = std::max(0.0, (active_obs_.s_rear - config::EGO_FRONT_OVERHANG_M) - s_ego);
        std::ostringstream kss, dss;
        kss << "BYPASS|" << active_obs_.id << "|" << side_str << "|"
            << std::fixed << std::setprecision(1) << active_obs_.d_target;
        log_key = kss.str();
        dss << "BYPASS_" << side_str << " | Target d=" << std::showpos << std::fixed << std::setprecision(2)
            << active_obs_.d_target << std::noshowpos << "m | Obstacle at s=" << std::setprecision(1)
            << active_obs_.s_rear << "m (cách mũi xe " << dist_to_o << "m)";
        current_decision = dss.str();
    } else if (std::abs(d_ego) > 0.20) {
        log_key = "RETURNING_TO_CENTER";
        current_decision = "RETURNING_TO_CENTER";
    } else {
        log_key = "TRACKING_LANE_CENTER";
        current_decision = "TRACKING_LANE_CENTER";
    }

    if (log_key != last_log_state_) {
        last_log_state_ = log_key;
        std::ostringstream ctag;
        ctag << "Road: [" << std::showpos << std::fixed << std::setprecision(2)
             << current_road_min_ << "m; " << current_road_max_ << "m]" << std::noshowpos;
        if (is_roadblock_) {
            std::cout << "\n[MPC DECISION - DỪNG LẠI (STOP)] s_ego=" << std::fixed << std::setprecision(1)
                      << s_ego << "m | " << ctag.str() << " | " << current_decision << std::endl;
        } else if (log_key.find("BYPASS") != std::string::npos) {
            std::cout << "\n[MPC DECISION - NÉ VẬT CẢN] s_ego=" << std::fixed << std::setprecision(1)
                      << s_ego << "m | " << ctag.str() << " | " << current_decision << std::endl;
        }
    }
}

std::vector<double> LinearizedMpcController::compute_safe_speed_profile(
    const std::array<double, 4>& x_init,
    const std::vector<double>& v_ref_base) const {

    std::vector<double> v_adapted = v_ref_base;
    if (x_traj_.empty()) return v_adapted;

    auto proj_ego = math::project_to_frenet(x_init[0], x_init[1], x_traj_, y_traj_, theta_traj_, s_traj_, current_traj_idx_);
    double s_ego = proj_ego.s;

    const types::ObstacleFrenetOBB* ref_obs = nullptr;
    if (has_active_obs_) {
        ref_obs = &active_obs_;
    } else if (!active_chain_.empty()) {
        double max_front = -1e9;
        for (const auto& [_, item] : active_chain_) {
            double gf = (item.group_s_front > item.s_front) ? item.group_s_front : item.s_front;
            if (gf > max_front) {
                max_front = gf;
                ref_obs = &item;
            }
        }
    }

    if (ref_obs != nullptr) {
        double s_rear = (ref_obs->group_s_rear > 0.0) ? ref_obs->group_s_rear : ref_obs->s_rear;
        double s_front = (ref_obs->group_s_front > ref_obs->s_front) ? ref_obs->group_s_front : ref_obs->s_front;

        double dist_to_obs = (s_rear - config::EGO_FRONT_OVERHANG_M) - s_ego;
        double dist_after_obs = s_ego - (s_front + config::S_CURVE_HOLD_DIST_M);

        double v_cap = config::TARGET_SPEED_MS;
        if (dist_to_obs > config::S_CURVE_APPROACH_MARGIN_M) {
            double ratio = std::clamp((dist_to_obs - config::S_CURVE_APPROACH_MARGIN_M) /
                                      std::max(config::OBSTACLE_SLOWDOWN_DISTANCE_M, 1.0), 0.0, 1.0);
            v_cap = config::CAUTION_SPEED_MS + ratio * (config::TARGET_SPEED_MS - config::CAUTION_SPEED_MS);
            if (dist_to_obs <= config::OBSTACLE_SLOWDOWN_DISTANCE_M + config::S_CURVE_APPROACH_MARGIN_M + 4.0) {
                v_cap = std::min(v_cap, std::max(x_init[3] + 0.3, config::CAUTION_SPEED_MS));
            }
        } else if (dist_after_obs <= 0.0) {
            v_cap = config::CAUTION_SPEED_MS;
        } else {
            double ratio = std::clamp(dist_after_obs / std::max(config::S_CURVE_RETURN_RAMP_M, 1.0), 0.0, 1.0);
            v_cap = config::CAUTION_SPEED_MS + ratio * (config::TARGET_SPEED_MS - config::CAUTION_SPEED_MS);
        }

        for (auto& v_val : v_adapted) {
            v_val = std::min(v_val, v_cap);
        }
    }

    return v_adapted;
}

bool LinearizedMpcController::solve_qp_iteration(
    const std::array<double, 4>& x_init,
    const std::vector<double>& x_ref_mod,
    const std::vector<double>& y_ref_mod,
    const std::vector<double>& theta_ref_mod,
    const std::vector<double>& v_ref,
    const std::vector<double>& x_ref_orig,
    const std::vector<double>& y_ref_orig,
    const std::vector<double>& theta_ref_orig,
    const std::vector<types::ObstacleFrenetOBB>& obstacles,
    const types::CorridorBoundsData& corridor_data,
    bool is_roadblock,
    std::vector<std::array<double, 4>>& out_x_opt,
    std::vector<std::array<double, 2>>& out_u_opt,
    double& out_cost) {

    const int n_u = 2 * horizon_;

    // 1. Truyền trạng thái động học tuyến tính hóa X_k(U) = c_k + S_k * U dọc theo chân trời N bước
    // Đảm bảo thỏa mãn chính xác tuyệt đối X_{k+1} = A_k X_k + B_k U_k + d_k
    std::vector<std::array<double, 4>> c_vec(horizon_ + 1, {0.0, 0.0, 0.0, 0.0});
    std::vector<std::array<std::vector<double>, 4>> S_mat(
        horizon_ + 1, {std::vector<double>(n_u, 0.0), std::vector<double>(n_u, 0.0),
                       std::vector<double>(n_u, 0.0), std::vector<double>(n_u, 0.0)});

    c_vec[0] = x_init;

    for (int k = 0; k < horizon_; ++k) {
        std::array<std::array<double, 4>, 4> A_k;
        std::array<std::array<double, 2>, 4> B_k;
        std::array<double, 4> d_k;
        get_linearized_matrices(x_nom_[k], u_nom_[k], A_k, B_k, d_k);

        for (int i = 0; i < 4; ++i) {
            double c_next = d_k[i];
            for (int j = 0; j < 4; ++j) {
                c_next += A_k[i][j] * c_vec[k][j];
            }
            c_vec[k + 1][i] = c_next;

            for (int col = 0; col < 2 * (k + 1); ++col) {
                double s_val = 0.0;
                for (int j = 0; j < 4; ++j) {
                    s_val += A_k[i][j] * S_mat[k][j][col];
                }
                if (col == 2 * k) {
                    s_val += B_k[i][0];
                } else if (col == 2 * k + 1) {
                    s_val += B_k[i][1];
                }
                S_mat[k + 1][i][col] = s_val;
            }
        }
    }

    // 2. Xác định hành lang pháp lý dọc chân trời N bước
    std::array<double, types::MPC_HORIZON> horizon_road_min = corridor_data.road_min;
    std::array<double, types::MPC_HORIZON> horizon_road_max = corridor_data.road_max;
    std::array<bool, types::MPC_HORIZON> horizon_is_yellow_solid = corridor_data.is_yellow_solid;

    if (corridor_tracker_ != nullptr && !s_queries_.empty()) {
        auto h_corr = corridor_tracker_->get_horizon_corridor_at_s(s_queries_);
        horizon_road_min = h_corr.road_min;
        horizon_road_max = h_corr.road_max;
        horizon_is_yellow_solid = h_corr.is_yellow_solid;
    }

    if (yellow_solid_active_ && yellow_blocks_left_ && yellow_road_limit_.has_value()) {
        for (int k = 0; k < horizon_; ++k) {
            double delta_h_min = horizon_road_min[k] - horizon_road_min[0];
            if (!horizon_is_yellow_solid[0]) {
                delta_h_min = std::min(0.0, delta_h_min);
            }
            horizon_road_min[k] = std::max(horizon_road_min[k], *yellow_road_limit_ + delta_h_min);
            horizon_is_yellow_solid[k] = true;
        }
    }

    bool any_yellow_solid = false;
    for (int k = 0; k < horizon_; ++k) {
        horizon_road_min[k] = std::min(horizon_road_min[k], config::STANDARD_ROAD_MIN_M);
        horizon_road_max[k] = std::max(horizon_road_max[k], 0.45);
        if (horizon_is_yellow_solid[k]) any_yellow_solid = true;
    }

    // 3. Khởi tạo bài toán QP chuẩn OSQP với biến bù Slack tường minh (eps >= 0), đồng bộ 100% CasADi Python:
    // Biến tối ưu z (kích thước n_vars = 8*N):
    //   - [0 .. 2N-1]   : U_k = [delta_k, a_k] (2N biến điều khiển)
    //   - [2N .. 3N-1]  : eps_road_left[k] >= 0 (N biến bù biên trái / vạch vàng)
    //   - [3N .. 4N-1]  : eps_road_right[k] >= 0 (N biến bù biên phải)
    //   - [4N .. 8N-1]  : eps_obs[m, k] >= 0 (4*N biến bù siêu phẳng cho tối đa 4 vật cản gần nhất)
    constexpr int max_obs = 4;
    const int n_vars = n_u + (2 + max_obs) * horizon_;
    const int n_cons = (11 + max_obs) * horizon_;
    constexpr double kInf = 1.0e20;

    auto idx_eps_left  = [&](int k) { return n_u + k; };
    auto idx_eps_right = [&](int k) { return n_u + horizon_ + k; };
    auto idx_eps_obs   = [&](int m, int k) { return n_u + (2 + m) * horizon_ + k; };

    // Quy ước OSQP: min 0.5 * z^T P z + q^T z -> hạng tử w * (g^T U + r)^2 đóng góp P += 2*w*g*g^T, q += 2*w*r*g
    std::vector<std::vector<double>> P(n_vars, std::vector<double>(n_vars, 0.0));
    std::vector<double> q(n_vars, 0.0);
    std::vector<std::vector<double>> A_cons(n_cons, std::vector<double>(n_vars, 0.0));
    std::vector<double> l_cons(n_cons, -kInf);
    std::vector<double> u_cons(n_cons,  kInf);

    auto add_quadratic_term = [&](double weight, const std::vector<double>& g, double r, double lin_slope = 0.0) {
        if (weight <= 0.0 && lin_slope == 0.0) return;
        for (int i = 0; i < n_u; ++i) {
            if (std::abs(g[i]) < 1e-12) continue;
            q[i] += (2.0 * weight * r + lin_slope) * g[i];
            for (int j = 0; j <= i; ++j) {
                if (std::abs(g[j]) < 1e-12) continue;
                double val = 2.0 * weight * g[i] * g[j];
                P[i][j] += val;
                if (i != j) P[j][i] += val;
            }
        }
    };

    double xy_coef_x = is_roadblock ? (0.10 * config::FINE_X_COEF) : config::FINE_X_COEF;
    double xy_coef_y = is_roadblock ? (0.10 * config::FINE_Y_COEF) : config::FINE_Y_COEF;
    double q_v = is_roadblock ? 120.0 : config::FINE_V_COEF;

    // 4. Đóng góp chi phí bám quỹ đạo, góc hướng, vận tốc, sai lệch ngang, độ mượt & biến bù hành lang
    // Đồng bộ 100% với src/linearized_mpc_controller.py (dòng 1330-1380): X[:, k] ứng với x_ref[k] tại t = k * dt
    for (int k = 0; k < horizon_; ++k) {
        int st_idx = k;
        const auto& S_k = S_mat[st_idx];
        const auto& c_k = c_vec[st_idx];

        // (a) Sai số X, Y, V, Theta
        add_quadratic_term(xy_coef_x, S_k[0], c_k[0] - x_ref_mod[k]);
        add_quadratic_term(xy_coef_y, S_k[1], c_k[1] - y_ref_mod[k]);
        add_quadratic_term(q_v,       S_k[3], c_k[3] - v_ref[k]);
        add_quadratic_term(config::FINE_THETA_COEF, S_k[2], math::normalize_angle(c_k[2] - theta_ref_mod[k]));

        // (b) Độ lệch ngang vuông góc chuẩn Frenet (calculate_lateral_deviation)
        if (k < horizon_ - 1) {
            double dx_r = x_ref_mod[k + 1] - x_ref_mod[k];
            double dy_r = y_ref_mod[k + 1] - y_ref_mod[k];
            double len_r = std::hypot(dx_r, dy_r);
            if (len_r > 1e-5) {
                double inv_len = 1.0 / len_r;
                std::vector<double> g_lat(n_u, 0.0);
                for (int col = 0; col < n_u; ++col) {
                    g_lat[col] = (S_k[0][col] * dy_r - S_k[1][col] * dx_r) * inv_len;
                }
                double r_lat = ((c_k[0] - x_ref_mod[k]) * dy_r - (c_k[1] - y_ref_mod[k]) * dx_r) * inv_len;
                add_quadratic_term(config::FINE_LATERAL_COEF, g_lat, r_lat);
            }
        }

        // (c) Nỗ lực điều khiển U_k = [delta_k, a_k]
        int idx_delta = 2 * k;
        int idx_a = 2 * k + 1;
        P[idx_delta][idx_delta] += 2.0 * config::FINE_STEER_COEF;
        P[idx_a][idx_a]         += 2.0 * config::FINE_ACC_COEF;

        // (d) Phạt tốc độ bẻ lái và đạo hàm gia tốc (k=0 so với u_prev_, k>=1 so với k-1)
        if (k == 0) {
            P[idx_delta][idx_delta] += 2.0 * config::FINE_STEER_DOT_COEF;
            q[idx_delta]            -= 2.0 * config::FINE_STEER_DOT_COEF * u_prev_[0];

            P[idx_a][idx_a]         += 2.0 * config::FINE_ACC_DOT_COEF;
            q[idx_a]                -= 2.0 * config::FINE_ACC_DOT_COEF * u_prev_[1];
        } else {
            int prev_delta = 2 * (k - 1);
            int prev_a = 2 * (k - 1) + 1;

            double w_sd = 2.0 * config::FINE_STEER_DOT_COEF;
            P[idx_delta][idx_delta]   += w_sd;
            P[prev_delta][prev_delta] += w_sd;
            P[idx_delta][prev_delta]  -= w_sd;
            P[prev_delta][idx_delta]  -= w_sd;

            double w_ad = 2.0 * config::FINE_ACC_DOT_COEF;
            P[idx_a][idx_a]   += w_ad;
            P[prev_a][prev_a] += w_ad;
            P[idx_a][prev_a]  -= w_ad;
            P[prev_a][idx_a]  -= w_ad;
        }

        // (e) Ràng buộc mềm Hành lang Pháp lý Động với biến bù eps_road_left[k] >= 0, eps_road_right[k] >= 0
        int i_el = idx_eps_left(k);
        int i_er = idx_eps_right(k);

        double w_left_q = horizon_is_yellow_solid[k] ? config::MPC_W_YELLOW_QUAD : config::MPC_W_ROAD_RIGHT;
        double w_left_l = horizon_is_yellow_solid[k] ? config::MPC_W_YELLOW_LINEAR : config::MPC_W_ROAD_RIGHT_LINEAR;
        P[i_el][i_el] = 2.0 * w_left_q;
        q[i_el]       = w_left_l;

        P[i_er][i_er] = 2.0 * config::MPC_W_ROAD_RIGHT;
        q[i_er]       = config::MPC_W_ROAD_RIGHT_LINEAR;

        // Ràng buộc không âm cho eps_road_left[k] >= 0 và eps_road_right[k] >= 0
        int row_el_box = 3 * horizon_ + k;
        A_cons[row_el_box][i_el] = 1.0;
        l_cons[row_el_box] = 0.0;
        u_cons[row_el_box] = kInf;

        int row_er_box = 4 * horizon_ + k;
        A_cons[row_er_box][i_er] = 1.0;
        l_cons[row_er_box] = 0.0;
        u_cons[row_er_box] = kInf;

        // d_lat_k = g_dlat^T * U + r_dlat
        double nx_orig = -std::sin(theta_ref_orig[k]);
        double ny_orig =  std::cos(theta_ref_orig[k]);
        double r_dlat = (c_k[0] - x_ref_orig[k]) * nx_orig + (c_k[1] - y_ref_orig[k]) * ny_orig;

        int row_road_r = 9 * horizon_ + k;
        int row_road_l = 10 * horizon_ + k;
        for (int col = 0; col < n_u; ++col) {
            double g_col = S_k[0][col] * nx_orig + S_k[1][col] * ny_orig;
            A_cons[row_road_r][col] = g_col;
            A_cons[row_road_l][col] = g_col;
        }
        // d_lat_k - eps_road_right[k] <= horizon_road_max[k]
        A_cons[row_road_r][i_er] = -1.0;
        l_cons[row_road_r] = -kInf;
        u_cons[row_road_r] = horizon_road_max[k] - r_dlat;

        // d_lat_k + eps_road_left[k] >= horizon_road_min[k]
        A_cons[row_road_l][i_el] = 1.0;
        l_cons[row_road_l] = horizon_road_min[k] - r_dlat;
        u_cons[row_road_l] = kInf;
    }

    // Khởi tạo mặc định cho 4*N biến bù siêu phẳng vật cản eps_obs[m, k]
    for (int m = 0; m < max_obs; ++m) {
        for (int k = 0; k < horizon_; ++k) {
            int i_eo = idx_eps_obs(m, k);
            P[i_eo][i_eo] = 2.0 * config::MPC_W_SLACK;
            q[i_eo]       = config::MPC_W_SLACK_LINEAR;

            int row_eo_box = (5 + m) * horizon_ + k;
            A_cons[row_eo_box][i_eo] = 1.0;
            l_cons[row_eo_box] = 0.0;
            u_cons[row_eo_box] = 0.0; // Khóa bằng 0 khi không kích hoạt

            int row_obs_ineq = (11 + m) * horizon_ + k;
            A_cons[row_obs_ineq][i_eo] = 1.0;
            l_cons[row_obs_ineq] = -kInf;
            u_cons[row_obs_ineq] =  kInf;
        }
    }

    // 5. Ràng buộc Siêu phẳng phân tách vật cản với biến bù eps_obs[m, k] >= 0 (SKILL.md Mục 5.1)
    if (!is_roadblock && !obstacles.empty()) {
        double tx_ego = std::cos(x_init[2]);
        double ty_ego = std::sin(x_init[2]);

        std::vector<types::ObstacleFrenetOBB> ahead_obs;
        for (const auto& obs : obstacles) {
            double lon_rel = tx_ego * (obs.center_x - x_init[0]) + ty_ego * (obs.center_y - x_init[1]);
            if (lon_rel >= -2.5) {
                ahead_obs.push_back(obs);
            }
        }
        std::sort(ahead_obs.begin(), ahead_obs.end(), [&](const auto& a, const auto& b) {
            return std::hypot(a.center_x - x_init[0], a.center_y - x_init[1]) <
                   std::hypot(b.center_x - x_init[0], b.center_y - x_init[1]);
        });
        if (static_cast<int>(ahead_obs.size()) > max_obs) ahead_obs.resize(max_obs);

        double r_safe = config::R_CLEARANCE_EDGE; // 1.35m

        for (size_t m = 0; m < ahead_obs.size(); ++m) {
            const auto& obs = ahead_obs[m];
            double s_zone_start = obs.s_rear - config::EGO_FRONT_OVERHANG_M - 0.5;
            double s_zone_end   = obs.s_front + config::EGO_REAR_OVERHANG_M + 0.5;
            double obs_d_center = 0.5 * (obs.d_min + obs.d_max);

            for (int k = 0; k < horizon_; ++k) {
                double s_k = (k < static_cast<int>(s_queries_.size())) ? s_queries_[k] : 0.0;
                if (s_k < s_zone_start || s_k > s_zone_end) continue;

                int st_idx = k;
                double nx_k = -std::sin(theta_ref_orig[k]);
                double ny_k =  std::cos(theta_ref_orig[k]);
                double d_rk = (x_ref_mod[k] - x_ref_orig[k]) * nx_k + (y_ref_mod[k] - y_ref_orig[k]) * ny_k;

                // QUY TẮC BẤT BIẾN (SKILL.md Mục 5.1):
                // Hướng siêu phẳng pass_sign bắt buộc so sánh d_rk với tâm vật cản obs_d_center,
                // tuyệt đối không so sánh với 0.0!
                double pass_sign = 1.0;
                if (std::abs(obs.side_sign) > 0.1) {
                    pass_sign = obs.side_sign;
                } else if (std::abs(d_rk - obs_d_center) > 0.05) {
                    pass_sign = (d_rk > obs_d_center) ? 1.0 : -1.0;
                } else {
                    pass_sign = (d_rk >= 0.0) ? 1.0 : -1.0;
                }

                // Khi có vạch vàng SOLID: nếu đi bên trái vật cản ép tâm xe vượt quá horizon_road_min[k],
                // bắt buộc đặt siêu phẳng đẩy sang bên PHẢI vật cản để bảo vệ vạch vàng
                if (any_yellow_solid && pass_sign < 0.0 && (obs.d_min - r_safe) < (horizon_road_min[k] - 0.05)) {
                    pass_sign = 1.0;
                }

                int i_eo = idx_eps_obs(static_cast<int>(m), k);
                int row_eo_box = (5 + static_cast<int>(m)) * horizon_ + k;
                u_cons[row_eo_box] = kInf; // Mở khóa biến bù eps_obs[m, k] >= 0

                int row_obs_ineq = (11 + static_cast<int>(m)) * horizon_ + k;
                double r_dlat = (c_vec[st_idx][0] - x_ref_orig[k]) * nx_k + (c_vec[st_idx][1] - y_ref_orig[k]) * ny_k;

                if (pass_sign > 0.0) {
                    // Né bên PHẢI vật cản: d_lat_k + eps_obs[m, k] >= obs.d_max + r_safe
                    for (int col = 0; col < n_u; ++col) {
                        A_cons[row_obs_ineq][col] = S_mat[st_idx][0][col] * nx_k + S_mat[st_idx][1][col] * ny_k;
                    }
                    A_cons[row_obs_ineq][i_eo] = 1.0;
                    l_cons[row_obs_ineq] = (obs.d_max + r_safe) - r_dlat;
                    u_cons[row_obs_ineq] = kInf;
                } else {
                    // Né bên TRÁI vật cản: -d_lat_k + eps_obs[m, k] >= -obs.d_min + r_safe
                    for (int col = 0; col < n_u; ++col) {
                        A_cons[row_obs_ineq][col] = -(S_mat[st_idx][0][col] * nx_k + S_mat[st_idx][1][col] * ny_k);
                    }
                    A_cons[row_obs_ineq][i_eo] = 1.0;
                    l_cons[row_obs_ineq] = r_dlat - (obs.d_min - r_safe);
                    u_cons[row_obs_ineq] = kInf;
                }
            }
        }
    }

    // 6. Thiết lập ràng buộc biên điều khiển vật lý & vận tốc: l_cons <= A_cons * z <= u_cons
    double min_v = is_roadblock ? 0.0 : std::min(x_init[3], 1.5);

    for (int k = 0; k < horizon_; ++k) {
        int idx_delta = 2 * k;
        int idx_a = 2 * k + 1;

        // Ràng buộc góc lái
        A_cons[2 * k][idx_delta] = 1.0;
        l_cons[2 * k] = -config::MAX_WHEEL_ANGLE_RAD;
        u_cons[2 * k] =  config::MAX_WHEEL_ANGLE_RAD;

        // Ràng buộc gia tốc
        A_cons[2 * k + 1][idx_a] = 1.0;
        l_cons[2 * k + 1] = config::MAX_BRAKING_MS2;
        u_cons[2 * k + 1] = config::MAX_ACCELERATION_MS2;

        // Ràng buộc vận tốc: min_v <= c_{k+1}[3] + S_{k+1}[3] * U <= 15.0
        int row_v = 2 * horizon_ + k;
        for (int col = 0; col < n_u; ++col) {
            A_cons[row_v][col] = S_mat[k + 1][3][col];
        }
        l_cons[row_v] = min_v - c_vec[k + 1][3];
        u_cons[row_v] = 15.0  - c_vec[k + 1][3];
    }

    qp_solver_.setup(n_vars, n_cons);
    std::vector<double> z_flat;
    double qp_obj = 0.0;
    if (!qp_solver_.solve_dense(P, q, A_cons, l_cons, u_cons, z_flat, &qp_obj) ||
        static_cast<int>(z_flat.size()) < n_u) {
        return false;
    }

    out_u_opt.resize(horizon_);
    for (int k = 0; k < horizon_; ++k) {
        out_u_opt[k][0] = std::clamp(z_flat[2 * k], -config::MAX_WHEEL_ANGLE_RAD, config::MAX_WHEEL_ANGLE_RAD);
        out_u_opt[k][1] = std::clamp(z_flat[2 * k + 1], config::MAX_BRAKING_MS2, config::MAX_ACCELERATION_MS2);
    }

    // Khôi phục chuỗi trạng thái tối ưu X_opt từ phương trình tuyến tính hóa
    out_x_opt.resize(horizon_ + 1);
    out_x_opt[0] = x_init;
    for (int k = 0; k < horizon_; ++k) {
        for (int i = 0; i < 4; ++i) {
            double val = c_vec[k + 1][i];
            for (int col = 0; col < n_u; ++col) {
                val += S_mat[k + 1][i][col] * z_flat[col];
            }
            out_x_opt[k + 1][i] = val;
        }
    }

    out_cost = qp_obj;
    return true;
}

MpcOutput LinearizedMpcController::solve(
    const std::array<double, 4>& x_init,
    const std::vector<double>& x_ref_in,
    const std::vector<double>& y_ref_in,
    const std::vector<double>& theta_ref_in,
    const std::vector<double>& v_ref_in,
    const std::vector<types::ObstacleFrenetOBB>& detected_obstacles,
    const types::CorridorBoundsData& corridor_data,
    double dt_loop) {

    MpcOutput out;

    // 1. Đồng bộ bước không gian quỹ đạo tham chiếu theo vận tốc thực tế
    std::vector<double> x_sync, y_sync, theta_sync, v_sync;
    resample_ref_by_speed(x_init, x_ref_in, y_ref_in, theta_ref_in, v_ref_in, detected_obstacles,
                          x_sync, y_sync, theta_sync, v_sync);

    // 2. Dời tạm tham chiếu cục bộ S-Curve né vật cản & kiểm tra Roadblock
    bool has_shift = false;
    compute_local_reference_shift(x_init, x_sync, y_sync, theta_sync, v_sync,
                                  detected_obstacles, corridor_data, dt_loop,
                                  x_ref_mod_, y_ref_mod_, theta_ref_mod_, has_shift);

    // 3. Tập hợp danh sách vật cản hợp lệ trong hành lang để đưa vào ràng buộc Siêu phẳng QP
    double r_clear_obs = config::R_CLEARANCE_EDGE;
    double road_min = config::STANDARD_ROAD_MIN_M, road_max = config::STANDARD_ROAD_MAX_M;
    bool is_yellow_solid = true;
    get_legal_corridor_bounds(current_traj_idx_, std::nullopt, corridor_data, road_min, road_max, is_yellow_solid);

    std::vector<types::ObstacleFrenetOBB> all_obs;
    std::unordered_set<int32_t> seen_ids;
    if (has_active_obs_) {
        all_obs.push_back(active_obs_);
        seen_ids.insert(active_obs_.id);
    }
    for (const auto& obs : detected_obstacles) {
        if (seen_ids.count(obs.id)) continue;
        if (active_chain_.count(obs.id)) {
            all_obs.push_back(active_chain_[obs.id]);
            seen_ids.insert(obs.id);
            continue;
        }
        double obs_map_min = road_min, obs_map_max = road_max;
        bool obs_map_yl = is_yellow_solid;
        get_legal_corridor_bounds(std::nullopt, obs.s_rear, corridor_data, obs_map_min, obs_map_max, obs_map_yl);
        if ((is_yellow_solid || obs_map_yl) && (obs.d_max + r_clear_obs) <= (std::max(road_min, obs_map_min) + 0.05)) {
            continue;
        }
        if ((obs.d_max + r_clear_obs) <= (obs_map_min + 0.15) || (obs.d_min - r_clear_obs) >= (obs_map_max - 0.15)) {
            continue;
        }
        all_obs.push_back(obs);
        seen_ids.insert(obs.id);
    }

    // 4. Xử lý kịch bản dừng Roadblock (Mode 1 Safe Stop + Limousine Stop + Anti-Jerk)
    if (is_roadblock_) {
        double d_stop = std::max(0.0, roadblock_barrier_dist_ - config::ROADBLOCK_SAFE_STOP_DIST_M);
        double curr_v = x_init[3];

        // Chốt dừng tĩnh (roadblock_stopped_latch_ = true) khi v < 0.20 m/s hoặc d_stop <= 0.35m (SKILL.md Mục 5.2)
        if (d_stop <= 0.35 || (curr_v < 0.20 && d_stop <= 12.0)) {
            roadblock_stopped_latch_ = true;
        }

        if (roadblock_stopped_latch_) {
            out.steer_rad = 0.0;
            out.acceleration_m_s2 = -1.2;
            out.is_success = true;
            u_prev_ = {0.0, -1.2};
            std::vector<double> zero_v(horizon_, 0.0);
            init_nominal_trajectory(x_init, x_ref_mod_, y_ref_mod_, theta_ref_mod_, zero_v);
            return out;
        }

        auto v_ref_adapted = compute_safe_speed_profile(x_init, v_sync);
        double a_req = (curr_v * curr_v) / (2.0 * std::max(d_stop, 0.35));
        double a_decel = std::clamp(std::max(config::ROADBLOCK_DECELERATION_MS2 * 0.75, a_req),
                                    0.6, std::abs(config::MAX_BRAKING_MS2));
        for (int k = 0; k < horizon_; ++k) {
            double v_target_k = std::max(0.0, curr_v - a_decel * (k + 1) * dt_);
            v_ref_adapted[k] = std::min(v_ref_adapted[k], v_target_k);
        }

        if (!nom_initialized_) {
            init_nominal_trajectory(x_init, x_ref_mod_, y_ref_mod_, theta_ref_mod_, v_ref_adapted);
        }

        std::vector<std::array<double, 4>> x_opt;
        std::vector<std::array<double, 2>> u_opt;
        double cost_val = 0.0;
        bool qp_ok = solve_qp_iteration(x_init, x_ref_mod_, y_ref_mod_, theta_ref_mod_, v_ref_adapted,
                                        x_sync, y_sync, theta_sync, all_obs, corridor_data, true,
                                        x_opt, u_opt, cost_val);
        if (qp_ok) {
            double steer_rad = u_opt[0][0];
            double acc_m_s2 = u_opt[0][1];
            if (d_stop <= 10.0 && curr_v > 0.15) {
                double req_decel = -a_req - 0.10;
                acc_m_s2 = std::min(acc_m_s2, std::max(config::MAX_BRAKING_MS2, req_decel));
            }
            // Nhả phanh đệm mượt mà (Brake Feathering / Limousine Stop): v < 2.2 m/s & d_stop > 1.8m -> a >= -0.95 m/s^2
            if (curr_v < 2.2 && d_stop > 1.8) {
                acc_m_s2 = std::max(acc_m_s2, -0.95);
            }
            // Tuyệt đối không cấp ga dương khi đang dừng Roadblock
            acc_m_s2 = std::min(0.0, acc_m_s2);
            // Rate Limiter chống Jerk: |da| <= 0.45 m/s^2
            double prev_acc = std::min(0.0, u_prev_[1]);
            acc_m_s2 = std::clamp(acc_m_s2, prev_acc - config::MAX_ACCEL_DELTA_PER_100MS,
                                            prev_acc + config::MAX_ACCEL_DELTA_PER_100MS);
            u_opt[0][1] = acc_m_s2;
            u_prev_ = {steer_rad, acc_m_s2};
            shift_nominal_trajectory(x_opt, u_opt, dt_loop);

            out.steer_rad = steer_rad;
            out.acceleration_m_s2 = acc_m_s2;
            out.cost = cost_val;
            out.is_success = true;
            return out;
        } else {
            out.steer_rad = 0.0;
            out.acceleration_m_s2 = -config::ROADBLOCK_DECELERATION_MS2;
            out.is_success = false;
            return out;
        }
    }

    // 5. Chế độ bám quỹ đạo & né vật cản bình thường bằng Successive Linearization QP-MPC
    auto v_ref_adapted = compute_safe_speed_profile(x_init, v_sync);
    if (!nom_initialized_) {
        init_nominal_trajectory(x_init, x_ref_mod_, y_ref_mod_, theta_ref_mod_, v_ref_adapted);
    }

    std::vector<std::array<double, 4>> x_opt;
    std::vector<std::array<double, 2>> u_opt;
    double cost_val = 0.0;
    bool qp_ok = false;

    for (int it = 0; it < config::MPC_MAX_LINEARIZATION_ITER; ++it) {
        qp_ok = solve_qp_iteration(x_init, x_ref_mod_, y_ref_mod_, theta_ref_mod_, v_ref_adapted,
                                   x_sync, y_sync, theta_sync, all_obs, corridor_data, false,
                                   x_opt, u_opt, cost_val);
        if (qp_ok) {
            x_nom_ = x_opt;
            u_nom_ = u_opt;
        } else {
            break;
        }
    }

    if (qp_ok && !u_opt.empty()) {
        double steer_rad = u_opt[0][0];
        double acc_m_s2 = u_opt[0][1];

        // Van một chiều bảo vệ vạch vàng trong dự báo MPC (Yellow Line One-Way Safety Valve)
        bool is_crossing_yellow = false;
        if (is_yellow_solid && has_active_obs_ && !x_traj_.empty()) {
            auto proj_now = math::project_to_frenet(x_init[0], x_init[1], x_traj_, y_traj_, theta_traj_, s_traj_, current_traj_idx_);
            double obs_s_front = (active_obs_.group_s_front > active_obs_.s_front) ? active_obs_.group_s_front : active_obs_.s_front;
            if (obs_s_front >= proj_now.s && active_obs_.side_sign <= 0.0) {
                auto p0 = math::project_to_frenet(x_nom_[0][0], x_nom_[0][1], x_traj_, y_traj_, theta_traj_, s_traj_, current_traj_idx_);
                for (int k = 4; k <= horizon_; ++k) {
                    auto pk = math::project_to_frenet(x_nom_[k][0], x_nom_[k][1], x_traj_, y_traj_, theta_traj_, s_traj_, current_traj_idx_);
                    double rmin_k = road_min, rmax_k = road_max;
                    bool yl_k = is_yellow_solid;
                    get_legal_corridor_bounds(std::nullopt, pk.s, corridor_data, rmin_k, rmax_k, yl_k);
                    if (yl_k && pk.d < (rmin_k - 0.25) && pk.d < (p0.d - 0.05)) {
                        is_crossing_yellow = true;
                        break;
                    }
                }
                if (is_crossing_yellow) {
                    is_roadblock_ = true;
                    roadblock_reason_ = "Quỹ đạo dự đoán MPC cắt qua vạch vàng nét liền; kích hoạt phanh dừng an toàn!";
                    steer_rad = std::clamp(steer_rad, -0.15, 0.15);
                    acc_m_s2 = -config::ROADBLOCK_DECELERATION_MS2;
                }
            }
        }

        // Bộ giới hạn tốc độ biến thiên gia tốc (Anti-Jerk Slew Rate Limiter: |da/dt| <= 2.0 m/s^3)
        // Bỏ qua khi Van bảo vệ vạch vàng kích hoạt để ưu tiên phanh dừng an toàn tức thời
        // Đồng bộ nghiệm u_opt[0][1] để Warm-start hoàn toàn nhất quán với u_prev_
        if (!is_crossing_yellow) {
            double max_da = 0.20 * std::clamp(dt_loop / 0.10, 0.20, 1.20);
            acc_m_s2 = std::clamp(acc_m_s2, u_prev_[1] - max_da, u_prev_[1] + max_da);
        }
        u_opt[0][1] = acc_m_s2;
        u_prev_ = {steer_rad, acc_m_s2};

        for (int k = 0; k < horizon_ - 1; ++k) {
            buffer_steer_[k] = u_opt[k + 1][0];
            buffer_accel_[k] = u_opt[k + 1][1];
        }
        buffer_steer_[horizon_ - 1] = u_opt[horizon_ - 1][0];
        buffer_accel_[horizon_ - 1] = u_opt[horizon_ - 1][1];
        buffer_idx_ = 0;

        // Dịch chuyển nghiệm Warm-start cho chu kỳ tiếp theo (tương ứng tỷ lệ dt_loop / dt_)
        shift_nominal_trajectory(x_opt, u_opt, dt_loop);

        out.steer_rad = steer_rad;
        out.acceleration_m_s2 = acc_m_s2;
        out.cost = cost_val;
        out.is_success = true;
    } else {
        out.is_success = false;
        if (buffer_idx_ < buffer_steer_.size()) {
            out.steer_rad = buffer_steer_[buffer_idx_];
            out.acceleration_m_s2 = buffer_accel_[buffer_idx_];
            buffer_idx_++;
        } else {
            out.steer_rad = 0.0;
            out.acceleration_m_s2 = -1.0;
        }
        init_nominal_trajectory(x_init, x_ref_mod_, y_ref_mod_, theta_ref_mod_, v_ref_adapted);
    }

    return out;
}

} // namespace av::mpc
