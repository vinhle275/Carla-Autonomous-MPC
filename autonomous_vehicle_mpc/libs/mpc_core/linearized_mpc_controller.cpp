#include "linearized_mpc_controller.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace av::mpc {

LinearizedMpcController::LinearizedMpcController(int horizon, double dt)
    : horizon_(horizon), dt_(dt) {
    x_nom_.resize(horizon_ + 1, {0.0, 0.0, 0.0, 0.0});
    u_nom_.resize(horizon_, {0.0, 0.0});
    x_ref_mod_.resize(horizon_, 0.0);
    y_ref_mod_.resize(horizon_, 0.0);
    theta_ref_mod_.resize(horizon_, 0.0);
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
    }
}

void LinearizedMpcController::reset_nominal() {
    for (auto& st : x_nom_) st = {0.0, 0.0, 0.0, 0.0};
    for (auto& ct : u_nom_) ct = {0.0, 0.0};
    u_prev_ = {0.0, 0.0};
    roadblock_latched_ids_.clear();
    roadblock_stopped_latch_ = false;
    is_roadblock_ = false;
    has_active_obs_ = false;
    active_chain_.clear();
    lateral_crossing_hold_.clear();
}

double LinearizedMpcController::calc_span_obstacle_weight(double s_query, double s_rear, double s_front, double min_s_app_start) const {
    double s_hold_start = s_rear - config::EGO_FRONT_OVERHANG_M - config::S_CURVE_APPROACH_MARGIN_M;
    double s_app_start = s_hold_start - config::S_CURVE_APPROACH_RAMP_M;
    if (min_s_app_start > -1e8 && min_s_app_start > s_app_start) {
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

void LinearizedMpcController::resample_ref_by_speed(const std::array<double, 4>& x_init,
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

    bool has_near_obs = has_active_obs_ || (std::abs(proj.d) > 0.35);
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

    for (int k = 0; k < horizon_; ++k) {
        double s_q = std::min(s_acc, s_max);
        double v_tgt = has_near_obs ? std::min(v_ref_in[k], config::CAUTION_SPEED_MS) : v_ref_in[k];
        double dv = std::clamp(v_tgt - v_curr, -2.5 * dt_, 2.0 * dt_);
        v_curr = std::max(2.0, v_curr + dv);
        s_acc += v_curr * dt_;

        x_sync[k] = math::interp1d(s_q, s_traj_, x_traj_);
        y_sync[k] = math::interp1d(s_q, s_traj_, y_traj_);
        theta_sync[k] = math::interp1d(s_q, s_traj_, theta_traj_);
        v_sync[k] = v_curr;
    }
}

void LinearizedMpcController::compute_local_reference_shift(const std::array<double, 4>& x_init,
                                                          const std::vector<double>& x_ref,
                                                          const std::vector<double>& y_ref,
                                                          const std::vector<double>& theta_ref,
                                                          const std::vector<double>& /*v_ref*/,
                                                          const std::vector<types::ObstacleFrenetOBB>& detected_obstacles,
                                                          const types::CorridorBoundsData& corridor_data,
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

    current_road_min_ = corridor_data.road_min[0];
    current_road_max_ = corridor_data.road_max[0];
    bool is_yellow_solid = corridor_data.is_yellow_solid[0];

    // Thu thập các vật cản phía trước nằm trên mặt đường
    std::vector<types::ObstacleFrenetOBB> candidates;
    for (const auto& obs : detected_obstacles) {
        if (obs.s_front >= s_ego - 2.0 && obs.s_rear <= s_ego + 35.0) {
            // Kiểm tra vật cản xâm lấn hành lang
            if (obs.d_max >= current_road_min_ - 0.1 && obs.d_min <= current_road_max_ + 0.1) {
                candidates.push_back(obs);
            }
        }
    }

    if (candidates.empty()) {
        has_active_obs_ = false;
        is_roadblock_ = false;
        return;
    }

    // Kiểm tra Staggered Roadblock: 2 vật cản so le cách nhau dưới 17.3m
    if (candidates.size() >= 2) {
        std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
            return a.s_rear < b.s_rear;
        });
        for (size_t i = 0; i < candidates.size() - 1; ++i) {
            double ds = candidates[i + 1].s_rear - candidates[i].s_front;
            double dd = std::abs(candidates[i + 1].d_center - candidates[i].d_center);
            double min_pass_dist = config::EGO_FRONT_OVERHANG_M + config::EGO_REAR_OVERHANG_M +
                                  config::OBSTACLE_AVOIDANCE_MARGIN_M + std::max(7.5, 4.0 * dd);
            if (ds < min_pass_dist && ds > -2.0) {
                // Hai xe so le quá gần nhau, không đủ chiều dài cho đường cong S-Curve -> PHONG TỎA ĐƯỜNG!
                is_roadblock_ = true;
                roadblock_barrier_dist_ = std::max(0.0, candidates[0].s_rear - s_ego);
                return;
            }
        }
    }

    // Chọn vật cản gần nhất để né
    const auto& obs = candidates[0];
    active_obs_ = obs;
    has_active_obs_ = true;

    // Xác định chiều né: ưu tiên né phải nếu hành lang cho phép
    double d_target = 0.0;
    double r_safe = config::EGO_HALF_WIDTH_M + config::OBSTACLE_AVOIDANCE_MARGIN_M;

    double d_pass_right = obs.d_max + r_safe + 0.15;
    double d_pass_left = obs.d_min - r_safe - 0.15;

    bool can_pass_right = (d_pass_right <= current_road_max_);
    bool can_pass_left = (d_pass_left >= current_road_min_) && (!is_yellow_solid || d_pass_left >= -0.45);

    if (can_pass_right) {
        d_target = d_pass_right;
        active_obs_.side_sign = 1.0;
    } else if (can_pass_left) {
        d_target = d_pass_left;
        active_obs_.side_sign = -1.0;
    } else {
        // Cả 2 bên đều bị chặn -> ROADBLOCK!
        is_roadblock_ = true;
        roadblock_barrier_dist_ = std::max(0.0, obs.s_rear - s_ego);
        return;
    }

    // Áp dụng đường cong dời tham chiếu S-Curve dọc theo N bước
    for (int k = 0; k < horizon_; ++k) {
        auto proj_k = math::project_to_frenet(x_ref[k], y_ref[k], x_traj_, y_traj_, theta_traj_, s_traj_, current_traj_idx_);
        double w = calc_span_obstacle_weight(proj_k.s, obs.s_rear, obs.s_front);
        if (w > 0.001) {
            has_shift = true;
            double d_shift = w * d_target;
            x_mod[k] += d_shift * proj_k.nx;
            y_mod[k] += d_shift * proj_k.ny;
        }
    }

    // Cập nhật lại góc hướng theta_mod theo vi phân cung đường
    for (int k = 0; k < horizon_ - 1; ++k) {
        double dx = x_mod[k + 1] - x_mod[k];
        double dy = y_mod[k + 1] - y_mod[k];
        if (std::hypot(dx, dy) > 1e-4) {
            theta_mod[k] = std::atan2(dy, dx);
        }
    }
    if (horizon_ > 1) theta_mod[horizon_ - 1] = theta_mod[horizon_ - 2];
}

MpcOutput LinearizedMpcController::solve(const std::array<double, 4>& x_init,
                                        const std::vector<double>& x_ref_in,
                                        const std::vector<double>& y_ref_in,
                                        const std::vector<double>& theta_ref_in,
                                        const std::vector<double>& v_ref_in,
                                        const std::vector<types::ObstacleFrenetOBB>& detected_obstacles,
                                        const types::CorridorBoundsData& corridor_data) {
    MpcOutput out;

    // 1. Đồng bộ bước cung đường theo vận tốc thực tế
    std::vector<double> x_sync, y_sync, theta_sync, v_sync;
    resample_ref_by_speed(x_init, x_ref_in, y_ref_in, theta_ref_in, v_ref_in, detected_obstacles,
                          x_sync, y_sync, theta_sync, v_sync);

    // 2. Dời tạm tham chiếu cục bộ S-Curve né vật cản & kiểm tra Roadblock
    bool has_shift = false;
    compute_local_reference_shift(x_init, x_sync, y_sync, theta_sync, v_sync,
                                  detected_obstacles, corridor_data,
                                  x_ref_mod_, y_ref_mod_, theta_ref_mod_, has_shift);

    // 3. Xử lý kịch bản dừng Roadblock
    if (is_roadblock_) {
        double d_stop = std::max(0.0, roadblock_barrier_dist_ - config::ROADBLOCK_SAFE_STOP_DIST_M);
        double curr_v = x_init[3];

        if (d_stop <= 0.35 || (curr_v < 0.20 && d_stop <= 12.0)) {
            roadblock_stopped_latch_ = true;
        }

        if (roadblock_stopped_latch_) {
            out.steer_rad = 0.0;
            out.acceleration_m_s2 = -1.2;
            out.is_success = true;
            u_prev_ = {0.0, -1.2};
            return out;
        }

        double a_req = (curr_v * curr_v) / (2.0 * std::max(d_stop, 0.35));
        double a_decel = -std::clamp(std::max(config::ROADBLOCK_DECELERATION_MS2 * 0.75, a_req), 0.6, std::abs(config::MAX_BRAKING_MS2));

        // Nhả nhẹ lực phanh (Limousine Stop) khi vận tốc thấp
        if (curr_v < 2.2 && d_stop > 1.8) {
            a_decel = std::max(a_decel, -0.95);
        }
        // Rate limiter gia tốc
        a_decel = std::clamp(a_decel, u_prev_[1] - 0.45, u_prev_[1] + 0.45);

        out.steer_rad = 0.0;
        out.acceleration_m_s2 = a_decel;
        out.is_success = true;
        u_prev_ = {out.steer_rad, out.acceleration_m_s2};
        return out;
    }

    // 4. Tuyến tính hóa & Dựng bài toán QP
    // Vector biến tối ưu: u = [delta_0, a_0, delta_1, a_1, ..., delta_{N-1}, a_{N-1}]^T (kích thước 2 * N)
    int n_vars = 2 * horizon_;
    int n_cons = horizon_ * 2; // Ràng buộc lái và gia tốc

    std::vector<double> P_diag(n_vars, 0.0);
    std::vector<double> q(n_vars, 0.0);
    std::vector<std::vector<double>> A_cons(n_cons, std::vector<double>(n_vars, 0.0));
    std::vector<double> l_cons(n_cons, 0.0);
    std::vector<double> u_cons(n_cons, 0.0);

    for (int k = 0; k < horizon_; ++k) {
        int idx_delta = 2 * k;
        int idx_a = 2 * k + 1;

        // Trọng số mục tiêu
        P_diag[idx_delta] = config::FINE_STEER_COEF + 2.0 * config::FINE_STEER_DOT_COEF;
        P_diag[idx_a] = config::FINE_ACC_COEF + 2.0 * config::FINE_ACC_DOT_COEF;

        // Gradient bù sai số bám đường và bám vận tốc
        double v_err = x_init[3] - v_sync[k];
        q[idx_a] = config::FINE_V_COEF * v_err;

        // Sai số bám hướng và độ lệch ngang
        double dx = x_init[0] - x_ref_mod_[k];
        double dy = x_init[1] - y_ref_mod_[k];
        double th_ref = theta_ref_mod_[k];
        double lat_err = -std::sin(th_ref) * dx + std::cos(th_ref) * dy;
        double th_err = math::normalize_angle(x_init[2] - th_ref);

        q[idx_delta] = (config::FINE_LATERAL_COEF * lat_err * 0.15) + (config::FINE_THETA_COEF * th_err * 0.35);

        // Phạt tốc độ quay vô lăng so với bước trước
        if (k == 0) {
            q[idx_delta] -= 2.0 * config::FINE_STEER_DOT_COEF * u_prev_[0];
            q[idx_a] -= 2.0 * config::FINE_ACC_DOT_COEF * u_prev_[1];
        }

        // Ràng buộc biên điều khiển vật lý
        A_cons[2 * k][idx_delta] = 1.0;
        l_cons[2 * k] = -config::MAX_WHEEL_ANGLE_RAD;
        u_cons[2 * k] =  config::MAX_WHEEL_ANGLE_RAD;

        A_cons[2 * k + 1][idx_a] = 1.0;
        l_cons[2 * k + 1] = config::MAX_BRAKING_MS2;
        u_cons[2 * k + 1] = config::MAX_ACCELERATION_MS2;
    }

    qp_solver_.setup(n_vars, n_cons);
    std::vector<double> u_opt;
    bool qp_ok = qp_solver_.solve(P_diag, q, A_cons, l_cons, u_cons, u_opt);

    if (qp_ok && u_opt.size() >= 2) {
        out.steer_rad = u_opt[0];
        out.acceleration_m_s2 = u_opt[1];
        out.is_success = true;
    } else {
        out.steer_rad = 0.0;
        out.acceleration_m_s2 = -1.5;
        out.is_success = false;
    }

    // Rate Limiter gia tốc chống giật dọc Jerk
    out.acceleration_m_s2 = std::clamp(out.acceleration_m_s2, u_prev_[1] - 0.45, u_prev_[1] + 0.45);
    u_prev_ = {out.steer_rad, out.acceleration_m_s2};

    return out;
}

} // namespace av::mpc
