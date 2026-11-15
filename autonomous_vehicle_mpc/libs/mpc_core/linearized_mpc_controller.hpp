// ==============================================================================
// SUCCESSIVE LINEARIZATION QP-MPC CONTROLLER
// Modern C++20 | Hyperplane Obstacle Avoidance & Soft Corridor Constraints
// ==============================================================================

#pragma once

#include <vector>
#include <array>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include "common/autosar_types.hpp"
#include "common/config_constants.hpp"
#include "common/frenet_utils.hpp"
#include "qp_solver.hpp"

namespace av::mpc {

struct MpcOutput {
    double steer_rad{0.0};
    double acceleration_m_s2{0.0};
    double cost{0.0};
    bool is_success{false};
};

class LinearizedMpcController {
public:
    LinearizedMpcController(int horizon = config::MPC_N, double dt = config::DT_CORE);

    void set_global_trajectory(const std::vector<double>& x,
                               const std::vector<double>& y,
                               const std::vector<double>& theta,
                               const std::vector<double>& v);

    void reset_nominal();

    MpcOutput solve(const std::array<double, 4>& x_init,
                    const std::vector<double>& x_ref_in,
                    const std::vector<double>& y_ref_in,
                    const std::vector<double>& theta_ref_in,
                    const std::vector<double>& v_ref_in,
                    const std::vector<types::ObstacleFrenetOBB>& detected_obstacles,
                    const types::CorridorBoundsData& corridor_data);

    // Getters
    bool is_roadblock() const { return is_roadblock_; }
    bool is_roadblock_stopped() const { return roadblock_stopped_latch_; }
    double get_current_road_min() const { return current_road_min_; }
    double get_current_road_max() const { return current_road_max_; }
    const std::vector<double>& get_x_ref_mod() const { return x_ref_mod_; }
    const std::vector<double>& get_y_ref_mod() const { return y_ref_mod_; }
    const types::ObstacleFrenetOBB* get_active_obstacle() const {
        return has_active_obs_ ? &active_obs_ : nullptr;
    }

private:
    void resample_ref_by_speed(const std::array<double, 4>& x_init,
                               const std::vector<double>& x_ref_in,
                               const std::vector<double>& y_ref_in,
                               const std::vector<double>& theta_ref_in,
                               const std::vector<double>& v_ref_in,
                               const std::vector<types::ObstacleFrenetOBB>& detected_obstacles,
                               std::vector<double>& x_sync,
                               std::vector<double>& y_sync,
                               std::vector<double>& theta_sync,
                               std::vector<double>& v_sync);

    void compute_local_reference_shift(const std::array<double, 4>& x_init,
                                       const std::vector<double>& x_ref,
                                       const std::vector<double>& y_ref,
                                       const std::vector<double>& theta_ref,
                                       const std::vector<double>& v_ref,
                                       const std::vector<types::ObstacleFrenetOBB>& detected_obstacles,
                                       const types::CorridorBoundsData& corridor_data,
                                       std::vector<double>& x_mod,
                                       std::vector<double>& y_mod,
                                       std::vector<double>& theta_mod,
                                       bool& has_shift);

    double calc_span_obstacle_weight(double s_query, double s_rear, double s_front, double min_s_app_start = -1e9) const;

    int horizon_;
    double dt_;
    double L_{config::WHEELBASE_L};

    std::vector<double> x_traj_;
    std::vector<double> y_traj_;
    std::vector<double> theta_traj_;
    std::vector<double> v_traj_;
    std::vector<double> s_traj_;
    size_t current_traj_idx_{0};

    // Warm-start state
    std::vector<std::array<double, 4>> x_nom_;
    std::vector<std::array<double, 2>> u_nom_;
    std::array<double, 2> u_prev_{0.0, 0.0};

    // Modified reference
    std::vector<double> x_ref_mod_;
    std::vector<double> y_ref_mod_;
    std::vector<double> theta_ref_mod_;

    // Roadblock & State latching
    bool is_roadblock_{false};
    bool roadblock_stopped_latch_{false};
    double roadblock_barrier_dist_{99.0};
    double roadblock_s_barrier_{0.0};
    std::unordered_set<int32_t> roadblock_latched_ids_;
    std::unordered_map<int32_t, int> lateral_crossing_hold_;

    // Active obstacle
    bool has_active_obs_{false};
    types::ObstacleFrenetOBB active_obs_{};
    std::unordered_map<int32_t, types::ObstacleFrenetOBB> active_chain_;

    double current_road_min_{-3.90};
    double current_road_max_{3.90};

    QpSolver qp_solver_;
};

} // namespace av::mpc
