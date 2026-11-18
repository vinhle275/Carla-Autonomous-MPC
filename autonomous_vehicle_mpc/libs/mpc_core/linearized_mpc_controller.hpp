// ==============================================================================
// SUCCESSIVE LINEARIZATION QP-MPC CONTROLLER
// Modern C++20 | Full State-Space Linearization, Separating Hyperplane,
// S-Curve Outer Envelope, Lateral Crossing Hold & Staggered Roadblock
// ==============================================================================

#pragma once

#include <vector>
#include <array>
#include <string>
#include <optional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include "common/autosar_types.hpp"
#include "common/config_constants.hpp"
#include "common/frenet_utils.hpp"
#include "perception_core/road_corridor.hpp"
#include "perception_core/yellow_line_detector.hpp"
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
    LinearizedMpcController(int horizon = config::MPC_N, double dt_mpc = config::DT_MPC);

    void set_global_trajectory(const std::vector<double>& x,
                               const std::vector<double>& y,
                               const std::vector<double>& theta,
                               const std::vector<double>& v);

    void set_corridor_tracker(const perception::RoadCorridorTracker* tracker) {
        corridor_tracker_ = tracker;
    }

    void update_yellow_line_constraint(const perception::LaneDetectionResult& yellow_result,
                                       double dt_loop = config::DT_CORE);

    void reset_nominal();

    MpcOutput solve(const std::array<double, 4>& x_init,
                    const std::vector<double>& x_ref_in,
                    const std::vector<double>& y_ref_in,
                    const std::vector<double>& theta_ref_in,
                    const std::vector<double>& v_ref_in,
                    const std::vector<types::ObstacleFrenetOBB>& detected_obstacles,
                    const types::CorridorBoundsData& corridor_data,
                    double dt_loop = config::DT_CORE);

    // Getters
    bool is_roadblock() const { return is_roadblock_; }
    bool is_roadblock_stopped() const { return roadblock_stopped_latch_; }
    double get_roadblock_barrier_dist() const { return roadblock_barrier_dist_; }
    const std::string& get_roadblock_reason() const { return roadblock_reason_; }
    double get_current_road_min() const { return current_road_min_; }
    double get_current_road_max() const { return current_road_max_; }
    const std::vector<double>& get_x_ref_mod() const { return x_ref_mod_; }
    const std::vector<double>& get_y_ref_mod() const { return y_ref_mod_; }
    const types::ObstacleFrenetOBB* get_active_obstacle() const {
        return has_active_obs_ ? &active_obs_ : nullptr;
    }
    const std::unordered_map<int32_t, types::ObstacleFrenetOBB>& get_active_chain() const {
        return active_chain_;
    }

    static std::pair<std::optional<double>, std::pair<double, double>> select_optimal_shift_from_gaps(
        const std::vector<std::pair<double, double>>& valid_gaps,
        std::optional<double> preferred_sign = std::nullopt,
        std::optional<double> enforce_sign = std::nullopt,
        bool yellow_solid_active = false,
        double yellow_limit = config::STANDARD_ROAD_MIN_M,
        bool has_oncoming_traffic = false,
        double min_gap_width = 0.20
    );

private:
    struct CrossSlice {
        std::vector<types::ObstacleFrenetOBB> obs_list;
        double s_rear{0.0};
        double s_front{0.0};
        std::vector<std::pair<double, double>> blocked;
        std::vector<std::pair<double, double>> gaps;
        double d_target{0.0};
        bool affects_lane{false};
        bool has_min_s_app_start{false};
        double min_s_app_start{-1e9};
    };

    struct CorridorGroup {
        double s_rear{0.0};
        double s_front{0.0};
        double side_sign{1.0};
        double A_shift{0.0};
        double d_target{0.0};
        bool has_min_s_app_start{false};
        double min_s_app_start{-1e9};
    };

    std::array<double, 4> eval_discrete_dynamics(const std::array<double, 4>& x,
                                                 const std::array<double, 2>& u) const;

    void get_linearized_matrices(const std::array<double, 4>& x_bar,
                                 const std::array<double, 2>& u_bar,
                                 std::array<std::array<double, 4>, 4>& A,
                                 std::array<std::array<double, 2>, 4>& B,
                                 std::array<double, 4>& d) const;

    void init_nominal_trajectory(const std::array<double, 4>& x_init,
                                 const std::vector<double>& x_ref,
                                 const std::vector<double>& y_ref,
                                 const std::vector<double>& theta_ref,
                                 const std::vector<double>& v_ref);

    void get_legal_corridor_bounds(std::optional<size_t> idx,
                                   std::optional<double> s,
                                   const types::CorridorBoundsData& corridor_data,
                                   double& out_road_min,
                                   double& out_road_max,
                                   bool& out_is_yellow_solid);

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
                                       double dt_loop,
                                       std::vector<double>& x_mod,
                                       std::vector<double>& y_mod,
                                       std::vector<double>& theta_mod,
                                       bool& has_shift);

    std::vector<double> compute_safe_speed_profile(const std::array<double, 4>& x_init,
                                                   const std::vector<double>& v_ref_base) const;

    bool solve_qp_iteration(const std::array<double, 4>& x_init,
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
                            double& out_cost);

    double calc_span_obstacle_weight(double s_query, double s_rear, double s_front,
                                     bool has_min_s_app = false, double min_s_app_start = -1e9) const;

    int horizon_;
    double dt_;
    double L_{config::WHEELBASE_L};

    std::vector<double> x_traj_;
    std::vector<double> y_traj_;
    std::vector<double> theta_traj_;
    std::vector<double> v_traj_;
    std::vector<double> s_traj_;
    std::vector<double> s_queries_;
    size_t current_traj_idx_{0};
    const perception::RoadCorridorTracker* corridor_tracker_{nullptr};

    // Warm-start state
    bool nom_initialized_{false};
    std::vector<std::array<double, 4>> x_nom_;
    std::vector<std::array<double, 2>> u_nom_;
    std::array<double, 2> u_prev_{0.0, 0.0};

    // Control buffer fallback
    std::vector<double> buffer_steer_;
    std::vector<double> buffer_accel_;
    size_t buffer_idx_{0};

    // Modified reference
    std::vector<double> x_ref_mod_;
    std::vector<double> y_ref_mod_;
    std::vector<double> theta_ref_mod_;

    // Roadblock & State latching
    bool is_roadblock_{false};
    bool roadblock_stopped_latch_{false};
    double roadblock_barrier_dist_{99.0};
    std::optional<double> roadblock_s_barrier_{std::nullopt};
    std::string roadblock_reason_;
    std::string last_log_state_;
    std::unordered_set<int32_t> roadblock_latched_ids_;
    std::unordered_map<int32_t, double> lateral_crossing_timer_s_; // Giữ trễ 2.5s (tương đương 25 bước @ 10Hz)

    // Active obstacle & chain
    bool has_active_obs_{false};
    types::ObstacleFrenetOBB active_obs_{};
    std::unordered_map<int32_t, types::ObstacleFrenetOBB> active_chain_;

    // Yellow line & CV corridor state
    bool yellow_solid_active_{false};
    bool yellow_blocks_left_{false};
    std::optional<double> yellow_road_limit_{std::nullopt};
    double yellow_d_line_{-1.75};
    double yellow_solid_hold_timer_s_{0.0};
    std::optional<double> cv_road_min_{std::nullopt};
    std::optional<double> cv_road_max_{std::nullopt};
    double cv_confidence_{0.0};
    std::string cv_source_{"NONE"};

    double current_road_min_{config::STANDARD_ROAD_MIN_M};
    double current_road_max_{config::STANDARD_ROAD_MAX_M};

    QpSolver qp_solver_;
};

} // namespace av::mpc
