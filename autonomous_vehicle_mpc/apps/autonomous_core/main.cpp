// ==============================================================================
// PROCESS 2: Autonomous_Core_App (ASIL-B/D - 50Hz, dt=0.05s, N=20)
// - Perception, Dynamic Frenet Corridor, SAT Danger Zone
// - Successive Linearization QP-MPC (OSQP C++) + Anti-Jerk + Limousine Stop
// - Safety Guard 2-Mode State Machine & Actuator Mapping
// ==============================================================================

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include "common/autosar_types.hpp"
#include "common/config_constants.hpp"
#include "common/trajectory_loader.hpp"
#include "common/frenet_utils.hpp"
#include "ipc_transport/shared_memory_ipc.hpp"
#include "mpc_core/linearized_mpc_controller.hpp"
#include "safety_guard/emergency_safety_manager.hpp"
#include "safety_guard/actuation_mapper.hpp"
#include "perception_core/road_corridor.hpp"
#include "perception_core/obstacle_detector.hpp"
#include "perception_core/yellow_line_detector.hpp"

using namespace av;

int main(int argc, char* argv[]) {
    std::cout << "========================================================\n"
              << "  PROCESS 2: Autonomous_Core_App (50Hz - ASIL-B/D)      \n"
              << "========================================================" << std::endl;

    std::string traj_path = "configs/trajectories/reference_trajectory.csv";
    if (argc >= 2) traj_path = argv[1];

    // 1. Nạp quỹ đạo tham chiếu toàn cục
    auto traj = data::TrajectoryLoader::load_reference_trajectory(traj_path, true, config::TARGET_SPEED_MS);
    if (traj.empty()) {
        std::cerr << "[AUTONOMOUS CORE] LỖI: Không nạp được file quỹ đạo: " << traj_path << std::endl;
        return 1;
    }

    // 2. Khởi tạo các module lõi
    mpc::LinearizedMpcController mpc_controller(config::MPC_N, config::DT_CORE);
    mpc_controller.set_global_trajectory(traj.x, traj.y, traj.theta, traj.v);

    safety::EmergencySafetyManager safety_manager;
    perception::ObstacleDetector obstacle_detector;
    perception::RoadCorridorTracker corridor_tracker;
    corridor_tracker.build_trajectory_corridor(traj.x, traj.y, traj.theta);

    perception::YellowLineDetector yellow_detector;

    // 3. Khởi tạo cổng ara::com Shared Memory
    ipc::AraComPort<types::VehicleStateData> state_sub("VehicleStateService", false);
    ipc::AraComPort<types::ActuatorCommandData> cmd_pub("ActuatorCommandService", true);
    ipc::AraComPort<types::TelemetryData> telemetry_pub("TelemetryService", true);

    const auto loop_period = std::chrono::milliseconds(20); // 50Hz (20ms)
    size_t current_idx = 0;
    uint32_t step_count = 0;

    double prev_v = 0.0;
    double prev_accel = 0.0;
    double jerk_filtered = 0.0;

    std::cout << "[AUTONOMOUS CORE] Đã sẵn sàng. Bắt đầu vòng lặp điều khiển 50Hz..." << std::endl;

    while (true) {
        auto t_start = std::chrono::steady_clock::now();
        step_count++;

        // 2.1 Thu nhận trạng thái xe từ Process 1 (PAIO) qua ara::com
        types::VehicleStateData state{};
        bool has_state = state_sub.read(state);
        if (!has_state) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        double x0 = state.x;
        double y0 = state.y;
        double theta0_raw = state.yaw_rad;
        double v0 = state.velocity_m_s;

        // Tính toán gia tốc tức thời và độ giật dọc Jerk (EMA filtered)
        double accel_ego = (v0 - prev_v) / config::DT_CORE;
        double jerk_raw = (accel_ego - prev_accel) / config::DT_CORE;
        jerk_filtered = 0.15 * jerk_raw + 0.85 * jerk_filtered;
        prev_v = v0;
        prev_accel = accel_ego;

        // Cập nhật waypoint gần nhất
        auto proj = math::project_to_frenet(x0, y0, traj.x, traj.y, traj.theta, traj.s, current_idx);
        current_idx = proj.closest_idx;

        if (current_idx >= traj.size() - 2) {
            std::cout << "[AUTONOMOUS CORE] Đã hoàn thành toàn bộ lộ trình!" << std::endl;
            types::ActuatorCommandData stop_cmd{};
            stop_cmd.brake = 1.0;
            stop_cmd.is_valid = true;
            cmd_pub.write(stop_cmd);
            break;
        }

        // Lấy đoạn tham chiếu N bước phía trước
        std::vector<double> x_ref(config::MPC_N), y_ref(config::MPC_N), theta_ref(config::MPC_N), v_ref(config::MPC_N);
        for (int k = 0; k < config::MPC_N; ++k) {
            size_t idx = std::min(current_idx + k, traj.size() - 1);
            x_ref[k] = traj.x[idx];
            y_ref[k] = traj.y[idx];
            theta_ref[k] = traj.theta[idx];
            v_ref[k] = traj.v[idx];
        }

        // Chuẩn hóa góc theta0 liên tục với theta_ref[0]
        double theta0 = theta_ref[0] + math::normalize_angle(theta0_raw - theta_ref[0]);
        std::array<double, 4> x_state = {x0, y0, theta0, v0};

        // 2.2 Nhận diện vạch vàng & Hành lang
        auto lane_res = yellow_detector.detect_from_features(x0, y0, theta0_raw, x_ref[0], y_ref[0], theta_ref[0], true);
        auto corridor_data = corridor_tracker.get_horizon_corridor(current_idx, config::MPC_N);

        // 2.3 Quét nhận thức vật cản OBB & Local Object Map
        std::vector<perception::RawActorData> raw_actors; // Nhận từ cảm biến
        auto detected_obs = obstacle_detector.process_scan(x0, y0, theta0_raw, raw_actors,
                                                          traj.x, traj.y, traj.theta, traj.s,
                                                          current_idx, config::DT_CORE);

        // 2.4 Đánh giá an toàn SAT Danger Zone & 2 Chế độ điều khiển
        auto safety_info = safety_manager.update(x0, y0, theta0_raw, v0, detected_obs, config::DT_CORE);

        types::ActuatorCommandData cmd_out{};
        cmd_out.timestamp_ns = ipc::get_current_time_ns();
        cmd_out.sequence_id = step_count;
        cmd_out.is_valid = true;

        double solve_time_ms = 0.0;
        double qp_cost = 0.0;

        if (safety_info.is_emergency) {
            // =============================================================
            // CHẾ ĐỘ 2: PHANH GẤP TỐI ĐA (EMERGENCY BRAKE)
            // =============================================================
            cmd_out.mode = types::VehicleMode::MODE_2_EMERGENCY_BRAKE;
            cmd_out.throttle = 0.0;
            cmd_out.brake = 1.0;
            cmd_out.steer = 0.0;
            cmd_out.is_emergency_stop = true;
            mpc_controller.reset_nominal();
        } else {
            // =============================================================
            // CHẾ ĐỘ 1: BÁM ĐƯỜNG NÉ VẬT CẢN (QP-MPC CONTROLLER)
            // =============================================================
            cmd_out.mode = types::VehicleMode::MODE_1_NORMAL;
            if (safety_info.mode_changed) {
                mpc_controller.reset_nominal();
            }

            auto t_solve_start = std::chrono::steady_clock::now();
            auto mpc_res = mpc_controller.solve(x_state, x_ref, y_ref, theta_ref, v_ref, detected_obs, corridor_data);
            auto t_solve_end = std::chrono::steady_clock::now();
            solve_time_ms = std::chrono::duration<double, std::milli>(t_solve_end - t_solve_start).count();
            qp_cost = mpc_res.cost;

            // Ánh xạ tín hiệu sang throttle, brake, steer
            auto act = safety::ActuationMapper::map_controls(mpc_res.steer_rad, mpc_res.acceleration_m_s2, v0);
            cmd_out.throttle = act.throttle;
            cmd_out.brake = act.brake;
            cmd_out.steer = act.steer;
            cmd_out.wheel_angle_rad = mpc_res.steer_rad;
            cmd_out.acceleration_cmd = mpc_res.acceleration_m_s2;
            cmd_out.is_roadblock_latch = mpc_controller.is_roadblock_stopped();
        }

        // Xuất bản tin điều khiển sang Process 1 qua ara::com
        cmd_pub.write(cmd_out);

        // 2.5 Đóng gói Telemetry cho Process 3 (HMI)
        types::TelemetryData telem{};
        telem.timestamp_ns = cmd_out.timestamp_ns;
        telem.step_count = step_count;
        telem.x = x0;
        telem.y = y0;
        telem.yaw_rad = theta0_raw;
        telem.v_ego_m_s = v0;
        telem.v_ref_m_s = v_ref[0];
        telem.lateral_error_m = proj.d;
        telem.heading_error_rad = math::normalize_angle(theta0_raw - theta_ref[0]);
        telem.steer_cmd = cmd_out.steer;
        telem.throttle_cmd = cmd_out.throttle;
        telem.brake_cmd = cmd_out.brake;
        telem.accel_cmd = cmd_out.acceleration_cmd;
        telem.jerk_filtered_m_s3 = jerk_filtered;
        telem.qp_cost = qp_cost;
        telem.solve_time_ms = solve_time_ms;
        telem.mode = cmd_out.mode;
        telem.is_roadblock = mpc_controller.is_roadblock();
        std::snprintf(telem.status_text, sizeof(telem.status_text), "%s", safety_info.status_text.c_str());

        telemetry_pub.write(telem);

        // Giữ nhịp 50Hz chính xác
        auto t_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
        if (elapsed < loop_period) {
            std::this_thread::sleep_for(loop_period - elapsed);
        }
    }

    return 0;
}
