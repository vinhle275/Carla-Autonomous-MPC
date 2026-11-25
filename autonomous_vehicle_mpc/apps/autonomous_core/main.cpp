// ==============================================================================
// PROCESS 2: Autonomous_Core_App (ASIL-B/D - 50Hz, dt_loop=0.02s, dt_mpc=0.05s, N=20)
// - Perception, Dynamic Frenet Corridor, SAT Danger Zone
// - Successive Linearization QP-MPC (Full SPD Hessian + Hyperplanes) + Anti-Jerk + Limousine Stop
// - Safety Guard 2-Mode State Machine & Actuator Mapping
// ==============================================================================

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
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
    perception::RoadCorridorTracker corridor_tracker;
    corridor_tracker.build_trajectory_corridor(traj.x, traj.y, traj.theta, traj.s);

    mpc::LinearizedMpcController mpc_controller(config::MPC_N, config::DT_MPC);
    mpc_controller.set_global_trajectory(traj.x, traj.y, traj.theta, traj.v);
    mpc_controller.set_corridor_tracker(&corridor_tracker);

    safety::EmergencySafetyManager safety_manager;
    perception::ObstacleDetector obstacle_detector;
    perception::YellowLineDetector yellow_detector;

    // 3. Khởi tạo cổng ara::com Zero-Copy Shared Memory
    ipc::AraComPort<types::VehicleStateData> state_sub("VehicleStateService", false);
    ipc::AraComPort<types::PerceptionSensorData> sensor_sub("PerceptionSensorService", false);
    ipc::AraComPort<types::ActuatorCommandData> cmd_pub("ActuatorCommandService", true);
    ipc::AraComPort<types::TelemetryData> telemetry_pub("TelemetryService", true);

    const auto loop_period = std::chrono::milliseconds(20); // 50Hz (20ms)
    size_t current_idx = 0;
    uint32_t step_count = 0;
    uint32_t last_state_seq = 0;
    bool has_prev_v = false;

    double prev_v = 0.0;
    double prev_accel = 0.0;
    double jerk_filtered = 0.0;
    auto last_cycle_clock = std::chrono::steady_clock::now();
    auto last_jerk_clock = last_cycle_clock;

    std::cout << "[AUTONOMOUS CORE] Đã sẵn sàng. Bắt đầu vòng lặp điều khiển 50Hz..." << std::endl;

    while (true) {
        // 2.1 Thu nhận trạng thái xe từ Process 1 (PAIO) qua ara::com
        types::VehicleStateData state{};
        if (!state_sub.read(state) || !state.is_valid || state.sequence_id == last_state_seq) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        last_state_seq = state.sequence_id;

        auto t_start = std::chrono::steady_clock::now();
        double dt_loop = std::chrono::duration<double>(t_start - last_cycle_clock).count();
        dt_loop = std::clamp(dt_loop, 0.02, 0.15);
        last_cycle_clock = t_start;
        step_count++;

        if (!corridor_tracker.has_precomputed_map()) {
            corridor_tracker.load_opendrive_corridor_cache();
        }

        double x0 = state.x;
        double y0 = state.y;
        double theta0_raw = state.yaw_rad;
        double v0 = state.velocity_m_s;

        // Tính toán gia tốc tức thời và độ giật dọc Jerk (EMA alpha = 0.35, chu kỳ lấy mẫu chuẩn 100ms đồng bộ src/main.py)
        if (!has_prev_v) {
            prev_v = v0;
            prev_accel = 0.0;
            jerk_filtered = 0.0;
            has_prev_v = true;
            last_jerk_clock = t_start;
        } else {
            double dt_j = std::chrono::duration<double>(t_start - last_jerk_clock).count();
            if (dt_j >= 0.095) {
                double accel_ego = (v0 - prev_v) / std::max(dt_j, 0.10);
                double jerk_raw = (accel_ego - prev_accel) / std::max(dt_j, 0.10);
                jerk_filtered = 0.35 * jerk_raw + 0.65 * jerk_filtered;
                prev_v = v0;
                prev_accel = accel_ego;
                last_jerk_clock = t_start;
            }
        }

        // Cập nhật waypoint gần nhất
        auto proj = math::project_to_frenet(x0, y0, traj.x, traj.y, traj.theta, traj.s, current_idx);
        current_idx = proj.closest_idx;

        if (current_idx >= traj.size() - 2) {
            std::cout << "\n========================================================" << std::endl;
            std::cout << "  [AUTONOMOUS CORE] XE ĐÃ HOÀN THÀNH QUỸ ĐẠO VÀ VỀ ĐÍCH AN TOÀN!" << std::endl;
            std::cout << "========================================================" << std::endl;
            types::ActuatorCommandData stop_cmd{};
            stop_cmd.brake = 1.0;
            stop_cmd.throttle = 0.0;
            stop_cmd.steer = 0.0;
            stop_cmd.is_valid = true;
            cmd_pub.write(stop_cmd);

            // Gửi bản tin Telemetry cuối cùng cho HMI Diagnostics hiển thị
            types::TelemetryData final_telem{};
            final_telem.timestamp_ns = ipc::get_current_time_ns();
            final_telem.step_count = step_count;
            final_telem.v_ego_m_s = 0.0;
            final_telem.v_ref_m_s = 0.0;
            final_telem.brake_cmd = 1.0;
            std::snprintf(final_telem.status_text, sizeof(final_telem.status_text), "[VỀ ĐÍCH AN TOÀN]");
            telemetry_pub.write(final_telem);

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            break;
        }

        // Lấy đoạn tham chiếu N bước phía trước bằng nội suy cung đường động (đồng bộ DT_MPC)
        std::vector<double> x_ref, y_ref, theta_ref, v_ref;
        math::get_ref_trajectory(traj.x, traj.y, traj.theta, traj.v, traj.s,
                                 current_idx, x0, y0, config::MPC_N, config::DT_MPC,
                                 x_ref, y_ref, theta_ref, v_ref);

        // Chuẩn hóa góc theta0 liên tục với theta_ref[0]
        double theta0 = theta_ref[0] + math::normalize_angle(theta0_raw - theta_ref[0]);
        std::array<double, 4> x_state = {x0, y0, theta0, v0};

        // 2.2 Nhận dữ liệu vật cản, hành lang bản đồ và khung hình camera từ Process 1 qua Shared Memory
        std::vector<perception::RawActorData> raw_actors;
        bool is_junction = false;
        bool map_has_yellow = true;
        double map_rmin = config::STANDARD_ROAD_MIN_M;
        double map_rmax = config::STANDARD_ROAD_MAX_M;
        types::PerceptionSensorData sensor_pkt{};
        if (sensor_sub.read(sensor_pkt) && sensor_pkt.is_valid) {
            uint32_t n_act = std::min(sensor_pkt.num_actors, static_cast<uint32_t>(types::MAX_RAW_ACTORS));
            raw_actors.reserve(n_act);
            for (uint32_t i = 0; i < n_act; ++i) {
                raw_actors.push_back(sensor_pkt.actors[i]);
            }
            is_junction = (sensor_pkt.is_junction != 0);
            map_has_yellow = (sensor_pkt.map_is_yellow_solid != 0);
            map_rmin = sensor_pkt.map_road_min;
            map_rmax = sensor_pkt.map_road_max;
            corridor_tracker.update_live_map_bounds(current_idx, map_rmin, map_rmax, map_has_yellow, is_junction);
        }

        // 2.3 Nhận diện vạch vàng & Hành lang pháp lý từ Camera RGB mũi xe (IPM + HSV + Sliding Window) + OpenDRIVE Map fallback
        const uint8_t* rgb_ptr = (sensor_pkt.is_valid && sensor_pkt.has_cam_frame) ? sensor_pkt.cam_rgb.data() : nullptr;
        int cam_w = (sensor_pkt.is_valid && sensor_pkt.has_cam_frame) ? static_cast<int>(sensor_pkt.cam_w) : 0;
        int cam_h = (sensor_pkt.is_valid && sensor_pkt.has_cam_frame) ? static_cast<int>(sensor_pkt.cam_h) : 0;

        auto lane_res = yellow_detector.detect_from_rgb_image(
            rgb_ptr, cam_w, cam_h,
            x0, y0, theta0_raw, x_ref[0], y_ref[0], theta_ref[0],
            map_has_yellow, mpc_controller.is_roadblock(), is_junction, dt_loop,
            map_rmin, map_rmax);

        mpc_controller.update_yellow_line_constraint(lane_res, dt_loop);

        auto corridor_data = corridor_tracker.get_horizon_corridor(current_idx, config::MPC_N);
        corridor_data.is_junction = is_junction;

        // 2.4 Quét nhận thức vật cản OBB & Local Object Map
        auto detected_obs = obstacle_detector.process_scan(
            x0, y0, theta0_raw, raw_actors, traj.x, traj.y, traj.theta, traj.s, current_idx, dt_loop);

        // 2.5 Đánh giá an toàn SAT Danger Zone & 2 Chế độ điều khiển
        auto safety_info = safety_manager.update(x0, y0, theta0_raw, v0, detected_obs, dt_loop);

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
            cmd_out.acceleration_cmd = config::MAX_BRAKING_MS2;
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
            auto mpc_res = mpc_controller.solve(x_state, x_ref, y_ref, theta_ref, v_ref, detected_obs, corridor_data, dt_loop);
            auto t_solve_end = std::chrono::steady_clock::now();
            solve_time_ms = std::chrono::duration<double, std::milli>(t_solve_end - t_solve_start).count();
            qp_cost = mpc_res.cost;

            // Ánh xạ tín hiệu sang throttle, brake, steer (kèm chốt dừng tĩnh brake = 0.18)
            bool rb_latch = mpc_controller.is_roadblock_stopped();
            auto act = safety::ActuationMapper::map_controls(mpc_res.steer_rad, mpc_res.acceleration_m_s2, v0, rb_latch);
            cmd_out.throttle = act.throttle;
            cmd_out.brake = act.brake;
            cmd_out.steer = act.steer;
            cmd_out.wheel_angle_rad = mpc_res.steer_rad;
            cmd_out.acceleration_cmd = mpc_res.acceleration_m_s2;
            cmd_out.is_roadblock_latch = rb_latch;
        }

        // Bảo vệ tuyệt đối không chạm vạch vàng (Yellow Line Safety Shield <= 1.05m - chỉ áp dụng ở Mode 1)
        if (!safety_info.is_emergency && lane_res.line_type == types::LineType::SOLID && lane_res.d_yellow_line_m <= 1.05) {
            double v_lat_ego = v0 * std::sin(theta0 - theta_ref[0]);
            if (mpc_controller.is_roadblock() || (lane_res.d_yellow_line_m <= 1.00 && v_lat_ego < -0.05 && v0 > 0.5)) {
                cmd_out.throttle = 0.0;
                cmd_out.brake = 1.0;
                cmd_out.steer = std::max(0.05, cmd_out.steer);
            } else if (v_lat_ego < -0.02) {
                double push_right = std::clamp(0.02 + (1.05 - lane_res.d_yellow_line_m) * 0.25, 0.02, 0.06);
                cmd_out.steer = std::max(push_right, cmd_out.steer);
            }
        }

        // Đóng gói thông tin Telemetry & Horizon 3D cho Cockpit Viewer
        cmd_out.v_ref_m_s = v_ref[0];
        cmd_out.jerk_filtered_m_s3 = jerk_filtered;
        cmd_out.road_min_m = mpc_controller.get_current_road_min();
        cmd_out.road_max_m = mpc_controller.get_current_road_max();
        cmd_out.s_ego_m = proj.s;
        cmd_out.s_total_m = traj.s.empty() ? 0.0 : traj.s.back();
        cmd_out.d_yellow_line_m = lane_res.d_yellow_line_m;
        cmd_out.detect_time_ms = lane_res.detect_time_ms;
        cmd_out.current_idx = static_cast<uint32_t>(current_idx);
        cmd_out.total_idx = static_cast<uint32_t>(traj.size());
        cmd_out.obs_count = static_cast<uint32_t>(detected_obs.size());
        const auto* act_obs = mpc_controller.get_active_obstacle();
        if (act_obs != nullptr && std::abs(act_obs->d_target) > 0.05) {
            cmd_out.active_obs_id = act_obs->id;
            cmd_out.bypass_side = (act_obs->d_target > 0.0) ? 1 : -1;
        } else {
            cmd_out.active_obs_id = -1;
            cmd_out.bypass_side = 0;
        }
        cmd_out.yellow_line_type = static_cast<uint8_t>(lane_res.line_type);
        cmd_out.yellow_source = (lane_res.source == "CV") ? 1 : ((lane_res.source == "MAP") ? 2 : 0);
        cmd_out.is_roadblock = mpc_controller.is_roadblock();

        const auto& hx = mpc_controller.get_x_ref_mod();
        const auto& hy = mpc_controller.get_y_ref_mod();
        double last_hx = !hx.empty() ? hx.back() : (!x_ref.empty() ? x_ref.back() : x0);
        double last_hy = !hy.empty() ? hy.back() : (!y_ref.empty() ? y_ref.back() : y0);
        for (size_t k = 0; k < 20; ++k) {
            cmd_out.horizon_x[k] = (k < hx.size()) ? hx[k] : (k < x_ref.size() ? x_ref[k] : last_hx);
            cmd_out.horizon_y[k] = (k < hy.size()) ? hy[k] : (k < y_ref.size() ? y_ref[k] : last_hy);
        }

        // Xuất bản tin điều khiển sang Process 1 qua ara::com
        cmd_pub.write(cmd_out);

        // 2.6 Đóng gói Telemetry cho Process 3 (HMI)
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
        telem.road_min_m = mpc_controller.get_current_road_min();
        telem.road_max_m = mpc_controller.get_current_road_max();
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
