// ==============================================================================
// UNIFIED AUTONOMOUS MPC RUNNER
// Runs Process 1 (100Hz), Process 2 (50Hz), and Process 3 (20Hz)
// in synchronized real-time threads communicating via ara::com Zero-Copy SHM.
// Pure C++ executable that automatically supervises the CARLA adapter daemon.
// ==============================================================================

#include <iostream>
#include <iomanip>
#include <thread>
#include <atomic>
#include <csignal>
#include <chrono>
#include <vector>
#include <mutex>
#include <string>
#include <cstdlib>
#include <algorithm>

#ifdef _WIN32
  #include <windows.h>
#else
  #include <unistd.h>
  #include <sys/types.h>
  #include <sys/wait.h>
  #include <signal.h>
#endif

#include "common/autosar_types.hpp"
#include "common/config_constants.hpp"
#include "common/trajectory_loader.hpp"
#include "common/frenet_utils.hpp"
#include "ipc_transport/shared_memory_ipc.hpp"
#include "hal/carla_bridge_client.hpp"
#include "mpc_core/linearized_mpc_controller.hpp"
#include "safety_guard/emergency_safety_manager.hpp"
#include "safety_guard/actuation_mapper.hpp"
#include "perception_core/road_corridor.hpp"
#include "perception_core/obstacle_detector.hpp"
#include "perception_core/yellow_line_detector.hpp"

using namespace av;

static std::atomic<bool> g_running{true};

#ifdef _WIN32
static PROCESS_INFORMATION g_bridge_pi{};
static bool g_bridge_spawned = false;
#else
static pid_t g_bridge_pid = -1;
static bool g_bridge_spawned = false;
#endif

void signal_handler(int) {
    g_running = false;
}

// Hàm tìm đường dẫn file script bridge phù hợp dù chạy từ thư mục gốc hay build/
std::string resolve_bridge_script() {
    std::vector<std::string> candidates = {
        "tools/carla_bridge/carla_bridge_server.py",
        "../tools/carla_bridge/carla_bridge_server.py",
        "tools\\carla_bridge\\carla_bridge_server.py",
        "..\\tools\\carla_bridge\\carla_bridge_server.py"
    };
    for (const auto& c : candidates) {
        FILE* fp = fopen(c.c_str(), "r");
        if (fp) {
            fclose(fp);
            return c;
        }
    }
    return "tools/carla_bridge/carla_bridge_server.py";
}

// Hàm khởi động CARLA Bridge Daemon (kèm Cockpit Viewer) nếu chưa có
bool launch_carla_bridge_daemon(const std::string& carla_host, int carla_port, int bridge_port,
                                bool with_obstacles, bool with_viewer) {
    std::string obs_flag = with_obstacles ? "--with-obstacles" : "--no-obstacles";
    std::string viewer_flag = with_viewer ? "--viewer" : "--no-viewer";
    std::string script_path = resolve_bridge_script();

#ifdef _WIN32
    std::vector<std::string> python_candidates;
    if (const char* userprofile = std::getenv("USERPROFILE")) {
        std::string up(userprofile);
        python_candidates.push_back(up + "\\miniconda3\\envs\\carla_env\\python.exe");
        python_candidates.push_back(up + "\\anaconda3\\envs\\carla_env\\python.exe");
    }
    python_candidates.push_back("C:\\Users\\Admin\\AppData\\Local\\Programs\\Python\\Python37\\python.exe");
    python_candidates.push_back("venv\\Scripts\\python.exe");
    python_candidates.push_back("..\\venv\\Scripts\\python.exe");
    python_candidates.push_back("python.exe");

    std::string python_bin = "python.exe";
    for (const auto& cand : python_candidates) {
        if (cand == "python.exe") break;
        DWORD attrib = GetFileAttributesA(cand.c_str());
        if (attrib != INVALID_FILE_ATTRIBUTES && !(attrib & FILE_ATTRIBUTE_DIRECTORY)) {
            python_bin = cand;
            break;
        }
    }

    std::string cmd = "\"" + python_bin + "\" \"" + script_path + "\" --host " +
                      carla_host + " --port " + std::to_string(carla_port) +
                      " --bridge-port " + std::to_string(bridge_port) + " " +
                      obs_flag + " " + viewer_flag;

    std::cout << "[SYSTEM] Đang tự động khởi chạy CARLA Simulation Adapter & Cockpit Viewer: " << cmd << std::endl;

    STARTUPINFOA si{};
    si.cb = sizeof(si);
    si.dwFlags = 0; // Cho phép hiển thị cửa sổ Pygame Cockpit Viewer trên Windows

    std::vector<char> cmd_buf(cmd.begin(), cmd.end());
    cmd_buf.push_back('\0');

    if (CreateProcessA(NULL, cmd_buf.data(), NULL, NULL, FALSE, 0, NULL, NULL, &si, &g_bridge_pi)) {
        g_bridge_spawned = true;
        return true;
    }
    return false;
#else
    pid_t pid = fork();
    if (pid == 0) {
        std::string p_port = std::to_string(carla_port);
        std::string b_port = std::to_string(bridge_port);

        std::vector<std::string> py_candidates;
        if (const char* home = std::getenv("HOME")) {
            std::string hm(home);
            py_candidates.push_back(hm + "/miniconda3/envs/carla_env/bin/python");
            py_candidates.push_back(hm + "/anaconda3/envs/carla_env/bin/python");
        }
        py_candidates.push_back("/home/tam-phuc/miniconda3/envs/carla_env/bin/python");
        py_candidates.push_back("/home/tam-phuc/anaconda3/envs/carla_env/bin/python");
        py_candidates.push_back("venv/bin/python3");
        py_candidates.push_back("../venv/bin/python3");
        py_candidates.push_back("python3");

        std::string py_bin = "python3";
        for (const auto& cand : py_candidates) {
            if (cand == "python3" || access(cand.c_str(), X_OK) == 0) {
                py_bin = cand;
                break;
            }
        }

        char* const args[] = {
            const_cast<char*>(py_bin.c_str()),
            const_cast<char*>(script_path.c_str()),
            const_cast<char*>("--host"), const_cast<char*>(carla_host.c_str()),
            const_cast<char*>("--port"), const_cast<char*>(p_port.c_str()),
            const_cast<char*>("--bridge-port"), const_cast<char*>(b_port.c_str()),
            const_cast<char*>(obs_flag.c_str()),
            const_cast<char*>(viewer_flag.c_str()),
            nullptr
        };
        execvp(args[0], args);
        _exit(1);
    } else if (pid > 0) {
        g_bridge_pid = pid;
        g_bridge_spawned = true;
        return true;
    }
    return false;
#endif
}

void terminate_carla_bridge_daemon() {
    if (!g_bridge_spawned) return;
    std::cout << "[SYSTEM] Đang dừng tiến trình CARLA Bridge Daemon ngầm..." << std::endl;
#ifdef _WIN32
    if (g_bridge_pi.hProcess) {
        TerminateProcess(g_bridge_pi.hProcess, 0);
        WaitForSingleObject(g_bridge_pi.hProcess, 2000);
        CloseHandle(g_bridge_pi.hProcess);
        CloseHandle(g_bridge_pi.hThread);
        g_bridge_pi = {};
    }
#else
    if (g_bridge_pid > 0) {
        kill(g_bridge_pid, SIGINT);
        int status = 0;
        waitpid(g_bridge_pid, &status, 0);
        g_bridge_pid = -1;
    }
#endif
    g_bridge_spawned = false;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

#ifdef _WIN32
    // Bật hỗ trợ UTF-8 và ANSI escape sequences cho Windows Terminal / PowerShell / CMD
    SetConsoleOutputCP(CP_UTF8);
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hOut != INVALID_HANDLE_VALUE) {
        DWORD dwMode = 0;
        if (GetConsoleMode(hOut, &dwMode)) {
            SetConsoleMode(hOut, dwMode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
        }
    }
#endif

    std::cout << "====================================================================\n"
              << "       AUTONOMOUS VEHICLE MPC - PURE C++ AUTOSAR AP RUNTIME        \n"
              << "====================================================================" << std::endl;

    std::string traj_path = "configs/trajectories/reference_trajectory.csv";
    std::string carla_host = "127.0.0.1";
    int carla_port = 2000;
    int bridge_port = 20050;
    bool with_obstacles = true;
    bool with_viewer = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--traj" && i + 1 < argc) traj_path = argv[++i];
        else if (arg == "--host" && i + 1 < argc) carla_host = argv[++i];
        else if (arg == "--port" && i + 1 < argc) carla_port = std::stoi(argv[++i]);
        else if (arg == "--bridge-port" && i + 1 < argc) bridge_port = std::stoi(argv[++i]);
        else if (arg == "--with-obstacles") with_obstacles = true;
        else if (arg == "--no-obstacles") with_obstacles = false;
        else if (arg == "--viewer") with_viewer = true;
        else if (arg == "--no-viewer") with_viewer = false;
    }

    // Tự động kiểm tra đường dẫn file quỹ đạo nếu chạy từ build/
    {
        FILE* fp = fopen(traj_path.c_str(), "r");
        if (fp) {
            fclose(fp);
        } else {
            std::string alt = "../" + traj_path;
            FILE* fp_alt = fopen(alt.c_str(), "r");
            if (fp_alt) {
                fclose(fp_alt);
                traj_path = alt;
            }
        }
    }

    auto traj = data::TrajectoryLoader::load_reference_trajectory(traj_path, true, config::TARGET_SPEED_MS);
    if (traj.empty()) {
        std::cerr << "[ERROR] Không thể nạp quỹ đạo từ " << traj_path << std::endl;
        return 1;
    }
    std::cout << "[SYSTEM] Đã nạp thành công " << traj.size() << " waypoints từ " << traj_path << std::endl;

    // Shared Memory Ports (ara::com Zero-Copy IPC)
    auto state_port  = std::make_shared<ipc::AraComPort<types::VehicleStateData>>("VehicleStateService", true);
    auto sensor_port = std::make_shared<ipc::AraComPort<types::PerceptionSensorData>>("PerceptionSensorService", true);
    auto cmd_port    = std::make_shared<ipc::AraComPort<types::ActuatorCommandData>>("ActuatorCommandService", true);
    auto telem_port  = std::make_shared<ipc::AraComPort<types::TelemetryData>>("TelemetryService", true);

    auto hal_client = std::make_shared<hal::CarlaBridgeClient>();

    // Kết nối tới CARLA Bridge (nếu chưa có, tự động kích hoạt daemon kèm Cockpit Viewer)
    bool connected = hal_client->connect("127.0.0.1", bridge_port);
    if (!connected) {
        std::cout << "[SYSTEM] Chưa phát hiện Bridge Server. Tự động khởi chạy adapter & Cockpit Viewer..." << std::endl;
        launch_carla_bridge_daemon(carla_host, carla_port, bridge_port, with_obstacles, with_viewer);

        for (int retry = 0; retry < 40 && !connected && g_running; ++retry) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            connected = hal_client->connect("127.0.0.1", bridge_port);
        }
    }

    if (!connected) {
        std::cerr << "[WARNING] Không thể kết nối tới CARLA Bridge! Chuyển sang chế độ SIL nội bộ." << std::endl;
    }

    // Vị trí xuất phát ban đầu
    types::VehicleStateData initial_state{};
    initial_state.x = traj.x[0];
    initial_state.y = traj.y[0];
    initial_state.yaw_rad = traj.theta[0];
    initial_state.velocity_m_s = 0.0;
    initial_state.is_valid = 1;
    state_port->write(initial_state);
    hal_client->set_initial_state(initial_state);

    // --------------------------------------------------------------------------
    // THREAD 1: PROCESS 1 - PAIO VEHICLE INTERFACE (100Hz)
    // --------------------------------------------------------------------------
    std::thread p1_thread([&]() {
        uint32_t cycle = 0;
        uint64_t last_cmd_time = ipc::get_current_time_ns();
        double last_valid_steer = 0.0;
        types::ActuatorCommandData current_cmd{};
        current_cmd.brake = 0.18;
        current_cmd.road_min_m = config::STANDARD_ROAD_MIN_M;
        current_cmd.road_max_m = config::STANDARD_ROAD_MAX_M;
        std::vector<perception::RawActorData> actors;

        while (g_running) {
            auto t_start = std::chrono::steady_clock::now();
            cycle++;

            types::VehicleStateData st{};
            types::PerceptionSensorData sensor_pkt{};
            if (hal_client->read_vehicle_state(st, actors, &sensor_pkt)) {
                uint64_t now_stamp = ipc::get_current_time_ns();
                st.timestamp_ns = now_stamp;
                st.sequence_id = cycle;
                st.is_valid = 1;
                state_port->write(st);

                sensor_pkt.timestamp_ns = now_stamp;
                sensor_pkt.sequence_id = cycle;
                uint32_t n_act = static_cast<uint32_t>(std::min(actors.size(), types::MAX_RAW_ACTORS));
                sensor_pkt.num_actors = n_act;
                for (uint32_t i = 0; i < n_act; ++i) {
                    sensor_pkt.actors[i] = actors[i];
                }
                sensor_pkt.is_valid = 1;
                sensor_port->write(sensor_pkt);
            } else if (connected && !hal_client->is_connected()) {
                // Nếu đóng cửa sổ Cockpit Viewer (hoặc ngắt kết nối CARLA Bridge), kết thúc runtime êm ái
                g_running = false;
                break;
            }

            types::ActuatorCommandData cmd{};
            uint64_t cmd_time = 0;
            if (cmd_port->read(cmd, &cmd_time) && cmd.is_valid && cmd_time > last_cmd_time) {
                current_cmd = cmd;
                last_cmd_time = cmd_time;
                last_valid_steer = cmd.steer;
            }

            // Watchdog kiểm tra timeout (100ms chuẩn config::WATCHDOG_TIMEOUT_MS)
            uint64_t now_ns = ipc::get_current_time_ns();
            if (last_cmd_time > 0 && (now_ns - last_cmd_time) / 1000000ULL > config::WATCHDOG_TIMEOUT_MS) {
                current_cmd.throttle = 0.0;
                current_cmd.brake = 1.0;
                current_cmd.steer = last_valid_steer;
                current_cmd.is_emergency_stop = true;
            }

            current_cmd.throttle = std::clamp(current_cmd.throttle, 0.0, 1.0);
            current_cmd.brake = std::clamp(current_cmd.brake, 0.0, 1.0);
            current_cmd.steer = std::clamp(current_cmd.steer, -1.0, 1.0);

            hal_client->apply_actuator_control(current_cmd);

            auto t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t_start);
            if (t_elapsed < std::chrono::milliseconds(10)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10) - t_elapsed);
            }
        }
    });

    // --------------------------------------------------------------------------
    // THREAD 2: PROCESS 2 - AUTONOMOUS CORE (50Hz)
    // --------------------------------------------------------------------------
    std::thread p2_thread([&]() {
        perception::RoadCorridorTracker corr_trk;
        corr_trk.build_trajectory_corridor(traj.x, traj.y, traj.theta, traj.s);

        mpc::LinearizedMpcController mpc_ctrl(config::MPC_N, config::DT_MPC);
        mpc_ctrl.set_global_trajectory(traj.x, traj.y, traj.theta, traj.v);
        mpc_ctrl.set_corridor_tracker(&corr_trk);

        safety::EmergencySafetyManager safety_mgr;
        perception::ObstacleDetector obs_det;
        perception::YellowLineDetector yellow_det;

        size_t current_idx = 0;
        uint32_t step = 0;
        uint32_t last_state_seq = 0;
        bool has_prev_v = false;
        double prev_v = 0.0, prev_a = 0.0, jerk_f = 0.0;
        auto last_cycle_clock = std::chrono::steady_clock::now();
        auto last_jerk_clock = last_cycle_clock;

        while (g_running) {
            types::VehicleStateData st{};
            if (!state_port->read(st) || !st.is_valid || (connected && st.sequence_id == last_state_seq)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            last_state_seq = st.sequence_id;

            auto t_start = std::chrono::steady_clock::now();
            double dt_loop = std::chrono::duration<double>(t_start - last_cycle_clock).count();
            dt_loop = std::clamp(dt_loop, 0.02, 0.15);
            last_cycle_clock = t_start;
            step++;

            if (!corr_trk.has_precomputed_map()) {
                corr_trk.load_opendrive_corridor_cache();
            }

            double x0 = st.x, y0 = st.y, th0_raw = st.yaw_rad, v0 = st.velocity_m_s;
            if (!has_prev_v) {
                prev_v = v0;
                prev_a = 0.0;
                jerk_f = 0.0;
                has_prev_v = true;
                last_jerk_clock = t_start;
            } else {
                double dt_j = std::chrono::duration<double>(t_start - last_jerk_clock).count();
                if (dt_j >= 0.095) {
                    double a_ego = (v0 - prev_v) / std::max(dt_j, 0.10);
                    double j_raw = (a_ego - prev_a) / std::max(dt_j, 0.10);
                    jerk_f = 0.35 * j_raw + 0.65 * jerk_f;
                    prev_v = v0;
                    prev_a = a_ego;
                    last_jerk_clock = t_start;
                }
            }

            auto proj = math::project_to_frenet(x0, y0, traj.x, traj.y, traj.theta, traj.s, current_idx);
            current_idx = proj.closest_idx;

            if (current_idx >= traj.size() - 2) {
                std::cout << "\n[XE ĐÃ HOÀN THÀNH QUỸ ĐẠO VÀ VỀ ĐÍCH AN TOÀN]" << std::endl;
                g_running = false;
                break;
            }

            std::vector<double> xr, yr, thr, vr;
            math::get_ref_trajectory(traj.x, traj.y, traj.theta, traj.v, traj.s,
                                     current_idx, x0, y0, config::MPC_N, config::DT_MPC,
                                     xr, yr, thr, vr);

            double th0 = thr[0] + math::normalize_angle(th0_raw - thr[0]);
            std::array<double, 4> x_state = {x0, y0, th0, v0};

            // Đọc dữ liệu vật cản, biên bản đồ OpenDRIVE và khung hình Camera RGB từ Shared Memory
            std::vector<perception::RawActorData> current_actors;
            bool is_junction = false;
            bool map_has_yellow = true;
            double map_rmin = config::STANDARD_ROAD_MIN_M;
            double map_rmax = config::STANDARD_ROAD_MAX_M;
            types::PerceptionSensorData sensor_pkt{};
            if (sensor_port->read(sensor_pkt) && sensor_pkt.is_valid) {
                uint32_t n_act = std::min(sensor_pkt.num_actors, static_cast<uint32_t>(types::MAX_RAW_ACTORS));
                current_actors.reserve(n_act);
                for (uint32_t i = 0; i < n_act; ++i) {
                    current_actors.push_back(sensor_pkt.actors[i]);
                }
                is_junction = (sensor_pkt.is_junction != 0);
                map_has_yellow = (sensor_pkt.map_is_yellow_solid != 0);
                map_rmin = sensor_pkt.map_road_min;
                map_rmax = sensor_pkt.map_road_max;
                corr_trk.update_live_map_bounds(current_idx, map_rmin, map_rmax, map_has_yellow, is_junction);
            }

            // Nhận diện làn đường & vạch vàng từ Camera RGB mũi xe (IPM + HSV + Sliding Window) + OpenDRIVE Map fallback
            const uint8_t* rgb_ptr = (sensor_pkt.is_valid && sensor_pkt.has_cam_frame) ? sensor_pkt.cam_rgb.data() : nullptr;
            int cam_w = (sensor_pkt.is_valid && sensor_pkt.has_cam_frame) ? static_cast<int>(sensor_pkt.cam_w) : 0;
            int cam_h = (sensor_pkt.is_valid && sensor_pkt.has_cam_frame) ? static_cast<int>(sensor_pkt.cam_h) : 0;

            auto lane_res = yellow_det.detect_from_rgb_image(
                rgb_ptr, cam_w, cam_h,
                x0, y0, th0_raw, xr[0], yr[0], thr[0],
                map_has_yellow, mpc_ctrl.is_roadblock(), is_junction, dt_loop,
                map_rmin, map_rmax);

            mpc_ctrl.update_yellow_line_constraint(lane_res, dt_loop);

            auto corr_data = corr_trk.get_horizon_corridor(current_idx, config::MPC_N);
            corr_data.is_junction = is_junction;

            auto det_obs = obs_det.process_scan(
                x0, y0, th0_raw, current_actors, traj.x, traj.y, traj.theta, traj.s, current_idx, dt_loop);
            auto s_info = safety_mgr.update(x0, y0, th0_raw, v0, det_obs, dt_loop);

            types::ActuatorCommandData cmd_out{};
            cmd_out.timestamp_ns = ipc::get_current_time_ns();
            cmd_out.sequence_id = step;
            cmd_out.is_valid = true;

            double solve_ms = 0.0;
            double cost = 0.0;

            if (s_info.is_emergency) {
                cmd_out.mode = types::VehicleMode::MODE_2_EMERGENCY_BRAKE;
                cmd_out.throttle = 0.0;
                cmd_out.brake = 1.0;
                cmd_out.steer = 0.0;
                cmd_out.acceleration_cmd = config::MAX_BRAKING_MS2;
                cmd_out.is_emergency_stop = true;
                mpc_ctrl.reset_nominal();
            } else {
                cmd_out.mode = types::VehicleMode::MODE_1_NORMAL;
                if (s_info.mode_changed) mpc_ctrl.reset_nominal();

                auto t_s = std::chrono::steady_clock::now();
                auto res = mpc_ctrl.solve(x_state, xr, yr, thr, vr, det_obs, corr_data, dt_loop);
                solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_s).count();
                cost = res.cost;

                bool rb_latch = mpc_ctrl.is_roadblock_stopped();
                auto act = safety::ActuationMapper::map_controls(res.steer_rad, res.acceleration_m_s2, v0, rb_latch);
                cmd_out.throttle = act.throttle;
                cmd_out.brake = act.brake;
                cmd_out.steer = act.steer;
                cmd_out.wheel_angle_rad = res.steer_rad;
                cmd_out.acceleration_cmd = res.acceleration_m_s2;
                cmd_out.is_roadblock_latch = rb_latch;
            }

            // BẢO VỆ TUYỆT ĐỐI KHÔNG CHẠM VẠCH VÀNG (Yellow Line Safety Shield <= 1.05m - chỉ áp dụng ở Mode 1)
            if (!s_info.is_emergency && lane_res.line_type == types::LineType::SOLID && lane_res.d_yellow_line_m <= 1.05) {
                double v_lat_ego = v0 * std::sin(th0 - thr[0]);
                if (mpc_ctrl.is_roadblock() || (lane_res.d_yellow_line_m <= 1.00 && v_lat_ego < -0.05 && v0 > 0.5)) {
                    cmd_out.throttle = 0.0;
                    cmd_out.brake = 1.0;
                    cmd_out.steer = std::max(0.05, cmd_out.steer);
                } else if (v_lat_ego < -0.02) {
                    double push_right = std::clamp(0.02 + (1.05 - lane_res.d_yellow_line_m) * 0.25, 0.02, 0.06);
                    cmd_out.steer = std::max(push_right, cmd_out.steer);
                }
            }

            // Đóng gói đầy đủ thông tin Telemetry & Horizon 3D cho Cockpit Viewer
            cmd_out.v_ref_m_s = vr[0];
            cmd_out.jerk_filtered_m_s3 = jerk_f;
            cmd_out.road_min_m = mpc_ctrl.get_current_road_min();
            cmd_out.road_max_m = mpc_ctrl.get_current_road_max();
            cmd_out.s_ego_m = proj.s;
            cmd_out.s_total_m = traj.s.empty() ? 0.0 : traj.s.back();
            cmd_out.d_yellow_line_m = lane_res.d_yellow_line_m;
            cmd_out.detect_time_ms = lane_res.detect_time_ms;
            cmd_out.current_idx = static_cast<uint32_t>(current_idx);
            cmd_out.total_idx = static_cast<uint32_t>(traj.size());
            cmd_out.obs_count = static_cast<uint32_t>(det_obs.size());
            const auto* act_obs = mpc_ctrl.get_active_obstacle();
            if (act_obs != nullptr && std::abs(act_obs->d_target) > 0.05) {
                cmd_out.active_obs_id = act_obs->id;
                cmd_out.bypass_side = (act_obs->d_target > 0.0) ? 1 : -1;
            } else {
                cmd_out.active_obs_id = -1;
                cmd_out.bypass_side = 0;
            }
            cmd_out.yellow_line_type = static_cast<uint8_t>(lane_res.line_type);
            cmd_out.yellow_source = (lane_res.source == "CV") ? 1 : ((lane_res.source == "MAP") ? 2 : 0);
            cmd_out.is_roadblock = mpc_ctrl.is_roadblock();

            const auto& hx = mpc_ctrl.get_x_ref_mod();
            const auto& hy = mpc_ctrl.get_y_ref_mod();
            double last_hx = !hx.empty() ? hx.back() : (!xr.empty() ? xr.back() : x0);
            double last_hy = !hy.empty() ? hy.back() : (!yr.empty() ? yr.back() : y0);
            for (size_t k = 0; k < 20; ++k) {
                cmd_out.horizon_x[k] = (k < hx.size()) ? hx[k] : (k < xr.size() ? xr[k] : last_hx);
                cmd_out.horizon_y[k] = (k < hy.size()) ? hy[k] : (k < yr.size() ? yr[k] : last_hy);
            }

            cmd_port->write(cmd_out);

            types::TelemetryData telem{};
            telem.timestamp_ns = cmd_out.timestamp_ns;
            telem.step_count = step;
            telem.x = x0;
            telem.y = y0;
            telem.yaw_rad = th0_raw;
            telem.v_ego_m_s = v0;
            telem.v_ref_m_s = vr[0];
            telem.lateral_error_m = proj.d;
            telem.heading_error_rad = math::normalize_angle(th0_raw - thr[0]);
            telem.steer_cmd = cmd_out.steer;
            telem.throttle_cmd = cmd_out.throttle;
            telem.brake_cmd = cmd_out.brake;
            telem.accel_cmd = cmd_out.acceleration_cmd;
            telem.jerk_filtered_m_s3 = jerk_f;
            telem.qp_cost = cost;
            telem.solve_time_ms = solve_ms;
            telem.road_min_m = cmd_out.road_min_m;
            telem.road_max_m = cmd_out.road_max_m;
            telem.mode = cmd_out.mode;
            telem.is_roadblock = cmd_out.is_roadblock;
            std::snprintf(telem.status_text, sizeof(telem.status_text), "%s", s_info.status_text.c_str());

            telem_port->write(telem);

            auto t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t_start);
            if (t_elapsed < std::chrono::milliseconds(20)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20) - t_elapsed);
            }
        }
    });

    // --------------------------------------------------------------------------
    // THREAD 3: PROCESS 3 - HMI & LOGGING (20Hz)
    // --------------------------------------------------------------------------
    std::thread p3_thread([&]() {
        uint64_t last_time = 0;
        while (g_running) {
            types::TelemetryData telem{};
            uint64_t t_stamp = 0;
            if (telem_port->read(telem, &t_stamp) && t_stamp > last_time) {
                last_time = t_stamp;
                const char* mode_tag = telem.is_roadblock ? "ROADBLOCK" :
                                       (telem.mode == types::VehicleMode::MODE_2_EMERGENCY_BRAKE ? "EMERG_BRK" : "MODE1_MPC");
                std::cout << "\033[2K\r" << std::fixed
                          << "Step:" << telem.step_count
                          << " | V:" << std::setprecision(1) << (telem.v_ego_m_s * 3.6) << "/" << (telem.v_ref_m_s * 3.6) << "km/h"
                          << " | Lat:" << std::setprecision(2) << telem.lateral_error_m << "m"
                          << " | Road: [" << std::showpos << telem.road_min_m << "m; " << telem.road_max_m << "m]" << std::noshowpos
                          << " | Str:" << telem.steer_cmd
                          << " | Thr:" << telem.throttle_cmd
                          << " | Brk:" << telem.brake_cmd
                          << " | QP:" << std::setprecision(2) << telem.solve_time_ms << "ms"
                          << " | " << mode_tag << std::flush;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    if (p1_thread.joinable()) p1_thread.join();
    if (p2_thread.joinable()) p2_thread.join();
    if (p3_thread.joinable()) p3_thread.join();

    hal_client->disconnect();
    terminate_carla_bridge_daemon();

    std::cout << "\n[SYSTEM] Hệ thống tự hành C++ đã dừng hoàn tất và an toàn." << std::endl;
    return 0;
}
