// ==============================================================================
// UNIFIED AUTONOMOUS MPC RUNNER
// Runs Process 1 (100Hz), Process 2 (50Hz), and Process 3 (20Hz)
// in synchronized real-time threads communicating via ara::com Zero-Copy SHM.
// Pure C++ executable that automatically supervises the CARLA adapter daemon.
// ==============================================================================

#include <iostream>
#include <thread>
#include <atomic>
#include <csignal>
#include <chrono>
#include <vector>
#include <mutex>
#include <string>
#include <cstdlib>

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

// Hàm khởi động CARLA Bridge Daemon ngầm nếu chưa có
bool launch_carla_bridge_daemon(const std::string& carla_host, int carla_port, int bridge_port, bool with_obstacles) {
    std::string obs_flag = with_obstacles ? "--with-obstacles" : "--no-obstacles";
    std::string script_path = resolve_bridge_script();

#ifdef _WIN32
    // Danh sách các trình thông dịch Python trên Windows
    std::vector<std::string> python_candidates = {
        "C:\\Users\\Admin\\AppData\\Local\\Programs\\Python\\Python37\\python.exe",
        "venv\\Scripts\\python.exe",
        "..\\venv\\Scripts\\python.exe",
        "python.exe"
    };

    std::string python_bin = "python.exe";
    for (const auto& cand : python_candidates) {
        DWORD attrib = GetFileAttributesA(cand.c_str());
        if (attrib != INVALID_FILE_ATTRIBUTES && !(attrib & FILE_ATTRIBUTE_DIRECTORY)) {
            python_bin = cand;
            break;
        }
    }

    std::string cmd = "\"" + python_bin + "\" \"" + script_path + "\" --host " +
                      carla_host + " --port " + std::to_string(carla_port) +
                      " --bridge-port " + std::to_string(bridge_port) + " " + obs_flag;

    std::cout << "[SYSTEM] Đang tự động khởi chạy CARLA Simulation Adapter: " << cmd << std::endl;

    STARTUPINFOA si{};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESHOWWINDOW;
    si.wShowWindow = SW_HIDE; // Chạy ngầm trong suốt

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
        // Child process
        std::string p_port = std::to_string(carla_port);
        std::string b_port = std::to_string(bridge_port);
        const char* py_bin = "venv/bin/python3";
        if (access(py_bin, X_OK) != 0) {
            py_bin = "python3";
        }
        char* const args[] = {
            const_cast<char*>(py_bin),
            const_cast<char*>("tools/carla_bridge/carla_bridge_server.py"),
            const_cast<char*>("--host"), const_cast<char*>(carla_host.c_str()),
            const_cast<char*>("--port"), const_cast<char*>(p_port.c_str()),
            const_cast<char*>("--bridge-port"), const_cast<char*>(b_port.c_str()),
            const_cast<char*>(obs_flag.c_str()),
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

    std::cout << "====================================================================\n"
              << "       AUTONOMOUS VEHICLE MPC - PURE C++ AUTOSAR AP RUNTIME        \n"
              << "====================================================================" << std::endl;

    std::string traj_path = "configs/trajectories/reference_trajectory.csv";
    std::string carla_host = "127.0.0.1";
    int carla_port = 2000;
    int bridge_port = 2002;
    bool with_obstacles = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--traj" && i + 1 < argc) traj_path = argv[++i];
        else if (arg == "--host" && i + 1 < argc) carla_host = argv[++i];
        else if (arg == "--port" && i + 1 < argc) carla_port = std::stoi(argv[++i]);
        else if (arg == "--bridge-port" && i + 1 < argc) bridge_port = std::stoi(argv[++i]);
        else if (arg == "--with-obstacles") with_obstacles = true;
        else if (arg == "--no-obstacles") with_obstacles = false;
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

    // Shared Memory Ports
    auto state_port = std::make_shared<ipc::AraComPort<types::VehicleStateData>>("VehicleStateService", true);
    auto cmd_port   = std::make_shared<ipc::AraComPort<types::ActuatorCommandData>>("ActuatorCommandService", true);
    auto telem_port = std::make_shared<ipc::AraComPort<types::TelemetryData>>("TelemetryService", true);

    auto hal_client = std::make_shared<hal::CarlaBridgeClient>();

    // Kết nối tới CARLA Bridge (nếu chưa có, tự động kích hoạt daemon ngầm)
    bool connected = hal_client->connect("127.0.0.1", bridge_port);
    if (!connected) {
        std::cout << "[SYSTEM] Chưa phát hiện Bridge Server. Tự động khởi chạy adapter..." << std::endl;
        launch_carla_bridge_daemon(carla_host, carla_port, bridge_port, with_obstacles);

        // Thử kết nối lại trong vòng 10 giây
        for (int retry = 0; retry < 20 && !connected && g_running; ++retry) {
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

    // Kênh truyền tải danh sách vật cản từ Thread 1 sang Thread 2
    std::mutex actors_mutex;
    std::vector<perception::RawActorData> latest_actors;

    // --------------------------------------------------------------------------
    // THREAD 1: PROCESS 1 - PAIO VEHICLE INTERFACE (100Hz)
    // --------------------------------------------------------------------------
    std::thread p1_thread([&]() {
        uint32_t cycle = 0;
        uint64_t last_cmd_time = ipc::get_current_time_ns();
        double last_valid_steer = 0.0;
        types::ActuatorCommandData current_cmd{};
        current_cmd.brake = 0.18;
        std::vector<perception::RawActorData> actors;

        while (g_running) {
            auto t_start = std::chrono::steady_clock::now();
            cycle++;

            types::VehicleStateData st{};
            if (hal_client->read_vehicle_state(st, actors)) {
                st.timestamp_ns = ipc::get_current_time_ns();
                st.sequence_id = cycle;
                st.is_valid = 1;
                state_port->write(st);

                // Cập nhật danh sách vật cản cho Thread 2
                std::lock_guard<std::mutex> lk(actors_mutex);
                latest_actors = actors;
            }

            types::ActuatorCommandData cmd{};
            uint64_t cmd_time = 0;
            if (cmd_port->read(cmd, &cmd_time) && cmd.is_valid && cmd_time > last_cmd_time) {
                current_cmd = cmd;
                last_cmd_time = cmd_time;
                last_valid_steer = cmd.steer;
            }

            // Watchdog kiểm tra timeout (300ms)
            uint64_t now_ns = ipc::get_current_time_ns();
            if (last_cmd_time > 0 && (now_ns - last_cmd_time) / 1000000ULL > 300) {
                current_cmd.throttle = 0.0;
                current_cmd.brake = 1.0;
                current_cmd.steer = last_valid_steer;
                current_cmd.is_emergency_stop = true;
            }

            hal_client->apply_actuator_control(
                std::clamp(current_cmd.throttle, 0.0, 1.0),
                std::clamp(current_cmd.brake, 0.0, 1.0),
                std::clamp(current_cmd.steer, -1.0, 1.0)
            );

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
        mpc::LinearizedMpcController mpc_ctrl(config::MPC_N, config::DT_CORE);
        mpc_ctrl.set_global_trajectory(traj.x, traj.y, traj.theta, traj.v);

        safety::EmergencySafetyManager safety_mgr;
        perception::ObstacleDetector obs_det;
        perception::RoadCorridorTracker corr_trk;
        corr_trk.build_trajectory_corridor(traj.x, traj.y, traj.theta);
        perception::YellowLineDetector yellow_det;

        size_t current_idx = 0;
        uint32_t step = 0;
        double prev_v = 0.0, prev_a = 0.0, jerk_f = 0.0;

        while (g_running) {
            auto t_start = std::chrono::steady_clock::now();
            step++;

            types::VehicleStateData st{};
            if (!state_port->read(st) || !st.is_valid) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            double x0 = st.x, y0 = st.y, th0_raw = st.yaw_rad, v0 = st.velocity_m_s;
            double a_ego = (v0 - prev_v) / config::DT_CORE;
            double j_raw = (a_ego - prev_a) / config::DT_CORE;
            jerk_f = 0.15 * j_raw + 0.85 * jerk_f;
            prev_v = v0;
            prev_a = a_ego;

            auto proj = math::project_to_frenet(x0, y0, traj.x, traj.y, traj.theta, traj.s, current_idx);
            current_idx = proj.closest_idx;

            if (current_idx >= traj.size() - 2) {
                std::cout << "\n[XE ĐÃ HOÀN THÀNH QUY ĐẠO VÀ VỀ ĐÍCH AN TOÀN]" << std::endl;
                g_running = false;
                break;
            }

            std::vector<double> xr(config::MPC_N), yr(config::MPC_N), thr(config::MPC_N), vr(config::MPC_N);
            for (int k = 0; k < config::MPC_N; ++k) {
                size_t idx = std::min(current_idx + k, traj.size() - 1);
                xr[k] = traj.x[idx];
                yr[k] = traj.y[idx];
                thr[k] = traj.theta[idx];
                vr[k] = traj.v[idx];
            }

            double th0 = thr[0] + math::normalize_angle(th0_raw - thr[0]);
            std::array<double, 4> x_state = {x0, y0, th0, v0};

            auto lane_res = yellow_det.detect_from_features(x0, y0, th0_raw, xr[0], yr[0], thr[0], true);
            auto corr_data = corr_trk.get_horizon_corridor(current_idx, config::MPC_N);

            // Nhận danh sách vật cản từ Thread 1
            std::vector<perception::RawActorData> current_actors;
            {
                std::lock_guard<std::mutex> lk(actors_mutex);
                current_actors = latest_actors;
            }
            auto det_obs = obs_det.process_scan(x0, y0, th0_raw, current_actors, traj.x, traj.y, traj.theta, traj.s, current_idx, config::DT_CORE);
            auto s_info = safety_mgr.update(x0, y0, th0_raw, v0, det_obs, config::DT_CORE);

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
                cmd_out.is_emergency_stop = true;
                mpc_ctrl.reset_nominal();
            } else {
                cmd_out.mode = types::VehicleMode::MODE_1_NORMAL;
                if (s_info.mode_changed) mpc_ctrl.reset_nominal();

                auto t_s = std::chrono::steady_clock::now();
                auto res = mpc_ctrl.solve(x_state, xr, yr, thr, vr, det_obs, corr_data);
                solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_s).count();
                cost = res.cost;

                auto act = safety::ActuationMapper::map_controls(res.steer_rad, res.acceleration_m_s2, v0);
                cmd_out.throttle = act.throttle;
                cmd_out.brake = act.brake;
                cmd_out.steer = act.steer;
                cmd_out.wheel_angle_rad = res.steer_rad;
                cmd_out.acceleration_cmd = res.acceleration_m_s2;
                cmd_out.is_roadblock_latch = mpc_ctrl.is_roadblock_stopped();
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
            telem.mode = cmd_out.mode;
            telem.is_roadblock = mpc_ctrl.is_roadblock();
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
                if (telem.step_count % 5 == 0) {
                    std::cout << "[MPC 50Hz] Step: " << telem.step_count
                              << " | Pos: (" << telem.x << ", " << telem.y << ")"
                              << " | V: " << (telem.v_ego_m_s * 3.6) << "/" << (telem.v_ref_m_s * 3.6) << " km/h"
                              << " | LatErr: " << telem.lateral_error_m << "m"
                              << " | Steer: " << telem.steer_cmd
                              << " | Thr: " << telem.throttle_cmd
                              << " | Brk: " << telem.brake_cmd
                              << " | QP: " << telem.solve_time_ms << "ms"
                              << " | " << telem.status_text << "\r" << std::flush;
                }
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
