// ==============================================================================
// PROCESS 3: HMI_Diagnostics_App (QM - 10-20Hz)
// - Cockpit HUD & Diagnostics Terminal
// - Black-box SSD Logger (simulation_telemetry.csv)
// ==============================================================================

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include "common/autosar_types.hpp"
#include "ipc_transport/shared_memory_ipc.hpp"

using namespace av;

int main(int argc, char* argv[]) {
    std::cout << "========================================================\n"
              << "  PROCESS 3: HMI_Diagnostics_App (10-20Hz - QM)         \n"
              << "========================================================" << std::endl;

    std::string csv_log_path = "simulation_telemetry.csv";
    if (argc >= 2) csv_log_path = argv[1];

    std::ofstream log_file(csv_log_path);
    if (log_file.is_open()) {
        log_file << "step,time_s,x,y,yaw_rad,v_ego,v_ref,lat_err,head_err,steer,throttle,brake,accel,jerk,solve_time_ms,mode\n";
    }

    ipc::AraComPort<types::TelemetryData> telemetry_sub("TelemetryService", false);

    const auto loop_period = std::chrono::milliseconds(50); // 20Hz
    uint64_t last_time_ns = 0;

    std::cout << std::left
              << std::setw(8)  << "Step"
              << std::setw(12) << "Speed(km/h)"
              << std::setw(12) << "Ref(km/h)"
              << std::setw(12) << "LatErr(m)"
              << std::setw(10) << "Steer"
              << std::setw(10) << "Throttle"
              << std::setw(10) << "Brake"
              << std::setw(12) << "Jerk(m/s3)"
              << std::setw(10) << "QP(ms)"
              << "Status" << "\n"
              << std::string(95, '-') << std::endl;

    while (true) {
        types::TelemetryData telem{};
        uint64_t t_stamp = 0;
        if (telemetry_sub.read(telem, &t_stamp)) {
            if (t_stamp > last_time_ns) {
                last_time_ns = t_stamp;

                double v_kmh = telem.v_ego_m_s * 3.6;
                double v_ref_kmh = telem.v_ref_m_s * 3.6;

                // Ghi vào file log SSD
                if (log_file.is_open()) {
                    log_file << telem.step_count << ","
                             << std::fixed << std::setprecision(4)
                             << (telem.timestamp_ns * 1e-9) << ","
                             << telem.x << "," << telem.y << "," << telem.yaw_rad << ","
                             << telem.v_ego_m_s << "," << telem.v_ref_m_s << ","
                             << telem.lateral_error_m << "," << telem.heading_error_rad << ","
                             << telem.steer_cmd << "," << telem.throttle_cmd << "," << telem.brake_cmd << ","
                             << telem.accel_cmd << "," << telem.jerk_filtered_m_s3 << ","
                             << telem.solve_time_ms << ","
                             << static_cast<int>(telem.mode) << "\n";
                    log_file.flush();
                }

                // In HUD ra màn hình console
                if (telem.step_count % 5 == 0) { // In mỗi 100ms
                    std::cout << std::left
                              << std::setw(8)  << telem.step_count
                              << std::setw(12) << std::fixed << std::setprecision(1) << v_kmh
                              << std::setw(12) << std::fixed << std::setprecision(1) << v_ref_kmh
                              << std::setw(12) << std::fixed << std::setprecision(2) << telem.lateral_error_m
                              << std::setw(10) << std::fixed << std::setprecision(2) << telem.steer_cmd
                              << std::setw(10) << std::fixed << std::setprecision(2) << telem.throttle_cmd
                              << std::setw(10) << std::fixed << std::setprecision(2) << telem.brake_cmd
                              << std::setw(12) << std::fixed << std::setprecision(2) << telem.jerk_filtered_m_s3
                              << std::setw(10) << std::fixed << std::setprecision(1) << telem.solve_time_ms
                              << telem.status_text << "\n";
                }
            }
        }
        std::this_thread::sleep_for(loop_period);
    }

    return 0;
}
