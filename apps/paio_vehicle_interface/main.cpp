// ==============================================================================
// PROCESS 1: PAIO_VehicleInterface_App (ASIL-D - 100Hz, dt=0.01s)
// - CAN / CARLA State & Actor Acquisition (100Hz) & Publish to P2 (50Hz)
// - Watchdog Safe-State Fail-Safe (100ms Timeout -> hold last_valid_steer + brake)
// - Range Check & Physical Actuation (100Hz)
// ==============================================================================

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
#include "common/autosar_types.hpp"
#include "common/config_constants.hpp"
#include "ipc_transport/shared_memory_ipc.hpp"
#include "hal/carla_bridge_client.hpp"

using namespace av;

int main(int argc, char* argv[]) {
    std::cout << "========================================================\n"
              << "  PROCESS 1: PAIO_VehicleInterface_App (100Hz - ASIL-D) \n"
              << "========================================================" << std::endl;

    std::string bridge_ip = "127.0.0.1";
    int bridge_port = 20050;
    if (argc >= 2) bridge_ip = argv[1];
    if (argc >= 3) bridge_port = std::stoi(argv[2]);

    // Khởi tạo HAL Client kết nối CARLA Bridge / CAN Bus
    hal::CarlaBridgeClient hal_client;
    bool connected = hal_client.connect(bridge_ip, bridge_port);
    if (!connected) {
        std::cout << "[PAIO] Lưu ý: Chưa kết nối được CARLA Bridge tại "
                  << bridge_ip << ":" << bridge_port << ". Chạy chế độ mô phỏng SIL nội bộ." << std::endl;
    }

    // Khởi tạo cổng ara::com Zero-Copy Shared Memory
    ipc::AraComPort<types::VehicleStateData> state_pub("VehicleStateService", true);
    ipc::AraComPort<types::PerceptionSensorData> sensor_pub("PerceptionSensorService", true);
    ipc::AraComPort<types::ActuatorCommandData> cmd_sub("ActuatorCommandService", false);

    const auto loop_period = std::chrono::milliseconds(10); // 100Hz
    uint64_t last_cmd_timestamp_ns = 0;
    double last_valid_steer = 0.0;
    uint32_t cycle_count = 0;

    types::ActuatorCommandData current_cmd{};
    current_cmd.brake = 0.18; // Khởi đầu giữ phanh tĩnh

    std::vector<perception::RawActorData> actors;

    while (true) {
        auto t_start = std::chrono::steady_clock::now();
        cycle_count++;

        // 1.1 Đọc trạng thái xe, hành lang bản đồ và khung hình camera từ HAL (100Hz)
        types::VehicleStateData state{};
        types::PerceptionSensorData sensor_pkt{};
        bool read_ok = hal_client.read_vehicle_state(state, actors, &sensor_pkt);
        if (read_ok) {
            uint64_t now_stamp = ipc::get_current_time_ns();
            state.timestamp_ns = now_stamp;
            state.sequence_id = cycle_count;
            state.is_valid = 1;

            // Xuất bản tin sang Process 2 qua Shared Memory mỗi chu kỳ 100Hz
            state_pub.write(state);

            sensor_pkt.timestamp_ns = now_stamp;
            sensor_pkt.sequence_id = cycle_count;
            uint32_t n_act = static_cast<uint32_t>(std::min(actors.size(), types::MAX_RAW_ACTORS));
            sensor_pkt.num_actors = n_act;
            for (uint32_t i = 0; i < n_act; ++i) {
                sensor_pkt.actors[i] = actors[i];
            }
            sensor_pkt.is_valid = 1;
            sensor_pub.write(sensor_pkt);
        } else if (connected && !hal_client.is_connected()) {
            break;
        }

        // 1.2 Đọc non-blocking lệnh chấp hành từ Process 2 (Core)
        types::ActuatorCommandData cmd_rx{};
        uint64_t cmd_time_ns = 0;
        if (cmd_sub.read(cmd_rx, &cmd_time_ns)) {
            if (cmd_rx.is_valid && cmd_time_ns > last_cmd_timestamp_ns) {
                current_cmd = cmd_rx;
                last_cmd_timestamp_ns = cmd_time_ns;
                last_valid_steer = current_cmd.steer;
            }
        }

        // 1.3 Giám sát Watchdog An Toàn 100ms (Fail-Safe Safe-State)
        uint64_t now_ns = ipc::get_current_time_ns();
        if (last_cmd_timestamp_ns > 0 && (now_ns - last_cmd_timestamp_ns) / 1000000ULL > config::WATCHDOG_TIMEOUT_MS) {
            // Quá 100ms không nhận được gói tin từ Process 2: Lập tức kích hoạt phanh an toàn!
            current_cmd.throttle = 0.0;
            current_cmd.brake = 1.0;
            // GIỮ NGUYÊN GÓC LÁI HỢP LỆ CUỐI CÙNG (last_valid_steer), không giật về 0.0 tránh văng ngang mất lái
            current_cmd.steer = last_valid_steer;
            current_cmd.is_emergency_stop = true;
        }

        // 1.4 Range Check & Xuất lệnh vật lý kèm Telemetry xuống CARLA / CAN Bus
        current_cmd.throttle = std::clamp(current_cmd.throttle, 0.0, 1.0);
        current_cmd.brake = std::clamp(current_cmd.brake, 0.0, 1.0);
        current_cmd.steer = std::clamp(current_cmd.steer, -1.0, 1.0);

        hal_client.apply_actuator_control(current_cmd);

        // Duy trì nhịp 100Hz chính xác
        auto t_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
        if (elapsed < loop_period) {
            std::this_thread::sleep_for(loop_period - elapsed);
        }
    }

    return 0;
}
